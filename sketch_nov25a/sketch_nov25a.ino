// ---------------- CHÂN ----------------
const int STBY_PIN = PB10;

const int PWMA_PIN = PA8;
const int AIN1_PIN = PB12;
const int AIN2_PIN = PB15;

const int PWMB_PIN = PA9;
const int BIN1_PIN = PB14;
const int BIN2_PIN = PB13;

const int sensorPins[8] = {PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0};
const int dipPins[4]    = {PA11, PA12, PB8, PB9};

// ---------------- CẤU HÌNH HIỆU CHUẨN & CHIỀU ----------------
const unsigned long CALIBRATION_TIME_MS = 3000;
const int MIN_SENSOR_SPAN = 200;
const bool BLACK_IS_HIGH_ADC = true; // Đen = ADC cao, Trắng = ADC thấp

bool INVERT_STEERING = false;
bool INVERT_MOTOR_A  = false;
bool INVERT_MOTOR_B  = false;

// ---------------- PWM 8 BIT, PIN 2S VÀ ĐỘNG CƠ 6 V ----------------
const int PWM_FULL_SCALE = 255;
const long MOTOR_RATED_MV = 6000;
const long BATTERY_FULL_MV = 8400;
const int MAX_DRIVE_PWM = PWM_FULL_SCALE * MOTOR_RATED_MV / BATTERY_FULL_MV; // 182
const int BASE_SPEED = 150;
const int SEARCH_OUTER_PWM = 150;
static_assert(BASE_SPEED <= MAX_DRIVE_PWM, "BASE_SPEED vuot gioi han PWM");

// ---------------- THÔNG SỐ PID (TUNE Kp TRƯỚC, Ki = 0, Kd = 0) ----------------
float Kp = 40.0f;
float Ki = 0.0f;
float Kd = 0.0f;

const float DEADBAND_ERROR = 0.03f;
const float ACTIVE_STRENGTH_THRESHOLD = 0.45f;
const float LINE_ACQUIRE_STRENGTH = 0.70f;
const float LINE_HOLD_STRENGTH = 0.55f;
const float TURN_ASSIST_START_ERROR = 0.30f;
const float TURN_ASSIST_FULL_ERROR = 0.62f;
const float SEARCH_DIRECTION_MIN_ERROR = 0.30f;
const float CENTER_ERROR_LIMIT = 0.24f;
const int CENTER_CONFIRM_SAMPLES = 3;
const unsigned long TURN_MEMORY_MS = 60;
const unsigned long SEARCH_TIMEOUT_MS = 180;
const unsigned long LOOP_INTERVAL_US = 2000; // 500 Hz nếu đọc ADC và log kịp
const unsigned long LOG_INTERVAL_MS = 50;

// ---------------- BIẾN TRẠNG THÁI ----------------
enum FollowState { WAIT_FOR_LINE, FOLLOW_LINE, SEARCH_LINE };
FollowState followState = WAIT_FOR_LINE;
int sensorMin[8];
int sensorMax[8];
bool sensorHealthy[8];
float lastStrength[8];

const float weights[8] = {-0.70f, -0.50f, -0.30f, -0.10f, 0.10f, 0.30f, 0.50f, 0.70f};
float lastError = 0.0f;
float integral = 0.0f;
int lastTurnDirection = 0;
int centeredSamples = 0;
unsigned long lastTurnMs = 0;
unsigned long searchStartMs = 0;
unsigned long lastLoopMicros = 0;
unsigned long lastPrintMs = 0;

void calibrateSensors() {
  Serial.println("=== BAT DAU HIEU CHUAN CAM BIEN (3s) ===");
  Serial.println("Den = ADC cao, Trang = ADC thap. Quet qua lai nen trang va vach den...");

  for (int i = 0; i < 8; i++) {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
    sensorHealthy[i] = false;
  }

  unsigned long startMs = millis();
  while (millis() - startMs < CALIBRATION_TIME_MS) {
    for (int i = 0; i < 8; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < sensorMin[i]) sensorMin[i] = val;
      if (val > sensorMax[i]) sensorMax[i] = val;
    }
    delay(2);
  }

  for (int i = 0; i < 8; i++) {
    int span = sensorMax[i] - sensorMin[i];
    sensorHealthy[i] = (span >= MIN_SENSOR_SPAN);
    Serial.print("M");
    Serial.print(i);
    Serial.print(" [");
    Serial.print(sensorMin[i]);
    Serial.print("..");
    Serial.print(sensorMax[i]);
    Serial.print("] span=");
    Serial.print(span);
    Serial.println(sensorHealthy[i] ? " OK" : " LOW_SPAN");
  }
  Serial.println("==========================================");
}

float readBlackStrength(int idx) {
  int val = analogRead(sensorPins[idx]);
  int span = sensorMax[idx] - sensorMin[idx];
  float norm = (float)(val - sensorMin[idx]) / (float)span;
  norm = constrain(norm, 0.0f, 1.0f);

  return BLACK_IS_HIGH_ADC ? norm : (1.0f - norm);
}

bool computeLineError(float &outError, float &outMaxStrength) {
  float weightedSum = 0.0f;
  float weightTotal = 0.0f;
  float maxStrength = 0.0f;

  for (int i = 0; i < 8; i++) {
    float strength = sensorHealthy[i] ? readBlackStrength(i) : 0.0f;
    lastStrength[i] = strength;
    if (strength > maxStrength) {
      maxStrength = strength;
    }

    if (strength >= ACTIVE_STRENGTH_THRESHOLD) {
      weightedSum += strength * weights[i];
      weightTotal += strength;
    }
  }

  outMaxStrength = maxStrength;

  const float lineThreshold = followState == FOLLOW_LINE ? LINE_HOLD_STRENGTH : LINE_ACQUIRE_STRENGTH;
  if (maxStrength < lineThreshold || weightTotal <= 0.001f) {
    outError = 0.0f;
    return false;
  }

  outError = weightedSum / weightTotal;
  return true;
}

void setSingleMotor(int pwmPin, int in1Pin, int in2Pin, bool invertDir, int speed) {
  const int pwm = constrain(speed, 0, MAX_DRIVE_PWM);
  if (pwm == 0) {
    // TB6612FNG: IN1 = IN2 = HIGH để phanh ngắn mạch bánh.
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, HIGH);
  } else {
    digitalWrite(in1Pin, invertDir ? LOW : HIGH);
    digitalWrite(in2Pin, invertDir ? HIGH : LOW);
  }
  analogWrite(pwmPin, pwm);
}

void stepLineFollowing(unsigned long elapsedUs) {
  const unsigned long nowMs = millis();
  float err = 0.0f;
  float maxStrength = 0.0f;
  const bool hasLine = computeLineError(err, maxStrength);
  const float effectiveErr = fabsf(err) < DEADBAND_ERROR ? 0.0f : err;
  const float steeringError = INVERT_STEERING ? -effectiveErr : effectiveErr;
  const float absError = fabsf(steeringError);
  const bool seesCenter = hasLine && absError <= CENTER_ERROR_LIMIT &&
      (lastStrength[3] >= ACTIVE_STRENGTH_THRESHOLD || lastStrength[4] >= ACTIVE_STRENGTH_THRESHOLD);

  if (followState == FOLLOW_LINE && !hasLine) {
    const bool knowsDirection = lastTurnDirection != 0 && nowMs - lastTurnMs <= TURN_MEMORY_MS;
    followState = knowsDirection ? SEARCH_LINE : WAIT_FOR_LINE;
    searchStartMs = nowMs;
    centeredSamples = 0;
    integral = 0.0f;
  }

  if (followState == SEARCH_LINE && nowMs - searchStartMs >= SEARCH_TIMEOUT_MS) {
    followState = WAIT_FOR_LINE;
    lastTurnDirection = 0;
  }

  if (followState != FOLLOW_LINE) {
    centeredSamples = seesCenter ? centeredSamples + 1 : 0;
    if (centeredSamples >= CENTER_CONFIRM_SAMPLES) {
      followState = FOLLOW_LINE;
      lastError = effectiveErr;
      lastTurnDirection = 0;
    }
  }

  int leftCmd = 0;
  int rightCmd = 0;
  float turnAssist = 0.0f;
  if (followState == FOLLOW_LINE) {
    if (absError >= SEARCH_DIRECTION_MIN_ERROR) {
      lastTurnDirection = steeringError > 0.0f ? 1 : -1;
      lastTurnMs = nowMs;
    }

    // Tính I và D theo số chu kỳ 2 ms thực tế, giữ thang hệ số cũ.
    const float elapsedSamples = (float)elapsedUs / LOOP_INTERVAL_US;
    integral = constrain(integral + effectiveErr * elapsedSamples, -10.0f, 10.0f);
    const float derivative = (effectiveErr - lastError) / elapsedSamples;
    float turn = (Kp * effectiveErr) + (Ki * integral) + (Kd * derivative);
    lastError = effectiveErr;
    if (INVERT_STEERING) turn = -turn;

    float left = constrain(BASE_SPEED + turn, 0.0f, (float)MAX_DRIVE_PWM);
    float right = constrain(BASE_SPEED - turn, 0.0f, (float)MAX_DRIVE_PWM);
    turnAssist = constrain((absError - TURN_ASSIST_START_ERROR) /
        (TURN_ASSIST_FULL_ERROR - TURN_ASSIST_START_ERROR), 0.0f, 1.0f);

    // Vạch càng sát mép thì phanh bánh trong càng mạnh. Bánh ngoài vẫn theo PID.
    if (steeringError < 0.0f) left *= 1.0f - turnAssist;
    if (steeringError > 0.0f) right *= 1.0f - turnAssist;
    leftCmd = (int)(left + 0.5f);
    rightCmd = (int)(right + 0.5f);
  } else if (followState == SEARCH_LINE) {
    // Quay quanh bánh trong để tìm lại vạch theo hướng đã thấy.
    if (lastTurnDirection > 0) leftCmd = SEARCH_OUTER_PWM;
    else rightCmd = SEARCH_OUTER_PWM;
  }

  setSingleMotor(PWMA_PIN, AIN1_PIN, AIN2_PIN, INVERT_MOTOR_A, leftCmd);
  setSingleMotor(PWMB_PIN, BIN1_PIN, BIN2_PIN, INVERT_MOTOR_B, rightCmd);

  if (nowMs - lastPrintMs >= LOG_INTERVAL_MS) {
    lastPrintMs = nowMs;
    if (followState == SEARCH_LINE) Serial.print("SEARCH ");
    else if (followState == WAIT_FOR_LINE) Serial.print("STOP ");
    else Serial.print(turnAssist > 0.0f ? "TURN " : "LINE ");
    Serial.print("err=");
    Serial.print(err, 2);
    Serial.print(" | maxS=");
    Serial.print(maxStrength, 2);
    Serial.print(" | L=");
    Serial.print(leftCmd);
    Serial.print(" R=");
    Serial.print(rightCmd);
    Serial.print(" | dt_us=");
    Serial.print(elapsedUs);
    Serial.print(" | s=");
    for (int i = 0; i < 8; i++) {
      char sensorMark = lastStrength[i] >= ACTIVE_STRENGTH_THRESHOLD ? '#' : '.';
      Serial.print(sensorHealthy[i] ? sensorMark : 'x');
    }
    Serial.println();
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, LOW);

  pinMode(PWMA_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);

  pinMode(PWMB_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);

  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT_ANALOG);
  }
  for (int i = 0; i < 4; i++) {
    pinMode(dipPins[i], INPUT_PULLUP);
  }

  analogReadResolution(12);
  analogWriteResolution(8);
  delay(500);

  calibrateSensors();
  delay(1000);
  setSingleMotor(PWMA_PIN, AIN1_PIN, AIN2_PIN, INVERT_MOTOR_A, 0);
  setSingleMotor(PWMB_PIN, BIN1_PIN, BIN2_PIN, INVERT_MOTOR_B, 0);
  digitalWrite(STBY_PIN, HIGH);
  lastLoopMicros = micros();
  lastPrintMs = millis();
}

void loop() {
  unsigned long now = micros();
  unsigned long elapsedUs = now - lastLoopMicros;
  if (elapsedUs < LOOP_INTERVAL_US) return;
  lastLoopMicros = now;

  stepLineFollowing(elapsedUs);
}
