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
const int SEARCH_OUTER_PWM = 100;
const int RUN_BASE_PWM = 60; // Mức nền đã thử trên xe
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
const unsigned long TURN_MEMORY_MS = 320;
const unsigned long FIRST_SWEEP_MS = 2000;
const unsigned long SEARCH_TIMEOUT_MS = FIRST_SWEEP_MS + 2800;
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
int lineFirst = -1;
int lineLast = -1;
bool hasStarted = false;
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
  // Chọn một cụm mắt liên tiếp. Nếu thấy cụm ở tâm, ưu tiên cụm đó.
  // Một đốm đen rời ở mép không kéo trọng tâm của vạch giữa sang bên.
  const float threshold = hasStarted ? LINE_HOLD_STRENGTH : LINE_ACQUIRE_STRENGTH;
  float bestTotal = 0.0f;
  bool bestAtCenter = false;
  lineFirst = lineLast = -1;
  outError = outMaxStrength = 0.0f;
  for (int i = 0; i < 8; ++i) {
    lastStrength[i] = sensorHealthy[i] ? readBlackStrength(i) : 0.0f;
    outMaxStrength = max(outMaxStrength, lastStrength[i]);
  }
  int i = 0;
  while (i < 8) {
    if (lastStrength[i] < ACTIVE_STRENGTH_THRESHOLD) {
      ++i;
      continue;
    }
    const int first = i;
    float total = 0, moment = 0, peak = 0;
    while (i < 8 && lastStrength[i] >= ACTIVE_STRENGTH_THRESHOLD) {
      total += lastStrength[i];
      moment += weights[i] * lastStrength[i];
      peak = max(peak, lastStrength[i]);
      ++i;
    }
    const int last = i - 1;
    const bool atCenter = first <= 4 && last >= 3;
    if (peak < threshold) continue;
    if ((atCenter && !bestAtCenter) || (atCenter == bestAtCenter && total > bestTotal)) {
      bestAtCenter = atCenter;
      bestTotal = total;
      lineFirst = first;
      lineLast = last;
      outError = moment / total;
    } else if (atCenter == bestAtCenter && total == bestTotal &&
               outError * moment < 0.0f) {
      // Hai cụm đối diện mạnh bằng nhau: tránh tự chọn trái do thứ tự quét.
      lineLast = last;
      outError = 0.0f;
    }
  }
  return lineFirst >= 0;
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
  const bool atCenter = hasLine && lineFirst <= 4 && lineLast >= 3 &&
      max(lastStrength[3], lastStrength[4]) >= ACTIVE_STRENGTH_THRESHOLD;
  const bool centered = atCenter && absError <= CENTER_ERROR_LIMIT && lineFirst >= 2 && lineLast <= 5;
  // Nhánh chữ L nối liền mắt giữa và mép về một phía.
  const bool leftWing = atCenter && lineFirst <= 2 && lineLast <= 4;
  const bool rightWing = atCenter && lineLast >= 5 && lineFirst >= 3;
  int wing = rightWing ? 1 : (leftWing ? -1 : 0);
  if (INVERT_STEERING) wing = -wing;

  // Ghi hướng kể cả ở mẫu đầu tiên vừa bắt lại vạch. Mẫu giữa không xóa lịch sử.
  if (hasLine && (hasStarted || centered)) {
    const int side = (steeringError > 0) - (steeringError < 0);
    const bool outsideCenter = lineLast <= 2 || lineFirst >= 5;
    const bool recent = lastTurnDirection != 0 && nowMs - lastTurnMs <= TURN_MEMORY_MS;
    const bool singleCenter = lineFirst == lineLast && (lineFirst == 3 || lineFirst == 4);
    // Mắt giữa đơn lẻ chỉ gợi hướng khi chưa có dấu vết rõ gần đây.
    if (wing != 0 || (side != 0 && (outsideCenter || absError >= SEARCH_DIRECTION_MIN_ERROR ||
                                    (singleCenter && !recent)))) {
      lastTurnDirection = wing != 0 ? wing : side;
      lastTurnMs = nowMs;
    }
  }
  if (followState == FOLLOW_LINE) {
    if (!hasLine) {
      const bool recent = lastTurnDirection != 0 && nowMs - lastTurnMs <= TURN_MEMORY_MS;
      followState = recent ? SEARCH_LINE : WAIT_FOR_LINE;
      if (!recent) lastTurnDirection = 0;
      searchStartMs = nowMs;
      centeredSamples = 0;
      integral = 0;
    }
  } else {
    if (!hasLine && followState == SEARCH_LINE && nowMs - searchStartMs >= SEARCH_TIMEOUT_MS) {
      followState = WAIT_FOR_LINE;
      lastTurnDirection = 0;
      centeredSamples = 0;
    }
    centeredSamples = centered ? centeredSamples + 1 : 0;
    if ((hasStarted && hasLine) || centeredSamples >= CENTER_CONFIRM_SAMPLES) {
      followState = FOLLOW_LINE;
      hasStarted = true;
      lastError = effectiveErr;
    }
  }
  int leftCmd = 0;
  int rightCmd = 0;
  float turnAssist = 0.0f;
  if (followState == FOLLOW_LINE) {


    // Tính I và D theo số chu kỳ 2 ms thực tế, giữ thang hệ số cũ.
    const float elapsedSamples = (float)elapsedUs / LOOP_INTERVAL_US;
    integral = constrain(integral + effectiveErr * elapsedSamples, -10.0f, 10.0f);
    const float derivative = (effectiveErr - lastError) / elapsedSamples;
    float turn = (Kp * effectiveErr) + (Ki * integral) + (Kd * derivative);
    lastError = effectiveErr;
    if (INVERT_STEERING) turn = -turn;

    const int base = min(BASE_SPEED, RUN_BASE_PWM);
    float left = constrain(base + turn, 0.0f, (float)MAX_DRIVE_PWM);
    float right = constrain(base - turn, 0.0f, (float)MAX_DRIVE_PWM);
    turnAssist = constrain((absError - TURN_ASSIST_START_ERROR) /
        (TURN_ASSIST_FULL_ERROR - TURN_ASSIST_START_ERROR), 0.0f, 1.0f);

    // Vạch càng sát mép thì phanh bánh trong càng mạnh. Bánh ngoài vẫn theo PID.
    if (steeringError < 0.0f) left *= 1.0f - turnAssist;
    if (steeringError > 0.0f) right *= 1.0f - turnAssist;
    leftCmd = (int)(left + 0.5f);
    rightCmd = (int)(right + 0.5f);
  } else if (followState == SEARCH_LINE) {
    // Quay quanh bánh trong để tìm lại vạch theo hướng đã thấy.
    int searchDirection = lastTurnDirection;
    if (nowMs - searchStartMs >= FIRST_SWEEP_MS) searchDirection = -searchDirection;
    if (searchDirection > 0) leftCmd = SEARCH_OUTER_PWM;
    else rightCmd = SEARCH_OUTER_PWM;
  }

  setSingleMotor(PWMA_PIN, AIN1_PIN, AIN2_PIN, INVERT_MOTOR_A, leftCmd);
  setSingleMotor(PWMB_PIN, BIN1_PIN, BIN2_PIN, INVERT_MOTOR_B, rightCmd);

  if (nowMs - lastPrintMs >= LOG_INTERVAL_MS) {
    lastPrintMs = nowMs;
    if (followState == SEARCH_LINE) {
      Serial.print(leftCmd > 0 ? "SEARCH_R " : "SEARCH_L ");
      Serial.print("ms=");
      Serial.print(nowMs - searchStartMs);
      Serial.print(" ");
    }
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
