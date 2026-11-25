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
const int RUN_BASE_PWM = 80;
const int STRAIGHT_PWM = RUN_BASE_PWM;
const unsigned long STRAIGHT_HOLD_MS = 150;
const unsigned long STRAIGHT_RAMP_MS = 160;
const int SEARCH_OUTER_PWM = 45;   // Tốc độ tiến khi xoay tìm vạch góc nhọn
const int SEARCH_REVERSE_PWM = 40; // Tốc độ lùi bánh trong khi xoay tại chỗ góc nhọn
static_assert(BASE_SPEED <= MAX_DRIVE_PWM, "BASE_SPEED vuot gioi han PWM");

// ---------------- THÔNG SỐ PID ----------------
float Kp = 55.0f; // Tăng từ 40.0f để ôm cua góc tù dứt khoát hơn
float Ki = 0.0f;
float Kd = 0.0f;

const float DEADBAND_ERROR = 0.03f;
const float ACTIVE_STRENGTH_THRESHOLD = 0.45f;
const float LINE_ACQUIRE_STRENGTH = 0.55f;
const float LINE_HOLD_STRENGTH = 0.55f;
const float TURN_ASSIST_START_ERROR = 0.20f; // Hạ từ 0.30f để hỗ trợ bẻ lái sớm hơn ở góc tù
const float TURN_ASSIST_FULL_ERROR = 0.60f;
const float SEARCH_DIRECTION_MIN_ERROR = 0.10f;
const float CENTER_ERROR_LIMIT = 0.24f;
const int CENTER_CONFIRM_SAMPLES = 1;           // Dừng xoay ngay khi mắt giữa chạm vạch đen
const unsigned long TURN_MEMORY_MS = 400;       // Giữ hướng rẽ PID
const unsigned long OUTER_MEMORY_MS = 600;      // Giữ hướng cảm biến mép cho góc nhọn / góc vuông
const unsigned long SEARCH_TIMEOUT_MS = 2400;
const int SEARCH_CENTER_SAMPLES = 3; // Xác nhận bắt lại tâm khi đang xoay
const unsigned long LOOP_INTERVAL_US = 2000;    // 500 Hz nếu đọc ADC và log kịp
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
float lastError = 0.0f;
float integral = 0.0f;
int lastTurnDirection = 0;
int lastOuterSide = 0; // Ghi nhớ riêng cảm biến mép thấy line đen (-1: trái, 1: phải)
int centeredSamples = 0;
bool hasStarted = false;
bool onStraight = false;
unsigned long straightSinceMs = 0;
unsigned long lastTurnMs = 0;
unsigned long lastOuterMs = 0;
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
  const float threshold = followState == FOLLOW_LINE ? LINE_HOLD_STRENGTH : LINE_ACQUIRE_STRENGTH;
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
    }
  }
  return lineFirst >= 0;
}

void setSingleMotor(int pwmPin, int in1Pin, int in2Pin, bool invertDir, int speed) {
  const int cmd = constrain(speed, -MAX_DRIVE_PWM, MAX_DRIVE_PWM);
  if (cmd == 0) {
    // TB6612FNG: IN1 = IN2 = HIGH để phanh ngắn mạch bánh.
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, 0);
  } else if (cmd > 0) {
    digitalWrite(in1Pin, invertDir ? LOW : HIGH);
    digitalWrite(in2Pin, invertDir ? HIGH : LOW);
    analogWrite(pwmPin, cmd);
  } else {
    // Quay ngược bánh để xoay tại chỗ khi vào góc nhọn (< 90 độ)
    digitalWrite(in1Pin, invertDir ? HIGH : LOW);
    digitalWrite(in2Pin, invertDir ? LOW : HIGH);
    analogWrite(pwmPin, -cmd);
  }
}

void stepLineFollowing(unsigned long elapsedUs) {
  const unsigned long nowMs = millis();
  float err = 0.0f;
  float maxStrength = 0.0f;
  const bool hasLine = computeLineError(err, maxStrength);
  const float effectiveErr = fabsf(err) < DEADBAND_ERROR ? 0.0f : err;
  const float steeringError = INVERT_STEERING ? -effectiveErr : effectiveErr;
  const float absError = fabsf(steeringError);
  const bool atCenter = hasLine && lineFirst <= 4 && lineLast >= 3;
  const bool centered = atCenter && absError <= CENTER_ERROR_LIMIT && lineFirst >= 2 && lineLast <= 5;

  // Bắt cảm biến mép trái/phải thấy vạch đen
  const bool outerLeft = lastStrength[0] >= LINE_HOLD_STRENGTH || lastStrength[1] >= LINE_HOLD_STRENGTH;
  const bool outerRight = lastStrength[6] >= LINE_HOLD_STRENGTH || lastStrength[7] >= LINE_HOLD_STRENGTH;

  if (followState == FOLLOW_LINE) {
    // Ghi nhớ độc lập bên mép thấy line đen cho góc vuông và góc nhọn
    if (outerLeft != outerRight) {
      int outerSide = outerRight ? 1 : -1;
      if (INVERT_STEERING) outerSide = -outerSide;
      lastOuterSide = outerSide;
      lastOuterMs = nowMs;
    } else if (nowMs - lastOuterMs > OUTER_MEMORY_MS) {
      lastOuterSide = 0;
    }

    if (nowMs - lastTurnMs > TURN_MEMORY_MS) {
      lastTurnDirection = 0;
    }

    if (hasLine && absError >= SEARCH_DIRECTION_MIN_ERROR) {
      lastTurnDirection = steeringError > 0.0f ? 1 : -1;
      lastTurnMs = nowMs;
      // Một cụm đã sang hẳn phía mới có giá trị hơn ký ức mép phía cũ.
      if (!outerLeft && !outerRight && !atCenter && lastOuterSide != lastTurnDirection) {
        lastOuterSide = 0;
      }
    }

    // Chỉ chuyển sang SEARCH_LINE khi thực sự mất vạch.
    // Khi còn thấy vạch (góc tù, cua cong), luôn để PID + turnAssist lái mượt.
    if (!hasLine) {
      if (lastOuterSide != 0 && nowMs - lastOuterMs <= OUTER_MEMORY_MS) {
        lastTurnDirection = lastOuterSide;
      } else if (lastTurnDirection == 0 && fabsf(lastError) >= DEADBAND_ERROR) {
        float prevSteer = INVERT_STEERING ? -lastError : lastError;
        lastTurnDirection = prevSteer > 0.0f ? 1 : -1;
      }
      followState = lastTurnDirection != 0 ? SEARCH_LINE : WAIT_FOR_LINE;
      searchStartMs = nowMs;
      centeredSamples = 0;
      integral = 0;
    }
  } else {
    const bool searchExpired = followState == SEARCH_LINE && nowMs - searchStartMs >= SEARCH_TIMEOUT_MS;
    const bool searching = followState == SEARCH_LINE;
    const bool clearCenter = centered && (!searching || (!outerLeft && !outerRight));
    const bool waitingAfterRun = !searching && hasStarted;
    const bool canResume = waitingAfterRun ? hasLine : clearCenter;
    centeredSamples = canResume ? centeredSamples + 1 : 0;
    const int requiredSamples = hasStarted ? SEARCH_CENTER_SAMPLES : CENTER_CONFIRM_SAMPLES;
    const bool edgeInTurnDirection = !atCenter && steeringError * lastTurnDirection > 0;
    const bool recoverAtEdge = searchExpired && hasLine && edgeInTurnDirection;
    if (centeredSamples >= requiredSamples || recoverAtEdge) {
      // Nếu vừa qua tâm rồi lại mất vạch, còn hướng để tiếp tục tìm.
      if ((!searching || recoverAtEdge) && absError >= SEARCH_DIRECTION_MIN_ERROR) {
        lastTurnDirection = steeringError > 0 ? 1 : -1;
      }
      if (lastTurnDirection != 0) lastTurnMs = nowMs;
      followState = FOLLOW_LINE;
      hasStarted = true;
      lastError = effectiveErr;
      lastOuterSide = 0;
    } else if (searchExpired) {
      followState = WAIT_FOR_LINE;
      lastTurnDirection = 0;
      lastOuterSide = 0;
      centeredSamples = 0;
    }
  }

  int leftCmd = 0;
  int rightCmd = 0;
  float turnAssist = 0.0f;
  const bool straight = followState == FOLLOW_LINE && centered && absError <= 0.10f && !outerLeft && !outerRight;
  if (straight && !onStraight) straightSinceMs = nowMs;
  onStraight = straight;
  if (followState == FOLLOW_LINE) {
    // Tính I và D theo số chu kỳ 2 ms thực tế, giữ thang hệ số cũ.
    const float elapsedSamples = (float)elapsedUs / LOOP_INTERVAL_US;
    integral = constrain(integral + effectiveErr * elapsedSamples, -10.0f, 10.0f);
    const float derivative = (effectiveErr - lastError) / elapsedSamples;
    float turn = (Kp * effectiveErr) + (Ki * integral) + (Kd * derivative);
    lastError = effectiveErr;
    if (INVERT_STEERING) turn = -turn;

    int base = min(BASE_SPEED, RUN_BASE_PWM);
    const unsigned long straightAge = nowMs - straightSinceMs;
    if (straight && straightAge > STRAIGHT_HOLD_MS) {
      const unsigned long ramp = min(straightAge - STRAIGHT_HOLD_MS, STRAIGHT_RAMP_MS);
      base += (min(BASE_SPEED, STRAIGHT_PWM) - base) * ramp / STRAIGHT_RAMP_MS;
    }
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
    // Xoay tại chỗ khi mất vạch ở đỉnh góc nhọn / góc vuông.
    const int fwd = min(SEARCH_OUTER_PWM, RUN_BASE_PWM);
    const int rev = -min(SEARCH_REVERSE_PWM, RUN_BASE_PWM);
    if (lastTurnDirection > 0) {
      leftCmd = fwd;
      rightCmd = rev;
    } else {
      leftCmd = rev;
      rightCmd = fwd;
    }
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
