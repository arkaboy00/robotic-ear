#include <Wire.h>
#include <AS5600.h>
#include <Preferences.h>

// ===== КОНФИГУРАЦИЯ =====
#define NUM_MOTORS 3
#define NUM_EARS 1
#define MOTORS_PER_EAR 3

#define MUX_ADDR 0x70
#define MCP_ADDR 0x20

// ШИМ пины ESP32 (первые 3 для прототипа)
const int pwmPins[NUM_MOTORS] = {13, 14, 15};

// Пины MCP23017 для направления: {IN1, IN2} (порт A)
const int dirPins[NUM_MOTORS][2] = {
  {0, 1},   // мотор 0
  {2, 3},   // мотор 1
  {4, 5}    // мотор 2
};

// Каналы мультиплексора для энкодеров моторов (0,1,2)
const int muxChannels[NUM_MOTORS] = {0, 1, 2};

// ===== НАСТРОЙКИ ПУЛЬТОВЫХ ЭНКОДЕРОВ (ОДИН МУЛЬТИПЛЕКСОР) =====
#define NUM_CONTROL_ENCODERS 3
#define CONTROL_MUX_ADDR MUX_ADDR          // используем тот же мультиплексор 0x70
const int controlMuxChannels[NUM_CONTROL_ENCODERS] = {3, 4, 5}; // свободные каналы

// ===== ПАРАМЕТРЫ ШИМ =====
#define PWM_FREQ 25000
#define PWM_RES 10
#define PWM_MAX ((1 << PWM_RES) - 1)

// ===== ПИД И ГИСТЕРЕЗИС (основной режим) =====
float Kp = 1.2, Ki = 0.4, Kd = 0.01;
int32_t HYS_LOW = 10;
int32_t HYS_HIGH = 40;

// ===== ПИД И ГИСТЕРЕЗИС (режим ассистента) =====
float assistKp = 0.3;
float assistKi = 0.05;
float assistKd = 0.0;
int32_t assistHysLow = 30;
int32_t assistHysHigh = 300;

// ===== PREFERENCES =====
Preferences preferences;

// ===== СТРУКТУРА МОТОРА =====
struct Motor {
  int pwmPin;
  int mcpAddr;
  int dirPin1, dirPin2;
  int muxChannel;

  int32_t targetTicks;
  int32_t position;          // накопленное абсолютное положение в тиках (может быть >4095)
  int32_t prevRaw;           // предыдущее сырое значение энкодера (0..4095)
  int32_t currentRaw;        // текущее сырое значение
  int32_t zeroRaw;           // сырое значение, соответствующее нулю (калибровка)

  float integral;
  float prevError;
  bool motorEnabled;
  unsigned long lastTime;

  float Kp, Ki, Kd;
  int32_t hysLow, hysHigh;

  bool assistantMode;
  float assistKp, assistKi, assistKd;
  int32_t assistHysLow, assistHysHigh;

  bool encoderInitialized;   // флаг, что encoder.begin() вызван для этого мотора
};

Motor motors[NUM_MOTORS];
AS5600 encoder;

// ===== СТРУКТУРА ПУЛЬТОВОГО ЭНКОДЕРА =====
struct ControlEncoder {
  int muxChannel;
  int32_t raw;
  int32_t prevRaw;
  int32_t baseRaw;       // запоминаем при входе в режим
  int32_t baseTarget;    // соответствующие targetTicks мотора
  int32_t position;      // накопленное смещение от базовой точки
  bool initialized;      // флаг инициализации
};

ControlEncoder controlEncoders[NUM_CONTROL_ENCODERS];
bool calibModeActive = false;      // активен ли режим ручного управления
int currentCalibEar = 0;           // ухо, которое сейчас калибруется (0..3)

// ===== КНОПКИ (пины) =====
const int btnCalibPin = 33;   // вход в калибровку / сохранение
const int btnNextPin = 32;    // переключение уха
const int btnReservePin = 35; // резерв

// ===== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ =====
void selectChannel(uint8_t channel, uint8_t muxAddr = MUX_ADDR) {
  if (channel > 7) return;
  Wire.beginTransmission(muxAddr);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delayMicroseconds(2000);
}

void mcpWriteRegister(int addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void setMotor(Motor &m, int speed) {
  speed = constrain(speed, -PWM_MAX, PWM_MAX);
  uint8_t gpioa = 0;
  if (speed > 0) {
    gpioa = (1 << m.dirPin1) | (0 << m.dirPin2);
  } else if (speed < 0) {
    gpioa = (0 << m.dirPin1) | (1 << m.dirPin2);
  } else {
    gpioa = 0;
  }
  mcpWriteRegister(m.mcpAddr, 0x12, gpioa);
  ledcWrite(m.pwmPin, abs(speed));
}

// ===== ОБНОВЛЕНИЕ ЭНКОДЕРА МОТОРА =====
void updateEncoder(Motor &m) {
  selectChannel(m.muxChannel);

  if (!m.encoderInitialized) {
    if (encoder.begin()) {
      m.encoderInitialized = true;
    } else {
      static bool errorPrinted[NUM_MOTORS] = {false};
      if (!errorPrinted[m.muxChannel]) {
        Serial.print("Motor encoder init FAILED on channel ");
        Serial.println(m.muxChannel);
        errorPrinted[m.muxChannel] = true;
      }
      return;
    }
  }

  int32_t raw = encoder.rawAngle();
  m.currentRaw = raw;

  if (m.prevRaw == -1) {
    m.prevRaw = raw;
    m.position = 0;
    return;
  }

  int32_t diff = raw - m.prevRaw;
  if (diff > 2048) diff -= 4096;
  else if (diff < -2048) diff += 4096;
  m.position += diff;
  m.prevRaw = raw;
}

// ===== ОБНОВЛЕНИЕ МОТОРА (PID) =====
void updateMotor(Motor &m, float dt) {
  updateEncoder(m);

  int32_t error = m.targetTicks - m.position;
  int32_t absError = abs(error);

  float Kp_eff = m.assistantMode ? m.assistKp : m.Kp;
  float Ki_eff = m.assistantMode ? m.assistKi : m.Ki;
  float Kd_eff = m.assistantMode ? m.assistKd : m.Kd;
  int32_t hysLow_eff = m.assistantMode ? m.assistHysLow : m.hysLow;
  int32_t hysHigh_eff = m.assistantMode ? m.assistHysHigh : m.hysHigh;

  if (m.motorEnabled) {
    if (absError < hysLow_eff) {
      m.motorEnabled = false;
      setMotor(m, 0);
      m.integral = 0.0;
      m.prevError = 0.0;
      return;
    }
  } else {
    if (absError > hysHigh_eff) {
      m.motorEnabled = true;
      m.integral = 0.0;
      m.prevError = 0.0;
    } else {
      return;
    }
  }

  float P = Kp_eff * error;
  m.integral += error * dt;
  m.integral = constrain(m.integral, -100.0, 100.0);
  float I = Ki_eff * m.integral;
  float derivative = (error - m.prevError) / dt;
  float D = Kd_eff * derivative;
  m.prevError = error;

  float output = P + I + D;
  output = constrain(output, -PWM_MAX, PWM_MAX);
  setMotor(m, (int)output);
}

// ===== КАЛИБРОВКА НУЛЯ =====
void calibrateZero(int idx) {
  if (idx < 0 || idx >= NUM_MOTORS) {
    Serial.println("ERR: Invalid motor index");
    return;
  }
  Motor &m = motors[idx];
  updateEncoder(m);
  m.zeroRaw = m.currentRaw;

  preferences.begin("motors", false);
  String key = "zero" + String(idx);
  preferences.putInt(key.c_str(), m.zeroRaw);
  preferences.end();

  m.prevRaw = m.zeroRaw;
  m.position = 0;
  m.targetTicks = 0;
  setMotor(m, 0);
  m.motorEnabled = false;
  m.integral = 0.0;
  m.prevError = 0.0;

  Serial.print("M"); Serial.print(idx);
  Serial.print(" zero calibrated at raw="); Serial.println(m.zeroRaw);
}

void loadZeroCalibration() {
  preferences.begin("motors", false);
  for (int i = 0; i < NUM_MOTORS; i++) {
    String key = "zero" + String(i);
    if (preferences.isKey(key.c_str())) {
      motors[i].zeroRaw = preferences.getInt(key.c_str(), 0);
      motors[i].prevRaw = motors[i].zeroRaw;
      motors[i].position = 0;
      Serial.print("M"); Serial.print(i);
      Serial.print(" zero loaded: raw="); Serial.println(motors[i].zeroRaw);
    } else {
      motors[i].zeroRaw = 0;
      motors[i].prevRaw = -1;
      Serial.print("M"); Serial.print(i);
      Serial.println(" no zero calibration found, using raw=0");
    }
  }
  preferences.end();
}

void printZeroStatus() {
  Serial.println("=== Zero calibration status ===");
  for (int i = 0; i < NUM_MOTORS; i++) {
    Serial.print("M"); Serial.print(i);
    Serial.print(" zeroRaw="); Serial.println(motors[i].zeroRaw);
  }
}

// ===== УПРАВЛЕНИЕ АССИСТЕНТОМ =====
void setAssistant(int idx, bool enable) {
  if (idx < 0 || idx >= NUM_MOTORS) {
    Serial.println("ERR: Invalid motor index");
    return;
  }
  motors[idx].assistantMode = enable;
  if (enable) {
    motors[idx].integral = 0.0;
    motors[idx].prevError = 0.0;
    motors[idx].motorEnabled = false;
    setMotor(motors[idx], 0);
    Serial.print("M"); Serial.print(idx); Serial.println(" assistant ON");
  } else {
    Serial.print("M"); Serial.print(idx); Serial.println(" assistant OFF");
  }
}

void setAssistantAll(bool enable) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    setAssistant(i, enable);
  }
}

void printAssistStatus() {
  Serial.println("=== Assistant settings ===");
  Serial.print("assistKp="); Serial.println(assistKp);
  Serial.print("assistKi="); Serial.println(assistKi);
  Serial.print("assistKd="); Serial.println(assistKd);
  Serial.print("assistHysLow="); Serial.println(assistHysLow);
  Serial.print("assistHysHigh="); Serial.println(assistHysHigh);
}

// ===== ПУЛЬТОВЫЕ ЭНКОДЕРЫ =====
void initControlEncoders() {
  for (int i = 0; i < NUM_CONTROL_ENCODERS; i++) {
    controlEncoders[i].muxChannel = controlMuxChannels[i];
    controlEncoders[i].raw = 0;
    controlEncoders[i].prevRaw = -1;
    controlEncoders[i].baseRaw = 0;
    controlEncoders[i].baseTarget = 0;
    controlEncoders[i].position = 0;
    controlEncoders[i].initialized = false;
  }
}

void updateControlEncoders() {
  if (!calibModeActive) return;

  for (int i = 0; i < NUM_CONTROL_ENCODERS; i++) {
    ControlEncoder &ce = controlEncoders[i];
    selectChannel(ce.muxChannel, CONTROL_MUX_ADDR);

    if (!ce.initialized) {
      if (encoder.begin()) {
        ce.initialized = true;
      } else {
        static bool errorPrinted[8] = {false};
        if (!errorPrinted[ce.muxChannel]) {
          Serial.print("Control encoder init FAILED on channel ");
          Serial.println(ce.muxChannel);
          errorPrinted[ce.muxChannel] = true;
        }
        continue;
      }
    }

    int32_t raw = encoder.rawAngle();
    ce.raw = raw;

    if (ce.prevRaw == -1) {
      ce.prevRaw = raw;
      ce.position = 0;
      continue;
    }

    int32_t diff = raw - ce.prevRaw;
    if (diff > 2048) diff -= 4096;
    else if (diff < -2048) diff += 4096;
    ce.position += diff;
    ce.prevRaw = raw;

    if (diff != 0) {
      int motorIndex = currentCalibEar * MOTORS_PER_EAR + i;
      if (motorIndex < NUM_MOTORS) {
        int32_t newTarget = ce.baseTarget + ce.position;
        motors[motorIndex].targetTicks = newTarget;
        motors[motorIndex].motorEnabled = true;
      }
    }
  }
}

void enterManualControl(int ear) {
  if (ear < 0 || ear >= NUM_EARS) return;
  currentCalibEar = ear;
  calibModeActive = true;

  for (int i = 0; i < NUM_CONTROL_ENCODERS; i++) {
    int motorIndex = ear * MOTORS_PER_EAR + i;
    controlEncoders[i].baseTarget = motors[motorIndex].targetTicks;
    controlEncoders[i].baseRaw = controlEncoders[i].raw;
    controlEncoders[i].position = 0;
    controlEncoders[i].prevRaw = controlEncoders[i].raw;
  }

  Serial.print("Manual control enabled for ear ");
  Serial.println(ear);
}

void exitManualControl() {
  calibModeActive = false;
  Serial.println("Manual control disabled");
}

// ===== ОБРАБОТКА КНОПОК =====
void processButtons() {
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50;
  static bool lastCalibState = HIGH;
  static bool lastNextState = HIGH;
  static bool calibPressed = false;
  static bool nextPressed = false;

  bool calib = digitalRead(btnCalibPin);
  bool next = digitalRead(btnNextPin);

  // === КНОПКА 1 (калибровка/сохранение) ===
  if (calib != lastCalibState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (calib == LOW && !calibPressed) {
      calibPressed = true;
      if (!calibModeActive) {
        enterManualControl(0);
      } else {
        Serial.print("Save current position for ear ");
        Serial.println(currentCalibEar);
        // Здесь будет сохранение в Preferences
      }
    }
    if (calib == HIGH && calibPressed) {
      calibPressed = false;
    }
  }

  // === КНОПКА 2 (переключение уха) ===
  if (next != lastNextState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (next == LOW && !nextPressed) {
      nextPressed = true;
      if (calibModeActive) {
        int nextEar = currentCalibEar + 1;
        if (nextEar >= NUM_EARS) {
          exitManualControl();
        } else {
          enterManualControl(nextEar);
        }
      }
    }
    if (next == HIGH && nextPressed) {
      nextPressed = false;
    }
  }

  lastCalibState = calib;
  lastNextState = next;
}

// ===== ОБРАБОТКА СЕРИАЛЬНЫХ КОМАНД =====
void processSerial() {
  static String input = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (input.length() > 0) {
        if (input == "help") {
          Serial.println("Commands:");
          Serial.println("  s - status");
          Serial.println("  m<idx> <ticks> - set target");
          Serial.println("  p<val>, i<val>, d<val> - PID coefficients");
          Serial.println("  h<val>, l<val> - hysteresis high/low");
          Serial.println("  ap<val>, ai<val>, ad<val> - assistant PID");
          Serial.println("  ah<val>, al<val> - assistant hysteresis");
          Serial.println("  astatus - show assistant settings");
          Serial.println("  calibzero<idx> - calibrate zero");
          Serial.println("  zerostatus - show zero calibration");
          Serial.println("  assist on <idx> / assist off <idx> - assistant mode");
          Serial.println("  assist all on / assist all off");
          Serial.println("  manual on / manual off - enable/disable control encoders");
        }
        else if (input == "s") {
          for (int i = 0; i < NUM_MOTORS; i++) {
            Serial.print("M"); Serial.print(i);
            Serial.print(" pos="); Serial.print(motors[i].position);
            Serial.print(" target="); Serial.print(motors[i].targetTicks);
            Serial.print(" raw="); Serial.print(motors[i].currentRaw);
            Serial.print(" en="); Serial.print(motors[i].motorEnabled ? "ON" : "OFF");
            Serial.print(" assist="); Serial.println(motors[i].assistantMode ? "ON" : "OFF");
          }
        }
        else if (input == "zerostatus") {
          printZeroStatus();
        }
        else if (input == "astatus") {
          printAssistStatus();
        }
        else if (input == "assist all on") {
          setAssistantAll(true);
        }
        else if (input == "assist all off") {
          setAssistantAll(false);
        }
        else if (input == "manual on") {
          enterManualControl(currentCalibEar);
        }
        else if (input == "manual off") {
          exitManualControl();
        }
        else if (input.charAt(0) == 'm') {
          int space = input.indexOf(' ');
          if (space != -1) {
            int idx = input.substring(1, space).toInt();
            int32_t pos = input.substring(space + 1).toInt();
            if (idx >= 0 && idx < NUM_MOTORS) {
              motors[idx].targetTicks = pos;
              motors[idx].integral = 0.0;
              motors[idx].prevError = 0.0;
              motors[idx].motorEnabled = true;
              Serial.print("M"); Serial.print(idx);
              Serial.print(" target="); Serial.println(pos);
            }
          }
        }
        else if (input.charAt(0) == 'p') {
          Kp = input.substring(1).toFloat();
          for (int i = 0; i < NUM_MOTORS; i++) motors[i].Kp = Kp;
          Serial.print("Kp="); Serial.println(Kp);
        }
        else if (input.charAt(0) == 'i') {
          Ki = input.substring(1).toFloat();
          for (int i = 0; i < NUM_MOTORS; i++) motors[i].Ki = Ki;
          Serial.print("Ki="); Serial.println(Ki);
        }
        else if (input.charAt(0) == 'd') {
          Kd = input.substring(1).toFloat();
          for (int i = 0; i < NUM_MOTORS; i++) motors[i].Kd = Kd;
          Serial.print("Kd="); Serial.println(Kd);
        }
        else if (input.charAt(0) == 'h') {
          HYS_HIGH = input.substring(1).toInt();
          for (int i = 0; i < NUM_MOTORS; i++) motors[i].hysHigh = HYS_HIGH;
          Serial.print("HYS_HIGH="); Serial.println(HYS_HIGH);
        }
        else if (input.charAt(0) == 'l') {
          HYS_LOW = input.substring(1).toInt();
          for (int i = 0; i < NUM_MOTORS; i++) motors[i].hysLow = HYS_LOW;
          Serial.print("HYS_LOW="); Serial.println(HYS_LOW);
        }
        else if (input.startsWith("calibzero")) {
          int idx = input.substring(strlen("calibzero")).toInt();
          calibrateZero(idx);
        }
        else if (input.startsWith("assist on ")) {
          int idx = input.substring(strlen("assist on ")).toInt();
          setAssistant(idx, true);
        }
        else if (input.startsWith("assist off ")) {
          int idx = input.substring(strlen("assist off ")).toInt();
          setAssistant(idx, false);
        }
        else if (input.charAt(0) == 'a' && input.charAt(1) == 'p') {
          assistKp = input.substring(2).toFloat();
          for (int i = 0; i < NUM_MOTORS; i++) motors[i].assistKp = assistKp;
          Serial.print("assistKp="); Serial.println(assistKp);
        }
        else if (input.charAt(0) == 'a' && input.charAt(1) == 'i') {
          assistKi = input.substring(2).toFloat();
          for (int i = 0; i < NUM_MOTORS; i++) motors[i].assistKi = assistKi;
          Serial.print("assistKi="); Serial.println(assistKi);
        }
        else if (input.charAt(0) == 'a' && input.charAt(1) == 'd') {
          assistKd = input.substring(2).toFloat();
          for (int i = 0; i < NUM_MOTORS; i++) motors[i].assistKd = assistKd;
          Serial.print("assistKd="); Serial.println(assistKd);
        }
        else if (input.charAt(0) == 'a' && input.charAt(1) == 'h') {
          assistHysHigh = input.substring(2).toInt();
          for (int i = 0; i < NUM_MOTORS; i++) motors[i].assistHysHigh = assistHysHigh;
          Serial.print("assistHysHigh="); Serial.println(assistHysHigh);
        }
        else if (input.charAt(0) == 'a' && input.charAt(1) == 'l') {
          assistHysLow = input.substring(2).toInt();
          for (int i = 0; i < NUM_MOTORS; i++) motors[i].assistHysLow = assistHysLow;
          Serial.print("assistHysLow="); Serial.println(assistHysLow);
        }
        else {
          Serial.println("Unknown command. Type 'help' for list.");
        }
        input = "";
      }
    } else {
      input += c;
    }
  }
}

// ===== НАСТРОЙКА =====
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  // Инициализация MCP23017
  mcpWriteRegister(MCP_ADDR, 0x00, 0x00);

  // Настройка моторов
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].pwmPin = pwmPins[i];
    motors[i].mcpAddr = MCP_ADDR;
    motors[i].dirPin1 = dirPins[i][0];
    motors[i].dirPin2 = dirPins[i][1];
    motors[i].muxChannel = muxChannels[i];

    motors[i].targetTicks = 0;
    motors[i].position = 0;
    motors[i].prevRaw = -1;
    motors[i].currentRaw = 0;
    motors[i].zeroRaw = 0;
    motors[i].integral = 0.0;
    motors[i].prevError = 0.0;
    motors[i].motorEnabled = false;
    motors[i].lastTime = 0;

    motors[i].Kp = Kp;
    motors[i].Ki = Ki;
    motors[i].Kd = Kd;
    motors[i].hysLow = HYS_LOW;
    motors[i].hysHigh = HYS_HIGH;

    motors[i].assistantMode = false;
    motors[i].assistKp = assistKp;
    motors[i].assistKi = assistKi;
    motors[i].assistKd = assistKd;
    motors[i].assistHysLow = assistHysLow;
    motors[i].assistHysHigh = assistHysHigh;

    motors[i].encoderInitialized = false;

    ledcAttach(motors[i].pwmPin, PWM_FREQ, PWM_RES);
    ledcWrite(motors[i].pwmPin, 0);
    setMotor(motors[i], 0);
  }

  // Загрузка калибровки нуля
  loadZeroCalibration();

  // Инициализация пультовых энкодеров
  initControlEncoders();

  // Диагностика энкодеров (моторных и пультовых)
  Serial.println("=== ENCODER DIAGNOSTIC ===");
  for (int i = 0; i < NUM_MOTORS; i++) {
    selectChannel(motors[i].muxChannel);
    Serial.print("M"); Serial.print(i);
    Serial.print(" channel "); Serial.print(motors[i].muxChannel);
    if (encoder.begin()) {
      motors[i].encoderInitialized = true;
      int32_t raw = encoder.rawAngle();
      Serial.print(" OK, raw="); Serial.println(raw);
    } else {
      Serial.println(" FAILED");
    }
  }
  for (int i = 0; i < NUM_CONTROL_ENCODERS; i++) {
    selectChannel(controlEncoders[i].muxChannel, CONTROL_MUX_ADDR);
    Serial.print("C"); Serial.print(i);
    Serial.print(" channel "); Serial.print(controlEncoders[i].muxChannel);
    if (encoder.begin()) {
      controlEncoders[i].initialized = true;
      int32_t raw = encoder.rawAngle();
      Serial.print(" OK, raw="); Serial.println(raw);
    } else {
      Serial.println(" FAILED");
    }
  }
  Serial.println("=== END DIAGNOSTIC ===");

  // Инициализация кнопок
  pinMode(btnCalibPin, INPUT_PULLUP);
  pinMode(btnNextPin, INPUT_PULLUP);
  pinMode(btnReservePin, INPUT_PULLUP);

  Serial.println("System ready. Type 'help' for commands.");
}

// ===== ГЛАВНЫЙ ЦИКЛ =====
void loop() {
  processSerial();
  processButtons();

  // Обновление пультовых энкодеров (если активен режим)
  updateControlEncoders();

  unsigned long now = micros();
  static unsigned long lastTime = 0;
  float dt = (now - lastTime) / 1000000.0;
  if (dt >= 0.01) {
    lastTime = now;
    for (int i = 0; i < NUM_MOTORS; i++) {
      updateMotor(motors[i], dt);
    }
  }
}