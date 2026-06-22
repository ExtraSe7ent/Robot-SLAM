// ═══════════════════════════════════════════════════════════
//  Robot STM32F103 — Final version
//  Fix: non-blocking AUTO (millis), thêm "OFF", bỏ dead code
// ═══════════════════════════════════════════════════════════

// ── Pin định nghĩa ────────────────────────────────────────
#define TRIG_PIN PB8
#define ECHO_PIN PB9
#define ENA PA0
#define ENB PA1
#define IN1 PA2
#define IN2 PA3
#define IN3 PA4
#define IN4 PA5

// ── Robot mode ────────────────────────────────────────────
enum RobotMode {
  ROBOT_STOP,
  ROBOT_FORWARD,
  ROBOT_BACK,
  ROBOT_LEFT,
  ROBOT_RIGHT,
  ROBOT_FWD_LEFT,
  ROBOT_FWD_RIGHT,
  ROBOT_BACK_LEFT,
  ROBOT_BACK_RIGHT,
  ROBOT_AUTO
};

// ── AUTO state machine (thay thế delay() blocking) ────────
// Cũ: delay(100)+delay(650)+delay(150) = 900ms chặn Serial
// Mới: millis()-based, Serial được đọc mỗi vòng loop (~30µs)
enum AutoState {
  AS_SCAN,   // đang tiến + đo khoảng cách
  AS_STOP,   // gặp vật, dừng lại 100ms
  AS_TURN,   // xoay tránh 650ms
  AS_PAUSE   // chờ ổn định 150ms
};

RobotMode currentMode = ROBOT_STOP;
AutoState autoState   = AS_SCAN;
unsigned long autoTimer = 0;

String inputString = "";
bool   stringComplete = false;

// ── Sensor siêu âm ───────────────────────────────────────
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // pulseIn timeout 30000µs = 30ms max — duy nhất điểm blocking còn lại
  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  return (dur == 0) ? 999.0f : (dur * 0.0343f) / 2.0f;
}

// ── Điều khiển motor ─────────────────────────────────────
void moveRobot(int L, int R) {
  analogWrite(ENA, abs(L));
  analogWrite(ENB, abs(R));
  // bool → HIGH(1)/LOW(0) tự động
  digitalWrite(IN1, L > 0); digitalWrite(IN2, L < 0);
  digitalWrite(IN3, R > 0); digitalWrite(IN4, R < 0);
}

// ── Đọc Serial (non-blocking, gọi mỗi loop) ──────────────
void readSerial() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n') stringComplete = true;
    else           inputString += c;
  }
}

// ── Xử lý lệnh từ Pi ────────────────────────────────────
void parseCommand() {
  if (!stringComplete) return;
  inputString.trim();

  if      (inputString == "ON")          currentMode = ROBOT_FORWARD;
  // FIX: "OFF" là lệnh joystick nhả ra, "MANUAL" là tắt AUTO từ web
  // uart_bridge.py gửi "OFF" khi nhả joystick và khi tắt AUTO
  else if (inputString == "OFF" ||
           inputString == "MANUAL")     { currentMode = ROBOT_STOP;
                                          moveRobot(0, 0);
                                          autoState = AS_SCAN; }  // reset AUTO state
  else if (inputString == "BACK")        currentMode = ROBOT_BACK;
  else if (inputString == "LEFT")        currentMode = ROBOT_LEFT;
  else if (inputString == "RIGHT")       currentMode = ROBOT_RIGHT;
  else if (inputString == "UP_LEFT")     currentMode = ROBOT_FWD_LEFT;
  else if (inputString == "UP_RIGHT")    currentMode = ROBOT_FWD_RIGHT;
  else if (inputString == "DOWN_LEFT")   currentMode = ROBOT_BACK_LEFT;
  else if (inputString == "DOWN_RIGHT")  currentMode = ROBOT_BACK_RIGHT;
  else if (inputString == "AUTO")       { currentMode = ROBOT_AUTO;
                                          autoState = AS_SCAN; }  // bắt đầu từ SCAN

  inputString = "";
  stringComplete = false;
}

// ── AUTO mode: state machine non-blocking ────────────────
// Worst-case Serial latency: ~30ms (chỉ pulseIn, không còn delay)
void handleAuto() {
  unsigned long now = millis();

  switch (autoState) {
    case AS_SCAN:
      if (getDistance() < 40.0f) {
        // Gặp vật: dừng lại
        moveRobot(0, 0);
        autoState = AS_STOP;
        autoTimer = now;
      } else {
        // Đường thông: tiến thẳng
        moveRobot(180, 180);
      }
      break;

    case AS_STOP:
      // Chờ 100ms rồi quay
      if (now - autoTimer >= 100UL) {
        moveRobot(-190, 190);   // xoay trái
        autoState = AS_TURN;
        autoTimer = now;
      }
      break;

    case AS_TURN:
      // Quay 650ms (~90°)
      if (now - autoTimer >= 650UL) {
        moveRobot(0, 0);
        autoState = AS_PAUSE;
        autoTimer = now;
      }
      break;

    case AS_PAUSE:
      // Chờ ổn định 150ms
      if (now - autoTimer >= 150UL) {
        autoState = AS_SCAN;   // quay về đo khoảng cách
      }
      break;
  }
}

// ── Setup ─────────────────────────────────────────────────
void setup() {
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(ENA, OUTPUT);      pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);      pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);      pinMode(IN4, OUTPUT);
  moveRobot(0, 0);
  Serial1.begin(115200);
  inputString.reserve(50);
}

// ── Loop ──────────────────────────────────────────────────
void loop() {
  readSerial();    // luôn đọc Serial trước tiên
  parseCommand();  // xử lý lệnh nếu có

  switch (currentMode) {
    case ROBOT_AUTO:        handleAuto();           break;
    case ROBOT_FORWARD:     moveRobot(180, 180);    break;
    case ROBOT_BACK:        moveRobot(-180, -180);  break;
    case ROBOT_LEFT:        moveRobot(-160, 160);   break;
    case ROBOT_RIGHT:       moveRobot(160, -160);   break;
    case ROBOT_FWD_LEFT:    moveRobot(80, 180);     break;
    case ROBOT_FWD_RIGHT:   moveRobot(180, 80);     break;
    case ROBOT_BACK_LEFT:   moveRobot(-80, -180);   break;
    case ROBOT_BACK_RIGHT:  moveRobot(-180, -80);   break;
    case ROBOT_STOP:        moveRobot(0, 0);        break;
  }
  // Không có delay() — loop chạy tự do, chỉ bị chặn ~30ms bởi pulseIn trong AS_SCAN
}
