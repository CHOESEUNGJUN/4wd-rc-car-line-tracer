/* 4휠 스마트 RC카 - 라인트레이서 + 초음파 회피  [검은 선 버전 : 밝은 바탕 + 검은 선]
 *  ★ 흰(밝은) 바탕에 검은 선을 따라가는 버전입니다. 시리얼 없음.
 *    어두운 바탕 + 흰 선 트랙은 RC_CAR_WHITE_LINE/RC_CAR_WHITE_LINE.ino 를 쓰세요.
 *    로그를 보며 디버깅하려면 RC_CAR_BLACK_LINE_DEBUG/RC_CAR_BLACK_LINE_DEBUG.ino 를 쓰세요.
 *    라인 따라가기 표는 센서 값 그대로(검은 선 = 1) 따로 씀. 회피 동작과 설정은 흰 선 버전과 같음.
 *    트랙이 바뀌면 라인센서 가변저항 감도를 다시 맞추세요.
 *  전원 켜면 바로 라인 따라 주행. 장애물을 만나면 폭을 재고 옆으로 돌아서 다시 선으로 복귀.
 *  고칠 때는 아래 [설정] 숫자만 바꾸면 됨. 시간은 전부 ms (1000 = 1초).
 */

#include <Servo.h>

// ============================================================
// [설정]
// ============================================================

// --- 속도 (0~255) ---
int SPEED            = 100;   // 직진
int TURN_SPEED       = 170;   // 라인 크게 보정 (코너에서 약하면 올리기, 흔들리면 내리기)
int AVOID_TURN_SPEED = 170;   // 회피 회전 (멈춘 상태에서 돌기 시작하므로 더 세게)
float CURVE_FAST     = 1.4;   // 살짝 보정 : 바깥 바퀴 = SPEED x 이 값
float CURVE_SLOW     = 0.4;   // 살짝 보정 : 안쪽 바퀴 = SPEED x 이 값
int CURVE_MIN_SPEED  = 50;    // 살짝 보정 : 안쪽 바퀴 최저 속도

// --- 회피 회전 시간 (ms) ※ 차마다 다름. TURN_TEST_BLACK_LINE.ino 로 재서 맞추기 ---
int TURN_RIGHT_90_MS = 550;
int TURN_LEFT_90_MS  = 900;
int TURN_RIGHT_45_MS = 500;
int TURN_LEFT_45_MS  = 500;

// --- 출발할 때 자동 회전 측정 (선 위에 차를 선과 나란히 올려놓고 켜기) ---
//   켤 때 제자리에서 오른쪽, 왼쪽으로 한 바퀴씩 돌면서 가운데 센서가 선을 만나는 시간을 재서
//   위 90도 / 45도 시간을 덮어씀. 선 위에 없거나 측정에 실패하면 위 값을 그대로 씀
int AUTO_CAL       = 1;      // 1 = 자동 측정, 0 = 위 값 그대로
int CAL_TIMEOUT_MS = 5000;   // 이 시간 안에 선을 못 만나면 측정 포기

// --- 회피 중 장애물과 거리 유지 (옆으로 나가기, 지나가기 구간) ---
//   처음 잰 옆 거리를 기억해 두고, 가까워지면 바깥쪽으로, 멀어지면 장애물 쪽으로 살짝 돌아서 맞춤
int KEEP_TOL_MM  = 30;       // 처음 거리에서 이만큼 넘게 달라지면 고침
int KEEP_TURN_MS = 60;       // 한 번 고칠 때 도는 시간

// --- 회피 직진 시간 (ms) ---
int PASS_STEP_MS  = 240;      // 지나갈 때 한 번에 전진하는 시간 (전진 -> 정지 -> 옆 확인 반복)
int PASS_EXTRA_MS = 600;      // 장애물이 안 보이게 된 뒤 더 가는 시간 (차 뒷부분까지 빠지게)
int PASS_MAX_MS   = 3600;     // 한 구간 최대 시간 (끝없이 가는 것 방지)
int RETURN_MAX_MS = 2800;     // 45도 꺾고 선 찾으며 직진하는 최대 시간
int LINE_OVER_MS  = 480;      // 선 발견 후 가운데 센서가 선을 넘어갈 때까지 더 가는 최대 시간
int LINE_TURN_MAX_MS = 700;   // 선 위에서 원래 방향으로 돌 때 최대 시간 (가운데 센서가 선을 보면 바로 멈춤)
int TURN_PAUSE_MS = 100;      // 돌고 나서 멈춰 있는 시간
int SETTLE_MS     = 150;      // 전진 후 멈춰서 차가 완전히 설 때까지 기다리는 시간

// --- 장애물 크기에 따라 달라지는 거리 (mm) -> MM_PER_SEC 로 시간으로 바꿈 ---
int MM_PER_SEC     = 250;     // SPEED 로 1초에 가는 거리
int SIDE_MARGIN_MM = 100;     // 옆으로 나갈 때 : 잰 장애물 폭 + 이 값만큼은 무조건 직진
int LEG2_MIN_MM    = 100;     // 지나가기 : 최소 (장애물까지 거리 + 이 값)은 가고 나서 판단

// --- 장애물 감지 / 초음파 ---
int AVOID_MM      = 250;      // 이 거리 안에 장애물 -> 멈춤 (아래 OBSTACLE_MODE 에 따라 기다리거나 피함)

// --- 장애물을 만났을 때 ---
int OBSTACLE_MODE = 0;        // 0 = 멈춰서 기다리다가 치우면 다시 출발,  1 = 돌아서 피하기 (회피 1~8단계)
int CLEAR_COUNT   = 5;        // 연속으로 이만큼 장애물이 안 보여야 치운 걸로 봄 (잡음 방지, 60ms 간격)
int CLEAR_WAIT_MS = 500;      // 치운 걸 확인하고 출발하기 전 기다리는 시간 (손 뺄 시간)
int PING_MS       = 60;       // 초음파 쏘는 간격 (너무 자주 쏘면 메아리가 섞여 가짜 값)
int MIN_VALID_MM  = 20;       // 이보다 작으면 잡음으로 버림
long ECHO_WAIT_US = 12000;    // 메아리 기다리는 최대 시간 (약 2m)
int SIDE_SEEN_EXTRA_MM = 200; // 옆을 볼 때 (예상 거리 + 이 값) 안쪽이면 "보임"

// --- 장애물 폭 재기 (서보) ---
int SWEEP_MAX_DEG  = 70;      // 정면 기준 좌우 몇 도까지
int SWEEP_STEP_DEG = 10;      // 몇 도씩
int SERVO_STEP_MS  = 150;     // 한 칸 움직이고 기다리는 시간
int SERVO_BIG_MS   = 500;     // 크게 돌릴 때 기다리는 시간
int OBST_EXTRA_MM  = 150;     // (정면 거리 + 이 값) 안쪽이면 같은 장애물
int SIDE_MAX_MM    = 400;     // 장애물 끝이 안 보이면 이 값으로 침
int SIDE_LOOK_DEG  = 90;      // 지나갈 때 옆을 보는 각도
int SERVO_CENTER   = 90;      // 서보 정면
int SERVO_FLIP     = 1;       // 서보 좌우가 반대로 달렸으면 1

// --- 기타 ---
int AVOID_SIDE = 0;                   // 0 자동(짧은 쪽), 1 무조건 오른쪽, 2 무조건 왼쪽 (테스트용)
// --- 선을 놓쳤을 때 : 제자리에서 좌우로 머리를 흔들며 찾기 ---
//   마지막으로 선을 본 쪽부터 흔들고, 한 번 흔들 때마다 폭을 넓힘. 흔드는 중에 선이 보이면 바로 라인 주행
int SWAY_START_MS = 150;            // 처음 흔드는 폭 (도는 시간)
int SWAY_GROW_MS  = 100;            // 한 번 흔들 때마다 늘어나는 폭
int SWAY_SPEED    = 150;            // 흔들 때 회전 속도
unsigned long SWAY_MAX_MS = 3000;   // 이 시간 동안 못 찾으면 정지 (선이 다시 보일 때까지 서 있음)
unsigned long LINE_GRACE_MS = 500;   // 양쪽 센서가 동시에 선을 봐도(1,?,1) 이 시간 동안은 하던 동작 유지. 그 뒤에도 계속이면 정지

// ============================================================
// [핀]
// ============================================================
int RightMotor_E_pin = 5, LeftMotor_E_pin = 6;     // 모터 속도(PWM)
int RightMotor_1_pin = 8, RightMotor_2_pin = 9;    // 오른쪽 모터 방향
int LeftMotor_3_pin = 10, LeftMotor_4_pin = 11;    // 왼쪽 모터 방향
int L_Line = A5, C_Line = A4, R_Line = A3;         // 라인센서 (그대로 읽으면 검은 선 = 1, 흰 바탕 = 0)
int trigPin = 13, echoPin = 12, servoPin = 2;      // 초음파, 서보

// ============================================================
Servo EduServo;
int last_state = -1;               // 직전 라인 상태 (바뀔 때만 모터 제어)
bool lostStopped = false;          // 흔들어서 찾다가 못 찾고 멈춘 상태
unsigned long both_since = 0;      // 양쪽 센서가 동시에 선을 보기 시작한 시각 (0 = 아님)
int lastSide = 0;                  // 마지막으로 선을 본 쪽 (-1 왼쪽, +1 오른쪽, 0 가운데) : 코너에서 놓치면 이쪽으로 돎
unsigned long lastPing = 0;
int obstLeftMM = 0, obstRightMM = 0;   // 잰 장애물 폭 (정면 기준 왼쪽 / 오른쪽)

void setup() {
  EduServo.attach(servoPin);
  EduServo.write(SERVO_CENTER);
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(RightMotor_E_pin, OUTPUT);
  pinMode(RightMotor_1_pin, OUTPUT);
  pinMode(RightMotor_2_pin, OUTPUT);
  pinMode(LeftMotor_3_pin, OUTPUT);
  pinMode(LeftMotor_4_pin, OUTPUT);
  pinMode(LeftMotor_E_pin, OUTPUT);
  calibrateTurns();                // 선 위에 올려놓고 켜면 회전 시간 자동 측정
}

void loop() {
  if (millis() - lastPing >= (unsigned long)PING_MS && checkObstacle(Ultrasonic())) return;
  followLine();                    // 라인센서는 쉬지 않고 계속 봄
}

// ============================================================
// 장애물 : 달리면서 정면만 확인 -> 보이면 멈추고 다시 재서 진짜면 회피
// ============================================================
// 장애물을 치울 때까지 멈춰서 기다림. CLEAR_COUNT 번 연속으로 장애물이 안 보이면 다시 출발
void waitUntilClear() {
  stopCar();
  EduServo.write(SERVO_CENTER);
  int clear = 0;
  while (clear < CLEAR_COUNT) {
    delay(PING_MS);
    if (Ultrasonic() >= AVOID_MM) clear++;
    else clear = 0;
  }
  delay(CLEAR_WAIT_MS);
}

bool checkObstacle(int d) {
  if (d >= AVOID_MM) return false;
  stopCar();
  delay(PING_MS);
  int d2 = Ultrasonic();
  if (d2 < AVOID_MM) {                // 다시 재도 가까우면 진짜 장애물
    if (OBSTACLE_MODE == 1) avoidMode(d2);   // 돌아서 피하기
    else                    waitUntilClear();  // 멈춰서 치울 때까지 기다리기
  }
  last_state = -1;                    // 라인 상태 처음부터 다시 판단
  lostStopped = false;
  return true;
}

// ============================================================
// 라인 따라가기 (검은 선 버전 : 센서 값 그대로 씀. 검은 선 = 1, 흰 바탕 = 0)
// ============================================================
void followLine() {
  int L = digitalRead(L_Line), C = digitalRead(C_Line), R = digitalRead(R_Line);

  if (L == 0 && C == 0 && R == 0) {   // 선 놓침 -> 좌우로 흔들며 찾기
    if (lostStopped) return;                   // 이미 흔들어 봤는데 못 찾고 멈춘 상태
    if (swaySearch()) { last_state = -1; return; }
    lostStopped = true;
    last_state = -1;
    return;
  }
  lostStopped = false;

  // 1,?,1 : 양쪽 센서가 동시에 선을 봄 -> 바로 멈추지 않고 LINE_GRACE_MS 동안 하던 동작 유지
  if (L == 1 && R == 1) {
    if (both_since == 0) both_since = millis();
    if (millis() - both_since < LINE_GRACE_MS) return;
  }
  else both_since = 0;

  if (R == 1 && L == 0) lastSide = +1;       // 선이 오른쪽에 있었음
  if (L == 1 && R == 0) lastSide = -1;       // 선이 왼쪽에 있었음
  if (L == 0 && C == 1 && R == 0) lastSide = 0;

  int state = L * 4 + C * 2 + R;
  if (state == last_state) return;
  last_state = state;

  //   L  C  R      (끝 센서 하나만 보면 많이 벗어난 것 -> 크게, 가운데와 같이 보면 -> 살짝)
  if      (L == 0 && C == 1 && R == 0) goForward();             // 직진
  else if (L == 0 && C == 1 && R == 1) curveRight();            // 오른쪽 살짝 보정
  else if (L == 0 && C == 0 && R == 1) spin(true, TURN_SPEED);  // 오른쪽 크게 보정 (코너)
  else if (L == 1 && C == 1 && R == 0) curveLeft();             // 왼쪽 살짝 보정
  else if (L == 1 && C == 0 && R == 0) spin(false, TURN_SPEED); // 왼쪽 크게 보정 (코너)
  else if (L == 1 && R == 1)           stopCar();               // 정지
}

// ============================================================
// 회피 : 1 폭 재기 -> 2 돌기 -> 3 옆으로 나가기 -> 4 다시 돌기 -> 5 지나가기
//        -> 6 45도 꺾기 -> 7 직진하며 라인 찾기 -> 8 원래 방향으로 정렬 (README 그림 번호)
//   toRight = true : 오른쪽으로 피함 (장애물은 왼쪽)
// ============================================================
void avoidMode(int d0) {
  bool toRight = true;
  int sideOff = 0;                 // 서보가 볼 방향 (장애물 쪽)
  int widthMM = 0;                 // 3번에서 옆으로 간 거리

  int step = 1;
  while (step != 0) {
    switch (step) {
      case 1:  // 폭 재기 -> 짧은 쪽으로 방향 정하기. 아무것도 없으면 가짜 감지라 취소
        measureObstacle(d0);
        if (obstLeftMM == 0 && obstRightMM == 0) { step = 0; break; }
        toRight = (obstRightMM <= obstLeftMM);
        if (AVOID_SIDE == 1) toRight = true;
        if (AVOID_SIDE == 2) toRight = false;
        sideOff = toRight ? -SIDE_LOOK_DEG : SIDE_LOOK_DEG;
        step = 2;
        break;

      case 2:  // 빈 쪽으로 90도
        turn90(toRight);
        step = 3;
        break;

      case 3:  // 옆으로 나가기 : 잰 폭 + 여유만큼은 무조건, 옆에 보이면 더
        widthMM = msToMm(passObstacle(sideOff, (toRight ? obstRightMM : obstLeftMM) + SIDE_MARGIN_MM,
                                      d0 + SIDE_SEEN_EXTRA_MM));
        step = 4;
        break;

      case 4:  // 장애물 쪽으로 90도 (원래 진행 방향)
        turn90(!toRight);
        step = 5;
        break;

      case 5:  // 지나가기 : 옆을 보며 조금 전진 -> 정지 -> 확인, 안 보일 때까지
        passObstacle(sideOff, d0 + LEG2_MIN_MM, widthMM + SIDE_SEEN_EXTRA_MM);
        lookAt(0, SERVO_BIG_MS);
        step = 6;
        break;

      case 6:  // 선 쪽으로 45도
        turn45(!toRight);
        step = 7;
        break;

      case 7:  // 직진하며 라인 찾기. 못 찾으면 회피 끝 (라인 주행이 전진하며 찾음)
        step = findLine() ? 8 : 0;
        break;

      case 8:  // 원래 방향으로 돌아 가운데 센서가 선을 볼 때까지 -> 라인 주행
        spin(toRight, AVOID_TURN_SPEED);
        runUntil(LINE_TURN_MAX_MS, C_Line);
        stopCar();
        step = 0;
        break;
    }
  }
}

// 선 찾으며 직진. 찾으면 가운데 센서가 선을 넘어갈 때까지 더 가고 true
bool findLine() {
  if (!moveMs(RETURN_MAX_MS, true)) return false;
  unsigned long start = millis();
  bool centerSeen = false;
  goForward();
  while (millis() - start < (unsigned long)LINE_OVER_MS) {
    if (readLine(C_Line) == 0) centerSeen = true;
    else if (centerSeen) break;    // 선 위였다가 벗어남 = 넘어감
  }
  stopCar();
  delay(SETTLE_MS);
  return true;
}

// 장애물을 옆에 두고 지나가기. 돌려주는 값 = 간 시간(ms)
//   minMM 만큼은 무조건 가고, 그 뒤 옆 거리가 seenMM 이상이면(안 보이면) 끝
int passObstacle(int sideOff, int minMM, int seenMM) {
  int minMs = mmToMs(minMM);
  int goneMs = 0;
  int ds = lookAt(sideOff, SERVO_BIG_MS);
  int keepMM = 0;                  // 처음 보인 옆 거리 (이 거리를 유지)
  while (goneMs < PASS_MAX_MS && !(ds >= seenMM && goneMs >= minMs)) {
    if (ds < seenMM) {             // 옆에 장애물 보임 -> 거리 유지
      if (keepMM == 0) keepMM = ds;
      else keepDistance(sideOff, ds, keepMM);
    }
    moveMs(PASS_STEP_MS, false);
    goneMs += PASS_STEP_MS;
    delay(SETTLE_MS);
    ds = lookAt(sideOff, SERVO_STEP_MS);
  }
  moveMs(PASS_EXTRA_MS, false);
  return goneMs + PASS_EXTRA_MS;
}

// 서보를 천천히 훑어서 장애물이 좌우로 얼마나 뻗어 있는지 저장
void measureObstacle(int d0) {
  obstLeftMM = obstRightMM = 0;
  int limit = d0 + OBST_EXTRA_MM;
  lookAt(-SWEEP_MAX_DEG, SERVO_BIG_MS);
  for (int off = -SWEEP_MAX_DEG; off <= SWEEP_MAX_DEG; off += SWEEP_STEP_DEG) {
    int d = lookAt(off, SERVO_STEP_MS);
    if (d >= limit) continue;
    int side = d * sin(radians(abs(off)));             // 옆으로 뻗은 길이
    if (abs(off) >= SWEEP_MAX_DEG || side > SIDE_MAX_MM) side = SIDE_MAX_MM;
    if (off < 0) obstLeftMM  = max(obstLeftMM, side);
    else         obstRightMM = max(obstRightMM, side);
  }
  lookAt(0, SERVO_BIG_MS);
}

// 서보를 정면 기준 offset 도 (- 왼쪽, + 오른쪽)로 돌리고 거리(mm) 재기. 항상 멈춘 상태에서 잼
int lookAt(int offset, int waitMs) {
  stopCar();
  EduServo.write(SERVO_CENTER + (SERVO_FLIP ? -offset : offset));
  delay(waitMs);
  return Ultrasonic();
}

int Ultrasonic() {                 // 정면 거리(mm). 없으면 9999
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, ECHO_WAIT_US);
  lastPing = millis();
  int mm = duration * 340L / 2000;
  return (duration == 0 || mm < MIN_VALID_MM) ? 9999 : mm;
}

// ============================================================
// 동작
// ============================================================
// 회피 중 선 찾기용 : 센서는 검은색에서 1 -> 뒤집어서 "선 = 0" 으로 맞춤 (runUntil, findLine 에서 씀)
int readLine(int pin) { return !digitalRead(pin); }

int mmToMs(int mm) { return (long)mm * 1000 / MM_PER_SEC; }
int msToMm(int ms) { return (long)ms * MM_PER_SEC / 1000; }

// ms 동안 전진하고 멈춤. watchLine 이면 가는 중 선 보이면 멈추고 true
bool moveMs(int ms, bool watchLine) {
  goForward();
  bool found = false;
  if (watchLine) found = runUntilAny(ms);
  else           delay(ms);
  stopCar();
  return found;
}

// ms 동안 기다리다가 센서 pin 이 선(0)을 보면 바로 true
bool runUntil(int ms, int pin) {
  unsigned long start = millis();
  while (millis() - start < (unsigned long)ms) if (readLine(pin) == 0) return true;
  return false;
}

// ms 동안 기다리다가 라인센서 중 하나라도 선을 보면 바로 true
bool runUntilAny(int ms) {
  unsigned long start = millis();
  while (millis() - start < (unsigned long)ms)
    if (readLine(L_Line) == 0 || readLine(C_Line) == 0 || readLine(R_Line) == 0) return true;
  return false;
}

// 출발할 때 자동 회전 측정 : 선 위에서 제자리로 돌며 가운데 센서가 선을 다시 만나는 시간을 잼
//   처음 다시 만남 = 약 180도, 그다음 = 약 360도 -> 1도당 시간과 출발할 때 늦는 시간으로 90도 / 45도 계산
void calibrateTurns() {
  if (!AUTO_CAL || !centerOnLine()) return;      // 선 위에 없으면 측정 안 함 (설정값 그대로)
  calibrateTurn(true);
  delay(300);
  calibrateTurn(false);
  delay(300);
}

bool calibrateTurn(bool toRight) {
  unsigned long start = millis();
  spin(toRight, AVOID_TURN_SPEED);
  long t180 = waitLineEntry(start);
  long t360 = (t180 > 0) ? waitLineEntry(start) : 0;
  stopCar();
  if (t180 <= 0 || t360 <= t180) return false;    // 실패 -> 설정값 그대로
  float msPerDeg = (t360 - t180) / 180.0;
  float lag = t180 - 180 * msPerDeg;               // 멈춘 상태에서 출발할 때 늦는 시간
  if (lag < 0) lag = 0;
  int t90 = lag + 90 * msPerDeg;
  int t45 = lag + 45 * msPerDeg;
  if (t90 < 200 || t90 > 2000) return false;       // 말이 안 되는 값이면 버림
  if (toRight) { TURN_RIGHT_90_MS = t90; TURN_RIGHT_45_MS = t45; }
  else         { TURN_LEFT_90_MS  = t90; TURN_LEFT_45_MS  = t45; }
  return true;
}

// 가운데 센서가 선에서 벗어났다가 다시 선을 만날 때까지 기다림. 돌려주는 값 = start 부터 걸린 ms (실패하면 0)
long waitLineEntry(unsigned long start) {
  while (centerOnLine())  if (millis() - start > (unsigned long)CAL_TIMEOUT_MS) return 0;
  delay(20);                                        // 선 가장자리에서 값이 튀는 것 무시
  while (!centerOnLine()) if (millis() - start > (unsigned long)CAL_TIMEOUT_MS) return 0;
  return millis() - start;
}

// 장애물과 거리 유지 : 처음 거리(keepMM)보다 가까우면 바깥쪽으로, 멀면 장애물 쪽으로 살짝 돌기
void keepDistance(int sideOff, int ds, int keepMM) {
  bool obstacleOnLeft = (sideOff < 0);
  if      (ds < keepMM - KEEP_TOL_MM) turn(obstacleOnLeft, KEEP_TURN_MS);    // 가까워짐 -> 장애물 반대쪽으로
  else if (ds > keepMM + KEEP_TOL_MM) turn(!obstacleOnLeft, KEEP_TURN_MS);   // 멀어짐 -> 장애물 쪽으로
}

bool centerOnLine() { return readLine(C_Line) == 0; }      // 가운데 센서가 검은 선 위 (readLine 이 뒤집어서 선 = 0)

// 선 놓침 : 제자리에서 좌우로 머리를 흔들며 찾기. 찾으면 true
//   마지막으로 선을 본 쪽부터 (모르면 왼쪽부터), 가운데를 지나 반대쪽으로 갈 때마다 폭을 SWAY_GROW_MS 씩 넓힘
bool swaySearch() {
  bool toRight = (lastSide > 0);
  int prevAmp = 0, amp = SWAY_START_MS;
  unsigned long start = millis();
  while (millis() - start < SWAY_MAX_MS) {
    spin(toRight, SWAY_SPEED);
    if (runUntilAny(prevAmp + amp)) { stopCar(); return true; }
    toRight = !toRight;
    prevAmp = amp;
    amp += SWAY_GROW_MS;
  }
  stopCar();
  return false;
}

void turn90(bool toRight) { turn(toRight, toRight ? TURN_RIGHT_90_MS : TURN_LEFT_90_MS); }
void turn45(bool toRight) { turn(toRight, toRight ? TURN_RIGHT_45_MS : TURN_LEFT_45_MS); }

void turn(bool toRight, int ms) {  // 멈춘 상태에서 ms 만큼 제자리 회전 후 멈춤
  stopCar();
  spin(toRight, AVOID_TURN_SPEED);
  delay(ms);
  stopCar();
  delay(TURN_PAUSE_MS);
}

void spin(bool toRight, int speed) {     // 제자리 회전 (오른쪽 : 오른쪽 바퀴 뒤, 왼쪽 바퀴 앞)
  if (toRight) drive(LOW, HIGH, speed, speed);
  else         drive(HIGH, LOW, speed, speed);
}

void goForward()  { drive(HIGH, HIGH, SPEED, SPEED); }
void stopCar()    { drive(HIGH, HIGH, 0, 0); }
void curveLeft()  { drive(HIGH, HIGH, min(SPEED * CURVE_FAST, 255), max(SPEED * CURVE_SLOW, CURVE_MIN_SPEED)); }
void curveRight() { drive(HIGH, HIGH, max(SPEED * CURVE_SLOW, CURVE_MIN_SPEED), min(SPEED * CURVE_FAST, 255)); }

void drive(int rightDir, int leftDir, int rightSpeed, int leftSpeed) {   // 방향 HIGH 앞 / LOW 뒤
  digitalWrite(RightMotor_1_pin, rightDir);
  digitalWrite(RightMotor_2_pin, !rightDir);
  digitalWrite(LeftMotor_3_pin, leftDir);
  digitalWrite(LeftMotor_4_pin, !leftDir);
  analogWrite(RightMotor_E_pin, rightSpeed);
  analogWrite(LeftMotor_E_pin, leftSpeed);
}
