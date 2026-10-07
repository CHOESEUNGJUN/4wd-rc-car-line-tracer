/* 4휠 스마트 RC카 - 라인트레이서 + 초음파 회피
 *  [미완성] 회피 주행 튜닝 중. 아직 완성된 코드가 아님
 *  전원 켜면 바로 라인 따라 주행
 *
 *  고칠 때는 아래 [설정] 부분 숫자만 바꾸면 됩니다.
 *  파일 구조 :  [설정] -> [핀] -> setup/loop -> 장애물 -> 라인 -> 회피 -> 기본 동작
 */

#include <Servo.h>

// ============================================================
// [설정]  여기 숫자만 바꾸면 됨
// ============================================================

// --- 디버그 (시리얼 모니터 9600) ---
int DEBUG    = 1;          // 1 = 상태를 계속 출력, 0 = 끔 (다 맞춘 뒤 0으로 하면 더 빠름)
int DEBUG_MS = 300;        // 주행 중 상태 출력 간격 (ms)

// --- 속도 (0~255) ---
int SPEED       = 140;     // 직진 속도
int TURN_SPEED  = 140;     // 제자리 좌회전 / 우회전 속도 (커브에서 약하면 올리기)
float CURVE_FAST = 1.4;    // 살짝 틀 때 바깥 바퀴 = SPEED x 이 값
float CURVE_SLOW = 0.4;    // 살짝 틀 때 안쪽 바퀴 = SPEED x 이 값 (작을수록 많이 휨)

int CURVE_MIN_SPEED = 50;  // 살짝 틀 때 안쪽 바퀴 최저 속도 (안쪽 바퀴가 멈추면 올리기)

// --- 장애물 거리 (mm) ---
int AVOID_MM     = 250;    // 이 거리 안에 장애물 -> 멈추고 회피 모드

// --- 초음파 잡음 걸러내기 ---
int PING_MS      = 60;     // 초음파 쏘는 간격 (ms). 너무 자주 쏘면 앞 메아리가 섞여서 가짜 값이 나옴
int MIN_VALID_MM = 20;     // 이 값보다 작게 나오면 잡음으로 보고 버림 (센서가 20mm 안쪽은 못 잼)
long ECHO_WAIT_US = 12000; // 초음파 메아리 기다리는 최대 시간 (마이크로초). 12000 = 약 2m 까지 봄

// --- 차 움직임 보정 (회피할 때 "몇 mm 가라 / 90도 돌아라" 를 시간으로 바꾸는 값) ---
//   직접 재서 맞추면 회피가 정확해짐. 건전지 약해지면 다시 맞춰야 함
int MM_PER_SEC = 250;      // SPEED 로 1초에 가는 거리(mm).  회피 때 너무 멀리 가면 올리고, 덜 가면 내리기
int TURN_90_MS = 600;      // 제자리에서 90도 도는 데 걸리는 시간(ms).  덜 돌면 올리고, 더 돌면 내리기
int TURN_PAUSE_MS = 100;   // 돌고 나서 잠깐 멈춰 있는 시간 (차가 흔들리면 늘리기)
int SETTLE_MS     = 150;   // 전진하다 멈춘 뒤, 차가 완전히 설 때까지 기다리는 시간 (재기 전에 흔들리면 늘리기)

// --- 회피 1단계 : 장애물 크기 재기 (서보를 천천히 좌우로 훑음) ---
int SWEEP_MAX_DEG  = 70;   // 정면 기준 좌우 몇 도까지 볼지
int SWEEP_STEP_DEG = 10;   // 몇 도씩 움직이며 잴지 (작을수록 정밀, 느림)
int SERVO_STEP_MS  = 150;  // 한 칸 움직이고 기다리는 시간 (값이 튀면 늘리기 = 더 천천히 봄)
int SERVO_BIG_MS   = 500;  // 서보를 크게 돌릴 때 기다리는 시간
int OBST_EXTRA_MM  = 150;  // 정면 거리 + 이 값 안쪽으로 보이면 "같은 장애물"로 침

// --- 회피 방향 (테스트용) ---
int AVOID_SIDE = 0;        // 0 = 자동(짧은 쪽),  1 = 무조건 오른쪽 회피,  2 = 무조건 왼쪽 회피

// --- 회피 2단계 : 빈 쪽으로 돌기 ---
int AVOID_TURN_DEG = 90;   // 빈 쪽으로 확실하게 도는 각도 (90, 80, 60 ... 덜/더 돌면 TURN_90_MS 보정)
int SIDE_MAX_MM    = 400;  // (크기 잴 때) 장애물 끝이 안 보이면 이 값으로 침

// --- 회피 3단계 : 장애물을 옆에 두고 지나가기 (두 번 반복 : 옆면 -> 돌기 -> 뒷면까지) ---
//   서보로 장애물 쪽 옆을 봄 -> 보이면 살짝 전진 -> 또 봄 -> 안 보이면 장애물 쪽으로 돌기
int SIDE_LOOK_DEG  = 90;   // 서보가 옆을 보는 각도 (90 = 완전히 옆)
int SIDE_MARGIN_MM = 100;  // 옆으로 나갈 때 : 잰 장애물 폭 + 이 값만큼은 무조건 직진 (차 반폭 + 여유). 긁히면 늘리기
int SIDE_SEEN_EXTRA_MM = 200; // 옆을 볼 때 : (장애물까지 예상 거리 + 이 값) 안쪽이면 "보임"
int PASS_STEP_MM   = 60;   // 한 번에 살짝 전진하는 거리
int PASS_EXTRA_MM  = 150;  // 안 보이게 된 뒤 더 가는 거리 (차 뒷부분까지 빠지게). 긁히면 늘리기
int PASS_MAX_MM    = 900;  // 한 구간에서 최대 이만큼만 감 (끝없이 가는 것 방지)
int LEG2_MIN_MM    = 100;  // 두 번째 구간 : 장애물 옆에 닿기 전이라 처음엔 안 보임 -> 최소 (정면거리 + 이 값)은 가고 나서 판단

// --- 회피 4단계 : 비스듬히 선으로 돌아오기 ---
int RETURN_TURN_DEG = 45;  // 선 쪽으로 비스듬히 도는 각도
int RETURN_MAX_MM   = 700; // 선 찾으며 직진하는 최대 거리
int LINE_OVER_MM    = 40;  // 선 발견 후 살짝 더 전진 (차 중심이 선 위에 오게)
int LINE_TURN_MAX_MS = 1200; // 선 위에서 원래 방향으로 돌 때 최대 시간 (가운데 센서가 선을 보면 바로 멈춤)

// --- 선을 놓쳤을 때 ---
unsigned long LOST_STOP_MS = 20000;   // 선 없는 상태가 이 시간 계속되면 정지 (20000 = 20초)

// --- 서보 ---
int SERVO_CENTER = 90;     // 정면 각도
int SERVO_FLIP   = 1;      // 좌우가 반대로 재지면 1 (장애물 있는 쪽으로 피하면 바꾸기)

// ============================================================
// [핀]  배선 바꿨을 때만 수정
// ============================================================
int RightMotor_E_pin = 5;    // 오른쪽 모터 속도(PWM)
int LeftMotor_E_pin  = 6;    // 왼쪽 모터 속도(PWM)
int RightMotor_1_pin = 8;    // 오른쪽 모터 방향 IN1
int RightMotor_2_pin = 9;    // 오른쪽 모터 방향 IN2
int LeftMotor_3_pin  = 10;   // 왼쪽 모터 방향 IN3
int LeftMotor_4_pin  = 11;   // 왼쪽 모터 방향 IN4

int L_Line = A5;             // 왼쪽 라인센서
int C_Line = A4;             // 가운데 라인센서
int R_Line = A3;             // 오른쪽 라인센서

int trigPin  = 13;           // 초음파 Trig
int echoPin  = 12;           // 초음파 Echo
int servoPin = 2;            // 서보

// ============================================================
// 내부 변수 (건드릴 필요 없음)
// ============================================================
Servo EduServo;
int last_state = -1;               // 직전 라인 상태 (바뀔 때만 모터/출력)
unsigned long lost_since = 0;      // 선 놓친 시각 (0 = 선 보는 중)
int obstLeftMM  = 0;               // 저장한 장애물 폭 : 정면 기준 왼쪽으로 뻗은 길이
int obstRightMM = 0;               //                     오른쪽으로 뻗은 길이
const char* action = "정지";       // 지금 하고 있는 동작 (디버그용)
bool searchLine = false;           // 회피 직후 : 선이 안 보이면 전진하며 찾기
unsigned long lastDebug = 0;
unsigned long lastPing = 0;        // 마지막으로 초음파 쏜 시각
int frontDist = 9999;              // 마지막으로 잰 정면 거리

// ============================================================
// setup / loop
// ============================================================
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

  Serial.begin(9600);
  Serial.println();
  Serial.println(F("=== 시작 (전원 켜짐 / 리셋) ==="));   // 주행 중에 이 줄이 또 나오면 전원이 순간 끊겨 리셋된 것 (서보/모터 전류)
  Serial.print(F("SPEED "));        Serial.print(SPEED);
  Serial.print(F(" | AVOID_MM "));  Serial.println(AVOID_MM);
  if (AVOID_MM < 100) Serial.println(F("경고 : AVOID_MM 이 100 보다 작음. 초음파는 20mm 안쪽을 못 재서, 멈추기 전에 부딪히면 장애물을 못 봄"));
}

void loop() {
  if (millis() - lastPing >= (unsigned long)PING_MS) {   // 초음파는 PING_MS 마다만 쏨
    frontDist = Ultrasonic();
    if (checkObstacle(frontDist)) { frontDist = 9999; return; }
  }
  debugStatus(frontDist);
  followLine();                    // 라인센서는 쉬지 않고 계속 봄
}

void debugStatus(int d) {          // 주행 중 상태를 DEBUG_MS 마다 한 줄 출력
  if (!DEBUG || millis() - lastDebug < (unsigned long)DEBUG_MS) return;
  lastDebug = millis();
  Serial.print(F("[주행] 거리 "));  Serial.print(d);
  Serial.print(F(" | 라인 "));      printLine();
  Serial.print(F(" | 동작 "));      Serial.println(action);
}

void printLine() {                 // 라인센서 3개 값 출력 (L,C,R)
  Serial.print(digitalRead(L_Line)); Serial.print(F(","));
  Serial.print(digitalRead(C_Line)); Serial.print(F(","));
  Serial.print(digitalRead(R_Line));
}

// ============================================================
// 장애물
// ============================================================
bool checkObstacle(int d) {        // d = 정면 거리(mm). 장애물 있어서 처리했으면 true
  if (d >= AVOID_MM) return false; // 장애물 없음

  stopCar();                       // 일단 멈추고 한 번 더 재서 확인 (가짜 값 방지)
  delay(PING_MS);
  int d2 = Ultrasonic();
  if (d2 >= AVOID_MM) {
    Serial.print(F("[장애물?] ")); Serial.print(d);
    Serial.print(F(" -> 다시 재니 ")); Serial.print(d2); Serial.println(F(" : 잡음, 무시"));
    last_state = -1;
    searchLine = true;
    return false;
  }

  Serial.print(F("[장애물] 거리 ")); Serial.print(d2);
  Serial.println(F(" -> 회피 모드 시작"));
  avoidMode(d2);
  Serial.println(F("[회피 모드 끝]"));

  last_state = -1;                 // 라인 상태 처음부터 다시 판단
  lost_since = 0;
  searchLine = true;               // 선이 안 보이면 멈춰 있지 말고 전진하며 찾기
  return true;
}

// ============================================================
// 라인트레이서
//   검은 바탕 + 흰 선 : 흰 선 위 = 0, 검은 바탕 = 1
// ============================================================
void followLine() {
  int L = digitalRead(L_Line);
  int C = digitalRead(C_Line);
  int R = digitalRead(R_Line);

  // 1,1,1 : 선 놓침 -> 하던 동작 유지, LOST_STOP_MS 지나면 정지
  if (L == 1 && C == 1 && R == 1) {
    if (searchLine) {              // 회피 직후인데 선이 없음 -> 전진하며 찾기 (멈춰 있지 않게)
      searchLine = false;
      goForward();
      Serial.println(F("선 없음 -> 전진하며 찾기"));
    }
    if (lost_since == 0) lost_since = millis();
    if (millis() - lost_since >= LOST_STOP_MS) {
      stopCar();
      Serial.println(F("선 없음 -> 정지"));
      lost_since = millis();
      last_state = -1;
    }
    return;
  }
  lost_since = 0;
  searchLine = false;

  int state = L * 4 + C * 2 + R;
  if (state == last_state) return; // 상태 그대로면 아무것도 안 함
  last_state = state;

  Serial.print(F("digital : "));
  Serial.print(L); Serial.print(F(", "));
  Serial.print(C); Serial.print(F(", "));
  Serial.print(R); Serial.print(F("   "));

  //   L  C  R                              동작
  if      (L == 1 && C == 0 && R == 1) { goForward();  Serial.println(F("직진")); }
  else if (L == 1 && C == 0 && R == 0) { curveLeft();  Serial.println(F("왼쪽 살짝")); }
  else if (L == 1 && C == 1 && R == 0) { spinLeft();   Serial.println(F("좌회전")); }
  else if (L == 0 && C == 0 && R == 1) { curveRight(); Serial.println(F("오른쪽 살짝")); }
  else if (L == 0 && C == 1 && R == 1) { spinRight();  Serial.println(F("우회전")); }
  else if (L == 0 && R == 0)           { stopCar();    Serial.println(F("정지")); }
  else                                 { Serial.println(); }
}

// ============================================================
// 회피 모드 : 장애물 크기를 먼저 재고(저장), 그 크기만큼 돌아서 지나감
//   1. 크기 재서 방향 정하기  2. 빈 쪽으로 돌기  3. 장애물을 옆에 두고 지나가기(2번)  4. 비스듬히 선으로 복귀
// ============================================================
void avoidMode(int d0) {           // d0 = 장애물까지 정면 거리(mm)
  stopCar();

  // ---- 1. 장애물 크기 재서 어느 쪽으로 갈지 정하기 ----
  measureObstacle(d0);
  bool toRight = (obstRightMM <= obstLeftMM);          // 짧은 쪽으로 돌아감
  if (AVOID_SIDE == 1) toRight = true;                 // 테스트용 : 방향 고정
  if (AVOID_SIDE == 2) toRight = false;

  Serial.print(F("[회피 1] 장애물 왼쪽 ")); Serial.print(obstLeftMM);
  Serial.print(F(" / 오른쪽 "));           Serial.print(obstRightMM);
  Serial.println(F(" mm"));

  if (obstLeftMM == 0 && obstRightMM == 0) {           // 훑어봤는데 아무것도 없음 = 가짜 감지
    Serial.println(F("[회피 1] 장애물 안 보임 -> 회피 취소"));
    return;
  }

  if (toRight) avoidRight(d0);
  else         avoidLeft(d0);
}

// ------------------------------------------------------------
// 오른쪽 회피 : 오른쪽으로 돌면 장애물은 "왼쪽"에 있음
// ------------------------------------------------------------
void avoidRight(int d0) {
  Serial.println(F("=== 오른쪽 회피 ==="));

  // 2. 오른쪽으로 확실하게 돌기
  turnRightDeg(AVOID_TURN_DEG);
  Serial.println(F("[R 2] 오른쪽으로 돎 (장애물은 왼쪽)"));

  // 3-1. 왼쪽을 보면서 : 보이면 살짝 전진, 안 보이면 끝
  //      저장해 둔 장애물 폭(obstRightMM) + 여유만큼은 무조건 직진
  int widthMM = passObstacle(-SIDE_LOOK_DEG, obstRightMM + SIDE_MARGIN_MM,
                             d0 + SIDE_SEEN_EXTRA_MM);
  Serial.print(F("[R 3-1] 옆으로 다 나감. 간 거리 ")); Serial.println(widthMM);

  // 3-2. 왼쪽으로 돌기 -> 장애물은 또 왼쪽 -> 같은 방식으로 지나감
  turnLeftDeg(AVOID_TURN_DEG);
  int depthMM = passObstacle(-SIDE_LOOK_DEG, d0 + LEG2_MIN_MM,
                             widthMM + SIDE_SEEN_EXTRA_MM);
  Serial.print(F("[R 3-2] 장애물 지나감. 간 거리 ")); Serial.println(depthMM);
  lookAt(0, SERVO_BIG_MS);                             // 서보 정면으로

  // 4. 왼쪽으로 비스듬히 돌고, 선 찾으면서 직진
  turnLeftDeg(RETURN_TURN_DEG);
  if (findLine()) {                                    // 선 만나면 오른쪽으로 돌아 원래 방향
    spinRight();
    waitCenterOnLine();
  }
}

// ------------------------------------------------------------
// 왼쪽 회피 : 왼쪽으로 돌면 장애물은 "오른쪽"에 있음  (오른쪽 회피를 좌우만 뒤집은 것)
// ------------------------------------------------------------
void avoidLeft(int d0) {
  Serial.println(F("=== 왼쪽 회피 ==="));

  // 2. 왼쪽으로 확실하게 돌기
  turnLeftDeg(AVOID_TURN_DEG);
  Serial.println(F("[L 2] 왼쪽으로 돎 (장애물은 오른쪽)"));

  // 3-1. 오른쪽을 보면서 : 보이면 살짝 전진, 안 보이면 끝
  //      저장해 둔 장애물 폭(obstLeftMM) + 여유만큼은 무조건 직진
  int widthMM = passObstacle(SIDE_LOOK_DEG, obstLeftMM + SIDE_MARGIN_MM,
                             d0 + SIDE_SEEN_EXTRA_MM);
  Serial.print(F("[L 3-1] 옆으로 다 나감. 간 거리 ")); Serial.println(widthMM);

  // 3-2. 오른쪽으로 돌기 -> 장애물은 또 오른쪽 -> 같은 방식으로 지나감
  turnRightDeg(AVOID_TURN_DEG);
  int depthMM = passObstacle(SIDE_LOOK_DEG, d0 + LEG2_MIN_MM,
                             widthMM + SIDE_SEEN_EXTRA_MM);
  Serial.print(F("[L 3-2] 장애물 지나감. 간 거리 ")); Serial.println(depthMM);
  lookAt(0, SERVO_BIG_MS);                             // 서보 정면으로

  // 4. 오른쪽으로 비스듬히 돌고, 선 찾으면서 직진
  turnRightDeg(RETURN_TURN_DEG);
  if (findLine()) {                                    // 선 만나면 왼쪽으로 돌아 원래 방향
    spinLeft();
    waitCenterOnLine();
  }
}

// 선 찾으면서 직진. 찾으면 차 중심이 선 위에 오게 살짝 더 가고 true
bool findLine() {
  if (!moveMM(RETURN_MAX_MM, true)) {
    Serial.println(F("[4] 선 못 찾음 -> 전진하며 찾기"));
    return false;
  }
  Serial.println(F("[4] 선 발견 -> 원래 방향으로 돌기"));
  moveMM(LINE_OVER_MM, false);
  return true;
}

// (돌고 있는 중에) 가운데 센서가 선을 볼 때까지 기다렸다가 멈춤
void waitCenterOnLine() {
  unsigned long start = millis();
  while (digitalRead(C_Line) != 0 && millis() - start < (unsigned long)LINE_TURN_MAX_MS) { }
  stopCar();
}

// 장애물을 옆에 두고 지나가기 : 옆을 봄 -> 보이면 살짝 전진 -> 또 봄 -> 안 보이면 끝
//   sideOff = 서보가 볼 방향
//   minMM   = 최소 이만큼은 무조건 직진 (그 전에는 안 보여도 계속 감)
//   seenMM  = 옆 거리가 이 값보다 가까우면 "장애물 보임"
//   돌려주는 값 = 간 거리(mm)
int passObstacle(int sideOff, int minMM, int seenMM) {
  int goneMM = 0;
  Serial.print(F("    최소 ")); Serial.print(minMM);
  Serial.print(F(" mm 직진, 옆 거리 ")); Serial.print(seenMM); Serial.println(F(" 안쪽이면 보임"));

  int ds = lookAt(sideOff, SERVO_BIG_MS);              // 먼저 옆을 봄
  while (goneMM < PASS_MAX_MM) {
    Serial.print(F("    간 거리 ")); Serial.print(goneMM);
    Serial.print(F(" | 옆 거리 "));  Serial.print(ds);
    if (ds >= seenMM && goneMM >= minMM) {             // 최소 거리 갔고 옆에 안 보임 -> 다 지나감
      Serial.println(F(" -> 안 보임, 끝"));
      break;
    }
    if (ds < seenMM) Serial.println(F(" -> 보임, 살짝 전진"));
    else             Serial.println(F(" -> 아직 최소 거리 전, 전진"));
    moveMM(PASS_STEP_MM, false);                       // 살짝 가고
    goneMM += PASS_STEP_MM;
    delay(SETTLE_MS);                                  // 멈춰서 완전히 설 때까지 기다리고
    ds = lookAt(sideOff, SERVO_STEP_MS);               // 재고 -> 위에서 판단
  }
  moveMM(PASS_EXTRA_MM, false);                        // 차 뒷부분까지 빠지게 조금 더
  return goneMM + PASS_EXTRA_MM;
}

// 서보를 천천히 훑어서 장애물이 좌우로 얼마나 뻗어 있는지 재서 저장
void measureObstacle(int d0) {
  obstLeftMM = 0;
  obstRightMM = 0;
  int limit = d0 + OBST_EXTRA_MM;                      // 이 거리 안쪽이면 같은 장애물

  lookAt(-SWEEP_MAX_DEG, SERVO_BIG_MS);                // 왼쪽 끝으로 가서 시작
  for (int off = -SWEEP_MAX_DEG; off <= SWEEP_MAX_DEG; off += SWEEP_STEP_DEG) {
    int d = lookAt(off, SERVO_STEP_MS);
    Serial.print(F("  각도 ")); Serial.print(off);
    Serial.print(F(" : "));     Serial.println(d);
    if (d >= limit) continue;                          // 장애물 아님

    int side = d * sin(radians(abs(off)));             // 정면 기준 옆으로 뻗은 길이
    if (abs(off) >= SWEEP_MAX_DEG || side > SIDE_MAX_MM) side = SIDE_MAX_MM;   // 끝 각도에서도 보임 = 끝을 못 찾음 -> 최대로 잡음
    if (off < 0) obstLeftMM  = max(obstLeftMM, side);
    else         obstRightMM = max(obstRightMM, side);
  }
  lookAt(0, SERVO_BIG_MS);
}

// 서보를 정면 기준 offset 도 (- 왼쪽, + 오른쪽) 로 돌리고, waitMs 기다린 뒤 거리(mm) 재기
//   규칙 : 서보로 잴 때는 반드시 차가 멈춘 상태 (여기서 무조건 멈춤)
int lookAt(int offset, int waitMs) {
  stopCar();
  EduServo.write(SERVO_CENTER + (SERVO_FLIP ? -offset : offset));
  delay(waitMs);
  return Ultrasonic();
}

// mm 만큼 전진하고 멈춤. watchLine 이면 가는 중 선 보이면 멈추고 true
bool moveMM(int mm, bool watchLine) {
  unsigned long ms = (unsigned long)mm * 1000UL / MM_PER_SEC;
  unsigned long start = millis();
  goForward();
  while (millis() - start < ms) {
    if (watchLine && onLine()) { stopCar(); return true; }
  }
  stopCar();
  return false;
}

void turnRightDeg(int deg) {       // 제자리에서 오른쪽으로 deg 도 돌고 멈춤 (TURN_90_MS 기준)
  spinRight();
  delay((unsigned long)TURN_90_MS * deg / 90);
  stopCar();
  delay(TURN_PAUSE_MS);
}

void turnLeftDeg(int deg) {        // 제자리에서 왼쪽으로 deg 도 돌고 멈춤
  spinLeft();
  delay((unsigned long)TURN_90_MS * deg / 90);
  stopCar();
  delay(TURN_PAUSE_MS);
}

bool onLine() {                    // 라인센서 중 하나라도 흰 선(0)을 보면 true
  return digitalRead(L_Line) == 0 || digitalRead(C_Line) == 0 || digitalRead(R_Line) == 0;
}

int Ultrasonic() {                 // 앞 장애물까지 거리(mm). 없으면 9999
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, ECHO_WAIT_US);
  lastPing = millis();
  if (duration == 0) return 9999;
  int mm = ((float)(340 * duration) / 1000) / 2;
  if (mm < MIN_VALID_MM) return 9999;              // 너무 작은 값 = 잡음
  return mm;
}

// ============================================================
// 기본 동작
// ============================================================
void goForward()  { action = "직진";   drive(HIGH, HIGH, SPEED, SPEED); }            // 직진
void spinLeft()   { action = "좌회전"; drive(HIGH, LOW,  TURN_SPEED, TURN_SPEED); }  // 제자리 좌회전 (오른쪽 앞, 왼쪽 뒤)
void spinRight()  { action = "우회전"; drive(LOW,  HIGH, TURN_SPEED, TURN_SPEED); }  // 제자리 우회전 (오른쪽 뒤, 왼쪽 앞)
void stopCar()    { action = "정지";   drive(HIGH, HIGH, 0, 0); }                    // 정지

void curveLeft()  {                                               // 왼쪽 살짝 (왼쪽 바퀴 느리게)
  action = "왼쪽 살짝";
  drive(HIGH, HIGH, min(SPEED * CURVE_FAST, 255), max(SPEED * CURVE_SLOW, CURVE_MIN_SPEED));
}
void curveRight() {                                               // 오른쪽 살짝 (오른쪽 바퀴 느리게)
  action = "오른쪽 살짝";
  drive(HIGH, HIGH, max(SPEED * CURVE_SLOW, CURVE_MIN_SPEED), min(SPEED * CURVE_FAST, 255));
}

// 모터 직접 제어 : 방향(HIGH 앞 / LOW 뒤), 속도(0~255)
void drive(int rightDir, int leftDir, int rightSpeed, int leftSpeed) {
  digitalWrite(RightMotor_1_pin, rightDir);
  digitalWrite(RightMotor_2_pin, !rightDir);
  digitalWrite(LeftMotor_3_pin, leftDir);
  digitalWrite(LeftMotor_4_pin, !leftDir);

  analogWrite(RightMotor_E_pin, rightSpeed);
  analogWrite(LeftMotor_E_pin, leftSpeed);
}
