/* 에듀이노(Eduino) 4휠 스마트 RC카 - 라인트레이서 + 초음파 회피
 *  전원 켜면 바로 라인 따라 주행
 *
 *  고칠 때는 아래 [설정] 부분 숫자만 바꾸면 됩니다.
 *  파일 구조 :  [설정] -> [핀] -> setup/loop -> 장애물 -> 라인 -> 회피 -> 기본 동작
 */

#include <Servo.h>

// ============================================================
// [설정]  여기 숫자만 바꾸면 됨
// ============================================================

// --- 속도 (0~255) ---
int SPEED       = 153;     // 직진 / 후진 속도
int TURN_SPEED  = 153;     // 제자리 좌회전 / 우회전 속도 (커브에서 약하면 올리기)
float CURVE_FAST = 1.4;    // 살짝 틀 때 바깥 바퀴 = SPEED x 이 값
float CURVE_SLOW = 0.4;    // 살짝 틀 때 안쪽 바퀴 = SPEED x 이 값 (작을수록 많이 휨)

int CURVE_MIN_SPEED = 50;  // 살짝 틀 때 안쪽 바퀴 최저 속도 (안쪽 바퀴가 멈추면 올리기)

// --- 장애물 거리 (mm) ---
int AVOID_MM     = 60;    // 이 거리 안에 장애물 -> 좌우 확인하고 회피
int TOO_CLOSE_MM = 20;     // 이 거리 안이면 너무 가까움 -> 먼저 후진

// --- 너무 가까울 때 ---
int TOO_CLOSE_BACK_MS = 200;   // 후진하는 시간

// --- 회피 순서별 시간 (ms). 이 동안은 라인센서를 안 봄 ---
int STEP0_BACK_MS     = 200;    // 0. 뒤로 살짝 빠지기      (0 이면 후진 안 함)
int STEP1_TURN_OUT_MS = 800;    // 1. 빈 쪽으로 돌기
int STEP2_GO_OUT_MS   = 1000;   // 2. 옆으로 나가기          (장애물에 긁히면 늘리기)
int STEP3_TURN_FWD_MS = 800;    // 3. 다시 앞쪽으로 돌기     (보통 1번과 같게)
// 4. 장애물 옆 지나가기 : 조금 전진 -> 멈춰서 양옆 확인 -> 장애물 안 보일 때까지 반복
int PASS_STEP_MS      = 300;    //    한 번에 전진하는 시간
int SIDE_CLEAR_MM     = 200;    //    양옆이 이 거리보다 멀면 "장애물 지나감"
int PASS_MAX_STEPS    = 8;      //    최대 반복 횟수 (이만큼 해도 안 끝나면 그냥 다음 단계로)
int STEP5_TURN_IN_MS  = 500;    // 5. 선 쪽으로 꺾기         (선을 못 찾으면 늘리기)

// --- 멈춤 / 서보 대기 시간 (ms) ---
int STOP_WAIT_MS  = 200;   // 장애물 보고 멈춘 뒤 잠깐 기다리는 시간
int AVOID_WAIT_MS = 500;   // 회피 시작 전 멈춰 있는 시간
int SERVO_MOVE_MS = 300;   // 서보가 돌아갈 때까지 기다리는 시간 (거리 값이 이상하면 늘리기)
int SERVO_REST_MS = 700;   // 회피 전 좌우 확인할 때 한쪽 재고 쉬는 시간 (줄이면 좌우 확인이 빨라짐)

// --- 선을 놓쳤을 때 ---
unsigned long LOST_STOP_MS = 20000;   // 선 없는 상태가 이 시간 계속되면 정지 (20000 = 20초)

// --- 서보 (좌우 확인 각도) ---
int SERVO_CENTER = 90;     // 정면
int SERVO_SIDE_A = 30;     // 한쪽
int SERVO_SIDE_B = 150;    // 반대쪽
// 장애물 있는 쪽으로 회피하면 아래를 1로 (좌우 판단 뒤집기)
int FLIP_AVOID_SIDE = 0;

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
  Serial.println("Welcome Eduino! Line Trace");
}

void loop() {
  if (checkObstacle()) return;     // 장애물 처리했으면 이번 바퀴는 라인 안 봄
  followLine();
}

// ============================================================
// 장애물
// ============================================================
bool checkObstacle() {             // 장애물 있어서 처리했으면 true
  int d = Ultrasonic();
  if (d >= AVOID_MM) return false; // 장애물 없음

  if (d < TOO_CLOSE_MM) {
    Serial.println("너무 가까움 -> 후진");
    goBack();    delay(TOO_CLOSE_BACK_MS);
    stopCar();   delay(STOP_WAIT_MS);
  }
  else {
    stopCar();   delay(STOP_WAIT_MS);
    Serial.println("장애물 -> 좌우 확인");
    avoid(lookBothSides());
  }

  last_state = -1;                 // 라인 상태 처음부터 다시 판단
  lost_since = 0;
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
    if (lost_since == 0) lost_since = millis();
    if (millis() - lost_since >= LOST_STOP_MS) {
      stopCar();
      Serial.println("선 없음 -> 정지");
      lost_since = millis();
      last_state = -1;
    }
    return;
  }
  lost_since = 0;

  int state = L * 4 + C * 2 + R;
  if (state == last_state) return; // 상태 그대로면 아무것도 안 함
  last_state = state;

  Serial.print("digital : ");
  Serial.print(L); Serial.print(", ");
  Serial.print(C); Serial.print(", ");
  Serial.print(R); Serial.print("   ");

  //   L  C  R                              동작
  if      (L == 1 && C == 0 && R == 1) { goForward();  Serial.println("직진"); }
  else if (L == 1 && C == 0 && R == 0) { curveLeft();  Serial.println("왼쪽 살짝"); }
  else if (L == 1 && C == 1 && R == 0) { spinLeft();   Serial.println("좌회전"); }
  else if (L == 0 && C == 0 && R == 1) { curveRight(); Serial.println("오른쪽 살짝"); }
  else if (L == 0 && C == 1 && R == 1) { spinRight();  Serial.println("우회전"); }
  else if (L == 0 && R == 0)           { stopCar();    Serial.println("정지"); }
  else                                 { Serial.println(); }
}

// ============================================================
// 회피 (시간으로만 움직임, 라인센서 무시)
//   toRight = true  : 오른쪽으로 피함
//   toRight = false : 왼쪽으로 피함
// ============================================================
void avoid(bool toRight) {
  Serial.println(toRight ? "회피 : 오른쪽" : "회피 : 왼쪽");

  stopCar();                          delay(AVOID_WAIT_MS);
  goBack();                           delay(STEP0_BACK_MS);      // 0. 뒤로 살짝

  if (toRight) spinRight(); else spinLeft();
                                      delay(STEP1_TURN_OUT_MS);  // 1. 빈 쪽으로 돌기
  goForward();                        delay(STEP2_GO_OUT_MS);    // 2. 옆으로 나가기
  if (toRight) spinLeft(); else spinRight();
                                      delay(STEP3_TURN_FWD_MS);  // 3. 다시 앞쪽으로 돌기
  for (int i = 0; i < PASS_MAX_STEPS; i++) {                     // 4. 장애물 옆 지나가기
    goForward();                      delay(PASS_STEP_MS);       //    조금 전진
    stopCar();
    if (sidesClear()) break;                                     //    양옆 확인 : 안 보이면 다 지나간 것
  }
  if (toRight) spinLeft(); else spinRight();
                                      delay(STEP5_TURN_IN_MS);   // 5. 선 쪽으로 꺾기

  Serial.println("회피 끝 -> 라인 찾기");
  goForward();                                                   // 6. 전진하면서 라인 찾기
}

bool sidesClear() {                // 서보로 양옆 확인. 양쪽 다 SIDE_CLEAR_MM 보다 멀면 true
  EduServo.write(SERVO_SIDE_A);  delay(SERVO_MOVE_MS);
  int dist_A = Ultrasonic();
  EduServo.write(SERVO_SIDE_B);  delay(SERVO_MOVE_MS);
  int dist_B = Ultrasonic();
  EduServo.write(SERVO_CENTER);  delay(SERVO_MOVE_MS);

  Serial.print("양옆 : "); Serial.print(dist_A);
  Serial.print(" / ");     Serial.println(dist_B);
  return dist_A > SIDE_CLEAR_MM && dist_B > SIDE_CLEAR_MM;
}

bool lookBothSides() {             // 서보로 좌우 거리 재기. true = 오른쪽으로 피하기
  EduServo.write(SERVO_SIDE_A);  delay(SERVO_MOVE_MS);
  int dist_A = Ultrasonic();     delay(SERVO_REST_MS);
  EduServo.write(SERVO_SIDE_B);  delay(SERVO_MOVE_MS);
  int dist_B = Ultrasonic();     delay(SERVO_REST_MS);
  EduServo.write(SERVO_CENTER);

  bool toRight = !(dist_A > dist_B);       // 원본 Servo_con() 과 같은 판단
  if (FLIP_AVOID_SIDE) toRight = !toRight;
  return toRight;
}

int Ultrasonic() {                 // 앞 장애물까지 거리(mm). 없으면 9999
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 12000);   // 12ms(약 2m)까지만 기다림
  if (duration == 0) return 9999;
  return ((float)(340 * duration) / 1000) / 2;
}

// ============================================================
// 기본 동작
// ============================================================
void goForward()  { drive(HIGH, HIGH, SPEED, SPEED); }            // 직진
void goBack()     { drive(LOW,  LOW,  SPEED, SPEED); }            // 후진
void spinLeft()   { drive(HIGH, LOW,  TURN_SPEED, TURN_SPEED); }  // 제자리 좌회전 (오른쪽 앞, 왼쪽 뒤)
void spinRight()  { drive(LOW,  HIGH, TURN_SPEED, TURN_SPEED); }  // 제자리 우회전 (오른쪽 뒤, 왼쪽 앞)
void stopCar()    { drive(HIGH, HIGH, 0, 0); }                    // 정지

void curveLeft()  {                                               // 왼쪽 살짝 (왼쪽 바퀴 느리게)
  drive(HIGH, HIGH, min(SPEED * CURVE_FAST, 255), max(SPEED * CURVE_SLOW, CURVE_MIN_SPEED));
}
void curveRight() {                                               // 오른쪽 살짝 (오른쪽 바퀴 느리게)
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
