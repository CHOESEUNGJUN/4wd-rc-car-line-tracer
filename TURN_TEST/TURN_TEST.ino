/* 회전 시간 맞추기 테스트 (본 코드 과 같은 모터 배선)
 *
 *  시리얼 모니터 9600 baud, 한 글자씩 보내기
 *    r : 오른쪽 90도      l : 왼쪽 90도
 *    e : 오른쪽 45도      q : 왼쪽 45도
 *    + : 방금 돈 시간 +20ms     - : 방금 돈 시간 -20ms
 *    p : 지금 값 전부 출력
 *
 *  차 바닥에 테이프로 방향 표시하고, 돌려보면서 +/- 로 맞춘 뒤
 *  마지막에 p 로 나온 숫자를 RC_CAR_WHITE_LINE/RC_CAR_WHITE_LINE.ino 의 TURN_RIGHT_90_MS / TURN_LEFT_90_MS /
 *  TURN_RIGHT_45_MS / TURN_LEFT_45_MS 에 옮겨 적기
 */

// --- 맞출 값 (ms) ---
int RIGHT_90_MS = 550;
int LEFT_90_MS  = 900;
int RIGHT_45_MS = 500;
int LEFT_45_MS  = 500;

int TURN_SPEED = 170;      // 본 코드 의 AVOID_TURN_SPEED 와 같게 (멈춘 상태에서 도는 속도)
int STEP_MS    = 20;       // +/- 한 번에 바뀌는 양

// --- 좌우 바퀴 속도 보정 (같은 값을 줘도 모터마다 힘이 달라서) ---
//   직진시켜서 한쪽으로 휘면 느린 쪽에 더하기. 오른쪽으로 휘면 오른쪽이 느린 것
int RIGHT_TRIM = 0;           // 오른쪽 바퀴에 더할 속도
int LEFT_TRIM  = 8;           // 왼쪽 바퀴에 더할 속도 (0 이면 오른쪽, 15 면 왼쪽으로 쏠림)

// --- 모터 핀 (본 코드 과 같음) ---
int RightMotor_E_pin = 5;
int LeftMotor_E_pin  = 6;
int RightMotor_1_pin = 8;
int RightMotor_2_pin = 9;
int LeftMotor_3_pin  = 10;
int LeftMotor_4_pin  = 11;

int* last = &RIGHT_90_MS;  // 방금 테스트한 값 (+/- 대상)

void setup() {
  pinMode(RightMotor_E_pin, OUTPUT);
  pinMode(RightMotor_1_pin, OUTPUT);
  pinMode(RightMotor_2_pin, OUTPUT);
  pinMode(LeftMotor_3_pin, OUTPUT);
  pinMode(LeftMotor_4_pin, OUTPUT);
  pinMode(LeftMotor_E_pin, OUTPUT);
  stopCar();

  Serial.begin(9600);
  Serial.println(F("=== 회전 시간 테스트 ==="));
  Serial.println(F("r/l : 오른쪽/왼쪽 90도   e/q : 오른쪽/왼쪽 45도   +/- : 20ms 조절   p : 값 출력"));
  printAll();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();

  if      (c == 'r') { last = &RIGHT_90_MS; turn(true,  RIGHT_90_MS, F("오른쪽 90도")); }
  else if (c == 'l') { last = &LEFT_90_MS;  turn(false, LEFT_90_MS,  F("왼쪽 90도")); }
  else if (c == 'e') { last = &RIGHT_45_MS; turn(true,  RIGHT_45_MS, F("오른쪽 45도")); }
  else if (c == 'q') { last = &LEFT_45_MS;  turn(false, LEFT_45_MS,  F("왼쪽 45도")); }
  else if (c == '+') { *last += STEP_MS; Serial.print(F("-> ")); Serial.print(*last); Serial.println(F(" ms (다시 돌려보기)")); }
  else if (c == '-') { *last = max(*last - STEP_MS, 0); Serial.print(F("-> ")); Serial.print(*last); Serial.println(F(" ms (다시 돌려보기)")); }
  else if (c == 'p') { printAll(); }
}

void turn(bool toRight, int ms, const __FlashStringHelper* name) {
  Serial.print(name); Serial.print(F(" : ")); Serial.print(ms); Serial.println(F(" ms"));
  delay(500);                                  // 손 떼고 차 놓을 시간
  if (toRight) drive(LOW, HIGH, TURN_SPEED, TURN_SPEED);   // 제자리 우회전 (오른쪽 뒤, 왼쪽 앞)
  else         drive(HIGH, LOW, TURN_SPEED, TURN_SPEED);   // 제자리 좌회전 (오른쪽 앞, 왼쪽 뒤)
  delay(ms);
  stopCar();
}

void printAll() {
  Serial.println(F("--- 지금 값 (본 코드 에 옮겨 적기) ---"));
  Serial.print(F("오른쪽 90도 : ")); Serial.println(RIGHT_90_MS);
  Serial.print(F("왼쪽   90도 : ")); Serial.println(LEFT_90_MS);
  Serial.print(F("오른쪽 45도 : ")); Serial.println(RIGHT_45_MS);
  Serial.print(F("왼쪽   45도 : ")); Serial.println(LEFT_45_MS);
}

void stopCar() { drive(HIGH, HIGH, 0, 0); }

void drive(int rightDir, int leftDir, int rightSpeed, int leftSpeed) {
  digitalWrite(RightMotor_1_pin, rightDir);
  digitalWrite(RightMotor_2_pin, !rightDir);
  digitalWrite(LeftMotor_3_pin, leftDir);
  digitalWrite(LeftMotor_4_pin, !leftDir);
  analogWrite(RightMotor_E_pin, rightSpeed == 0 ? 0 : constrain(rightSpeed + RIGHT_TRIM, 0, 255));
  analogWrite(LeftMotor_E_pin,  leftSpeed  == 0 ? 0 : constrain(leftSpeed  + LEFT_TRIM,  0, 255));
}
