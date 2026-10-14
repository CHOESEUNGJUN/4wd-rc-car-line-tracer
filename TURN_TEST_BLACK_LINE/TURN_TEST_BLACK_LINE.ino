/* 회전 시간 맞추기 테스트 [검은 선 버전 : 밝은 책상 + 검은 테이프] (본 코드와 같은 모터 배선)
 *  어두운 책상 + 흰 테이프로 재려면 TURN_TEST_WHITE_LINE.ino 를 쓰세요.
 *
 *  시리얼 모니터 9600 baud, 한 글자씩 보내기
 *    c : 자동 측정 (* 모양 테이프 위에서 오른쪽, 왼쪽으로 여러 바퀴 돌며 평균)
 *    r : 오른쪽 90도      l : 왼쪽 90도
 *    e : 오른쪽 45도      q : 왼쪽 45도
 *    + : 방금 돈 시간 +20ms     - : 방금 돈 시간 -20ms
 *    p : 지금 값 전부 출력
 *
 *  [자동 측정 c] 책상에 테이프 4줄을 한가운데에서 겹쳐 * 모양으로 붙임 (가운데에서 8갈래, 45도 간격)
 *    차의 회전 중심을 * 한가운데에, 가운데 라인센서가 테이프 한 줄 위에 오게 놓고 c 를 보냄
 *    -> 45도마다 테이프를 지나가는 시간을 CAL_REVS 바퀴 동안 재서 평균을 냄
 *    -> 멈춘 상태에서 출발할 때 늦는 시간까지 더해서 90도 / 45도 시간을 계산해 위 값에 넣음
 *    -> r / l / e / q 로 실제로 돌려보고 확인
 *
 *  손으로 맞출 때는 차 바닥에 테이프로 방향 표시하고, 돌려보면서 +/- 로 맞춘 뒤
 *  마지막에 p 로 나온 숫자를 RC_CAR_BLACK_LINE (검은 선) 주행용 / 디버그용의 TURN_RIGHT_90_MS / TURN_LEFT_90_MS /
 *  TURN_RIGHT_45_MS / TURN_LEFT_45_MS 에 옮겨 적기
 */

// --- 맞출 값 (ms) ---
int RIGHT_90_MS = 550;
int LEFT_90_MS  = 900;
int RIGHT_45_MS = 500;
int LEFT_45_MS  = 500;

int TURN_SPEED = 170;      // 본 코드의 AVOID_TURN_SPEED 와 같게 (멈춘 상태에서 도는 속도)
int STEP_MS    = 20;       // +/- 한 번에 바뀌는 양

// --- 자동 측정 (c) ---
int TAPE_IS_WHITE = 0;     // 검은 선 버전 : 밝은 책상 + 검은 테이프
int CAL_REVS      = 10;    // 몇 바퀴 돌며 평균 낼지 (많을수록 정확, 오래 걸림)
int CAL_RAYS      = 8;     // * 모양 갈래 수 (테이프 4줄 = 8갈래 = 45도마다)
int CAL_WAIT_MS   = 2000;  // 이 시간 안에 다음 테이프를 못 만나면 측정 실패
int C_Line        = A4;    // 가운데 라인센서

// --- 모터 핀 (본 코드와 같음) ---
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
  Serial.println(F("=== 회전 시간 테스트 (검은 선 : 밝은 책상 + 검은 테이프) ==="));
  Serial.println(F("c : 자동 측정(* 테이프)   r/l : 오른쪽/왼쪽 90도   e/q : 오른쪽/왼쪽 45도   +/- : 20ms 조절   p : 값 출력"));
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
  else if (c == 'c') {
    Serial.println(F("=== 자동 측정 (3초 뒤 시작, 차를 * 한가운데에 놓기) ==="));
    delay(3000);
    measure(true);
    delay(1000);
    measure(false);
    printAll();
  }
}

void turn(bool toRight, int ms, const __FlashStringHelper* name) {
  Serial.print(name); Serial.print(F(" : ")); Serial.print(ms); Serial.println(F(" ms"));
  delay(500);                                  // 손 떼고 차 놓을 시간
  if (toRight) drive(LOW, HIGH, TURN_SPEED, TURN_SPEED);   // 제자리 우회전 (오른쪽 뒤, 왼쪽 앞)
  else         drive(HIGH, LOW, TURN_SPEED, TURN_SPEED);   // 제자리 좌회전 (오른쪽 앞, 왼쪽 뒤)
  delay(ms);
  stopCar();
}

// 가운데 센서가 테이프 위인지
bool onTape() { return (digitalRead(C_Line) == 0) == (TAPE_IS_WHITE == 1); }   // 흰색 = 0, 검은색 = 1

// 테이프에서 벗어났다가 다음 테이프를 만날 때까지 기다림. 만난 시각(millis), 실패하면 0
unsigned long nextTape() {
  unsigned long t0 = millis();
  while (onTape())  if (millis() - t0 > (unsigned long)CAL_WAIT_MS) return 0;
  delay(5);                                     // 테이프 가장자리에서 값이 튀는 것 무시
  while (!onTape()) if (millis() - t0 > (unsigned long)CAL_WAIT_MS) return 0;
  return millis();
}

// 한 방향으로 CAL_REVS 바퀴 돌며 45도 간격 평균과 출발 지연을 재서 90도 / 45도 값 계산
void measure(bool toRight) {
  Serial.print(toRight ? F("[오른쪽] ") : F("[왼쪽] "));
  if (!onTape()) { Serial.println(F("가운데 센서가 테이프 위에 없음 -> 측정 안 함")); return; }

  int total = CAL_REVS * CAL_RAYS;              // 지나갈 테이프 수
  unsigned long start = millis();
  if (toRight) drive(LOW, HIGH, TURN_SPEED, TURN_SPEED);
  else         drive(HIGH, LOW, TURN_SPEED, TURN_SPEED);

  unsigned long first = nextTape(), prev = first, last = first;
  unsigned long minGap = 99999, maxGap = 0;
  int count = (first > 0) ? 1 : 0;
  while (count > 0 && count < total) {
    unsigned long t = nextTape();
    if (t == 0) break;
    unsigned long gap = t - prev;
    if (gap < minGap) minGap = gap;
    if (gap > maxGap) maxGap = gap;
    prev = last = t;
    count++;
  }
  stopCar();

  if (count < CAL_RAYS + 1) { Serial.println(F("테이프를 충분히 못 만남 -> 측정 실패 (감도, TAPE_IS_WHITE 확인)")); return; }

  float per45 = (float)(last - first) / (count - 1);   // 45도 평균 시간
  float lag   = (float)(first - start) - per45;        // 처음 테이프(45도)까지 걸린 시간 - 45도 시간 = 출발 지연
  if (lag < 0) lag = 0;
  int t90 = lag + 2 * per45;
  int t45 = lag + per45;
  if (toRight) { RIGHT_90_MS = t90; RIGHT_45_MS = t45; }
  else         { LEFT_90_MS  = t90; LEFT_45_MS  = t45; }

  Serial.print(count); Serial.print(F("번 지남 | 45도 평균 ")); Serial.print(per45, 1);
  Serial.print(F(" ms (최소 ")); Serial.print(minGap); Serial.print(F(" / 최대 ")); Serial.print(maxGap);
  Serial.print(F(") | 출발 지연 ")); Serial.print(lag, 0);
  Serial.print(F(" ms -> 90도 ")); Serial.print(t90); Serial.print(F(" / 45도 ")); Serial.println(t45);
}

void printAll() {
  Serial.println(F("--- 지금 값 (본 코드에 옮겨 적기) ---"));
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
  analogWrite(RightMotor_E_pin, rightSpeed);
  analogWrite(LeftMotor_E_pin, leftSpeed);
}
