#define BS 200
#define NODES 12
bool onStartMark = true;
byte p = 0b00000000;

#include <Adafruit_NeoPixel.h>

constexpr uint8_t LED_PIN   = A5;
constexpr uint8_t LED_COUNT = 8;

Adafruit_NeoPixel leds(
  LED_COUNT,
  LED_PIN,
  NEO_GRB + NEO_KHZ800
);

int place1[3] = { 0, 0, 0 };
int place2[3] = { 0, 0, 0 };
int place3[3] = { 0, 0, 0 };
int place4[3] = { 0, 0, 0 };

int pole[][3] = {
  { 0, 2, 3 },
  { 1, 10, 4 },
  { 2, 3, 3 },
  { 2, 0, 1 },
  { 3, 2, 1 },
  { 3, 4, 3 },
  { 3, 10, 2 },
  { 4, 3, 1 },
  { 4, 5, 3 },
  { 5, 4, 1 },
  { 5, 6, 3 },
  { 5, 7, 2 },
  { 6, 5, 1 },
  { 7, 5, 4 },
  { 7, 8, 2 },
  { 7, 10, 1 },
  { 8, 7, 4 },
  { 8, 9, 2 },
  { 9, 8, 4 },
  { 10, 7, 3 },
  { 10, 3, 4 },
  { 10, 11, 1 },
  { 10, 1, 2 },
  { 11, 10, 3 }
};

int poleSize = sizeof(pole) / sizeof(pole[0]);

int now = 0;
int poz = 3;

#include <Servo.h>
Servo servoGrip;
Servo servoUp;

volatile long encl = 0;
volatile long encr = 0;

int whiteRigth = 1000;
int whiteLeft = 1000;
int grey = 500;

void setup() {

  leds.begin();

  leds.setBrightness(40);

  leds.clear();
  leds.show();

  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  servoGrip.attach(11);
  servoUp.attach(12);
  servoGrip.write(35);
  servoUp.write(0);

  pinMode(10, INPUT);
  pinMode(13, OUTPUT);

  pinMode(A3, INPUT);

  attachInterrupt(digitalPinToInterrupt(2), l_enc, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), r_enc, FALLING);
  Serial.begin(9600);
}

void setLed(uint8_t number, uint8_t r, uint8_t g, uint8_t b) {
  if (number >= LED_COUNT) {
    return;
  }

  leds.setPixelColor(number, leds.Color(r, g, b));
  leds.show();
}

void clearLed(uint8_t number) {
  setLed(number, 0, 0, 0);
}

void setAll(uint8_t r, uint8_t g, uint8_t b) {
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    leds.setPixelColor(i, leds.Color(r, g, b));
  }

  leds.show();
}

void clearAll() {
  leds.clear();
  leds.show();
}

void l_enc() {
  if (digitalRead(9)) {
    encl++;
  } else {
    encl--;
  }
}

void r_enc() {
  if (digitalRead(8)) {
    encr--;
  } else {
    encr++;
  }
}

void resetticks() {
  encl = 0;
  encr = 0;
}

void sync(int s = BS) {
  int u = (encl - encr) * 30;
  runMotors(s - u, s + u);
}

void rotate(int s = BS) {
  long u = (encl + encr) * 30;
  runMotors(s - u, -s - u);
}

#define MM_TO_TICKS (1000.0 / 250.0)
void goDist(int d, int s = BS) {
  resetticks();
  int k = d < 0 ? -1 : 1;
  while (min(abs(encl), abs(encr)) < abs(d) * MM_TO_TICKS) {
    sync(s * k);
  }
  stop();
}

#define WHEEL_BASE_MM 180.0

void turnAngle(float angle, int s = BS) {
  resetticks();

  int k = angle < 0 ? -1 : 1;
  float wheelDist = PI * WHEEL_BASE_MM * abs(angle) / 360.0;
  long targetTicks = round(wheelDist * MM_TO_TICKS);

  while (min(abs(encl), abs(encr)) < targetTicks) {
    runMotors(s * k, -s * k);
  }

  stop();
}

void move(int lin, int ang) {
  float left = lin + ang;
  float right = lin - ang;
  float k = left / right;
}

int rightLine() {
  int x = analogRead(A0);
  int r = map(constrain(x, 0, whiteRigth), 0, whiteRigth, 0, 1000);
  return r;
}

int lCube(int s) {

  digitalWrite(13, LOW);
  delayMicroseconds(2);
  digitalWrite(13, HIGH);
  delayMicroseconds(10);
  digitalWrite(13, LOW);

  long duration = pulseIn(10, HIGH, 25000);

  if (duration == 0) {
    return -1;
  }

  int x = duration / 58.2;
  if (x > s) {
    return 0;
  } else {
    return 1;
  }
}

int rCube(int s) {
  int raw = analogRead(A3);
  if (raw < 80) return 0;

  int t = int(4800.0 / (raw - 20.0));
  if (t > s) return 0;

  showCubeDetected();
  return 1;
}

int leftLine() {
  int x = analogRead(A1);
  int r = map(constrain(x, 0, whiteLeft), 0, whiteLeft, 0, 1000);
  return r;
}

void runMotors(int speedL, int speedR) {
  digitalWrite(4, speedL < 0);
  digitalWrite(7, speedR < 0);
  analogWrite(5, constrain(abs(speedR), 0, 255));
  analogWrite(6, constrain(abs(speedL), 0, 255));
}

void ridelinepd(int s) {
  int err = leftLine() - rightLine();
  float k = 0.3;
  int u = err * k;
  if (s > 0) {
    runMotors(s + u, s - u);
  } else runMotors(s - u, s + u);
}

void stop() {
  long t0 = millis();
  while (millis() < t0 + 300) {
    runMotors(255, 255);
    delay(10);
    runMotors(-255, -255);
    delay(10);
  }
  runMotors(0, 0);
}

void tocross() {
  while (!(leftLine() < grey && rightLine() < grey)) {
    ridelinepd(200);
  }
  stop();
}

void toNcross(int n, int s) {
  int x = 0;
  bool flag = 0;
  while (n > x) {
    ridelinepd(s);
    if (leftLine() < grey and rightLine() < grey) {
      flag = 1;
    }
    if (leftLine() > grey and rightLine() > grey and flag == 1) {
      flag = 0;
      x++;
    }
  }
  stop();
}

void toNsocrossRight(int n, int s) {
  int x = 0;
  bool flag = 0;
  while (n > x) {
    ridelinepd(s);
    if (leftLine() > grey and rightLine() < grey) {
      flag = 1;
    }
    if (leftLine() > grey and rightLine() > grey and flag == 1) {
      flag = 0;
      x++;
    }
  }
  stop();
}

void toNsocrossLeft(int n, int s) {
  int x = 0;
  bool flag = 0;

  while (n > x) {
    ridelinepd(s);

    if (leftLine() < grey && rightLine() > grey) {
      flag = 1;
    }
    if (leftLine() > grey && rightLine() > grey && flag == 1) {
      flag = 0;
      x++;
    }
  }
  stop();
}

void turnRight() {

  runMotors(255, 255);
  delay(300);

  runMotors(-255, 255);
  delay(150);

  while (rightLine() > grey) {
  }
  runMotors(0, 0);
}

void turnLeft() {

  runMotors(255, 255);
  delay(300);

  runMotors(255, -255);
  delay(150);

  while (leftLine() > grey) {
  }
  runMotors(0, 0);
}

void greedRcube() {
  turnRight();
  servoUp.write(90);
  servoGrip.write(150);
  delay(1000);
  if (rightLine() > grey and leftLine() > grey) { toNcross(1, 200); }
  servoGrip.write(20);
  delay(1000);
  servoUp.write(0);
  delay(1000);
  turnLeft();
}

void scan_place2() {
  place2[2] = rCube(20);
  toNcross(1, 200);
  place2[1] =  rCube(20);
  toNcross(1, 200);
  place2[0] = rCube(20);
}

void scan_place1() {
  turnAngle(80);
  delay(200);
  place1[0] = rCube(15);
  goDist(-120, 150);
  delay(100);
  place1[1] = rCube(15);
  goDist(-120, 150);
  delay(100);

  place1[2] = rCube(15);

  goDist(240, 150);
  turnAngle(-80);
}

void scan_place3() {
  turnAngle(80);
  delay(200);
  place3[0] = rCube(15);
  goDist(-120, 150);
  delay(100);
  place3[1] = rCube(15);
  goDist(-120, 150);
  delay(100);

  place3[2] = rCube(15);

  goDist(240, 150);
  turnAngle(-80);
}

void scan_place4() {
  place4[2] = rCube(60);
  toNcross(1, 200);
  place4[1] = rCube(60);
  toNcross(1, 200);
  place4[0] = rCube(60);
}

void run(int a, int b) {
  if (a < 0 || a >= NODES || b < 0 || b >= NODES) return;
  if (a != now || a == b) return;

  if (onStartMark) {
    toNcross(1, 200);
    onStartMark = false;
  }

  const int STATES = NODES * 4;
  int cost[STATES];
  int previous[STATES];
  bool checked[STATES];

  for (int i = 0; i < STATES; i++) {
    cost[i] = 30000;
    previous[i] = -1;
    checked[i] = false;
  }

  int start = a * 4 + (poz - 1);
  cost[start] = 0;
  int finish = -1;

  for (int step = 0; step < STATES; step++) {
    int current = -1;

    for (int i = 0; i < STATES; i++) {
      if (!checked[i] && (current == -1 || cost[i] < cost[current])) {
        current = i;
      }
    }

    if (current == -1 || cost[current] == 30000) break;

    checked[current] = true;

    int point = current / 4;
    int currentPoz = current % 4 + 1;

    if (point == b) {
      finish = current;
      break;
    }

    for (int i = 0; i < poleSize; i++) {
      if (pole[i][0] != point) continue;

      int next = pole[i][1];
      int neededPoz = pole[i][2];
      int difference = (neededPoz - currentPoz + 4) % 4;

      int turnCost = 0;
      if (difference == 1 || difference == 3) turnCost = 2;
      if (difference == 2) turnCost = 4;

      int nextState = next * 4 + (neededPoz - 1);
      int newCost = cost[current] + 1 + turnCost;

      if (newCost < cost[nextState]) {
        cost[nextState] = newCost;
        previous[nextState] = current;
      }
    }
  }

  if (finish == -1) return;

  int path[STATES];
  int length = 0;

  for (int state = finish; state != start;
       state = previous[state]) {
    path[length++] = state;
  }

  for (int i = length - 1; i >= 0; i--) {
    int next = path[i] / 4;
    int neededPoz = path[i] % 4 + 1;
    int difference = (neededPoz - poz + 4) % 4;

    turnToPoz(neededPoz);

    bool currentIsHalf = (now == 8 || now == 9);
    bool nextIsHalf = (next == 8 || next == 9);

    int count = 1;

    if (difference == 2 &&
        currentIsHalf == nextIsHalf &&
        now != 6) {
      count = 2;
    }

    if (nextIsHalf) {
      if (neededPoz == 4) {
        toNsocrossLeft(count, 200);
      } else {
        toNsocrossRight(count, 200);
      }
    } else {
      toNcross(count, 200);
    }

    now = next;
  }
}

void showCubeDetected() {
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    if (i < 3) {
      leds.setPixelColor(i, leds.Color(255, 255, 255));
    } else if (i < 5) {
      leds.setPixelColor(i, leds.Color(0, 0, 255));
    } else {
      leds.setPixelColor(i, leds.Color(255, 0, 0));
    }
  }

  leds.show();
  delay(1000);
  leds.clear();
  leds.show();
}

void turnToPoz(int neededPoz) {
  int difference = (neededPoz - poz + 4) % 4;

  if (difference == 1) {
    turnRight();
  } else if (difference == 3) {
    turnLeft();
  } else if (difference == 2) {
    turnAngle(160, 150);

    while (rightLine() > grey) {
      runMotors(110, -110);
    }
    runMotors(0, 0);
  }

  poz = neededPoz;
}

void loop() {
  delay(1000);
  run(0, 6);
  scan_place1();
  run(6, 2);

  exit(0);
}
