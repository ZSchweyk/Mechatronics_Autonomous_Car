#include <Pixy2.h>
#include <DualMAX14870MotorShield.h>
#define encoder1PinA 2
#define encoder1PinB 3
#define encoder2PinA 18
#define encoder2PinB 19

// INPUTS
int DISTANCE_THRESHOLD = 7;
unsigned long TURN_START_DELAY = 3000;
unsigned long TURN_END_DELAY = 3000;
int speed = 150;
int encoderCountsOneRev = 118;
int QUARTER_TURN_COUNTS = encoderCountsOneRev * 0.5;
////////////////////////////////////////////////////


volatile long m1EncoderCount = 0;
volatile long m2EncoderCount = 0;
int signal = 13;
int distance = 0;

enum FSM {
  RIGHT,    // right
  LEFT,     // left
  TURN180,  // 180
  FORWARD,
  BACKWARD,
  STOP,
};


Pixy2 pixy;
DualMAX14870MotorShield motors;
FSM current_state;

void setup() {
  Serial.begin(115200);
  attachInterrupt(0, doEncoder1, RISING);  // PIN 2 generates the interrupt
  attachInterrupt(1, doEncoder2, RISING);  // PIN 2 generates the interrupt
  pinMode(signal, OUTPUT);
  pixy.init();
  current_state = FORWARD;
}

int measureDistance() {
  pinMode(signal, OUTPUT);
  digitalWrite(signal, LOW);
  delayMicroseconds(5);

  digitalWrite(signal, HIGH);
  delayMicroseconds(5);
  digitalWrite(signal, LOW);

  pinMode(signal, INPUT);
  unsigned long pulseduration = pulseIn(signal, HIGH);  // maybe make local
  return 0.0343 * pulseduration / 2;
}
void loop() {
  // delay(250);
  distance = measureDistance();

  Serial.print("distance: ");
  Serial.println(distance);

  pixy.ccc.getBlocks();
  uint16_t color;
  //color = pixy.ccc.blocks[0].m_signature;
  if (pixy.ccc.numBlocks) {
    color = pixy.ccc.blocks[0].m_signature;
    uint16_t x = pixy.ccc.blocks[0].m_x;
    uint16_t y = pixy.ccc.blocks[0].m_y;
    uint16_t width = pixy.ccc.blocks[0].m_width;

    Serial.print("Color: ");
    Serial.println(color);
    //Serial.print(" X: "); Serial.print(x);
    //Serial.print(" Y: "); Serial.print(y);
    //Serial.print(" Width: "); Serial.println(width);
  }

  switch (current_state) {
    case FORWARD:
      // check if distance is within threshold
      go_forward();
      Serial.print("Current state: ");
      Serial.println("forward");
      if (distance <= DISTANCE_THRESHOLD) {
        //stop();
        // check color and change state
        if (color == 1) {
          current_state = LEFT;
        } else if (color == 2) {
          current_state = RIGHT;
        } else if (color == 3) {
          current_state = TURN180;
        }
      }
      break;
    case RIGHT:
      stop();
      delay(TURN_START_DELAY);
      turn(1, 90);
      delay(TURN_END_DELAY);
      Serial.print(" Current state: ");
      Serial.println("right");
      current_state = FORWARD;
      break;
    case LEFT:
      stop();
      delay(TURN_START_DELAY);
      turn(-1, 90);
      delay(TURN_END_DELAY);
      Serial.print("Current state: ");
      Serial.println("left");
      current_state = FORWARD;
      break;
    case TURN180:
      stop();
      delay(TURN_START_DELAY);
      turn(1, 180);
      delay(TURN_END_DELAY);
      Serial.print(" Current state: ");
      Serial.println("180");
      current_state = FORWARD;
      break;
    case BACKWARD:
      // backward();
      Serial.print(" Current state: ");
      Serial.println("backward");
      current_state = FORWARD; // change depending on FSM
      break;
    case STOP:
      stop();
      delay(3000);
      current_state = FORWARD;
      break;
  }
}

/* 
  direction: -1 = left, 1 = right
*/
void turn(int direction, int deg) {
  motors.enableDrivers();
  int initialM2EncoderCount = m2EncoderCount;

  motors.setM1Speed(direction * speed);
  motors.setM2Speed(-direction * speed);
  
  while (abs(m2EncoderCount - initialM2EncoderCount) < QUARTER_TURN_COUNTS * deg / 90) {
    // wait
    // Serial.print("encoder count: ");
    Serial.print(m2EncoderCount - initialM2EncoderCount);
    Serial.print(" out of ");
    Serial.println(QUARTER_TURN_COUNTS * deg / 90);
  }
  stop();
}

void turn_right() {
  // motors.enableDrivers();
  // motors.setM1Speed(100);
  // motors.setM2Speed(0);
  // delay(2000);
  // stop();
  /*
  motors.enableDrivers();
  int initialM1EncoderCount = m1EncoderCount;
  motors.setM2Speed(50);
  //motors.setM2Speed(0);
  while (m1EncoderCount - initialM1EncoderCount < encoderCountsOneRev / 4) {
    // wait
  }
  stop();
*/
  Serial.println("right_function");
  motors.enableDrivers();
  int initialM2EncoderCount = m2EncoderCount;
  motors.setM1Speed(speed);
  motors.setM2Speed(-speed);
  while (abs(m2EncoderCount - initialM2EncoderCount) < encoderCountsOneRev * .5) {
    // wait
    //Serial.print("encoder count: ");
    //Serial.println(m2EncoderCount - initialM2EncoderCount);
  }
  delay(1000);
  stop();
}

void turn_left() {
  // motors.enableDrivers();
  // motors.setM1Speed(0);
  // motors.setM2Speed(200);
  // delay(2000);
  // stop();

  
  Serial.println("left_function");
  motors.enableDrivers();
  int initialM2EncoderCount = m2EncoderCount;
  motors.setM1Speed(-speed);
  motors.setM2Speed(speed);
  
  Serial.print("Max encoder count: ");
  Serial.println(encoderCountsOneRev * .5);
  while (abs(m2EncoderCount - initialM2EncoderCount) < encoderCountsOneRev * .5) {
    // wait
    // Serial.print("encoder count: ");
    Serial.print(m2EncoderCount - initialM2EncoderCount);
    Serial.print(" out of ");
    Serial.println(encoderCountsOneRev * .5);
  }
  stop();
  
}

void turn_180() {

  Serial.println("180_function");
  motors.enableDrivers();
  motors.setM1Speed(-speed);
  motors.setM2Speed(speed);
  delay(2000);
  stop();
}

void go_forward() {
  motors.enableDrivers();
  motors.setM1Speed(speed);
  motors.setM2Speed(speed);
}

void go_backward() {
  motors.enableDrivers();
  motors.setM1Speed(-speed);
  motors.setM2Speed(-speed);
  delay(1000);
  stop();
}

void stop() {
  motors.setM1Speed(0);
  motors.setM2Speed(0);
  motors.disableDrivers();
}




void doEncoder1() {
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    m1EncoderCount++;
  } else {
    m1EncoderCount--;
  }
}

void doEncoder2() {
  if (digitalRead(encoder2PinA) == digitalRead(encoder2PinB)) {
    m2EncoderCount++;
  } else {
    m2EncoderCount--;
  }
}
