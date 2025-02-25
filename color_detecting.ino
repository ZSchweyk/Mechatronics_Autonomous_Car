#include <Pixy2.h>
#include <DualMAX14870MotorShield.h>
#define encoder1PinA 2
#define encoder1PinB 3
#define encoder2PinA 18
#define encoder2PinB 19

volatile long m1EncoderCount = 0;
volatile long m2EncoderCount = 0;

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
//float dist_threshold = 15; // cm
int signal = 13;
int distance = 0;
unsigned long pulseduration = 0;

int encoderCountsOneRev = 118;


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
  pulseduration = pulseIn(signal, HIGH);  // maybe make local
  return 0.0343 * pulseduration / 2;
}
void loop() {

  distance = measureDistance();

  Serial.print("distance: ");
  Serial.print(distance);

  pixy.ccc.getBlocks();
  uint16_t color;
  //color = pixy.ccc.blocks[0].m_signature;
  if (pixy.ccc.numBlocks) {
    color = pixy.ccc.blocks[0].m_signature;
    uint16_t x = pixy.ccc.blocks[0].m_x;
    uint16_t y = pixy.ccc.blocks[0].m_y;
    uint16_t width = pixy.ccc.blocks[0].m_width;

    Serial.print(" Color: ");
    Serial.print(color);
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
      if (distance <= 4) {
        //stop();
        // check color and change state
        if (color == 1) {
          current_state = LEFT;
        }
        // currrent_state = LEFT;

        if (color == 2) {
          current_state = RIGHT;
        }

        if (color == 3) {
          current_state = TURN180;
        }
      }
      break;
    case RIGHT:
      turn_right();
      Serial.print(" Current state: ");
      Serial.println("right");
      current_state = FORWARD;
      break;
    case LEFT:
      turn_left();
      Serial.print("Current state: ");
      Serial.println("left");
      current_state = FORWARD;
      break;

    case TURN180:
      turn_180();
      Serial.print(" Current state: ");
      Serial.println("180");
      current_state = FORWARD;
      break;
    case BACKWARD:
      break;
    case STOP:
      stop();
      delay(3000);
      current_state = FORWARD;
      break;
  }
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
  motors.setM2Speed(-100);
  motors.setM1Speed(100);
  // while (abs(m2EncoderCount - initialM2EncoderCount) < encoderCountsOneRev * .5) {
  //   // wait
  //   //Serial.print("encoder count: ");
  //   //Serial.println(m2EncoderCount - initialM2EncoderCount);
  // }
  delay(1000);
  //stop();

  //Serial.println("right_end_function");
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
  motors.setM2Speed(100);
  motors.setM1Speed(-100);
  // while (abs(m2EncoderCount - initialM2EncoderCount) < encoderCountsOneRev * .5) {
  //   // wait
  //   //Serial.print("encoder count: ");
  //   //Serial.println(m2EncoderCount - initialM2EncoderCount);
  // }
  delay(1000);
  Serial.println("left-end");
  stop();
}

void turn_180() {

  Serial.println("180_function");
  motors.enableDrivers();
  motors.setM1Speed(100);
  motors.setM2Speed(-100);
  delay(2000);
  //stop();

  //Serial.println("180_end_function");
}

void go_forward() {
  motors.enableDrivers();
  motors.setM1Speed(100);
  motors.setM2Speed(100);
}

void go_backward() {
  motors.enableDrivers();
  motors.setM1Speed(-200);
  motors.setM2Speed(-200);
}

void stop() {
  motors.setM1Speed(0);
  motors.setM2Speed(0);
  motors.disableDrivers();
}
