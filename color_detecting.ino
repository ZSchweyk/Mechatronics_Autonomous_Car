#include <Pixy2.h>
#include <DualMAX14870MotorShield.h>
#define sensorL A8
#define encoder1PinA 2
#define encoder1PinB 3
#define encoder2PinA 18
#define encoder2PinB 19

volatile long m1EncoderCount = 0;
volatile long m2EncoderCount = 0;
int valL = 0;
float VoltageL;
float distanceL;

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
  valL = analogRead(sensorL);
  VoltageL = valL * 0.00488;
  distanceL = -8.8664 * VoltageL + 31.864;

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
  else {
    color = 0;
    Serial.print(" Color: ");
    Serial.print(color);
  }

  switch (current_state) {
    case FORWARD:
      // check if distance is within threshold
      Serial.print("IR distance: ");
      Serial.println(distanceL);
      if (distanceL <= 3 || distanceL >= 50){
        stop();
        motors.enableDrivers();
        motors.setM1Speed(100);
        //motors.setM2Speed(-100);
        delay(70);
        stop();
        Serial.print("IR distance: ");
        Serial.println(distanceL);
      }
      go_forward();
      Serial.print("Current state: ");
      Serial.println("forward");
      if (distance <= 4) {
        //stop();
        // check color and change state
        if (color == 1) {
          current_state = LEFT;
        }

        if (color == 2) {
          current_state = RIGHT;
        }

        if (color == 3) {
          current_state = TURN180;
        }
        if (color == 0) {
          distance = measureDistance();
          Serial.print(" Distance 2: ");
          Serial.println(distance);
          if (distance <= 2) {
            Serial.println("stop");
            current_state = STOP;
          }
        }
      }
      if (distance >= 250) {
        current_state = STOP;
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
  stop();

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
  delay(900);
  Serial.println("left-end");
  stop();
  delay(200);
}

void turn_180() {

  Serial.println("180_function");
  motors.enableDrivers();
  motors.setM1Speed(100);
  motors.setM2Speed(-100);
  delay(1900);
  stop();

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
