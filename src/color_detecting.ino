#include <Pixy2.h>
#include <DualMAX14870MotorShield.h>

enum FSM {
  RIGHT, // right
  LEFT, // left
  TURN180, // 180
  FORWARD,
  BACKWARD,
  STOP,
};


Pixy2 pixy;
DualMAX14870MotorShield motors;
float dist_threshold = 15; // cm


FSM current_state;

void setup() {
  Serial.begin(115200);
  pixy.init();
  current_state = FORWARD;
}

void loop() {
    pixy.ccc.getBlocks();
    
    if (pixy.ccc.numBlocks) {
        uint16_t color = pixy.ccc.blocks[0].m_signature;
        uint16_t x = pixy.ccc.blocks[0].m_x;
        uint16_t y = pixy.ccc.blocks[0].m_y;
        uint16_t width = pixy.ccc.blocks[0].m_width;

        Serial.print("Color: "); Serial.print(color);
        Serial.print(" X: "); Serial.print(x);
        Serial.print(" Y: "); Serial.print(y);
        Serial.print(" Width: "); Serial.println(width);
    }

    switch (current_state){
    case RIGHT:
      turn_right();
      current_state = FORWARD;
      break;
    case LEFT:
      turn_left();
      current_state = FORWARD;
      break;
    case FORWARD:
      // check if distance is within threshold
      // check color and change state

      break;
    case BACKWARD:
      break;
    case STOP:
      break;
  }
   
}

void turn_right() {
  motors.enableDrivers();
  motors.setM1Speed(200);
  motors.setM2Speed(0);
  delay(2000);
  stop();
}

void turn_left() {
  motors.enableDrivers();
  motors.setM1Speed(0);
  motors.setM2Speed(200);
  delay(2000);
  stop();
}

void turn_180() {
  motors.enableDrivers();
  motors.setM1Speed(200);
  motors.setM2Speed(-200);
  delay(2000);
  stop();
}

void go_forward() {
  motors.enableDrivers();
  motors.setM1Speed(200);
  motors.setM2Speed(200);
}

void go_backward() {
  motors.enableDrivers();
  motors.setM1Speed(-200);
  motors.setM2Speed(-200);
}

void stop(){
  motors.setM1Speed(0);
  motors.setM2Speed(0);
  motors.disableDrivers();
}
