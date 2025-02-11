#include <Pixy2.h>

Pixy2 pixy;

void setup() {
  Serial.begin(115200);
  pixy.init();

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
        
        /*
        // Example decision-making based on color and distance
        if (color == 1) {
            currentState = GO_FORWARD;
        } else if (color == 2) {
            currentState = TURN_LEFT;
        } else if (color == 3) {
            currentState = TURN_RIGHT;
        } else if (color == 4) {
            currentState = GO_BACKWARD;
        } else if (color == 5) {
            currentState = TURN_AROUND;
        }
    }

    executeState();
    */
    }
}
