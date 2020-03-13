#ifndef ArmServo_h
#define ArmServo_h

#include "Arduino.h"
#include <Servo.h>

#define MOVE_WINDOW 40
#define STEP 10
#define BREAKING_STEP 1
#define BREAKING_OFFSET 11

class ArmServo {

  public:    

    ArmServo(uint8_t,uint8_t); 
    void init();
    void moveTo(uint8_t);
    void update();
      
  protected:
  
    uint8_t pin;
    uint8_t start_pos;
    uint8_t act_pos;    
    uint8_t final_pos;
    uint8_t move_dir;
    bool in_move;
    Servo serwo;
    unsigned long startTime;
};



#endif
