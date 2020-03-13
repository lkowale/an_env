#ifndef DC_motor_h
#define DC_motor_h

#include "Arduino.h"

#define MOVE_LEFT 1
#define MOVE_RIGHT 0
#define SETTER_PWM 50
#define PWM 80
#define DEBOUNCE_TIME 50

class DC_motor {

  public:    



    DC_motor(uint8_t,uint8_t,uint8_t); 
      
    void moveToStart();
    void moveTo(bool,unsigned int);
    void moveTo(unsigned int);
    void moveUpdate();
    void reset();
    void readStartPosition();

  protected:
    unsigned long move_start;  
    unsigned long move_time;  
    unsigned long debounce_start_time;
    unsigned int dest_position,current_position;
    uint8_t pin_left,pin_right;
    uint8_t pin_on_start;
    bool on_start_pos,on_move,move_direction;    
};



#endif
