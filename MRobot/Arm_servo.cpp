    #include "Arm_servo.h"
    
    ArmServo::ArmServo(uint8_t pin_,uint8_t start_pos_){
      pin=pin_;
      start_pos=start_pos_;
      in_move=false;
    }
    void ArmServo::init(){
  //                Serial.println("ArmServo init");
      serwo.attach(pin); 
      serwo.write(start_pos); 
      act_pos=start_pos;      
    }
    
    void ArmServo::moveTo(uint8_t final_pos_){
      final_pos = final_pos_;
      in_move=true;
       //     Serial.println("ArmServo moveto");
      //calculate move direction
      int offset = final_pos-act_pos;
      if (offset < 0) move_dir = -1;
      if (offset > 0) move_dir = 1;
      if (offset == 0) in_move=false;           
      startTime=millis();   
    }
    
    void ArmServo::update(){
   // Serial.println("ArmServo update");
    if(in_move&&startTime+MOVE_WINDOW<millis())
      {
      int offset = abs(final_pos-act_pos);
      if(offset){
        int step;  
        if(offset>BREAKING_OFFSET) step=STEP;
          else step=BREAKING_STEP;
        act_pos+=(step*move_dir);
       // String info="update || in_move:"+String(in_move)+" startTime:"+String(startTime) +" offset:"+String(offset)+" step:"+String(step)+" act_pos:"+String(act_pos);
       // Serial.println(info);        
        serwo.write(act_pos);         
      }
      else
        in_move=false;
        
      startTime=millis();
      }
    }


