    #include "DC_motor.h"
    
    DC_motor::DC_motor(uint8_t pin_left_,uint8_t pin_right_,uint8_t pin_on_start_){
      pin_left=pin_left_;
      pin_right=pin_right_;   
      pin_on_start=pin_on_start_;
      on_move=false;
      current_position = 2000;
      // set on zero position
      reset();
    }
    
    void DC_motor::moveTo(bool move_direction_,unsigned int move_time_){
        on_move=true;
        move_start=millis();
        move_time=move_time_;
        move_direction=move_direction_;
//        String info="In moveTo move_direction:"+String(move_direction)+" move_time:"+String(move_time);
//        Serial.println(info);
        if(move_direction){
          //move left
//                  String info="In moveLeft pin_left:"+String(pin_left)+" pin_right:"+String(pin_right)+" PWM:"+String(PWM)+" current_position:"+String(current_position)+" dest_position:"+String(dest_position);
//                  Serial.println(info);
            analogWrite(pin_left, PWM);
            analogWrite(pin_right, 0);  
            on_start_pos=false;         
        }
        else
        {
          //move right
//                  String info="In moveRight pin_left:"+String(pin_left)+" pin_right:"+String(pin_right)+" PWM:"+String(PWM)+" current_position:"+String(current_position)+" dest_position:"+String(dest_position);
//                  Serial.println(info);          
             if(!on_start_pos){
              analogWrite(pin_right, PWM);
              analogWrite(pin_left, 0);
             }
        }
    }
    
       void DC_motor::moveTo(unsigned int dest_position_){
        dest_position = dest_position_;
        int difference = dest_position - current_position;
//            String info="In DC_motor::moveTo difference:"+String(difference)+" dest_position:"+String(dest_position)+" current_posiiton:"+String(current_position);
//            Serial.println(info);  
        if(difference > 0)
//        move left
          moveTo(1,difference);
        else if(difference < 0)
//        move right
          moveTo(0,abs(difference));     

        
    } 
    
    void DC_motor::moveToStart(){

    }

    void DC_motor::moveUpdate(){
//      if(on_move){
//        int diff=  millis()-move_start;
//        //move_direction==1 -> left ____ move_direction==0 -> right
//        if(!move_direction||diff>DEBOUNCE_TIME)
//          //read start position after debouce time passed - pin_on_start is PULLUP hence negation
//          on_start_pos=!digitalRead(pin_on_start);      
////          String info="In update millis()-move_start:"+String(diff)+" move_time:"+String(move_time)+" on_start_pos:"+String(on_start_pos)+" current_position:"+String(current_position)+" dest_position:"+String(dest_position);
////          Serial.println(info);      
////      //move lasts long enough - end it
//        if(diff>move_time){
//            analogWrite(pin_left, 0);
//            analogWrite(pin_right, 0);  
//            String info="In update move ends time:"+String(diff)+" move_time:"+String(move_time)+" on_start_pos:"+String(on_start_pos)+" current_position:"+String(current_position)+" dest_position:"+String(dest_position);
//            Serial.println(info);             
//            on_move=false;
//            current_position = dest_position;
//        }
//        //torso reached start position
//        if(on_start_pos){
//            analogWrite(pin_left, 0);
//            analogWrite(pin_right, 0);  
////            String info="In update move ends pos on start:"+String(diff)+" move_time:"+String(move_time)+" on_start_pos:"+String(on_start_pos);
////            Serial.println(info);             
//
////            /move to place righ after zero position
//            while(!digitalRead(pin_on_start))
//            {
//            analogWrite(pin_left, SETTER_PWM);
//            analogWrite(pin_right, 0);  
//            }
//            analogWrite(pin_left, 0);
//            analogWrite(pin_right, 0);
//            on_move=false;
//            current_position = 0;
//        }
//      }

      if(on_move)
      {

        /**********************************************************************/
        int diff=  millis()-move_start;
        // if time ends stop whetever it moves left or right and destination isn't zero position
        if(diff>move_time)
          if(dest_position!=0){
            analogWrite(pin_left, 0);
            analogWrite(pin_right, 0);  
//            String info="In update move ends time:"+String(diff)+" move_time:"+String(move_time)+" on_start_pos:"+String(on_start_pos)+" current_position:"+String(current_position)+" dest_position:"+String(dest_position);
//            Serial.println(info);             
            on_move=false;
            current_position = dest_position;
          }
          else
          {
            //keep moving right
              analogWrite(pin_right, PWM);
              analogWrite(pin_left, 0);
          }
        /**********************************************************************/
        // if reached start position must stop either moving left or right
        on_start_pos=!digitalRead(pin_on_start);
        if(on_start_pos){
          // if moving right
          if(!move_direction){
            //read start position after debouce time passed - pin_on_start is PULLUP hence negation                
            //torso reached start position
            if(on_start_pos){
                analogWrite(pin_left, 0);
                analogWrite(pin_right, 0);  
    //            String info="In update move ends pos on start:"+String(diff)+" move_time:"+String(move_time)+" on_start_pos:"+String(on_start_pos);
    //            Serial.println(info);             
    
                on_move=false;
                current_position = 0;
            }                 
          }
          // if reached start position moving left after debounce time then somethings gone terribly wrong
          else if(diff > DEBOUNCE_TIME)
          {
                analogWrite(pin_left, 0);
                analogWrite(pin_right, 0);  
                String info="EMERG STOP:"+String(diff)+" move_time:"+String(move_time)+" on_start_pos:"+String(on_start_pos);
                Serial.println(info);                   
          }
        }

      }
    }
    
    void DC_motor::reset(){

    }

