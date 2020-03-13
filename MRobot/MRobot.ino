//To be used with !3dMediumArm

#define RED_PIN A5
#define JOINT_1 7
#define JOINT_2 8
#define JOINT_3R 6
#define JOINT_3L 5
#define JOINT4 9
#define START_POS A4
#define WINDOW 1000
#define LED_WINDOW 100
#define CLOSE 0
#define OPEN 10

//#define J1MIN 0
//#define J1MAX 90
//#define J2MIN 0
//#define J2MAX 160

#define GRIPPER_MAX 60
#define ELBOW_MAX 90
#define SHOULDER_MAX 130
#define TORSO_MAX 2000

#define MAX_MESSAGE_LENGTH 128

#include "Arm_servo.h"
#include <MemoryFree.h>
#include "DC_motor.h"

ArmServo gripper=ArmServo(JOINT4,0);
ArmServo shoulder=ArmServo(JOINT_2,0);
ArmServo elbow=ArmServo(JOINT_1,0);
DC_motor torso=DC_motor(JOINT_3L,JOINT_3R,START_POS);

  int shoulder_position,elbow_position,dcm_dir,gripper_position,torso_position;

  

unsigned long led_start_time,start_time,loop_cnt=0;
bool led_led=false;

void setup() {

  pinMode(RED_PIN, OUTPUT);

  Serial.begin(9600);

  gripper.init();
  shoulder.init();
  elbow.init();
  
}


void loop() {


  if(recive_command())
    {
    parse_command();   
    if(params_in_bound())
      startMoving();
    }
    
  torso.moveUpdate(); 
  
  gripper.update();
  shoulder.update();
  elbow.update();    
}

bool params_in_bound()
{

    if(gripper_position>GRIPPER_MAX || elbow_position>ELBOW_MAX || shoulder_position>SHOULDER_MAX || torso_position> TORSO_MAX)
      return false;
    else 
      return true;  
}
void startMoving(){
//  Serial.println("Start moving");
  digitalWrite(RED_PIN, HIGH);    
  gripper.moveTo(gripper_position);
  shoulder.moveTo(shoulder_position);
  elbow.moveTo(elbow_position);
  torso.moveTo(torso_position);
}

char command[MAX_MESSAGE_LENGTH+1];
String commandString;


bool recive_command()
{
  if (Serial.available()) 
  {
   int t=0;
    while(Serial.available()&&t<MAX_MESSAGE_LENGTH) {
      *(((char*)command + t))=Serial.read();
      delay(2);
      t++;
    }
    *(((char*)command + t))=0; //end of char string
      commandString= String(command);  
    return true;
  }
  else
    return false;
}

void parse_command(){

  byte colon_index1=commandString.indexOf(",");
  byte colon_index2=commandString.indexOf(",",colon_index1+1);
  byte colon_index3=commandString.indexOf(",",colon_index2+1);
  byte colon_index4=commandString.indexOf(",",colon_index3+1);
      
  gripper_position=commandString.substring(0,colon_index1).toInt();
  elbow_position=commandString.substring(colon_index1+1,colon_index2).toInt();
  shoulder_position=commandString.substring(colon_index2+1,colon_index3).toInt();
  torso_position=commandString.substring(colon_index3+1,colon_index4).toInt();
  
  Serial.println(commandString);  

//  String info="Parse command colon_index1:"+String(colon_index1)+" colon_index2:"+String(colon_index2)+" colon_index3:"+String(colon_index3);
//  String info="gripper:"+String(gripper_position)+" elbow:"+String(elbow_position)+" shoulder:"+String(shoulder_position)+" torso:"+String(torso_position);
//  Serial.println(info);   

}

void send_ack()
{
  Serial.println("ack");
}
