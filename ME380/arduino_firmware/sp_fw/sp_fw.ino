#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//path storage
const int N_COMMANDS_MAX=300;
const int DATA_PRECISION=4;
const int TIME_PRECISION=6;
const int N_SERVOS=6;
const int PER_COMMAND_LENGTH=(N_SERVOS*DATA_PRECISION)+TIME_PRECISION;
int commands[N_COMMANDS_MAX][N_SERVOS+1];
int n_commands=0;


//servo calibration data
int servo_mapping[N_SERVOS]={6,5,4,3,2,1}; //translate from python servo numbering to hardware
//all of these arrays are indexed as TRUE SERVO NUMBER (wire label) - 1
int servo_min[N_SERVOS]={165,165,135,185,155,160};
int servo_max[N_SERVOS]={575,600,505,735,685,595};
int servo_zero[N_SERVOS]={300,300,300,300,300,300}; //horizontal
int servo_range[N_SERVOS]={180,180,180,180,180,180};
int servo_direction[N_SERVOS]={1,0,1,0,1,0}; //0 for unit CCW from outside base, 1 for CW from outside base
float servo_gain[N_SERVOS];

//servo positions
int servo_positions[N_SERVOS];

int poll_rate_ms=50;

// holds serial data for various program functions
String data;

void print_commands(int n_commands){
  for(int j=0;j<n_commands;j++){
    for(int i=0;i<N_SERVOS+1;i++)
    {
      Serial.print(commands[j][i]);
      Serial.print(",");
    }
    Serial.println();
  }
}

void exec_commands(int n_commands){
    int start_time=millis();
    for(int j=0;j<n_commands;j++){
      while(millis()-start_time <commands[j][0]){
        delay(5);
      }
      Serial.println(millis());
      for(int i=0;i<N_SERVOS+1;i++)
        {
          for (i=0; i<N_SERVOS; i++) {
            pwm.setPWM(servo_mapping[i], 0, commands[j][i]); // added +1 to match PWM port numbering (pints 1..6 used)
          }
        }
    
  }
  Serial.println("dn");
}

void convert_angle_to_PWM(){
  for(int j=0;j<n_commands;j++){
    for(int i=1;i<N_SERVOS+1;i++)
    {
      int real_servo_num=servo_mapping[i-1];
      int clocking=servo_direction[real_servo_num-1];
      int angle=(float)commands[j][i]/10.0;
      if(clocking==1){
        angle=180-angle;
      }
      else if (clocking==0){
        if(angle>180){
          angle=angle-360;
        }
      }
      int pwm=(angle*servo_gain[real_servo_num-1])+servo_zero[real_servo_num-1];
      commands[j][i]=pwm;
    }
  }
}

int read_encoded_commands(){
  int N_COMMANDS;
  int current_command=0;
  String time_data;
  String manip_data;
  
  while(true){
    while(!Serial.available())                                 
      {delay(poll_rate_ms);}//poll until data
    data=Serial.readStringUntil('\n');
    data.trim();
    if(data=="end"){
      break;
    }
    if(data.length() != (PER_COMMAND_LENGTH)){
      Serial.println(F("ERROR: data length received does not match servo configuration and angle precision combination"));
    }
    N_COMMANDS=(data.length()-TIME_PRECISION)/DATA_PRECISION;
    Serial.println("commands expected:");
    Serial.println(N_COMMANDS);
    time_data=data.substring(0,TIME_PRECISION);
    commands[current_command][0]=time_data.toInt();
    for(int i=0;i<N_COMMANDS;i++){
      manip_data=data.substring(i*(DATA_PRECISION)+TIME_PRECISION,((1+i)*DATA_PRECISION)+TIME_PRECISION);
      manip_data.trim();
      Serial.println("Substring:");
      Serial.println(manip_data);
      commands[current_command][i+1]=manip_data.toInt();
      
    }
    current_command+=1;
    Serial.println("ok");
  }
  data="";
  return current_command;
}
            
void setup()
{
  for(int i=0;i<N_SERVOS;i++){ //set servo positions to home positions
    servo_positions[i]=servo_zero[i];
  }
  //connect to things
  Serial.begin(115200); 
  pwm.begin();
  pwm.setPWMFreq(60);
  //calculate servo gains
  for(int i=0;i<N_SERVOS;i++){
    servo_gain[i]=(float)(servo_max[i]-servo_min[i])/(float)servo_range[i];
  }
          
}
void loop()
{

  data=Serial.readStringUntil('\n');
    
  data.trim();
  if(data == "start")                                                 
    {
    Serial.println(F("ok"));
    n_commands=read_encoded_commands();
    convert_angle_to_PWM();
    Serial.println(F("Received Path"));
    Serial.println(F("any character to execute:"));
    while(!Serial.available())                                 
      {delay(poll_rate_ms);}
    
    exec_commands(n_commands);
    data="";             
    }
    else if(data!=""){
      Serial.println("ERROR: COMMAND UNKNOWN");
      Serial.println(data);
      data="";
    }

}