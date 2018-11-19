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
int servo_mapping[N_SERVOS]={6,5,4,3,2,1};

 //translate from python servo numbering to hardware
//all of these arrays are indexed as TRUE SERVO NUMBER (wire label) - 1
int servo_min[N_SERVOS]={165,165,135,185,155,160};
int servo_max[N_SERVOS]={575,600,505,735,685,595};
int servo_zero[N_SERVOS]={340,362,410,325,330,372}; 
//int servo_zero[N_SERVOS]={369,370,430,321,350,376}; 
//horizontal
int servo_direction[N_SERVOS]={0,1,0,1,0,1}; //0 for unit CCW from outside base, 1 for CW from outside base
float servo_gain[N_SERVOS]={2.00,-2.32,1.84,-2.93,2.83,-2.43};

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
        delay(1);
      }
      Serial.println(millis());
      for(int i=0;i<N_SERVOS+1;i++)
        {
          for (i=0; i<N_SERVOS; i++) {
            Serial.println(i+1);
            Serial.println(commands[j][servo_mapping[i]]);
            pwm.setPWM(i+1, 0, commands[j][servo_mapping[i]]); // added +1 to match PWM port numbering (pints 1..6 used)
          }
        }
    
  }
  Serial.println("dn");
}

void convert_angle_to_PWM(){
  for(int j=0;j<n_commands;j++){
    for(int i=0;i<N_SERVOS;i++)
    {
      int clocking=servo_direction[i];
      int angle=(float)commands[j][i+1]/10.0;
      if(clocking==1){
        angle=180-angle;
      }
      else if (clocking==0){
        if(angle>180){
          angle=angle-360;
        }
      }
      int pwm=(angle*servo_gain[i])+servo_zero[i];
      
      commands[j][i+1]=pwm;
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

  for (int i=0; i<N_SERVOS; i++) {
    Serial.println(servo_mapping[i]);
    Serial.println(servo_zero[i]);
    pwm.setPWM(servo_mapping[i], 0, servo_zero[i]); // added +1 to match PWM port numbering (pints 1..6 used)
  }
          
}
void loop()
{
  while(!Serial.available())                                 
      {delay(poll_rate_ms);}
  data=Serial.readStringUntil('\n');
    
  data.trim();
  if(data == "start")                                                 
    {
    Serial.println(F("ok"));
    n_commands=read_encoded_commands();
    convert_angle_to_PWM();
    print_commands(n_commands);
    
    Serial.println(F("Received Path"));
    Serial.println(F("any character to execute:"));
    while(!Serial.available())                                 
      {delay(poll_rate_ms);}
    
    exec_commands(n_commands);
    data="";             
    }
  if(data=="repeat"){
    exec_commands(n_commands);
  }
    else if(data!=""){
      Serial.println("ERROR: COMMAND UNKNOWN");
      Serial.println(data);
      data="";
    }

}
