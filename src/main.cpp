#include <Arduino.h>
#include <SPI.h> 
#include "nRF24L01.h"
#include "RF24.h" 
#include <ESP32Servo.h>
#include "MPU9250.h"
#include <Wire.h>
#include<iostream>
#include<string>

// I2C device found at address 0x68
// I2C device found at address 0x76
/******************DEBUG****************/
#define DEBUG // for adding and removing debug code 
#define calib
//#define test//with local variables
//#define testy//with local variables
//#define webtest
//#define webtestyaw
//#define bmp_debug
//#define DEBUG_MPU
//#define DEBUG_GPS
//#define PID
//#define DEBUG_NRF
//#define DEBUG_MOTORS_SPEED
//#define DEBUG_PID_VALUES
//#define fixi
//#define fixir
//#define fixiy
//#define fixi_motors

/************data**********/
float data[5];



/****************radio mid points*****************/
//Yaw:1447 Roll:1443 Pitch :1438

/*****************nrf******************/
#define CE 12
#define CSN 14
#define SCK 26
#define MOSI 27
#define MISO 25


/****************LEDS***************/
#define Bled 4
#define Gled 15
#define Rled 2


/*************Defines_for_motor***********/
#define ESC_0 23 
#define ESC_1 19
#define ESC_2 5
#define ESC_3 18
// minimum and maximum PWM pulse width
#define MIN_THROTTLE 1000  
#define MAX_THROTTLE 1800 
//servo objects
Servo MOTOR_0;
Servo MOTOR_1;
Servo MOTOR_2;
Servo MOTOR_3;

/**********Defines_for_mpu***************/
MPU9250 mpu;
int16_t rollAvr = 0;
int16_t pitchAvr = 0;
int16_t yawAvr = 0;
int16_t Roll=0, Pitch=0, Yaw=0;
bool calibrate = true;


/***************PID coef************/
double PID_PITCH_kp =0.94;//2;
double PID_PITCH_kd =0.01;//0.3;
double PID_PITCH_ki =0;//0.002;

double PID_ROLL_kp =PID_PITCH_kp;
double PID_ROLL_kd =PID_PITCH_kd;
double PID_ROLL_ki =PID_PITCH_ki;

double PID_YAW_kp =1;//0.3;
double PID_YAW_kd =0;//0.02;
double PID_YAW_ki =0.001;

float previous_pitch_error = .0;
float previous_roll_error = .0;
float previous_yaw_error = .0;
/************functions declaration***************/
void inline MPU_Angles_Avr(int16_t &rollavr, int16_t &pitchavr, int16_t &yawavr);
void inline Measured_Roll_Pitch_Yaw(int16_t & Roll, int16_t & Pitch, int16_t & Yaw);
void update_data(double lng,double lat,double speed,double temp,double pressure);
void Readytogo();
void accCalib();
void magnCalib();
int16_t inline ExecutePitchPID(const int16_t& pitch_set_point, const int16_t& measured_pitch,const float &dt);
int16_t inline ExecuteRollPID(const int16_t& roll_set_point, const int16_t& measured_roll,const float &dt);
int16_t inline ExecuteYawPID(const int16_t& yaw_set_point, const int16_t& measured_yaw,const float &dt);
void inline UpdateMotorsValues( const int16_t throttle, const int16_t pitch_pid_output, const int16_t roll_pid_output, const int16_t yaw_pid_output);
void InfoGPS();
void Task1code( void *parameter);
void bmptask( void *parameter);
void gpstask( void *parameter);
void sendingtask( void *parameter);
float  parse_and_handle_message();
void handle_ble_serial();
/****************nrf***************/
//nrf oject win pins
RF24 receiver(12, 14, 26, 25, 27);
//RF24 (CE, CSN, SCK, MISO, MOSI); 

const uint64_t p= 0x01111111;//IMPORTANT: The same as in the transmitter
const uint64_t r= 0x00011001;
int16_t values_received[4];



//time 
long current_time=0,last_time=0;

//bluetooth
String message = "";
int end_message_signal = 10;
bool message_recieved = false;
bool poweroff=false;

void setup(void){
//set pins to 0
MOTOR_0.writeMicroseconds(MIN_THROTTLE);
MOTOR_1.writeMicroseconds(MIN_THROTTLE);
MOTOR_2.writeMicroseconds(MIN_THROTTLE);
MOTOR_3.writeMicroseconds(MIN_THROTTLE);


#ifdef DEBUG
Serial.begin(9600);
#endif
//LEDS setting
pinMode(Bled,OUTPUT);
pinMode(Gled,OUTPUT);
pinMode(Rled,OUTPUT);


// setting-up the motors //
MOTOR_0.attach(ESC_0, MIN_THROTTLE, MAX_THROTTLE);
MOTOR_1.attach(ESC_1, MIN_THROTTLE, MAX_THROTTLE);
MOTOR_2.attach(ESC_2, MIN_THROTTLE, MAX_THROTTLE);
MOTOR_3.attach(ESC_3, MIN_THROTTLE, MAX_THROTTLE);


// setting-up nRF module //


 receiver.begin();// Begin operation of the chip.
 receiver.setChannel(30);// Which RF channel to communicate on, 0-127
 receiver.setPayloadSize(32);//size of data trame 
 receiver.setDataRate(RF24_250KBPS);
 receiver.openWritingPipe(r);
 receiver.openReadingPipe(1,p);
 receiver.startListening();


//setting MPU module
  MPU9250Setting setting;
  setting.accel_fs_sel        = ACCEL_FS_SEL::A4G;
  setting.gyro_fs_sel         = GYRO_FS_SEL::G500DPS;
  setting.mag_output_bits     = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate    = FIFO_SAMPLE_RATE::SMPL_500HZ;
  setting.gyro_fchoice        = 0x03;
  setting.gyro_dlpf_cfg       = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice       = 0x01;
  setting.accel_dlpf_cfg      = ACCEL_DLPF_CFG::DLPF_45HZ;
  mpu.setMagneticDeclination(3.11f); //tunisia 26 mars


Wire.begin();
if (!mpu.setup(0x68))
{
#ifdef DEBUG_MPU
Serial.println("Failed to initialize MPU9250.");
#endif

  while (1)
  {
  }
  
}
Serial.println(" initialize MPU9250.");

  /* bluetooth and clear serial */
  while (Serial.available()) /* If data is available on serial port */
  { 
    Serial.read();
  }


#ifdef calib
    accCalib();
mpu.calibrateAccelGyro();
    digitalWrite(Bled,0);
     magnCalib();
 mpu.calibrateMag();
    digitalWrite(Gled,0);
    Readytogo();
#endif
 

}
void loop(){
current_time=millis();
Measured_Roll_Pitch_Yaw(Roll, Pitch, Yaw);
 handle_ble_serial();
if(poweroff){
MOTOR_0.writeMicroseconds(MIN_THROTTLE);
MOTOR_1.writeMicroseconds(MIN_THROTTLE);
MOTOR_2.writeMicroseconds(MIN_THROTTLE);
MOTOR_3.writeMicroseconds(MIN_THROTTLE);
}

else if(receiver.available()){
receiver.read(values_received, sizeof(values_received));
digitalWrite(Bled,1);
  digitalWrite(Rled,1);
delayMicroseconds(4000);
  #ifdef DEBUG_NRF
      Serial.print("throtlle:");
  Serial.print(values_received[0]);
   Serial.print("   Yaw:");
    Serial.print(values_received[1]);
   Serial.print("   Roll:");
    Serial.print(values_received[2]);
   Serial.print("   Pitch:");
    Serial.print(values_received[3]);
   Serial.println("   ");

   #endif 

   //UpdateMotorsValues(values_received[0],PitchPIDOutput,0,0);
   #ifdef fixi_motors
MOTOR_0.writeMicroseconds(values_received[0]);
MOTOR_1.writeMicroseconds(values_received[0]);
 MOTOR_2.writeMicroseconds(values_received[0]);
 MOTOR_3.writeMicroseconds(values_received[0]);
  #endif
if(values_received[0]>1080){
  current_time=micros();
  float dt=(current_time-last_time)/1000000;
int16_t RollPIDOutput   = ExecuteRollPID(values_received[2], Roll,dt);
int16_t PitchPIDOutput  = ExecutePitchPID(values_received[3], Pitch,dt);
int16_t YawPIDOutput  = ExecuteYawPID(values_received[1], Yaw,dt);
#ifdef PID
    Serial.print("RollPIDOutput:");
  Serial.print(RollPIDOutput);
   Serial.print("   PitchPIDOutput:");
    Serial.print(PitchPIDOutput);
   Serial.print("   YawPIDOutput :");
    Serial.println(YawPIDOutput );
#endif
UpdateMotorsValues(values_received[0],RollPIDOutput,PitchPIDOutput ,YawPIDOutput);

}
else{
MOTOR_0.writeMicroseconds(MIN_THROTTLE);
MOTOR_1.writeMicroseconds(MIN_THROTTLE);
MOTOR_2.writeMicroseconds(MIN_THROTTLE);
MOTOR_3.writeMicroseconds(MIN_THROTTLE);
}
}

if(!receiver.available()){
  digitalWrite(Bled,0);
  digitalWrite(Rled,1);
  #ifdef DEBUG_NRF
    Serial.println(" No signal !  ");
  #endif 
MOTOR_0.writeMicroseconds(MIN_THROTTLE);
MOTOR_1.writeMicroseconds(MIN_THROTTLE);
MOTOR_2.writeMicroseconds(MIN_THROTTLE);
MOTOR_3.writeMicroseconds(MIN_THROTTLE);
}






last_time=current_time;
//delay(10);


}






void inline MPU_Angles_Avr(int16_t & rollavr, int16_t & pitchavr, int16_t & yawavr){
#ifdef DEBUG_MPU
Serial.print("Calibration start");
#endif
for (int i = 0; i < 2000; i++)
{
mpu.update();
float x_angle = mpu.getRoll();
float y_angle = mpu.getPitch();
float z_angle = mpu.getYaw();
rollavr += (x_angle/2000);
pitchavr += (y_angle/2000);
yawavr += (z_angle/2000);
delay(1);
}
#ifdef DEBUG_MPU
Serial.println("Calibration done");
#endif
}
//this function measures the roll, pitch and yaw
void inline Measured_Roll_Pitch_Yaw(int16_t & Roll, int16_t & Pitch, int16_t & Yaw){
if (mpu.update()) {
Roll = mpu.getRoll() - rollAvr -2;
Pitch = mpu.getPitch() - pitchAvr +2;
Yaw = mpu.getYaw() - yawAvr;
#ifdef DEBUG_MPU
Serial.print("Roll: ");
Serial.print(Roll);
Serial.print(" Pitch: ");
Serial.print(Pitch);
Serial.print(" Yaw: ");
Serial.println(Yaw);
#endif
}

}


int16_t inline ExecutePitchPID(const int16_t& pitch_set_point, const int16_t& measured_pitch,const float&dt) {
long error;
float integral = .0;
float proportional;
float derivative;
long psp=map(pitch_set_point,1000,2000,-36,46);//46
psp=max((long)-50,min(psp,(long)50));
if(psp<=3&&psp>=-3)psp=0;
error = (psp- measured_pitch);
// if(error<=3 && error>=-3)
// error = 0;

proportional = PID_PITCH_kp * error;
derivative =PID_PITCH_kd* (error - (previous_roll_error));
integral += error*PID_PITCH_ki;
previous_roll_error = error;

#ifdef webtest
proportional = Kp * error;
derivative =Kd* (error - (previous_roll_error))/dt;
integral += Ki*error*dt;
previous_roll_error = error;
#endif
#ifdef fixi
// Serial.print("kp:");
//     Serial.print(Kp);
//     Serial.print(" kd:");
//     Serial.print(Kd);
//     Serial.print(" ki:");
//     Serial.print(Ki);
Serial.print(" correction:");
Serial.print( proportional +  integral + derivative  );
Serial.print(" previous_pitch_error:");
Serial.print(previous_pitch_error );
Serial.print(" mesured pitch:");
Serial.print(measured_pitch);
Serial.print(" pitch:");
Serial.println(psp);
#endif
int16_t a=proportional +  integral + derivative;
a=constrain(a,-400,400);
return a;
}

int16_t inline ExecuteRollPID(const int16_t& roll_set_point, const int16_t& measured_roll,const float&dt) {
long error;
float integral = .0;
float proportional;
float derivative;
long rsp=map(roll_set_point,1000,2000,-32,46);//48
rsp=max((long)-50,min(rsp,(long)50));
if(rsp<=3&&rsp>=-3)rsp=0;

error = (rsp- measured_roll);

// if(error<=3 && error>=-3)
// error = 0;

proportional = PID_ROLL_kp * error;
derivative =PID_ROLL_kd*(error - (previous_roll_error));
integral += error* PID_ROLL_ki;
previous_roll_error = error;

#ifdef webtest
proportional = Kp * error;
derivative =Kd* (error - (previous_roll_error))/dt;
integral += Ki*error*dt;
previous_roll_error = error;
#endif
#ifdef fixir
Serial.print(" error:");
Serial.print(proportional + integral + derivative);
Serial.print(" mesured roll:");
Serial.print(measured_roll);
Serial.print(" rsp:");
Serial.println(rsp);
#endif
int16_t a=proportional +  integral + derivative;
a=constrain(a,-400,400);
return a;
}
int16_t inline ExecuteYawPID(const int16_t& yaw_set_point, const int16_t& measured_yaw,const float&dt) {
  long error=0;
  static float integral = .0;
  float proportional;
  float derivative;
  long ysp=map(yaw_set_point,1000,2000,-52,68);
ysp=max((long)-50,min(ysp,(long)50));
if(ysp<=3&&ysp>=-3)ysp=0;

error = (ysp - measured_yaw);

//if(error<=3 && error>=-3)error = 0;


  proportional = PID_YAW_kp * error;
  derivative = PID_YAW_kd * (error - (previous_yaw_error));
  integral +=  PID_YAW_ki *error;

   #ifdef webtestyaw
  proportional = Kp * error;
  derivative = Kd* (error - (previous_yaw_error))/dt;
  integral +=  Ki*error*dt;
 #endif  
  previous_yaw_error = error;
#ifdef fixiy
Serial.print(" error:");
Serial.print(proportional + integral + derivative);
Serial.print(" mesured yaw:");
Serial.print(measured_yaw);
Serial.print(" ysp:");
Serial.println(ysp);

#endif
long long a=proportional + integral + derivative;
a=constrain(a,-400,400);
  return (a);
}
void accCalib(){
  for(int i=0;i<6;i++){
  if(i%2){
    digitalWrite(Bled,1);
    delay(300);
  }
  else{
    digitalWrite(Bled,0);
    delay(300);
      }

}
}
void magnCalib(){
  for(int i=0;i<6;i++){
  if(i%2){
    digitalWrite(Gled,1);
    delay(300);
  }
else{
    digitalWrite(Gled,0);
    delay(300);
}
}
}
void Readytogo(){
   
  digitalWrite(Gled,1);
  digitalWrite(Bled,1);
  MPU_Angles_Avr(rollAvr, pitchAvr, yawAvr);
  digitalWrite(Gled,0);
  digitalWrite(Bled,0);
  Serial.println("ready to go");
}
void inline UpdateMotorsValues( const int16_t throttle, const int16_t pitch_pid_output,
const int16_t roll_pid_output, const int16_t yaw_pid_output) {
long m2 = throttle + pitch_pid_output + roll_pid_output - yaw_pid_output;
long m1 = throttle + pitch_pid_output - roll_pid_output + yaw_pid_output;
long m0 = throttle - pitch_pid_output - roll_pid_output - yaw_pid_output;
long m3 = throttle - pitch_pid_output + roll_pid_output + yaw_pid_output;



m0=max((long)MIN_THROTTLE,min((long)MAX_THROTTLE,m0));
m1=max((long)MIN_THROTTLE,min((long)MAX_THROTTLE,m1));
m2=max((long)MIN_THROTTLE,min((long)MAX_THROTTLE,m2));
m3=max((long)MIN_THROTTLE,min((long)MAX_THROTTLE,m3));
#ifdef DEBUG_MOTORS_SPEED
Serial.print(" m0 : ");
Serial.print(m0);
Serial.print(" m1 : ");
Serial.print(m1);
Serial.print(" m2 : ");
Serial.print(m2);
Serial.print(" m3 : ");
Serial.print(m3);
Serial.print("\n");
#endif
 MOTOR_0.writeMicroseconds(m0);
MOTOR_1.writeMicroseconds(m1);
 MOTOR_2.writeMicroseconds(m2);
MOTOR_3.writeMicroseconds(m3);
}

void handle_ble_serial(){
  while (Serial.available()) /* If data is available on serial port */
  { 
    char c = Serial.read();
    message += c ;

    if (c == end_message_signal){
      message_recieved = 1;
      break;
    }
  }
  if (message_recieved){
    // handle message here !!!

    float x = parse_and_handle_message();
    if (x){
    String debug_message = "changing k"+ String(message[0]) + " to "+String(x)+">>\n";
    Serial.print(debug_message);
    }
    else {
    Serial.print(message + "some error occured \n");
    }
    // clear message
    message_recieved = 0;
    message = ""; 

  }
  
}
float  parse_and_handle_message(){
    
    if (message[0] == 'p' || message[0] == 'd' || message[0] == 'e' || message[0] == 'a'|| message[0] == 'o'   ) {
      
      float x = atof(message.substring(1).c_str());
      char type = message[0];
      
      if (type=='p'){
      
       PID_YAW_kp=x;
        Serial.println(PID_YAW_kp,5);//remove me
      }else if(type=='d'){

        PID_YAW_kd=x;
        Serial.println(PID_YAW_kd,5);//remove me
      }
      else if(type=='e'){

        PID_YAW_ki=x;
        Serial.println(PID_YAW_ki,5);//remove me
      }
      else if (type=='a'){
        //change Power
      }else if (type=='o'){
        //change on/off 
        // on x==2
        if(x==1){
          PID_YAW_kp=1.3;
          PID_YAW_kd=0;
          PID_YAW_ki=0;
        }
        else if(x==2){
          poweroff=!poweroff;
        }
        else if (x==7){
          Serial.println(PID_YAW_kp,8);//remove me
          Serial.println(PID_YAW_kd,8);//remove me
          Serial.println(PID_YAW_ki,8);//remove m
        }
      }
    return x;
    }
    
  
  return 0;

}
