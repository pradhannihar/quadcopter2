////////////////////IMU PARTS\\\\\\\\\\\\\\\\\\\\\\\\\\\

#include<Wire.h>
#include"IMU.h"
//hellow

#define degconvert 57.2957786 
#define accelw 0.004
#define gyrow 0.996
#define acc_smt 8



int calibration;
double dt;
float roll=0,pitch=0,yaw=0;
float gyro[3],acc[3];
float gyro_cal[3];
uint32_t loop_timer,pid_loop_timer;
bool flag=0;
float dcm[][3] = { 1, 0, 0,
                   0, 1, 0,
                   0, 0, 1 };
double dtheta[3] = { 0, 0, 0};              //roll,pitch,yaw

////////////////IMU PATRS ENDS


/////CONTROL PART STARTS HERE

////############################################\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//unsigned long current_time,throttle,throttle1,last_throttle,timer_1; byte last_channel_1;

////####GLOBAL VARIABLES##########\\\\\\\\\\\\

byte last_channel_1, last_channel_2, last_channel_3, last_channel_4,last_channel_5, last_channel_6;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4,timer_channel_5,timer_channel_6, esc_timer, esc_loop_timer;

unsigned long timer_1, timer_2, timer_3, timer_4,timer_5,timer_6, current_time;

unsigned long throttle,throttle1,rec_input_5,rec_input_6,last_throttle;             ///net throtle only input
double u2[4];                 ///error matrix
int esc_1, esc_2, esc_3, esc_4;      ///outputs to motors
////####COEFFICIENTS#########\\\\\\\

double Kp_phi=0;
double Kd_phi=0;
double Kp_theta=Kp_phi;
double Kd_theta=Kd_phi;
double Kp_shi=0;
double Kd_shi=0;
double pwm2f;

////SET POINTS\\\\

double theta_des = 0;  double thetaDes_dot= 0;
double phi_des =   0;  double phiDes_dot=   0;
double shi_des =   0;  double shiDes_dot=   0;

///SENSOR DATAS\\\

float theta ;  float theta_dot;
float phi   ;  float phi_dot;
float shi   ;  float shi_dot;




/////CONTROLL PART ENDS






void setup()
{
  ////////////////////IMU PARTS\\\\\\\\\\\\\\\\\\\\\\\\\\\

  Serial.begin(115200);
  Wire.begin();

  #if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #endif
  
setupmpu();
for(calibration = 0;calibration<2000;calibration++)
{
 record_data(); 
}
for(int i=0;i<3;i++)
{
gyro_cal[i]/=2000;
  
}
roll = atan2(acc[1],acc[2]);
pitch = atan2(-acc[0],acc[2]);                           //Starting Angle

float dcm[][3] = {cos(pitch) , sin(roll)*sin(pitch), cos(roll)*sin(pitch),
                  0          , cos(roll)           , -sin(roll)          ,
                  -sin(pitch), sin(roll)*cos(pitch), cos(roll)*cos(pitch)};

 

////////////////IMU PATRS ENDS


///////////CONTROLL PARTS BEGIN

  
  DDRD |= B11110000;                                                        //Configure digital poort 4, 5, 6 and 7 as output.
  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.  
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.THROTLE
  PCMSK0 |= (1 << PCINT4);                                                  //FOR ARMING QUADCOPTER
  PCMSK0 |= (1 << PCINT5);

  
   for (int cal_int = 0; cal_int < 1250 ; cal_int ++){                           //Wait 5 seconds before continuing.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);   }                                             //Wait 3000us.



    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.


////////CONTROL PARTS ENDS

loop_timer = micros();

}
////////////IMU function defination start
void setupmpu() {
  
    Wire.beginTransmission(mpu6050_address);                                      //Start communication with the address found during search.
    Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                                    //End the transmission with the gyro.

    Wire.beginTransmission(mpu6050_address);                                      //Start communication with the address found during search.
    Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(gyro_rate_select);                                                          //Set the register bits as 00011000 (2000dps full scale)
    Wire.endTransmission();                                                    //End the transmission with the gyro

                                                                              //End the transmission with the gyro

    Wire.beginTransmission(mpu6050_address);                                      //Start communication with the address found during search.
    Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(acc_rate_select);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();        
 
    Wire.beginTransmission(mpu6050_address);                                      //Start communication with the address found during search
    Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();  
   

}


void record_data(){


    Wire.beginTransmission(mpu6050_address);                                   //Start communication with the gyro.
    Wire.write(0x3B);                                                     //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(mpu6050_address,14);                                      //Request 14 bytes from the gyro.
   
    
    while(Wire.available() < 14);                                           //Wait until the 14 bytes are received.
    acc[0] = ((Wire.read()<<8|Wire.read())>>acc_smt)<<acc_smt;                               //Add the low and high byte to the acc_x variable.
    acc[1] = ((Wire.read()<<8|Wire.read())>>acc_smt)<<acc_smt;                               //Add the low and high byte to the acc_y variable.
    acc[2] = ((Wire.read()<<8|Wire.read())>>acc_smt)<<acc_smt;                               //Add the low and high byte to the acc_z variable.
    Wire.read()<<8|Wire.read();                                        //Add the low and high byte to the temperature variable.
    gyro[0] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro[1] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro[2] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.




  if(calibration == 2000){
    gyro[0] -= gyro_cal[0];                                       //Only compensate after the calibration.
    gyro[1] -= gyro_cal[1];                                       //Only compensate after the calibration.
    gyro[2] -= gyro_cal[2];                                       //Only compensate after the calibration.

      for(int i=0;i<3;i++)
  {
    gyro[i]/=gyrolsb;
    acc[i] /=acclsb;
  }
  
  }
 
  else
  {
    gyro_cal[0] += gyro[0];                                       //Only compensate after the calibration.
    gyro_cal[1] += gyro[1];                                       //Only compensate after the calibration.
    gyro_cal[2] += gyro[1];     

  }
}


void angle_calc()
{
 
  dtheta[0] = gyrow*gyro[0]*dt/degconvert + accelw*(atan2(acc[1],acc[2])-roll);
  dtheta[1] = gyrow*gyro[1]*dt/degconvert + accelw*(atan2(-acc[0],acc[2])-pitch);
  dtheta[2] = gyro[2]*dt/degconvert;


vec_rot();


roll  = atan2(dcm[2][1],dcm[2][2]);
pitch = atan2(-dcm[2][0],sqrt(dcm[2][1]*dcm[2][1]+dcm[2][2]*dcm[2][2]));
yaw   = atan2(dcm[1][0],dcm[0][0]);

  
}

void vec_rot()
{
float temp_dcm[3][3];
float dot = 0;

float euler[][3] = {   1         , -dtheta[2] , dtheta[1],
                       dtheta[2], 1         ,  -dtheta[0],
                       -dtheta[1] , dtheta[0],  1         
                   };



for(int i=0;i<3;i++)
{
  for(int j=0;j<3;j++)
  {
    temp_dcm[i][j] = 0;
    
    for(int k=0;k<3;k++)
    temp_dcm[i][j] += dcm[i][k]*euler[k][j];

   
  }
}

for(int i=0;i<3;i++){
for(int j=0;j<3;j++)
  {dcm[i][j] = temp_dcm[i][j];
  //Serial.print(dcm[i][j]);
  //Serial.print(" ");
  }
  //Serial.println();
  
}
//Serial.println("***********");
renorm();
//Serial.println();
//Serial.println();
//CALCULATING DOT PRODUCT FOR ERROR CORRECTION

for(int i=0;i<3;i++)
dot += dcm[i][0]*dcm[i][1];
//Serial.println(dot);

}

void renorm()
{

float rnorm = sqrt(dcm[0][0]*dcm[0][0] +  dcm[1][0]*dcm[1][0] +  dcm[2][0]*dcm[2][0]);
for(int i=0;i<3;i++)
{
  dcm[i][0]/=rnorm;
}



rnorm = sqrt(dcm[0][1]*dcm[0][1] +  dcm[1][1]*dcm[1][1] +  dcm[2][1]*dcm[2][1]);
for(int i=0;i<3;i++)
{
  dcm[i][1]/=rnorm;
}


rnorm = sqrt(dcm[0][2]*dcm[0][2] +  dcm[1][2]*dcm[1][2] +  dcm[2][2]*dcm[2][2]);
for(int i=0;i<3;i++)
{
  dcm[i][2]/=rnorm;
}



  
}
//////////////////IMU functions ends
unsigned long throttle2;

void loop()
{
 record_data();
       
       dt = (micros()-loop_timer)/1000000.0;
       
       angle_calc();
       loop_timer = micros();                                                    //Set the timer for the next loop.


Kp_phi = 0.05*rec_input_6 - 50;
Kp_theta = Kp_phi;
/*
Serial.print("Roll ");
Serial.print(int(roll*degconvert));
Serial.print(" Pitch ");
Serial.println(int(pitch*degconvert));
*/
////////#####


phi = int(roll*degconvert);
phi_dot = gyro[0];

theta = int(pitch*degconvert);
theta_dot = gyro[1];

u2[0] = Kp_phi*(phi_des-phi) + Kd_phi*(phiDes_dot-phi_dot);                     //.error in phi calculated
u2[1] = -Kp_theta*(theta_des-theta) - Kd_theta*(thetaDes_dot-theta_dot);        //.error in theta calculated
u2[2] = 0;                                                                      //YAW is not in self correction 

//Serial.print(phi_des-phi);Serial.print(",");Serial.print(thetaDes_dot-theta_dot);Serial.println();
//Serial.print("roll is ");Serial.print(roll*degconvert);Serial.println();
//Serial.println(throttle);


if(rec_input_5>1500){
   
    esc_1 = throttle - u2[1] + u2[0]; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + u2[1] + u2[0]; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + u2[1] - u2[0]; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - u2[1] - u2[0]; //Calculate the pulse for esc 4 (front-left - CW)

}
else
{
  esc_1 = 1000;
  esc_2 = 1000;
  esc_3 = 1000;
  esc_4 = 1000;


}

    esc_1 = constrain(esc_1,1000,2000);
    esc_2 = constrain(esc_2,1000,2000);
    esc_3 = constrain(esc_3,1000,2000);
    esc_4 = constrain(esc_4,1000,2000);
  
   /* Serial.print(esc_1);
    Serial.print(" ");
    Serial.print(esc_2);
    Serial.print(" ");
    Serial.print(esc_3);
    Serial.print(" ");
    Serial.print(esc_4);
    Serial.println();
   */

   while(micros() - pid_loop_timer < 5000);                                      //We wait until 4000us are passed.

       pid_loop_timer = micros();                                                    //Set the timer for the next loop.


  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + pid_loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + pid_loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + pid_loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + pid_loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.

  //Serial.println(int(roll*degconvert));

  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
  }
  
 
}





ISR(PCINT0_vect){
  current_time = micros();
  //Channel 3=========================================
  if(PINB & B00000001){                                                     //Is input 8 high?
    if(last_channel_1 == 0){                                                //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if(last_channel_1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    throttle1 = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }

  //Channel 5=========================================
  if(PINB & B00010000){                                                    
    if(last_channel_5 == 0){                                              
      last_channel_5 = 1;                                                   
      timer_5 = current_time;                                             
    }
  }
  else if(last_channel_5 == 1){                                             
    last_channel_5 = 0;                                                    
    rec_input_5 = current_time - timer_5;                            
  }

 /* //Channel 6=========================================
  if(PINB & B00100000){                                                    
    if(last_channel_6 == 0){                                              
      last_channel_6 = 1;                                                   
      timer_6 = current_time;                                             
    }
  }
  else if(last_channel_6 == 1){                                             
    last_channel_6 = 0;                                                    
    rec_input_6 = current_time - timer_6;                            
  }
*/
  
if(throttle1-last_throttle<5||throttle1-last_throttle>-5)
throttle = last_throttle;
last_throttle = throttle1;

}




