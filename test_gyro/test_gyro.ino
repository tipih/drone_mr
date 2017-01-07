#include <Wire.h>  
byte lowByte, highByte, type, gyro_address, error, clockspeed_ok;
unsigned long timer;
int address, cal_int;


//Gyro vars
float gyro_pitch, gyro_roll, gyro_yaw;
float gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
byte  gyro_check_byte;
byte roll_axis, pitch_axis, yaw_axis;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Test of Gyro");
  Serial.println("This test will ensure the the gyro has been mounted correct and that it works");
  pinMode(A3,OUTPUT);
  
  Wire.begin();             //Start the I2C as master
  error=0;
  delay(250);               //Give the gyro time to start
  
}


//Search for the gyro and check the Who_am_I register
byte search_gyro(int gyro_address, int who_am_i){
  Wire.beginTransmission(gyro_address);
  Wire.write(who_am_i);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 1);
  timer = millis() + 100;
  while(Wire.available() < 1 && timer > millis());
  lowByte = Wire.read();
  address = gyro_address;
  return lowByte;
}


void start_gyro(){
 
  //Setup the MPU-6050
  if(type == 1){
    
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(0x6B);                                            //PWR_MGMT_1 register
    Wire.write(0x00);                                            //Set to zero to turn on the gyro
    Wire.endTransmission();                                      //End the transmission
    
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(0x6B);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 1);                                //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x6B is set to:"));
    Serial.println(Wire.read(),BIN);
    
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(0x1B);                                            //GYRO_CONFIG register
    Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                      //End the transmission
    
    Wire.beginTransmission(address);                             //Start communication with the gyro (adress 1101001)
    Wire.write(0x1B);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 1);                                //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x1B is set to:"));
    Serial.println(Wire.read(),BIN);

  }
  else
  {
    Serial.print("Wrong Gyro should be type 1, got type=");
    Serial.println(type);
  }
}

void gyro_signalen(){
 
  if(type == 1){
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(0x43);                                            //Start reading @ register 43h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address,6);                                 //Request 6 bytes from the gyro
    while(Wire.available() < 6);                                 //Wait until the 6 bytes are received
    gyro_roll=Wire.read()<<8|Wire.read();                        //Read high and low part of the angular data
    if(cal_int == 2000)gyro_roll -= gyro_roll_cal;               //Only compensate after the calibration
    gyro_pitch=Wire.read()<<8|Wire.read();                       //Read high and low part of the angular data
    if(cal_int == 2000)gyro_pitch -= gyro_pitch_cal;             //Only compensate after the calibration
    gyro_yaw=Wire.read()<<8|Wire.read();                         //Read high and low part of the angular data
    if(cal_int == 2000)gyro_yaw -= gyro_yaw_cal;                 //Only compensate after the calibration
  }
}


//Check if the angular position of a gyro axis is changing within 10 seconds
void check_gyro_axes(byte movement){
  byte trigger_axis = 0;
  float gyro_angle_roll, gyro_angle_pitch, gyro_angle_yaw;
  //Reset all axes
  gyro_angle_roll = 0;
  gyro_angle_pitch = 0;
  gyro_angle_yaw = 0;
  gyro_signalen();
  timer = millis() + 10000;    
  while(timer > millis() && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_pitch > -30 && gyro_angle_pitch < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyro_signalen();
    if(type == 2 || type == 3){
      gyro_angle_roll += gyro_roll * 0.00007;              //0.00007 = 17.5 (md/s) / 250(Hz)
      gyro_angle_pitch += gyro_pitch * 0.00007;
      gyro_angle_yaw += gyro_yaw * 0.00007;
    }
    if(type == 1){
      gyro_angle_roll += gyro_roll * 0.0000611;          // 0.0000611 = 1 / 65.5 (LSB degr/s) / 250(Hz)
      gyro_angle_pitch += gyro_pitch * 0.0000611;
      gyro_angle_yaw += gyro_yaw * 0.0000611;
    }
    
    delayMicroseconds(3700); //Loop is running @ 250Hz. +/-300us is used for communication with the gyro
  }
  //Assign the moved axis to the orresponding function (pitch, roll, yaw)
  if((gyro_angle_roll < -30 || gyro_angle_roll > 30) && gyro_angle_pitch > -30 && gyro_angle_pitch < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyro_check_byte |= 0b00000001;
    if(gyro_angle_roll < 0)trigger_axis = 0b10000001;
    else trigger_axis = 0b00000001;
  }
  if((gyro_angle_pitch < -30 || gyro_angle_pitch > 30) && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyro_check_byte |= 0b00000010;
    if(gyro_angle_pitch < 0)trigger_axis = 0b10000010;
    else trigger_axis = 0b00000010;
  }
  if((gyro_angle_yaw < -30 || gyro_angle_yaw > 30) && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_pitch > -30 && gyro_angle_pitch < 30){
    gyro_check_byte |= 0b00000100;
    if(gyro_angle_yaw < 0)trigger_axis = 0b10000011;
    else trigger_axis = 0b00000011;
  }
  
  if(trigger_axis == 0){
    error = 1;
    Serial.println(F("No angular motion is detected in the last 10 seconds!!! (ERROR 4)"));
  }
  else
  if(movement == 1)roll_axis = trigger_axis;
  if(movement == 2)pitch_axis = trigger_axis;
  if(movement == 3)yaw_axis = trigger_axis;
  
}

void systemSetup(){
    Serial.println(F(""));
  Serial.println(F("==================================================="));
  Serial.println(F("System check"));
  Serial.println(F("==================================================="));
  delay(1000);
  Serial.println(F("Checking I2C clock speed."));
  delay(1000);
  
  TWBR = 12;                      //Set the I2C clock speed to 400kHz.
  
  #if F_CPU == 16000000L          //If the clock speed is 16MHz include the next code line when compiling
    clockspeed_ok = 1;            //Set clockspeed_ok to 1
  #endif                          //End of if statement

  if(TWBR == 12 && clockspeed_ok){
    Serial.println(F("I2C clock speed is correctly set to 400kHz."));
    
  }
  else{
    Serial.println(F("I2C clock speed is not set to 400kHz. (ERROR 8)"));
    error = 1;
  }
}
void loop() {
  // put your main code here, to run repeatedly:
systemSetup();
if (error==0){

 //What gyro is connected
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Gyro search"));
    Serial.println(F("==================================================="));
    delay(2000);
     Serial.println(F("Searching for MPU-6050 on address 0x68/104"));
    delay(1000);
    if(search_gyro(0x68, 0x75) == 0x68){
      Serial.println(F("MPU-6050 found on address 0x68"));
      type = 1;
      gyro_address = 0x68;
    }
    else {
      error=1;
      Serial.println("Did not find the MPU6050), please check connections");
    }
}
if(error==0){
  delay(3000);
      Serial.println(F(""));
      Serial.println(F("==================================================="));
      Serial.println(F("Gyro register settings"));
      Serial.println(F("==================================================="));
      start_gyro(); //Setup the gyro for further use
}
if(error == 0){
    digitalWrite(A3,HIGH);
    delay(3000);
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Gyro calibration"));
    Serial.println(F("==================================================="));
    Serial.println(F("Don't move the quadcopter!! Calibration starts in 3 seconds"));
    delay(3000);
    Serial.println(F("Calibrating the gyro, this will take +/- 8 seconds"));
    Serial.print(F("Please wait"));
    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (cal_int = 0; cal_int < 2000 ; cal_int ++){              //Take 2000 readings for calibration.
      if(cal_int % 100 == 0)Serial.print(F("."));                //Print dot to indicate calibration.
      gyro_signalen();                                           //Read the gyro output.
      gyro_roll_cal += gyro_roll;                                //Ad roll value to gyro_roll_cal.
      gyro_pitch_cal += gyro_pitch;                              //Ad pitch value to gyro_pitch_cal.
      gyro_yaw_cal += gyro_yaw;                                  //Ad yaw value to gyro_yaw_cal.
      delay(4);                                                  //Wait 3 milliseconds before the next loop.
    }

//Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
    gyro_roll_cal /= 2000;                                       //Divide the roll total by 2000.
    gyro_pitch_cal /= 2000;                                      //Divide the pitch total by 2000.
    gyro_yaw_cal /= 2000;                                        //Divide the yaw total by 2000.
    
    //Show the calibration results
    Serial.println(F(""));
    Serial.print(F("Axis 1 offset="));
    Serial.println(gyro_roll_cal);
    Serial.print(F("Axis 2 offset="));
    Serial.println(gyro_pitch_cal);
    Serial.print(F("Axis 3 offset="));
    Serial.println(gyro_yaw_cal);
    Serial.println(F(""));
    
    Serial.println(F("==================================================="));
    Serial.println(F("Gyro axes configuration"));
    Serial.println(F("==================================================="));

     //Detect the left wing up movement
    Serial.println(F("Lift the left side of the quadcopter to a 45 degree angle within 10 seconds"));
    //Check axis movement
    check_gyro_axes(1);
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(roll_axis & 0b00000011);
      if(roll_axis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));

      delay(5000);
     //Detect the nose up movement
      Serial.println(F(""));
      Serial.println(F(""));
      Serial.println(F("Lift the nose of the quadcopter to a 45 degree angle within 10 seconds"));
      //Check axis movement
      check_gyro_axes(2);
     }
      if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(pitch_axis & 0b00000011);
      if(pitch_axis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
     delay(5000);
      
      //Detect the nose right movement
      Serial.println(F(""));
      Serial.println(F(""));
      Serial.println(F("Rotate the nose of the quadcopter 45 degree to the right within 10 seconds"));
      //Check axis movement
      check_gyro_axes(3);
    }
     if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(yaw_axis & 0b00000011);
      if(yaw_axis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      
      delay(5000);
    }
      if(error == 0){
    Serial.println(F("Setup is finished."));
    Serial.println(F("Everything look ok"));
  }
     
     
 }
else{
  Serial.println("Error in setup");
 }
 while(1);
}
