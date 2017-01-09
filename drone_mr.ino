#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
#include <EEPROM.h>                        //Include the EEPROM.h library so we can store information onto the EEPROM
#define flightcontroller
#define enable_serial
#define gyro_adress 0x68
#define red_pin A3
#define white_pin 13
#define red_pin_b   B00010000
#define pin4_5_6_7_on  B11110000
#define pin4_5_6_7_off B00001111
#define pin12_13_on    B00100000
#define low_throttle 1200
#define throttle_low_min 990
#define throttle_low_max 1020

#define state_idle 1
#define state_running 2
#define state_stopped 3
#define state_lost_receiver 4
#define state_low_batt 5
#define state_no_receiver 6
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)
//****************************************************************************************************************



//****************************************************************************************************************
//General vars
int print_out;
boolean gyro_angles_set;
unsigned long loop_timer;
char mydata;                                //Input data var for serial port
int mode;                                   //Start mode 0 - 1 - 2 - 3
byte state; 
bool remote_present = false;                //Default value for flight controller is set to false, will be redefined if flightcontroller flag is set
boolean auto_level = true;                  //Auto level on (true) or off (false)
int cnt =0;



//****************************************************************************************************************
//PID vars
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
float roll_level_adjust, pitch_level_adjust;




//****************************************************************************************************************
//GYROS (6050 IMU) vars
int cal_int, start, gyro_address;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
int gyro_x, gyro_y, gyro_z;
int temperature;
int acc_axis[4], gyro_axis[4];
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long acc_x, acc_y, acc_z, acc_total_vector;




//****************************************************************************************************************
//esc and battery vars
int throttle, battery_voltage;
int esc_1, esc_2, esc_3, esc_4;




//****************************************************************************************************************
//Timer vars
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;





//****************************************************************************************************************
//RECEIVER vars
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;






//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//Start of program 
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************




//****************************************************************************************************************
//Setup section
#ifdef enable_serial
void setup_serial(){
 Serial.begin(115200);
 Serial.println("Starting Flight controller program"); 
}
#endif


void setup_mode_and_adress(){
 mode=0;                                                                    //Setup mode 0 for startup
 gyro_address = gyro_adress;                                                //Set gyro adress, as defined in top of program
}

void setup_wire(){
  Wire.begin();                                                             //Start the I2C as master.
  TWBR = 12;                                                                //Set the I2C clock speed to 400kHz.
}

void setup_configure_pins(){
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
  DDRD |= pin4_5_6_7_on;                                                    //Configure digital port 4, 5, 6 and 7 as output.
  DDRB |= pin12_13_on;                                                      //Configure digital port 12 and 13 as output.
  pinMode(A3,OUTPUT);
  pinMode(A2,OUTPUT);
}



void setup_pin_interrupt(){
    PCICR |= (1 << PCIE0);                                                  //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT4);                                                  //Set PCINT3 (digital input 12)to trigger an interrupt on state change.

}

void setup_vars() {
     start=0;                                                               //Setup state 0 for startup
    
}

void setup_look_for_controller(){
  int remote_cnt;                                                           //Local var, used to look for remote control 
  remote_present = true;                                                    //Make sure that the remote var is set to true;
  state=state_idle;                                                         //Setting the state to idle
  
  //Wait until the receiver is active and the throtle is set to the lower position.
  while((receiver_input_channel_3 < throttle_low_min || receiver_input_channel_3 > throttle_low_max || receiver_input_channel_4 < 1400) && (remote_present==true)){

    start ++;                                                               //While waiting increment start whith every loop.
                                                                            //We will be using this state var for loop increment
                                                                            //and reset it when finish
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
    
    PORTD |= pin4_5_6_7_on;                                                 //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= pin4_5_6_7_off;                                                //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
    if(start == 125){                                                       //Every 125 loops (500ms).
      digitalWrite(white_pin, !digitalRead(white_pin));                     //Change the led status.
      start = 0;                                                            //Start again at 0.
#ifdef enable_serial
       Serial.print("Looking for remote, and Throttle set to min");
       Serial.println(remote_cnt);
#endif
      remote_cnt++;
      if (remote_cnt==20){
        Serial.println("Did not find any remote");
        remote_present=false;                             //We did not find any remote
        state=state_no_receiver;                          //No receivers detected, setting the state to no receivers
      }
    }
  }
  start = 0;                                                                //Reset the state var.
}
//end of setup section
//****************************************************************************************************************



void setup() {
  // put your setup code here, to run once:

#ifdef enable_serial
  setup_serial();
#endif

  setup_mode_and_adress();
  //readPID();                                                                //Read the eprom
  setup_wire();                                                             //Setup I2C
  setup_configure_pins();                                                   //Setup output pins

  digitalWrite(red_pin,HIGH);                                               //Set red pin high, to indicate that program is starting 
  set_gyro_registers();                                                     //Set the specific gyro registers.

  for (cal_int = 0; cal_int < 1250 ; cal_int ++){                           //Wait 5 seconds before continuing.
                                                                            //We pulse the ESC to make them stop beeping
                                                                            //for the IMU to settel
    PORTD |= pin4_5_6_7_on;                                                 //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= pin4_5_6_7_off;                                                //Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                                                //Wait 3000us.
    }

  digitalWrite(red_pin,LOW);                                                //Set red_pin off, to indicate that we are finish waiting

  
  
  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
#ifdef enable_serial
  Serial.println("Calibrating");
#endif  
  Calibrating();                                                            //Call calibrate function for the gyro
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.

  


digitalWrite(red_pin,LOW);                                                  //Set red_pin to low, pin has been used inside the calibrating routine 
print_out=0;                                                                //Set print out to 0 used for serial output
throttle=low_throttle;                                                      //Setup throttle to 1200

setup_pin_interrupt();                                                      //Setup pin interrupt for pin 8-9-10-11, this is receiver input pins
setup_vars();                                                               //Setup some vars

#ifdef flightcontroller
setup_look_for_controller();                                                //Look for flight controller
#endif

reset_system_pid();                                                         //Reset the PID controller
digitalWrite(red_pin,LOW);
#ifdef enable_serial
Serial.println("Entering main loop");
#endif
 
 if (state==state_no_receiver) wait_for_receivers();
loop_timer = micros();                                                      //Set loop_time for first run    
reset_system_pid();
 digitalWrite(13,LOW);        
}





void loop() {
  // put your main code here, to run repeatedly:

 

  get_gyro_calculation();
  

  
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.
  
  if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
  }
  if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
  }

   //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
  angle_roll_acc -= 0.0;                                                    //Accelerometer calibration value for roll.
  
  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.



 

  pitch_level_adjust = angle_pitch * 20;                                    //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 20;                                      //Calculate the roll angle correction






//The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_1 > 1508)pid_roll_setpoint = (receiver_input_channel_1 - 1508);
  else if(receiver_input_channel_1 < 1492)pid_roll_setpoint = (receiver_input_channel_1 - 1492);

  pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.
 

  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_2 > 1508)pid_pitch_setpoint = (receiver_input_channel_2 - 1508)*-1;
  else if(receiver_input_channel_2 < 1492)pid_pitch_setpoint = (receiver_input_channel_2 - 1492)*-1;


  pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_3 > 1050){ //Do not yaw when turning off the motors.
    if(receiver_input_channel_4 > 1508)pid_yaw_setpoint = (receiver_input_channel_4 - 1508)/3.0;
    else if(receiver_input_channel_4 < 1492)pid_yaw_setpoint = (receiver_input_channel_4 - 1492)/3.0;
  }

 


if (throttle>1150) //only do pid calculation if  throttle is more the 1150, this should prevent the pid to start working before we start to gas up 
 calculate_pid(); 



//Check state mashine
if ((state==state_idle) || (state==state_stopped))
{
wait_for_remote_controll(); //If we are in Idle then we need to wait for the user to perform the correct sequence
reset_system_pid();
}

//We are now in the loop
//Stopping the motors: throttle low and yaw right.
if(state == state_running && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)state = state_stopped;



if (state==state_running){

throttle=receiver_input_channel_3;
if (throttle > 1800) throttle = 1800; 



    
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)


//Serial.print(pid_output_pitch);
//Serial.print("   ");
//Serial.println(pid_output_roll);


    if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.

    if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.
}
else {
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
}


#ifdef enable_serial
if(print_out>20)
 {
 serial_tuning();
 print_out=0;
 }
print_out++;
#endif






  if(micros() - loop_timer > 4050){digitalWrite(A3, HIGH);digitalWrite(A2, HIGH);digitalWrite(13,HIGH);}                   //Turn on the LED if the loop time exceeds 4050us.
  
while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
 

read_mpu_6050_data();

 while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
  }

}
//End of main loop
//*******************************************************************************


void get_gyro_calculation(){
  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
  
  gyro_roll=gyro_y;                                                    //Set the gyro_roll to the calibrated value
  gyro_pitch=gyro_x;                                                   //Set the gyro_pitch to the calibrated value
  gyro_yaw=gyro_z*-1;                                                     //Set the gyro_yaw to the calibrated value



  
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec. gyro_roll_input is calculated using a complementary filter.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec. gyro_pitch_input is calculated using a complementary filter.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec. gyro_yaw_input is calculated using a complementary filter.


}







void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable

}

void Calibrating(){
    Serial.println(" Calibrating gyro");
     for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
    if(cal_int % 125 == 0)
    {
    Serial.print(".");                              //Print a dot on the LCD every 125 readings
    PINB = PINB | red_pin_b; // toggle D12
   
    }
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
  Serial.println();
  
  Serial.print(" Gyro_x =");
  Serial.print(gyro_x_cal);
  Serial.print(" Gyro_y =");//+gyro_y_cal);
  Serial.print(gyro_y_cal);
  Serial.print(" Gyro_z_cal=");//+gyro_z_cal);
  Serial.print(gyro_z_cal);
  Serial.println();
  Serial.print("ACC x=");
  Serial.print(acc_x);
  Serial.print("  ACC y=");
  Serial.print(acc_y);
  Serial.print("  ACC z=");
  Serial.print(acc_z);
  Serial.println();
  delay(1000);
}


void set_gyro_registers(){
  //Setup the MPU-6050
  
    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                                    //End the transmission with the gyro.

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
    Wire.write(0x1B);                                                          //Start reading @ register 0x1B
    Wire.endTransmission();                                                    //End the transmission
    Wire.requestFrom(gyro_address, 1);                                         //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                               //Wait until the 6 bytes are received
    if(Wire.read() != 0x08){                                                   //Check if the value is 0x08
      digitalWrite(red_pin,HIGH);                                                   //Turn on the warning led
      while(1)delay(10);                                                       //Stay in this loop for ever
    }

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
    Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();                                                    //End the transmission with the gyro    

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Init of the PID controller
void reset_system_pid()
{
    angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    gyro_angles_set = true;                                                 //Set the IMU started flag.

    //Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID controller, this is the main function for the PID controller, here is where the magic is happening
//This function has 3 main calc. roll pitch and yaw
void calculate_pid(){
  //Roll calculations
  //gyro_roll_input, is calculated using a complementary filter, to filter out vibrations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;                           //Find the error between the input roll and the roll setpoint
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;                             //calculate the i mem for roll 
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;                 //limit the output
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

//Calculate the final output for the roll function, by the following
// out= p*error + i_mem_roll + d * (error - last_error)
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;               //Limit the output
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;                                         //Store current error to last error

  //Pitch calculations, this will be same as above
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;


  //Yaw calculations, this will be same as above
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Function to store PID values
void storePID(){
  unsigned int baseAddr = 0;
  EEPROM_writeDouble(baseAddr, pid_p_gain_roll);
  EEPROM_writeDouble(baseAddr+4, pid_i_gain_roll);
  EEPROM_writeDouble(baseAddr+8, pid_d_gain_roll);
  //TODO store for pitch and yaw
  
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Function to read PID values
void readPID(){
  unsigned int baseAddr = 0;
  pid_p_gain_roll=EEPROM_readDouble( baseAddr);
  pid_i_gain_roll=EEPROM_readDouble( baseAddr+4);
  pid_d_gain_roll=EEPROM_readDouble( baseAddr+8);
  //TODO read for pitch and yaw

  Serial.println(pid_p_gain_roll);
  Serial.println(pid_i_gain_roll);
  Serial.println(pid_d_gain_roll);
  
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Helper function to write a double to eeprom
void EEPROM_writeDouble(int ee, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
       EEPROM.write(ee++, *p++);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Helper function to read a double to eeprom
double EEPROM_readDouble(int ee)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
       *p++ = EEPROM.read(ee++);
   return value;
}


#ifdef enable_serial
void serial_tuning(){
  if (mode!=0) 
 Serial.print("mode =");
 
 if (mode==1){ 
 Serial.print("P-controller   ");
 Serial.println(pid_p_gain_roll);
 }
 else if(mode==2){
  Serial.print("I-controller   ");
 Serial.println(pid_i_gain_roll);
 }
 else if (mode==3){
  Serial.print("D-controller    ");
 Serial.println(pid_d_gain_roll);
 }
 else if (mode==99){
  Serial.print("Throttle    ");
 Serial.println(throttle);
 }
 else if (mode==90){
   Serial.println(pid_pitch_setpoint); 
 }
  
  if (Serial.available() > 0) {
   mydata=Serial.read();
   if (mydata=='1') mode=1; //Tuning P of pitch and roll
   if (mydata=='2') mode=2; //tuning I of pitch and roll
   if (mydata=='3') mode=3;  //tuning I of pitch and roll
   if (mydata=='0') mode=0; //End tuning
   if (mydata=='t') mode=99; //Store PID values
   if (mydata=='r') {pid_p_gain_roll = 3;               //Gain setting for the roll P-controller
                     pid_i_gain_roll = 0.02;              //Gain setting for the roll I-controller
                     pid_d_gain_roll = 35;              //Gain setting for the roll D-controller
   }
   if (mydata=='s') {
    storePID();
    readPID();
   }
   if (mydata=='q') mode=90;

   
   if ((mydata=='u')&&(mode==1)) pid_p_gain_roll+=.5;
   if ((mydata=='d')&&(mode==1)) pid_p_gain_roll-=.5;
   if ((mydata=='u')&&(mode==2)) pid_i_gain_roll+=.02;
   if ((mydata=='d')&&(mode==2)) pid_i_gain_roll-=.02;
   if ((mydata=='u')&&(mode==3)) pid_d_gain_roll+=5;
   if ((mydata=='d')&&(mode==3)) pid_d_gain_roll-=5;

   if ((mydata=='u')&&(mode==99)) throttle+=10;
   if ((mydata=='d')&&(mode==99)) throttle-=10;
  
   pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
   pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
   pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
   
  }

}
#endif

void wait_for_receivers(){
  while(1){
    digitalWrite(red_pin,HIGH);
  }
}

void wait_for_remote_controll(){
//Lets wait for the user to do the start sequence
  Serial.println("Waiting for remote");
  
  while(state!=state_running){
  Serial.print("receiver_input_channel_3=");
  Serial.print(receiver_input_channel_3 );
  Serial.print("   receiver_input_channel_4=");
  Serial.println(receiver_input_channel_4);
  //For starting the motors: throttle low and yaw left (step 1).
  if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;
  
  //When yaw stick is back in the center position start the motors (step 2).
  if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450){
   Serial.println("Going to running state");
    state=state_running;
    angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    digitalWrite(red_pin,LOW);
    loop_timer = micros();  
  }
 }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This routine is called every time input 8, 9, 10 or 11 changed state. This is used to read the receiver signals. 
//More information about this subroutine can be found in this video:
//https://youtu.be/bENjl1KQbvo
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT0_vect){
  
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                                     //Is input 8 high?
    if(last_channel_1 == 0){                                                //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if(last_channel_1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    receiver_input_channel_1 = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                                    //Is input 9 high?
    if(last_channel_2 == 0){                                                //Input 9 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if(last_channel_2 == 1){                                             //Input 9 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    receiver_input_channel_2 = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                                    //Is input 10 high?
    if(last_channel_3 == 0){                                                //Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
      
    }
  }
  else if(last_channel_3 == 1){                                             //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input_channel_3 = current_time - timer_3;                             //Channel 3 is current_time - timer_3.

  }
  //Channel 4=========================================
  if(PINB & B00010000 ){                                                    //Is input 12 high?
    if(last_channel_4 == 0){                                                //Input 12 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_4 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    receiver_input_channel_4 = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }
}


