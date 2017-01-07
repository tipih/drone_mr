
#define pin4_5_6_7_on  B11110000
#define pin4_5_6_7_off B00001111
#define pin12_13_on    B00100000

//****************************************************************************************************************
//esc and battery vars
int throttle, battery_voltage;
int esc_1, esc_2, esc_3, esc_4;

//****************************************************************************************************************
//Timer vars
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;

//****************************************************************************************************************
//static vars
char mydata;
byte currentState;
int cal_int;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Test of esc");
  Serial.println("Configurer pin 4 5 6 7 as output for each esc");
  Serial.println("You should check that all engiens runnig as they should in the correct direction");
  setup_configure_pins();
  loop_timer = micros();
  esc_1 = 1000;
  esc_2 = 1000;
  esc_3 = 1000;
  esc_4 = 1000; 
 

}


void setup_configure_pins(){
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
  DDRD |= pin4_5_6_7_on;                                                    //Configure digital port 4, 5, 6 and 7 as output.
  DDRB |= pin12_13_on;                                                      //Configure digital port 12 and 13 as output.

  for (cal_int = 0; cal_int < 625 ; cal_int ++){                           //Wait 5 seconds before continuing.
                                                                            //We pulse the ESC to make them stop beeping
                                                                            //for the IMU to settel
    PORTD |= pin4_5_6_7_on;                                                 //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= pin4_5_6_7_off;                                                //Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                                                //Wait 3000us.
    }
}



void get_serial_input(){
  if (Serial.available() > 0) {
   mydata=Serial.read();
  
  if(mydata=='\n') return;
  
  if (mydata==' '){
  Serial.println("Change to next state");
  currentState++;
  Serial.print("State = ");
  Serial.println(currentState);
  }

   if(mydata=='r') {
    currentState=0;
    esc_1=esc_2=esc_3=esc_4=0;
  
   }
    if(mydata=='s') {
    currentState=0;
    esc_1=esc_2=esc_3=esc_4=0;
  
   }
  
  
  switch (currentState){
   case 0: {
     Serial.println("We got a reset");
     break;
     }
   case 1:{  
    Serial.println("Testing ESC 1, Should be Right Front engien speed set to 1300");
    esc_1=1300;
    esc_2=1000;
    esc_3=1000;
    esc_4=1000;
    break;
  }
   case 2:{
        Serial.println("Testing ESC 2, Should be Right Rear engien speed set to 1300");
    esc_1=1000;
    esc_2=1300;
    esc_3=1000;
    esc_4=1000;
    break;
  }
   case 3:{
        Serial.println("Testing ESC 3, Should be Left Rear engien speed set to 1300");
    esc_1=1000;
    esc_2=1000;
    esc_3=1300;
    esc_4=1000;
    break;
  }
   case 4:{
        Serial.println("Testing ESC 4, Should be Left Rear engien speed set to 1300");
    esc_1=1000;
    esc_2=1000;
    esc_3=1000;
    esc_4=1300;
    break;
  }
  case 5:{
        Serial.println("Testing all engiens");
    esc_1=1080;
    esc_2=1080;
    esc_3=1080;
    esc_4=1080;
    break;
  }
  case 6:{
        Serial.println("Testing all engiens");
    esc_1=1000;
    esc_2=1000;
    esc_3=1000;
    esc_4=1000;
    currentState=0;
    break;
  }
   default:
  break;
  
 
  
  }
 }
}
void loop() {
  // put your main code here, to run repeatedly:
get_serial_input();



 
 if(micros() - loop_timer > 4050){digitalWrite(13, HIGH);}                   //Turn on the LED if the loop time exceeds 4050us.

  while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.





PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
 



 while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
    
  }

}
