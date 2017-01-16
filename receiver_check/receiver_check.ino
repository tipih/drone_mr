//Sketch for testing the distance of the receiver


//define pin for the Led
#define red_pin 13
#define white_pin A2
#define color_pin A3

//****************************************************************************************************************
//Timer vars
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;

//****************************************************************************************************************
//RECEIVER vars
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;

void setup() {
  // put your setup code here, to run once:

 Serial.begin(115200);
 Serial.println("Starting receiver check program");

  pinMode(white_pin,OUTPUT);
  pinMode(red_pin,OUTPUT);
  pinMode(color_pin,OUTPUT);

//Setup of pin change interrupt for pin 8-9-10-12 pin 11 is currently dead :(
    PCICR |= (1 << PCIE0);                                                  //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT4);                                                  //Set PCINT3 (digital input 12)to trigger an interrupt on state change.


}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(red_pin,LOW);
  digitalWrite(white_pin,LOW);
  digitalWrite(color_pin,LOW);
if (receiver_input_channel_1>1500) digitalWrite(red_pin,HIGH);
if (receiver_input_channel_2>1500) digitalWrite(white_pin,HIGH);
if (receiver_input_channel_3>1500) digitalWrite(color_pin,HIGH);
if (receiver_input_channel_4>1500) {
  digitalWrite(red_pin,HIGH);
  digitalWrite(white_pin,HIGH);
  digitalWrite(color_pin,HIGH);
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
