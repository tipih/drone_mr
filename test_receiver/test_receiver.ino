
//****************************************************************************************************************
//Timer vars
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;

//****************************************************************************************************************
//RECEIVER vars
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;

char mydata;                                //Input data var for serial port
byte currentState = 0;
static bool recieverDetected;
bool firstTime;
bool readyForNextState;
int maxValue, minValue;

void setup() {
  // put your setup code here, to run once:
 Serial.begin(115200);
 Serial.println("Setup the test program fro the recaiver");
 Serial.println("Use space on the serial terminal to move to test of next channel");
 
 Serial.println("Setting Pin change interrupt for pin 8-9-10-11");
 setup_pin_interrupt();
 currentState = 0;                                                          //Set current state to 0 first path of the program we have 5 state
 recieverDetected=false;
 firstTime=true;
 readyForNextState=false;
 maxValue=1000;
 minValue=2000;
 pinMode(11,INPUT);
}


void setup_pin_interrupt(){
    PCICR |= (1 << PCIE0);                                                  //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT4);                                                  //Set PCINT3 (digital input 11)to trigger an interrupt on state change.

}

void get_serial_input(){
  if (Serial.available() > 0) {
   mydata=Serial.read();
  }
  if (mydata==' '){
  Serial.println("Change to next state");
  currentState++;
  Serial.print("State = ");
  Serial.println(currentState);
  }
  if(mydata=='r') {
    currentState=0;
     maxValue=1000;
     minValue=2000;  
   }
  }
 


void loop() {
  
  // put your main code here, to run repeatedly:
 if (readyForNextState)
      get_serial_input();
 
 if (currentState==0)
 {
 //Print out only one time
  
  if(recieverDetected==true && firstTime==false){
    
    Serial.println(" Receiver detedted Ready for next state");
    readyForNextState=true;
    firstTime=true;
    currentState++;
    
  } 
  else
  {
   if (firstTime==true){
    Serial.println("Waiting for receiver"); 
    firstTime=false;                            //flag for only printing 1 time      
   }//Print out
  }

  
 } //State 0
 else if (currentState==1){
   if (firstTime==true){
    Serial.println("Press space to get to next state"); 
    firstTime=false;                            //flag for only printing 1 time      
   }//Print out
 } //state 1
 else if (currentState==2){
  Serial.print("Channel 1= ");
  Serial.println(receiver_input_channel_1);
  if (receiver_input_channel_1<minValue)minValue=receiver_input_channel_1;
  if (receiver_input_channel_1>maxValue)maxValue=receiver_input_channel_1;
  firstTime=true;
  
 }//state 2
 else if (currentState==3){
  if (firstTime==true){
  
  for (int a=0; a<100;a++) Serial.println("");
  Serial.print("minValue channel 1= ");
  Serial.println(minValue);
  Serial.print("maxValue channel 1= ");
  Serial.println(maxValue);
  firstTime=false;
     maxValue=1000;
     minValue=2000; 
  
  }
 }//state3
 else if (currentState==4){
  Serial.print("Channel 2= ");
  Serial.println(receiver_input_channel_2);
  if (receiver_input_channel_2<minValue)minValue=receiver_input_channel_2;
  if (receiver_input_channel_2>maxValue)maxValue=receiver_input_channel_2;
  firstTime=true;
 } //state 4
 else if (currentState==5){
  if (firstTime==true){
  
  for (int a=0; a<100;a++) Serial.println("");
  Serial.print("minValue channel 2= ");
  Serial.println(minValue);
  Serial.print("maxValue channel 2= ");
  Serial.println(maxValue);
  firstTime=false;
     maxValue=1000;
     minValue=2000; 
  }
 }//state5
 else if (currentState==6){
  Serial.print("Channel 3= ");
  Serial.println(receiver_input_channel_3);
  if (receiver_input_channel_3<minValue)minValue=receiver_input_channel_3;
  if (receiver_input_channel_3>maxValue)maxValue=receiver_input_channel_3;
  firstTime=true;
 } //state 6
 else if (currentState==7){
  if (firstTime==true){
  
  for (int a=0; a<100;a++) Serial.println("");
  Serial.print("minValue channel 3= ");
  Serial.println(minValue);
  Serial.print("maxValue channel 3= ");
  Serial.println(maxValue);
  firstTime=false;
     maxValue=1000;
     minValue=2000; 
  }
 }//state7
 else if (currentState==8){
  Serial.print("Channel 4= ");
  Serial.println(receiver_input_channel_4);
  if (receiver_input_channel_4<minValue)minValue=receiver_input_channel_4;
  if (receiver_input_channel_4>maxValue)maxValue=receiver_input_channel_4;
  firstTime=true;
 } //state 8
 else if (currentState==9){
  if (firstTime==true){
  
  for (int a=0; a<100;a++) Serial.println("");
  Serial.print("minValue channel 4= ");
  Serial.println(minValue);
  Serial.print("maxValue channel 4= ");
  Serial.println(maxValue);
  firstTime=false;
     maxValue=1000;
     minValue=2000; 
  }
 }//state9
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This routine is called every time input 8, 9, 10 or 11 changed state. This is used to read the receiver signals. 
//More information about this subroutine can be found in this video:
//https://youtu.be/bENjl1KQbvo
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT0_vect){
  
  recieverDetected=true;

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
  if(PINB & B00010000 ){                                                    //Is input 11 high?
    if(last_channel_4 == 0){                                                //Input 11 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_4 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    receiver_input_channel_4 = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
    
  }
}
