#define time_up 100

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting test program for doide indicatore");
  Serial.println("Diode connect to A2 will flash 10 timer");
  Serial.println("Then Pin13 and then Pin A3");
  //Set pin 12-13 to output;
  //located on Port Register B
  //PB4 = 12
  //PB5 = 13
  //Bit   1312111098
  DDRB = DDRB | B0100000; //Setting bit 12 and 13 to output
  pinMode(A3,OUTPUT);
  pinMode(A2,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

 Serial.println("Flashing pin 12");
 for (int a = 0;a<10;a++){
 delay(time_up);
 digitalWrite(A2,HIGH);

 delay(time_up);
digitalWrite(A2,LOW);
 
 }
Serial.println("Flashing pin 13");
 for (int a = 0;a<10;a++){
 
 delay(time_up);
 PORTB |=B0100000;

 delay(time_up);
 PORTB &=B0011111;
 }

Serial.println("Flashing pin A3");
  for (int a = 0;a<10;a++){
 
 delay(time_up);
digitalWrite(A3,HIGH);
 delay(time_up);
 //digitalWrite(A3,LOW);
 }
 digitalWrite(A3,LOW);
}
