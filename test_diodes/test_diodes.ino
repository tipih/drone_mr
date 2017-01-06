#define time_up 100

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting test program for doide indicatore");
  Serial.println("Diode connect to Pin12 will flash 10 timer");
  Serial.println("Then Pin13 and then Pin A3");
  //Set pin 12-13 to output;
  //located on Port Register B
  //PB4 = 12
  //PB5 = 13
  //Bit   1312111098
  DDRB = DDRB | B0110000; //Setting bit 12 and 13 to output
  pinMode(A3,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
 
 for (int a = 0;a<10;a++){
 delay(time_up);
 PORTB |=B0010000;

 delay(time_up);
 PORTB &=B0101111;
 
 }

 for (int a = 0;a<10;a++){
 
 delay(time_up);
 PORTB |=B0100000;

 delay(time_up);
 PORTB &=B0011111;
 }

  for (int a = 0;a<10;a++){
 
 delay(time_up);
digitalWrite(A3,HIGH);
 delay(time_up);
 //digitalWrite(A3,LOW);
 }
 digitalWrite(A3,LOW);
}
