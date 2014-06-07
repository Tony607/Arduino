volatile unsigned int pulseStart;
volatile unsigned int pulseEnd;
volatile unsigned int pulseInUs = 2013;
volatile unsigned char edge =0;
volatile unsigned int success = 0;
volatile unsigned int failure = 0;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int cnt = 0;
 
#define START_TIMER TCCR3B |= ((1 << CS31) | (1 << CS30))
#define STOP_TIMER TCCR3B &= ~((1 << CS31) | (1 << CS30))
#define TRIGPIN 7 
#define ECHOPIN 13 //PC7(ICP3)--the led pin on Arduino

void pulseSensor(void){
   //PORTB |= (1 << 0);
   digitalWrite(TRIGPIN,HIGH);
   //_delay_us(10);
   delayMicroseconds(1000);
   //PORTB &= ~(1 << 0);
   digitalWrite(TRIGPIN,LOW);
}
 
void initialisePorts(void){
   pinMode(TRIGPIN,OUTPUT);
   pinMode(ECHOPIN,INPUT);
   //DDRC &= ~(1 << 7);                  //PORTC.7 = INPUT
   //DDRB |= (1 << 0) | (1 << 1) | (1 << 2);   //PORTB.0 = PORTB.1 = OUTPUT
}

void setup() {
  // put your setup code here, to run once:
   initialisePorts();
   TIMSK3 |= ((1 << ICIE3)| (1 << TOIE3));   //Set capture interrupt
   sei();                           //Set global interrupt
   TCCR3B |= (1 << ICES3);               //Set capture rising edge
   pulseSensor();
   // initialize serial:
   Serial.begin(9600);
   // reserve 200 bytes for the inputString:
   inputString.reserve(40);
   START_TIMER;
}

void loop() {
  // put your main code here, to run repeatedly:
      if(success){
         success = 0;
         START_TIMER;
      }
      if(failure){
         failure = 0;
         PORTB |= (1 << 2);
      }
      pulseSensor();
      delay(100);
      Serial.print(success);
      Serial.print("pulseInUs = ");
      Serial.println(pulseInUs);
      
	  if (stringComplete) {
		Serial.print(success);
		Serial.print("pulseInUs = ");
		Serial.println(pulseInUs);
		// clear the string:
		inputString = "";
		stringComplete = false;
  }
}

ISR(TIMER3_CAPT_vect){
   if(edge == 0){
      pulseStart = ICR3;               //copy capture value
      TCCR3B &= ~(1 << ICES3);         //toggle capture edge
      edge = 1;                    
   }
   else{
      pulseEnd = ICR3;               //copy capture value
      TCCR3B |= (1 << ICES3);            //toggle capture edge
      edge = 0;
      STOP_TIMER;
      TCNT3 = 0;  
      pulseInUs = pulseEnd - pulseStart;
      success = 1;
   }
}
 
ISR(TIMER3_OVF_vect){
   failure = 1;
   TCCR3B |= (1 << ICES3);
   STOP_TIMER;
   TCNT3 = 0;
   edge = 0;
}
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
	if (inChar == '#') {//end sign
      stringComplete = true;
    }
  }
}
