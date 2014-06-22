#ifndef _sonarCaptureT1_h
#define _sonarCaptureT1_h
//Timer 1 capture ultrasonic sensor
#define CAPTURE 4//PD4(ICP1)
#define TRIG 5
volatile unsigned int pulseWidth = 2013;
void initSonarCaptureT1()
{
  noInterrupts();
  pinMode(CAPTURE, INPUT);
  pinMode(TRIG, OUTPUT);
  
  // initialize timer1 
             // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 65533;            // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= ((1 << ICIE1)| (1 << TOIE1));   //Set capture interrupt
  TCCR1B |= (1 << ICNC1);               // Input Capture Noise Canceler
  TCCR1B |= (1 << ICES1);               //Set capture rising edge
  interrupts();             // enable all interrupts
}

ISR(TIMER1_CAPT_vect){
	if(TCCR1B & (1 << ICES1)){
	    TCNT1  = 0;
	}else{
            pulseWidth = ICR1;               //copy capture value
	}
	TCCR1B ^= 1 << ICES1;
}
void trigPin()
{
	//Serial1.println(TCNT1);
	Serial1.print("D");
	Serial1.println(pulseWidth*0.2616);
	digitalWrite(TRIG, digitalRead(TRIG) ^ 1);
}

#endif
