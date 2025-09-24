// 20 Hz PWM frequency using ATmega Timer1

void setup( void )
{
  //TIMER1 setup
  TCCR1A=TCCR1B=0;
  //set waveform generator mode WGM1 3..0: 1110 (mode 14; FAST PWM) clear OC1A on compare match
  //set prescaler CS12..0: 100 (clkio/256)
  TCCR1A = (1 << COM1A1) | (1 << WGM11); 
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12);
  //set top and compare register
  ICR1 = 3124; //TOP for 20 Hz, 256 prescaler
  OCR1A = 1500; //default PWM compare

  //on Mega2560, OC1A function is tied to pin 11 (9 on ATmega328)
  pinMode( 9, OUTPUT );

}//setup

void loop( void ) {}