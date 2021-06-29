/*
  EFY PC OsciloScope
  Read A0 and send to UART at 115200.  

  Read the voltages as fast as possible and send to serial port.
  115200 baud pushes each byte at around 85us. 
  But the default ADC config by Arduino gives ADC at 116us. so here ADC config with additional lines of code and get samples faster than 85us.
  
  Data throughput is decided by the serial baud rate
  @ 115.2k baud we can get 12kSps
   
  Created on 18th Oct 2016
  by Balaji ramalingam, Robert Bosch, Banagalore
  Balajir@in.bosch.com
 
 */

// These constants won't change.  They're used to give names
// to the pins used:
const int analogInPin1 = A0;  // Analog input pin that the potentiometer is attached to
//static int ctr,flag_tog;
//static unsigned char adcval;
unsigned int adcval_L,adcval_H,adcval;
bool flag=0;
unsigned long val=0;

void setup()
{
  Serial.begin(115200);
  
  pinMode(13, OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(2,INPUT_PULLUP);

  ADCSRA = 0;             // clear ADCSRA register
  ADCSRB = 0;             // clear ADCSRB register
  ADMUX &= ~(1|(1<<1)|(1<<2)|(1<<3));    // set A0 analog input pin
  ADMUX |= (1 << REFS0);  // set reference voltage
  ADMUX &= ~(1 << ADLAR);  // left align ADC value to 8 bits from ADCH register

  // sampling rate is [ADC clock] / [prescaler] / [conversion clock cycles]
  // for Arduino Uno ADC clock is 16 MHz and a conversion takes 13 clock cycles
  //ADCSRA |= (1 << ADPS2) | (1 << ADPS0);    // 32 prescaler for 38.5 KHz
  ADCSRA |= (1 << ADPS2);                     // 16 prescaler for 76.9 KHz
  //ADCSRA |= (1 << ADPS1) | (1 << ADPS0);    // 8 prescaler for 153.8 KHz

  ADCSRA |= (1 << ADATE); // enable auto trigger
  ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN);  // enable ADC
  ADCSRA |= (1 << ADSC);  // start ADC measurements
  //Timer pwm
  TCCR1A |= (1<<5)|(1<<7)|(1<<4);
  TCCR1A &= ~((1<<1)|1|(1<<6));
  TCCR1B |= 1|(1<<4);
  TCCR1B &= ~((1<<3)|(1<<2)|(1<<1));
  TCNT1 = 0;
  ICR1 = 16000;  
  OCR1A=8000;
  OCR1B=8000;
  // Timer triangular wave
  TCCR0A |= (1<<5)|(1<<7)|(1<<4)|1;
  TCCR0A &= ~((1<<1)|(1<<6));
  TCCR0B |= 1|(1<<1);
  TCCR0B &= ~((1<<3)|(1<<2));
  TCNT0 = 0;    
  OCR0A=126;
  OCR0B=126;
}

ISR(ADC_vect)
{
  //adcval = ADH;
  adcval_L = ADCL;
  adcval_H = ADCH;  // read 8 bit value from ADC   
}
  
void loop()
{
  adcval = (adcval_H*256)+adcval_L; 
  
  /*
  for (int i=0;i<100;i++)  
  {
    val += adcval;
  }
  val /=100;
  */
  Serial.println(adcval);
  //flag = digitalRead(2);
  //adcval = (adcval_H*256)+adcval_L;
  //adcval = map(adcval,0,65535,0,255);
  //Serial.write(adcval);
  //if (flag==1)
  //Serial.write(TCNT0);
  //else
  //Serial.write(adcval);
  /*
  // Following code to generate ref signal at pin 13 @ 50HZ. You can connect A0 to see the waveform in PCScope.exe
  ctr++;
  if(ctr>117) //117=10.03ms
  {
   ctr=0; 
   flag_tog = !flag_tog; 
   digitalWrite(13, flag_tog);
  }
  */
}
