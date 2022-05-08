
//#include <Servo.h>
#include <avr/io.h> // // I/O and register definitions
#include <avr/interrupt.h> // interrupt functions and definitions
#define BIT(a) (1 << (a)) // for turing bits on using BIT(a) definition


//***************PID controller************

void PID_controller();

//Servo servo1, servo2;
//*************ADC read*********************
float read_ADC_voltage_register(int channel, int n);
volatile boolean adcDone = false; // ADC completion interrupt flag

// **************servo pulse**************8
// Timer 1 initial value (used for pulse generation)
long int timer1_start = 65535 - (20.0e-3) * 16.0e6/8.0 ;
void servo_pulsewrite(unsigned int Servo, unsigned int z);


// initial time of test
float t0;


void setup() 
{	
	// Serial startup sequence below ensures reliable / predictable startup /////
	char ch;
	
	Serial.begin(2000000);
	
	//wait until Serial Monitor or save_serial program is started
//	while( !Serial ) delay(1);
	
//	while(1) {
//	if( Serial.available() > 0 ) { // is there incoming data
//			ch = Serial.read(); // read one character / byte
//			if( ch == 's' ) break;
//		}
//		delay(10); // leave some time for printing message above, etc.
//	}
	
	// give some time for user to look at robot
	delay(3000);
	
	//***********servo_setup ************//
	
	// Default 1500us as 0 degree on servo
	
	
	// setting 1500us as 0 degree for pulse generation
	
	long int Timer1_compare = (65535 - (1500.0*1.0e-6) * (16.0e6/8.0) );
	
	// Servo attach to pin 7 and 8

	DDRB |= BIT(0); // Pin 8
	DDRD |= BIT(7); // Pin 7
	
	// Reset Timer1 Controls
	TCCR1A = 0;
	TCCR1B = 0;

	// set timer1 prescaler to 8
	TCCR1B |= BIT(CS11);

	// clear previous overflow interrupts so an overflow interrupt doesn't 
	// immediately occur when enabled below
	// +
	// Compare match interrupt
	TIFR1 |= BIT(TOV1) | BIT(OCF1A) | BIT(OCF1B);

	// set timer1 overflow interrupt bit in interrupt mask / register
	TIMSK1 = 0;
	TIMSK1 |= BIT(TOIE1) | BIT(OCIE1A) | BIT(OCIE1B);

	// set timer1 initial value 
	TCNT1 = timer1_start;
	// set timer compare register A
	// -- a compare match interrupt will occur
	// when the timer reaches this value.
	OCR1A = Timer1_compare;
	OCR1B = Timer1_compare;
	

	
	
	

	//servo1.attach(7); 
	//servo2.attach(8);

	t0 = micros()*1.0e-6; // initial time (s)

	while(1) {
		
		PID_controller();
	}
	delay(1000);
	exit(0); // not using loop()	
}


void PID_controller()
{		
	int n;
	float y1=0, y2=0, y3;
	float wb, wf;
	float u1=0, u2;
	int pw1, pw2;
	int w1, w2;	
	float t;

	

	const int   PW_MAX = 2000, PW_0 = 1500, PW_MIN = 1000;
  const float PW_R = 1.0 / (PW_MAX - PW_0);
	const float wmax = 810.0; // maximum back wheel speed (rad/s)	
	const float V_bat = 12.0; // lipo battery voltage
	const float V_bat_inv = 1/V_bat;	

	// read clock time (s)
	t = micros()*1.0e-6 - t0;


      
	
	n = 130; // number of averaging sample 
  	
	// reading output (feedback) signal
	
	y1 = read_ADC_voltage_register(A1,n);
 if(y1>2.37 && y1 <2.67) y1 =2.50; // like a filter
	y2 = read_ADC_voltage_register(A3,n);
	if(y2>2.37 && y2 <2.67) y2 =2.50; // like a filter
	y3 = read_ADC_voltage_register(A5,n);

	
	float r;
	
	float y,dy; 
   float yp = 0.0;
	float e,ee;
	float ep = 0.0,epp = 0.0;
	float tp = 0.0;
	float dt;
	
	float kp,kd,ki,kpp,kdd,kii;
	float de = 0.0,dee = 0.0;
	float ie = 0.0,iee = 0.0;
	float ie_max;
	float z;
  static float Rw = 3.2e-2; /// (m) 3.2 cm tire radius
  float tol = 1.0e-10;
  
    
  float s;
  float wdes = 800.0;
  float wbdes = 100.0; 
	//y = (y1-2.5)*0.4*V_bat; // transforming sensor vvoltage (tacho) to corresponding 
	                        // battery voltage. 
	// note: y1 and y2 tacho are in voltage 0 to 2.5 to 5v we convert it to angular velocity

  
  
	if(t <=15) {          // referrence input is considered in a way to examine both 
                          // traction and speed controller effects on speed and slip 	
		r = 0.0;         // ratio. 
	} 
	else if(t<=45){
       r = wdes ; 
	}	   
 else
 {
  r=wbdes;
 }
 
 
  y1 = (y1 -2.5)* 0.4 * wmax; // voltage to angular velocity
  y2 = (y2 -2.5)* 0.4 * wmax; // voltage to angular velocity

  s = (( y1 * Rw) - (y2 *Rw) ) / ( fabs(y2 *Rw) + tol );   
  Serial.print(s);
  Serial.print(",");  
 
	
	dt = t-tp;
	
	tp = t;
	
	
	ee = (0.01-s);    // error. cascaded with slip ratio
  kpp = 2.0;  // these constants are designed for cases of traction or break 
	kdd = 0.1;
	kii = 180.0;
	 
	epp = ee;  // save error for next step time. 
	
	dee = (ee - epp)/dt;  // derivative of error. 
	
	iee += ee*dt;  // integration of error. 

  
	
  e = (r-y1);    // error. cascaded with slip ratio
  kp = 15.0;  // these constants are designed for cases of traction or break 
  kd = 0.1;
  ki = 0.3;
   
  ep = e;  // save error for next step time. 
  
  de = (e - ep)/dt;  // derivative of error. 
  
  ie += e*dt;  // integration of error. 

  //u1 = (kp*e + kd*de + ki*ie);
  if (r !=wbdes)
  {
  u1 = (kp*e + kd*de + ki*ie)+(kpp*ee + kdd*dee + kii*iee);
  }
  else
  {
  u1 = 0.1*(kp*e + kd*de + ki*ie)+1.1*(kpp*ee + kdd*dee + kii*iee); 
  }

  // prevent u[1] from going out of range
  if( u1 > V_bat)  u1 = V_bat;
  else if (u1 < 0) u1 = -V_bat;

	u2 = 0.0;
		
	w1 = u1 * V_bat_inv * (PW_MAX - PW_0) + PW_0; // input voltage to pulse width
  
	if(w1 > PW_MAX) w1 = PW_MAX;
	if(w1 < PW_MIN) w1 = PW_MIN;	
	
	pw1 = w1; // u1

	pw2 = 1500;//u2
	
	
	//servo1.writeMicroseconds(pw1);
	//servo2.writeMicroseconds(pw2);
	
	servo_pulsewrite(0,pw1); // Pin 7
	servo_pulsewrite(1,pw2); // Pin 8
	
	
	wb = y1 ;
  
	wf = y2;
	
	u2 = 0.0;

	
	
	
	//Serial.print(u1);
	//Serial.print(",");
	
	Serial.print(r);
	Serial.print(",");	

  
	Serial.print(wb);
	Serial.print(",");		
	
	Serial.print(wf);
	Serial.print("\n");	
	
	delay(30);
}

	// read and calculate the average of N ADC values
	float read_ADC_voltage_register(int channel, int n) 
{
	// select ADC channel based on the pin
	if ( channel==A1 ) {
		ADMUX = 0;
		ADMUX |= BIT(MUX0); // activate A1 pin
	} else if ( channel==A3 ) {
		ADMUX = 0;
		ADMUX |= BIT(MUX0) | BIT(MUX1); // activate A3 pin
	} else if ( channel==A5 ) {
		ADMUX = 0;
		ADMUX |= BIT(MUX0) | BIT(MUX2); // activate A5 pin
	} else Serial.print("\nwrong analog pin name!\n"); 

	// set ADC reference (max input voltage) to 5V (Vcc) from micrcontroller
	ADMUX |= BIT(REFS0);
 
	// set ADC control and status register A
	ADCSRA = 0;
	
	ADCSRA |= BIT(ADEN); // ADC enable
		
	ADCSRA |= BIT(ADIE); // ADC interrupt enable

	// 128 prescaler selected
	ADCSRA |= BIT(ADPS0) | BIT(ADPS1) | BIT(ADPS2); // 128 prescaler 
	
	// start ADC manually (start ADC conversion)
	ADCSRA |= BIT(ADSC);
	
	int ADC_counter = 0;
	unsigned long int sum=0;
	float ave , voltage_ADC = 0;
	const float ADC_to_V = 1.0/1023.0*5; // convert ADC values to voltage
	while (1){
	
	if (adcDone){
	adcDone = false;
	ADC_counter++;
	sum += ADC; // calculate running summation
	
	if (ADC_counter>=n){
	ave = sum / ADC_counter; // average analog input

	voltage_ADC = ave * ADC_to_V; // convert the value to volts
	
	return voltage_ADC;
	}
	}
	}	
	}
	
	
// ADC conversion complete interrupt
ISR(ADC_vect)
{
adcDone = true;
ADCSRA |= BIT(ADSC); // start new ADC conversion
}
	
	
void servo_pulsewrite(unsigned int Servo, unsigned int z){ // Only 2 Servos
	
	if (Servo == 0) { //1st Servo
		// clear previous overflow interrupts so an overflow interrupt doesn't 
		// immediately occur when enabled below
		// +
		// Compare match interrupt
		TIFR1 |= BIT(OCF1A);

		// New Compare match
		float T1_compar_1_val = 65535 - (z*1e-6) * (16.0e6/8.0) ;

		// set timer compare register A
		// -- a compare match interrupt will occur
		// when the timer reaches this value.
		OCR1A = T1_compar_1_val;
		
	}else{
		//Same as above
		TIFR1 |= BIT(OCF1B);
		float T1_compare_2_val = 65535 - (z*1e-6) * (16.0e6/8.0) ;
		OCR1B = T1_compare_2_val;
		
	}

}

ISR(TIMER1_COMPA_vect){
  // turn pin 7 on
    PORTD |= BIT(7);
}

ISR(TIMER1_COMPB_vect){
  // turn pin 8 on
    PORTB |= BIT(0);
}

ISR(TIMER1_OVF_vect){
  // turn pin 7 off
  PORTD &= ~BIT(7);
  // turn pin 8 off
  PORTB &= ~BIT(0);
  TCNT1 = timer1_start;
}
	


void loop()
{
	// not used
}
