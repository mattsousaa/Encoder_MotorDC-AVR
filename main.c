/*
 * Carro pancadao.c
 *
 * Created: 06/06/2018 13:22:33
 * Author : Mateus Sousa
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <aterrupt.h>

#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)

/************ SPEED OF THE CAR - STRAIGHT, CURVES AND STOPS ************/
#define left_speed 80
#define right_speed 80
#define fast_left_speed 85
#define fast_right_speed 85
#define stop_left 0
#define stop_right 0

/************ GLOBAL VARIABLES ************/
volatile unsigned long millis_value;		// Store value of millisecond since uC started
int rpm = 0;								// Store RPM of motor left
int rpm2 = 0;								// Store RPM of motor right
char start_car = 0x00;						// Start the car through UART
int pulsos = 0;								// Count the pulses of encoder 1 - Motor Left
int pulsos2 = 0;							// Count the pulses of encoder 2 - Motor Right
unsigned long time_old = 0;					// Time base for function millis()
volatile unsigned long milis_count = 0;		// This var is shared between function and an ISR, so it must be declared volatile
unsigned int pulsos_por_volta = 20;			// Number of holes in encoders


/************ FUNCTIONS - UART ************/
void USART_init(void);
void USART_send(unsigned char data);
unsigned char USART_Receive(void);
void USART_putstring(char* StringPtr);

/************ TWO SENSORS ON BLACK LINE ************/
/*

00 - STOP COMPLETE - WHITE LINE
01 - PWM - 150
10 - PWM - 150
11 - GO STRAIGHT - BLACK LINE

*/

/************ THREE SENSORS ON BLACK LINE ************/
/*

000 - STOP COMPLETE - WHITE LINE
001 - PWM - 170
011 - PWM - 150
100 - PWM - 170
110 - PWM - 150
111 - GO STRAIGHT - BLACK LINE

*/

/************ FOUR SENSORS ON BLACK LINE ************/
/*

0000 - STOP COMPLETE - WHITE LINE
0001 - PWM - 190
0011 - PWM - 170
0111 - PWM - 150
1000 - PWM - 190
1100 - PWM - 170
1110 - PWM - 150
1111 - GO STRAIGHT - BLACK LINE

*/

/*
ISR(INT0_vect){
	pulsos++;
}

ISR(INT1_vect){
	pulsos2++;
}*/

ISR(PCINT0_vect){
	
	if ((!(PINB & (1 << PINB0))) || (!(PINB & (1 << PINB1)))){
		start_car = 0x00;
		OCR0A = stop_left;
		OCR0B = stop_right;
	} else{
		start_car = 0xFF;
		OCR0A = left_speed;
		OCR0B = right_speed;
	}
	
	if (!(PINB & (1 << PINB4))){
		OCR0B = stop_right;
		OCR0A = fast_left_speed;
	}
	
	if (!(PINB & (1 << PINB5))){
		OCR0A = stop_left;
		OCR0B = fast_right_speed;
	}
	
}

ISR(TIMER1_COMPA_vect){
	//milis_count++;    //Just add 1 to our milis value
	
	if(start_car == 0xFF){
		// Car go straight - Black line
		if((PINB & (1 << PINB4)) && (PINB & (1 << PINB5))){
			//if(rpm >= 162 && rpm2 <= 100){	// Let the two motors on the same speed after a curve
				//OCR0A  = 235;
				//OCR0B  = 235;
			//}
			
			//if(rpm <= 100 && rpm2 >= 162){	// Let the two motors on the same speed after a curve
				OCR0A  = left_speed;
				OCR0B  = right_speed;
			//}
		}
		
		/*if(!((PINB & (1 << PINB4))) && !((PINB & (1 << PINB5)))){
			OCR0A  = stop_left;
			OCR0B  = fast_right_speed;
			
		}*/
	}
}

ISR(USART_RX_vect){
	
	char ReceivedByte;
	ReceivedByte = USART_Receive();
	//UDR0 = ReceivedByte; // Echo back the received byte back to the computer
	
	if(ReceivedByte == '1'){
		start_car = 0xFF;
	} else{
		start_car = 0x00;
	  	OCR0A  = stop_left;
	 	OCR0B  = stop_right;
	 }
	 
	//USART_send(ReceivedByte+1);
}

/*
unsigned long millis(void){
	
	cli();   //Disable all interrupts so we can read our long variable atomically
	//This is to ensure that no interrupt is fired while reading the long variable
	//And possibly trash the readed value
	unsigned long milis_value = milis_count;    //Copy the value and return it
	sei();        //Enable all interrupt again
	return milis_value;
	
}*/

void setup_uc(){
	
	/************IO PORT CONFIG************/
	//SET all PORTD as output, except D0, D2, D3 and D4
	DDRD = 0b01100000;
	//PORTD initial value
	PORTD = 0x00;
	
	DDRB = 0b00000000;
	PORTB = 0b00110011;
	
	//Config INT0 and INT1 active on falling edge
	//EICRA = 0b00001010;
	//INT0 and INT1 Interrupt MASK
	//EIMSK = 0x03;
	
	//Enable Pin Change INT[2,1,0] and PCINT interrupt
	//PCICR = 0b00000101;
	
	PCMSK0 = 0b00110111;
	
	/************TIMER 0 CONFIG************/
	//Timer/Counter Control Register A | COM0A1 COM0A0 COM0B1 COM0B0 -- WGM01 WGM00
	//Fast PWM TOP = 0xFF
	
	//Timer/Counter Control Register B | FOC0A FOC0B -- WGM02 CS02 CS01 CS00
	//Timer configuration
	TCCR0A = ((1<<COM0B1)|(1<<COM0A1)|(1<<WGM01)|(1<<WGM00));    //Enable pwm mode in pin PD6 and PD5 and set the WGM bits to Fast pwm mode
	TCCR0B = ((1<<CS01)|(1<<CS00));                  //Set prescaler to 32
	
	//Timer/Counter Register
	TCNT0  = 0x00;
	//Output Compare Register A
	//OCR0A  = 0;
	//Output Compare Register B
	//OCR0B  = 0;
	//Timer/Counter Interrupt Mask Register | ----- OCIE0B OCIE0A TOIE0
	TIMSK0 = 0x00;
	
	/************TIMER 1 CONFIG************/
	TCCR1B = ((1<<WGM12)|(1<<CS10));    //Timer in CTC mode, prescaler set to 1
	OCR1A = 15999;                      //Our target count to have an 1ms interrupt
	TIMSK1 = (1<<OCIE1A);               //Enable interrupt when OCR1A value is reached	

}

int main(void){
	
	setup_uc();
	
	USART_init();
	
	sei();
	
    /* Replace with your application code */
    while (1) {
		
		/*
		if(millis() - time_old >= 1000){	//if(millis() - time_old >= 1000) -> 1s
		
		//clr_bit(EIMSK,INT0);
		//clr_bit(EIMSK,INT1);
		
		cli();
		
		rpm = (60 * 1000 / pulsos_por_volta ) / (millis() - time_old) * pulsos;
		rpm2 = (60 * 1000 / pulsos_por_volta ) / (millis() - time_old) * pulsos2;
		time_old = millis();
		pulsos = 0;
		pulsos2 = 0;
		
		//if(rpm >= 130){
		//	cpl_bit(PORTB, 0);
		//}
	
		
		//cpl_bit(PORTB, 1);
		
		USART_send(rpm);
		
		USART_send(rpm2);
		
		//set_bit(EIMSK,INT0);
		//set_bit(EIMSK,INT1);
		
		sei();
		
		}*/
    }
}

void USART_init(void){
	
	/************************************************************************/
	/*UART			                                                        */
	/************************************************************************/
	
	/*Set baud rate */
	UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
	UBRR0L = (uint8_t)(BAUD_PRESCALLER);
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1 << RXCIE0)|(1 << TXCIE0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}
//------------------------------------------------------------------------------------
void USART_send(unsigned char data){
	/* Wait for empty transmit buffer */
	while(!(UCSR0A & (1<<UDRE0)));
	
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

//------------------------------------------------------------------------------------
unsigned char USART_Receive(void){
	/* Wait for data to be received */
	while (!(UCSR0A & (1<<RXC0)));
	/* Get and return received data from buffer */
	return UDR0;
}
//------------------------------------------------------------------------------------
void USART_putstring(char* StringPtr){
	// sends the characters from the string one at a time to the USART
	while(*StringPtr != 0x00)
	{
		USART_send(*StringPtr);
		StringPtr++;
	}
}