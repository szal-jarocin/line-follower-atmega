/*
 * MGR.c
 *
 * Created: 06.04.2019 14:51:00
 * Author : Mateusz Szalkowski
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define sonic_on	PORTD |= (1<<PORTD3)
#define sonic_off	PORTD &= ~(1<<PORTD3)
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1


volatile char center=0, lewy1=0, lewy2=0, lewy3=0, prawy1=0, prawy2=0, prawy3=0, P_flag, przeszkoda=0, trig, znak=0;
volatile signed int uchyb=0, uchyb_poprzedni=0, regul=0, regul_poprzedni=0, mianownik, mianownik_poprzedni=0;
int predkosc=85, Kp=8;
uint16_t dzielnik=0;
volatile uint16_t progowa=500, dystans=50;
volatile uint32_t czas=0, czas_linia=0, czas_jazda=0;



unsigned char USART_Receive(void)
{
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) );
	/* Get and return received data from buffer */
	return UDR0;
}

void USART_Transmit(unsigned char data)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

ISR(ADC_vect)
{
	_delay_us(100);
	switch (ADMUX)
	{
		case 0x40:
		if (ADC>progowa)
		{
			lewy3=0;
		}
		else
		{
			lewy3=1;
		}
		break;
		case 0x41:
		if (ADC>progowa)
		{
			lewy2=0;
		}
		else
		{
			lewy2=1;
		}
		break;
		case 0x42:
		if (ADC>progowa)
		{
			lewy1=0;
		}
		else
		{
			lewy1=1;
		}
		break;
		case 0x43:
		if (ADC>progowa)
		{
			center=0;
		}
		else
		{
			center=1;
		}
		break;
		case 0x44:
		if (ADC>progowa)
		{
			prawy1=0;
		}
		else
		{
			prawy1=1;
		}
		break;
		case 0x45:
		if (ADC>progowa)
		{
			prawy2=0;
		}
		else
		{
			prawy2=1;
		}
		break;
		case 0x46:
		if (ADC>progowa)
		{
			prawy3=0;
		}
		else
		{
			prawy3=1;
		}
		break;
		case 0x47:
		progowa=ADC;
		break;
		default:
		break;
	}
	if (ADMUX<0x47)
	{
		ADMUX++;
	} 
	else
	{
		ADMUX=0x40;
	}
	
}

ISR(TIMER2_COMPA_vect)
{
	if (czas<1200)
	{
		czas++;
	}
	else
	{
		TCCR2B &= ~(1<<CS20);					//stop timera jeœli za d³ugo trwa
		czas=500;
	}
}

ISR(TIMER1_COMPA_vect)
{
	if (przeszkoda == 0)
	{
		P_flag=1;
		if (dzielnik==1000)
		{
			USART_Transmit('p');
		}
	}
	else
	{
		P_flag=0;
		czas_jazda++;
		if (dzielnik==1000) USART_Transmit('P');
	}
	if (dzielnik<1000)
	{
		dzielnik++;
	}
	else
	{
		dzielnik=0;
		if (PORTB & (1<<PORTB1)) USART_Transmit('S');
		else USART_Transmit('s');
	}
}

ISR(INT0_vect)
{
	if (trig==1)
	{
		TCCR2B |= (1<<CS20);					//start timera bez prescalera
		trig=2;
	}
	else if (trig==2)
	{
		TCCR2B &= ~(1<<CS20);					//stop timera
		dystans=0;
		dystans=czas/10;
		if (dystans<=24 && czas>40)
		{
			OCR0A = 0;
			OCR0B = 0;
		PORTB |= (1<<PORTB5);
			przeszkoda=1;
		}
		else
		{
			PORTB &= ~(1<<PORTB5);
			przeszkoda=0;
			trig=0;
		}
		czas=0;
	}
}

ISR(USART_RX_vect)
{
	znak = UDR0;
	switch (znak)
	{
		case 'D':
			PORTB |= (1<<PORTB1);
			USART_Transmit('S');
			przeszkoda=0;
			trig=0;
			P_flag=1;
			PORTB &= ~(1<<PORTB5);
			PORTB |= (1<<PORTB3);
			PORTB &= ~(1<<PORTB2);
			PORTD |= (1<<PORTD7);
			PORTB &= ~(1<<PORTB0);
			OCR0A=predkosc;
			OCR0B=predkosc;		
			break;
		case 'd':
			PORTB &= ~(1<<PORTB1);
			USART_Transmit('s');
			break;
		default:
			break;
	}
	
}

void szukaj(void)
{
	TCCR1B &= ~(1<<CS10);		//stop timera do wyznaczania czasu regulatora
	trig=3;

	OCR0A = 0;
	OCR0B = 0;
	czas_linia=0;
	czas_jazda=0;
	//zmiana kierunku prawego silnika
	PORTB |= (1<<PORTB2);
	PORTB &= ~(1<<PORTB3);
	OCR0A = 60;
	OCR0B = 60;
	_delay_us(500);
	OCR0A = 0;
	OCR0B = 0;
	//normalny kierunek obrotu
	PORTB |= (1<<PORTB3);
	PORTB &= ~(1<<PORTB2);
	
	TCCR1B |= (1<<CS10);		//odliczanie czasu od linii do nastêpnego zdarzenia
	OCR0A = 70;
	OCR0B = 70;
	while (lewy3==0 && prawy3==0 && czas_jazda<1500);			//czeka a¿ czas przekroczy czas albo znajdzie liniê
	TCCR1B &= ~(1<<CS10);
	OCR0A = 0;
	OCR0B = 0;
	//zmiana kierunku lewego silnika
	PORTB |= (1<<PORTB0);
	PORTD &= ~(1<<PORTD7);
	czas_linia=czas_jazda;
	czas_jazda=0;
	OCR0A = 60;
	OCR0B = 60;
		
	if (lewy3+prawy3>0)
	{	
		_delay_us(280);			//obraca o 60
	
		if (prawy3+prawy2+prawy1+center<3)
		{
			OCR0A = 0;
			OCR0B = 0;
										//KONIEC OMIJANIA
			trig=0;
			przeszkoda=0;
			USART_Transmit('p');
		}
		
		else
		{
			_delay_us(400);				//dokrêca do 180
		}
	}
	else
	{
		_delay_us(680);					//pe³ny obrót o 180
	}
	OCR0A = 0;
	OCR0B = 0;
	
	
	
	if (przeszkoda!=0)
	{
		//normalny kierunek obrotu
		PORTD |= (1<<PORTD7);
		PORTB &= ~(1<<PORTB0);
		OCR0A = 70;
		OCR0B = 70;
		TCCR1B |= (1<<CS10);
		while (czas_jazda<(czas_linia+500));

		while (lewy3==0 && prawy3==0 && czas_jazda<4000);			//czeka a¿ czas przekroczy albo znajdzie liniê
		OCR0A = 0;
		OCR0B = 0;
		
		//zmiana kierunku prawego silnika
		PORTB |= (1<<PORTB2);
		PORTB &= ~(1<<PORTB3);
		OCR0A = 60;
		OCR0B = 60;
		if (lewy3+prawy3>0)
		{
			_delay_us(280);			//obraca dopóki prawy nie wróci na bia³e
	
			if (lewy3+lewy2+lewy1+center<3)
			{
				OCR0A = 0;
				OCR0B = 0;
				//KONIEC OMIJANIA
				trig=0;
				przeszkoda=0;
				USART_Transmit('p');
			}
			else
			{
				OCR0A = 0;
				OCR0B = 0;
				PORTB &= ~(1<<PORTB1);
				USART_Transmit('s');
			}
		}
		else
		{
			OCR0A = 0;
			OCR0B = 0;
			PORTB &= ~(1<<PORTB1);
			USART_Transmit('s');
		}
	}
	TCCR1B &= ~(1<<CS10);
	if (przeszkoda==0)
	{
		//normalny kierunek obrotu
		PORTB |= (1<<PORTB3);
		PORTB &= ~(1<<PORTB2);
		PORTD |= (1<<PORTD7);
		PORTB &= ~(1<<PORTB0);
		OCR0A=predkosc;
		OCR0B=predkosc;
		PORTB &= ~(1<<PORTB5);
	}
	czas_jazda=0;
	czas_linia=0;
	TCCR1B |= (1<<CS10);			//bo trig p flag
}

void trigger(void)
{
	trig=1;
	sonic_off;
	_delay_us(2);
	sonic_on;
	_delay_us(10);
	sonic_off;
}

void timer2_freq(uint32_t freq)
{
	OCR2A = (F_CPU/(freq*2)-1);				//wzór obl OCR2A z czêstotliwoœci
	TIMSK2 |= (1<<OCIE2A);					//zezwolenie na przerwania od compare a
	TCCR2A |= (1<<WGM21);					//clear on compare, w³¹czyæ w przerw int0
}

void timer1_freq(uint32_t freq)
{
	OCR1A = (F_CPU/(freq*2)-1);				//wzór obl OCR1A z czêstotliwoœci
	TIMSK1 |= (1<<OCIE1A);					//zezwolenie na przerwania od compare a
	TCCR1B |= (1<<WGM12) | (1<<CS10);		//clear on compare, start bez presc
}

void regulator(void)
{
	mianownik = prawy3+prawy2+prawy1+center+lewy1+lewy2+lewy3;	
	if (mianownik != 0 && przeszkoda==0)
	{
		uchyb=(9*prawy3+6*prawy2+3*prawy1-(3*lewy1 + 6*lewy2 + 9*lewy3))/mianownik;
		
		regul = Kp*uchyb;
		if (mianownik==7 && mianownik==mianownik_poprzedni)
		{
			OCR0A = 0;
			OCR0B = 0;
		}
		else
		{
			OCR0A = predkosc-regul;					//silnik prawy
			OCR0B = predkosc+regul;					//silnik lewy
		}
		regul_poprzedni=regul;
	}
	mianownik_poprzedni=mianownik;
}

void int0_init(void)
{
	EIMSK |= (1<<INT0);						//zezwolenie na przerwanie int0
	EICRA |= (1<<ISC00);					//reakcja int 0 na zmianê stanu
}

void pwm_init(void)
{
	TCCR0A |= (1<<COM0A1) | (1<<COM0B1) | (1<<WGM01) | (1<<WGM00);				//timer w trybie fastPWM, oba wyjœcia w tym samym trybie
	TCCR0B |= (1<<CS00);														//start timera
}

void adc_init(void)
{
	ADMUX |= (1<<REFS0);					//zaczyna przetwarzanie od pierwszego kana³u
	ADCSRA |= (1<<ADEN) | (1<<ADSC) | (1<<ADATE) | (1<<ADIE) | (1<<ADPS0) | (1<<ADPS2);
}

void motor_init(void)
{
	
	PORTB |= (1<<PORTB3);					//obrót silników w ró¿ne strony 
	PORTD |= (1<<PORTD7);
}

void USART_Init(unsigned int ubrr)
{
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/*Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = (1<<UCSZ00)|(1<<UCSZ01);
}


int main(void)
{
	DDRD |= (1<<DDD3) | (1<<DDD6) | (1<<DDD5) | (1<<DDD7);								//wyjœcia sygna³u PWM i trig
	DDRB |= (1<<DDB0) | (1<<DDB1) | (1<<DDB2) | (1<<DDB3) | (1<<DDB5);		//wyjœcia na sterowanie kierunkiem obrotu
	DDRD &= ~(1<<DDD2);
	int0_init();
	timer2_freq(1000000);
	timer1_freq(100);
	pwm_init();
	adc_init();
	motor_init();
	USART_Init(MYUBRR);
	sei();
	
    while (1)
	{
		_delay_us(1);
		if ((przeszkoda==1) && (PORTB & (1<<PORTB1)))
		{
			szukaj();
		}
		
		if (P_flag==1)
		{
			regulator();
			P_flag=0;
		}
		
		if (trig==0) trigger();
	}
}