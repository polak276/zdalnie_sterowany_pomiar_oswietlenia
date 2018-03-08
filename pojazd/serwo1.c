//funkcja do obs�ugi serwomechanizmu

#include <avr/io.h>

void serwo1 (int wartosc)
{
	DDRD |= (1<<PD5);	//kierunek portu D jako wyj�ciowy

	TCCR1A |= (1<<WGM11);//tryb fast PWM ustalany w dw�ch rejestrach
	TCCR1B |= (1<<WGM13) | (1<<WGM12);

	TCCR1B |= (1<<CS11);//ustawiamy preskaler na 8, cz�stotliwo�� impuls�w w timerze wynosi 1MHz

	//ustawiamy aby wyj�cie OC1A mia�o stan niski i wchodzi�o na wysoki przy por�wnaniu z OCR1A
	TCCR1A |= (1<<COM1A1) | (1<<COM1A0);

	//ustalamy do jakiej warto�ci ma liczy� timer 16 bitowy, w ten spos�b zmieniamy okres
	//i otrzymamy 50HZ
	ICR1 = 20000;

	//warto��, z kt�r� por�wnujemy licznik
	OCR1A = ICR1 - wartosc;
}

