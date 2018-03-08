//funkcja do obs³ugi serwomechanizmu

#include <avr/io.h>

void serwo1 (int wartosc)
{
	DDRD |= (1<<PD5);	//kierunek portu D jako wyjœciowy

	TCCR1A |= (1<<WGM11);//tryb fast PWM ustalany w dwóch rejestrach
	TCCR1B |= (1<<WGM13) | (1<<WGM12);

	TCCR1B |= (1<<CS11);//ustawiamy preskaler na 8, czêstotliwoœæ impulsów w timerze wynosi 1MHz

	//ustawiamy aby wyjœcie OC1A mia³o stan niski i wchodzi³o na wysoki przy porównaniu z OCR1A
	TCCR1A |= (1<<COM1A1) | (1<<COM1A0);

	//ustalamy do jakiej wartoœci ma liczyæ timer 16 bitowy, w ten sposób zmieniamy okres
	//i otrzymamy 50HZ
	ICR1 = 20000;

	//wartoœæ, z któr¹ porównujemy licznik
	OCR1A = ICR1 - wartosc;
}

