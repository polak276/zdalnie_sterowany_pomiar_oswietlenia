//do³¹czenie bibliotek oraz plików nag³ówkowych

#include <avr/io.h>
#include <util/delay.h>
#include "MK_RFM70/rfm70.h"
#include "LCD/lcd44780.h"

//przyciski steruj¹ce
#define POMIAR_PIN (1<<PC0)
#define W_LEWO_PIN (1<<PC1)
#define W_PRAWO_PIN (1<<PC2)
#define DO_PRZODU_PIN (1<<PC3)
#define DO_TYLU_PIN (1<<PC4)

#define WCISNIETY_POMIAR !(PINC & POMIAR_PIN)
#define WCISNIETY_W_LEWO !(PINC & W_LEWO_PIN)
#define WCISNIETY_W_PRAWO !(PINC & W_PRAWO_PIN)
#define WCISNIETY_DO_PRZODU !(PINC & DO_PRZODU_PIN)
#define WCISNIETY_DO_TYLU !(PINC & DO_TYLU_PIN)

volatile unsigned char rx_tmp[20];
volatile uint8_t len_moje = 0;
int i=0;

int main(void) //funkcja g³ówna main
{
	//piny z przyciskami jako wejœcia
	DDRC &= ~(WCISNIETY_POMIAR | WCISNIETY_W_LEWO | WCISNIETY_W_PRAWO |
			WCISNIETY_DO_PRZODU | WCISNIETY_DO_TYLU);

	//podciagniecie wejsc do 4 rezystorow
	PORTC |=(POMIAR_PIN);
	PORTC |=(W_LEWO_PIN);
	PORTC |=(W_PRAWO_PIN);
	PORTC |=(DO_PRZODU_PIN);
	PORTC |=(DO_TYLU_PIN);

	//funkcje inicjalizacyjne
	lcd_init();
	RFM70_Init();
	SetChannelNum(41);

	//podswietlenie LCD
	DDRA |= (1<<PA7);
	PORTA |= (1<<PA7);

	//wyswietlenie napisu na 2 sekundy i start urz¹dzenia
	lcd_str("Mobilny pojazd");
	lcd_locate(1,0);
	lcd_str("z luksomierzem");
	_delay_ms(2000);

	//zdefiniowanie klepsydry, która symbolizuje czas oczekiwania na pomiar
	uint8_t klepsydra0[] = {31,17,10,4,4,10,17,31}; //klepsydra 1
	uint8_t klepsydra1[] = {31,31,10,4,4,10,17,31}; //klepsydra 2
	uint8_t klepsydra2[] = {31,31,14,4,4,10,17,31}; //klepsydra 3
	uint8_t klepsydra3[] = {31,31,14,4,4,14,17,31}; //klepsydra 5
	uint8_t klepsydra4[] = {31,31,14,4,4,14,31,31}; //klepsydra 6

	uint8_t el[] = {32,14,17,31,16,14,4,2}; //ê
	uint8_t zet[] = {4,32,31,2,4,8,31,32}; //¿
	uint8_t es[] = {4,32,14,16,14,1,30,32}; //œ

	lcd_defchar(0x80, klepsydra0);
	lcd_defchar(0x81, klepsydra1);
	lcd_defchar(0x82, klepsydra2);
	lcd_defchar(0x83, klepsydra3);
	lcd_defchar(0x84, klepsydra4);

	lcd_defchar(0x85, el);
	lcd_defchar(0x86, zet);
	lcd_defchar(0x87, es);

	while(1) //pêtla while
	{
		if(WCISNIETY_POMIAR) //przycisk pomiaru
		{
			_delay_ms(80);
			if(WCISNIETY_POMIAR)
			{
				for(i=0;i<100;i++) //dla pewnosci wysylamy az 100 ramek, zeby miec pewnosc ze dojdzie
				{
				Send_Packet( W_TX_PAYLOAD_NOACK_CMD, "1", 5);
				}
				for(i=0;i<100;i++) //50 razy czekamy na wynik zwrotny
				{
					len_moje = Receive_Packet(rx_tmp); //funkcja do odbioru danych radiowych

					if (len_moje == 5) //jeœli odebrano ramkê 5 bajtow¹
						{
								lcd_cls(); //czyszczenie ekranu
								lcd_str("Oczekiwania na");
								lcd_locate(1,0); //funkcja ustawiania kursora na LCD
								lcd_str("pomiar ");
								lcd_str("\x80");
								_delay_ms(500); //opóŸnienie
								lcd_locate(1,7);
								lcd_str("\x81");
								_delay_ms(500);
								lcd_locate(1,7);
								lcd_str("\x82");
								_delay_ms(500);
								lcd_locate(1,7);
								lcd_str("\x83");
								_delay_ms(500);
								lcd_locate(1,7);
								lcd_str("\x84");
								_delay_ms(500);
								lcd_cls();
								lcd_str("Nat""\x85""\x86""enie o""\x87""w.:");
								lcd_locate(1,0);
								lcd_str(rx_tmp);
								lcd_str(" [lux]");
								_delay_ms(100);
								break;
						}
				}
			}
		}

		if(WCISNIETY_W_LEWO) //przycisk w lewo
		{
			_delay_ms(80);
			if(WCISNIETY_W_LEWO)
			{
				lcd_cls();
				lcd_str("Skret w lewo");
				Send_Packet( W_TX_PAYLOAD_NOACK_CMD, "2", 5);
			}
		}

		if(WCISNIETY_W_PRAWO) //przycisk w prawo
		{
			_delay_ms(80);
			if(WCISNIETY_W_PRAWO)
			{
				lcd_cls();
				lcd_str("Skret w prawo");
				Send_Packet( W_TX_PAYLOAD_NOACK_CMD, "3", 5);
			}
		}

		if(WCISNIETY_DO_PRZODU) //przycisk do przodu
		{
			_delay_ms(80);
			if(WCISNIETY_DO_PRZODU)
			{
				lcd_cls();
				lcd_str("Do przodu");
				Send_Packet( W_TX_PAYLOAD_NOACK_CMD, "4", 5);
			}
		}

		if(WCISNIETY_DO_TYLU) //przycisk do ty³u
		{
			_delay_ms(80);
			if(WCISNIETY_DO_TYLU)
			{
				lcd_cls();
				lcd_str("Do tylu");
				Send_Packet( W_TX_PAYLOAD_NOACK_CMD, "5", 5);
			}
		}
	}
}



