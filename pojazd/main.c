#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include "MK_RFM70/rfm70.h"
#include "I2C_TWI/i2c_twi.h"
#include "LCD/lcd44780.h"
#include "serwo1.h"

#define ALS 0x70 //adres SFH7773

typedef unsigned char  u08;
typedef unsigned short u16;

volatile unsigned char rx_tmp[20];
volatile uint8_t len_moje = 0;

#define LED_PIN (1<<PB0) //led
#define LED_ON_TEST PORTB &= ~LED_PIN;
#define LED_OFF_TEST PORTB |= LED_PIN;

//piny obs³ugi silnika
#define WE_A PD1
#define WE_B PD0
#define RUCH_W_LEWO PORTD &= ~(1<<WE_A); PORTD |= (1<<WE_B)
#define RUCH_W_PRAWO PORTD |= (1<<WE_A); PORTD &= ~(1<<WE_B)
#define STOP PORTD &= ~(1<<WE_A); PORTD &= ~(1<<WE_B)

//funkcja main
int main(void)
{
	uint8_t bufor[2];		// bufor do zczytywania danych z I2C
	uint16_t suma=0;
	char suma_w[2];
	uint8_t aktywuj_als=0x33;
	int i=0;

	i2cSetBitrate(100);	// prêdkoœæ I2C na 100kHz

	TWI_write_buf(0x70, 0x80, 1, &aktywuj_als);	//ustawienie SFH7773 w tryb FREE-RUNNING MODE
	_delay_ms(110);

	DDRD |= ((1<<WE_A) | (1<<WE_B));

	int kat_serwa=1600; //polozenie srodkowe serwa

	RFM70_Init();	//init RFM73
    SetChannelNum(41);

    DDRB |= (1<<PB0);
    PORTB |= (1<<PB0);

    while(1)
    {
    	//urz¹dzenie ci¹gle odbiera dane
        len_moje = Receive_Packet(&rx_tmp);

        if (len_moje == 5)
			{
				if(strcmp("1",&rx_tmp)==0) //gdy nadszed³ rozkaz odes³ania pomiaru
				{
					TWI_read_buf(0x70, 0x8C, 2,bufor);
					suma = ((bufor[1]<<8) | bufor[0]);
					STOP;
					utoa(suma/2,suma_w, 10);

					for(i=0;i<50;i++)
					{
						Send_Packet( W_TX_PAYLOAD_NOACK_CMD, suma_w, 5);
						LED_ON_TEST; //zapalenie diodê
					}

					_delay_ms(100);
					LED_OFF_TEST;
				}

				if(strcmp("2",&rx_tmp)==0) //ruch serwomechanizmu w prawo
				{
					serwo1(kat_serwa-50);
					if(kat_serwa>1300) //ograniczenie maksymalnego skrêtu serwa w prawo
					kat_serwa=kat_serwa-50; //aktualna wartoœæ ustawienia serwa
				}

				if(strcmp("3",&rx_tmp)==0) //ruch serwomechanizmu w lewo
				{
					serwo1(kat_serwa+50);
					if(kat_serwa<1900) //ograniczenie maksymalnego skrêtu serwa w lewo
					kat_serwa=kat_serwa+50;
				}

				if(strcmp("4",&rx_tmp)==0)
				{
					RUCH_W_LEWO;
					_delay_ms(100);
					STOP;
				}

				if(strcmp("5",&rx_tmp)==0)
				{
					RUCH_W_PRAWO;
					_delay_ms(100);
					STOP;
				}
			}

    }
}





