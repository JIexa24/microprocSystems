#ifndef MPS_H
#define MPS_H

#include <Arduino.h>
#include <BINARY.h>
#include <util/twi.h>
/* Pins */
#define P_D0 0
#define P_D1 1
#define P_D2 2
#define P_D3 3
#define P_D4 4
#define P_D5 5
#define P_D6 6
#define P_D7 7
#define P_D8 8
#define P_D9 9
#define P_D10 10
#define P_D11 11
#define P_D12 12
#define P_D13 13

#define P_D0_T 0
#define P_D1_T 1
#define P_D2_T 2
#define P_D3_T 3
#define P_D4_T 4
#define P_D5_T 5
#define P_D6_T 6
#define P_D7_T 7
#define P_D8_T 8
#define P_D9_T 9
#define P_D10_T 10
#define P_D11_T 11
#define P_D12_T 12
#define P_D13_T 13

#define P_A0 0
#define P_A1 1
#define P_A2 2
#define P_A3 3
#define P_A4 4
#define P_A5 5
#define P_A6 6
#define P_A7 7

#define P_A0_T 14
#define P_A1_T 15
#define P_A2_T 16
#define P_A3_T 17
#define P_A4_T 18
#define P_A5_T 19
#define P_A6_T 20
#define P_A7_T 21

#define TX_PIN P_D0
#define RX_PIN P_D1
#define LED_PIN P_D13

#define PWM_PIN1 P_D3_T
#define PWM_PIN2 P_D5_T
#define PWM_PIN3 P_D6_T
#define PWM_PIN4 P_D9_T
#define PWM_PIN5 P_D10_T
#define PWM_PIN6 P_D11_T

#define RESET_PIN 6

#define SDA_PIN P_A4_T
#define SCL_PIN P_A5_T

#define INT0_PIN P_D2_T
#define INT1_PIN P_D3_T

#define I2C_READY 0
#define I2C_MRX   1
#define I2C_MTX   2
#define I2C_SRX   3
#define I2C_STX   4
#define I2C_STATUS  (TWSR & 0xF8)

/*LCD*/
#define LEFT 0
#define RIGHT 1
#define DS_ON 1
#define DS_OFF 0
#define DS_VISIBLE 1
#define DS_BLINK 1
#define DS_NOBLINK 0
#define DS_HIDE 0
#define SH_SYMBOLS 1
#define SH_CURSOR 0
#define SH_SHIFT_NO 0
#define SH_SHIFT_YES 1
#define WM_8BIT 1
#define WM_4BIT 0
#define WM_2LINE 1
#define WM_1LINE 0
#define WM_5X10 1
#define WM_5X8 0
#define LCD_COMMAND_CLR() (0x1)
#define LCD_COMMAND_R() (0x1 << 1)
#define LCD_COMMAND_SHIFT_MODE(ID,SHIFTSYM) ((0x1 << 2) | (ID << 1) | SHIFTSYM)
#define LCD_COMMAND_DISPLAY(DISPLAY,CURSOR,BLINK) ((0x1 << 3) | (DISPLAY << 2) | (CURSOR << 1) | BLINK)
#define LCD_COMMAND_SHIFT(SH_PARAM,VECT) ((0x1 << 4) | (SH_PARAM << 3) | ((VECT) << 2))
#define LCD_COMMAND_WORK_MODE(DL,LINES,SYM_TYPE) ((0x1 << 5) | (DL << 4) | (LINES << 3) | (SYM_TYPE << 2))
#define LCD_COMMAND_SET_CURSOR(ADDR) ((0x1 << 7) | ADDR)

/*Makros*/
#define PORTSET(PORT, BIT) (PORT =((PORT) | (1 << (BIT))))
#define PORTCLEAR(PORT, BIT) (PORT = ((PORT) & ~(1 << (BIT))))
#define PORTCHECK(PORT, BIT) (((PORT) >> BIT) & 1)

#define noInt() PORTCLEAR(SREG, 7)
#define yesInt() PORTSET(SREG, 7)


#define readPortB() PINB
#define readPortD() PIND
#define readPortC() PINC

#define SEC_TO_MILSEC(X) (X * 1000)

//page 82 - int vectors
//PCICR - pin change interrupt control register(page 92)
/* PCINT0 - PCINT7 (0 - 7 PB) */
#define PCINT0_EN(X) { PORTSET(PCICR, 0); \
                       PCMSK0 = X & 0x1F; }
/* PCINT8 - PCINT14 (0 - 5 PC) */
#define PCINT1_EN(X) { PORTSET(PCICR, 1); \
                       PCMSK1 = X & 0x1F; }
/* PCINT16 - PCINT23 (0 - 7 PD) */
#define PCINT2_EN(X) { PORTSET(PCICR, 2); \
                       PCMSK2 = X & 0xFF; }
#define PCINT0_DIS() PORTCLEAR(PCICR, 0)
#define PCINT1_DIS() PORTCLEAR(PCICR, 1)
#define PCINT2_DIS() PORTCLEAR(PCICR, 2)

//ECICR - external interrupt control register(17.2.1)

#define INT0_EN_LOW() { PORTCLEAR(EICRA, 0); \
                        PORTCLEAR(EICRA, 1); \
                        PORTSET(EIMSK, 0);   }
#define INT1_EN_LOW() { PORTCLEAR(EICRA, 2); \
                        PORTCLEAR(EICRA, 3); \
                        PORTSET(EICRA, 1);   }

#define INT0_EN_LOG() { PORTSET(EICRA, 0);   \
                        PORTCLEAR(EICRA, 1); \
                        PORTSET(EIMSK, 0);   }
#define INT1_EN_LOG() { PORTSET(EICRA, 2);   \
                        PORTCLEAR(EICRA, 3); \
                        PORTSET(EIMSK, 1);   }

#define INT0_EN_FALL() { PORTCLEAR(EICRA, 0); \
                         PORTSET(EICRA, 1);   \
                         PORTSET(EIMSK, 0);   }
#define INT1_EN_FALL() { PORTCLEAR(EICRA, 2); \
                         PORTSET(EICRA, 3);   \
                         PORTSET(EIMSK, 1);   }


#define INT0_EN_RISE() { PORTSET(EICRA, 0); \
                         PORTSET(EICRA, 1); \
                         PORTSET(EIMSK, 0); }
#define INT1_EN_RISE() { PORTSET(EICRA, 2); \
                         PORTSET(EICRA, 3); \
                         PORTSET(EIMSK, 1); }

#define INT0_DIS() PORTCLEAR(EIMSK, 0)
#define INT1_DIS() PORTCLEAR(EIMSK, 1)


#define TIMER1_OVF_EN() { PORTSET(TIMSK1, 0); }
#define TIMER1_OVF_DIS() PORTCLEAR(TIMSK1, 0)

#define TIMER1_COMPA_EN() { PORTSET(TIMSK1, 1); }
#define TIMER1_COMPA_DIS() PORTCLEAR(TIMSK1, 1)

#define TIMER2_COMPA_EN() { PORTSET(TIMSK2, 1); }
#define TIMER2_COMPA_DIS() PORTCLEAR(TIMSK2, 1)

#define FREQ_CORE F_CPU
// Count of 1 microseconds ~1/16microsec
#define FREQ_PER_MS() (FREQ_CORE / 1000000UL)
// Overflow Timing in microseconds
#define FREQ_TO_MS(X) ((X) / FREQ_PER_MS())
// Count to overflow
#define COUNT_TIMER0 (256)

// 1.024 milliseconds to overflow
#define MS_TIMER1_OVERFLOW (FREQ_TO_MS(64 * COUNT_TIMER1 )) 

#define MILLISECONDS_INCREMENT (MS_TIMER1_OVERFLOW * 2/ 1000)

// 0.024 - eps. If 1000 microseconds -> 1111101000 >> 3 = 125. 1 byte
#define OVERFLOW_FIX_INCREMENT ((MS_TIMER1_OVERFLOW * 2 % 1000) >> 3)
#define OWF_MAX (1000 >> 3) 


void soft_reset();
int dWrite(int pin, bool mode);
int dRead(int pin);
int readPortFromPin(int pin);
int pMode(int pin, byte mode);
int pin_Arduino_Nano_3_0_to_timer(int PIN);
void onPWM(int pin, int val);
void offPWM(int pin);
void INT_init();
void PWM_init();
unsigned long milliseconds();
void delayMillis(unsigned long int mil);

#endif
