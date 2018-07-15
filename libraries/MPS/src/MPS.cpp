#include "MPS.h"

volatile unsigned long milliseconds_timer1 = 0;
volatile unsigned int fmilliseconds_timer1 = 0;
volatile unsigned long timer1 = 0;

void soft_reset() {
  asm volatile ("jmp 0x00\n\t");
}

int dWrite(int pin, bool mode) {
  char SREG_tmp = SREG;
  noInt();
  offPWM(pin);
  if (P_D0_T <= pin && pin <= P_D7_T) {
    if (mode == HIGH) {
      PORTSET(PORTD, pin);
    } else if (mode == LOW) {
      PORTCLEAR(PORTD, pin);
    }
  } else if (P_D8_T <= pin && pin <= P_D13_T) {
    if (mode == HIGH) {
      PORTSET(PORTB, pin - 8);
    } else if (mode == LOW) {
      PORTCLEAR(PORTB, pin - 8);
    }
  } else if (P_A0_T <= pin && pin <= P_A5_T) {
    if (mode == HIGH) {
      PORTSET(PORTC, pin - 14);
    } else if (mode == LOW) {
      PORTCLEAR(PORTC, pin - 14);
    }
  } else {
    SREG = SREG_tmp;
    return -1;
  }
  SREG = SREG_tmp;
  return 0;
}

int dRead(int pin) {
  int level = 0;
  if (P_D0_T <= pin && pin <= P_D7_T) {
    level = PORTCHECK(PIND, pin);
    if (level == 1) {  
      return HIGH;
    } else if (level == 0) {  
      return LOW;
    }
  } else if (P_D8_T <= pin && pin <= P_D13_T) {
    level = PORTCHECK(PINB, pin - 8);
    if (level == 1) {  
      return HIGH;
    } else if (level == 0) {  
      return LOW;
    }
  } else if (P_A0_T <= pin && pin <= P_A5_T) {
    level = PORTCHECK(PINC, pin - 14);
    if (level == 1) {  
      return HIGH;
    } else if (level == 0) {  
      return LOW;
    }
  } else {
    return LOW;
  }
}


int readPortFromPin(int pin) {
  int level = 0;
  if (P_D0_T <= pin && pin <= P_D7_T) {
    return PIND;
  } else if (P_D8_T <= pin && pin <= P_D13_T) {
    return PINB;
  } else if (P_A0_T <= pin && pin <= P_A5_T) {
    return PINC;
  } else {
    return -1;
  }
}

int pMode(int pin, byte mode) {
  char SREG_tmp = SREG;
  noInt();
  if (P_D0_T <= pin && pin <= P_D7_T) {
    if (mode == OUTPUT) {
      PORTSET(DDRD, pin);
    } else if (mode == INPUT) {
      PORTCLEAR(DDRD, pin);
      PORTCLEAR(PORTD, pin);
    } else if (mode == INPUT_PULLUP) {
      PORTCLEAR(DDRD, pin);
      PORTSET(PORTD, pin);
    }
  } else if (P_D8_T <= pin && pin <= P_D13_T) {
    if (mode == OUTPUT) {
      PORTSET(DDRB, pin - 8);
    } else if (mode == INPUT) {
      PORTCLEAR(DDRB, pin - 8);
      PORTCLEAR(PORTB, pin - 8);
    } else if (mode == INPUT_PULLUP) {
      PORTCLEAR(DDRB, pin - 8);
      PORTSET(PORTB, pin - 8);
    }
  } else if (P_A0_T <= pin && pin <= P_A5_T) {
    if (mode == OUTPUT) {
      PORTSET(DDRC, pin - 14);
    } else if (mode == INPUT) {
      PORTCLEAR(DDRC, pin - 14);
      PORTCLEAR(PORTC, pin - 14);
    } else if (mode == INPUT_PULLUP) {
      PORTCLEAR(DDRC, pin - 14);
      PORTSET(PORTC, pin - 14);
    }
  } else {
    SREG = SREG_tmp;
    return -1;
  }
  SREG = SREG_tmp;
  return 0;
}


void check_function() __attribute__((weak));
void check_function() { 
  asm volatile ("nop\n\t");
}
  
int pin_Arduino_Nano_3_0_to_timer(int PIN) { 
  if (PIN == PWM_PIN1) return TIMER2B;
  else if (PIN == PWM_PIN2) return TIMER0B;
  else if (PIN == PWM_PIN3) return TIMER0A;
  else if (PIN == PWM_PIN4) return TIMER1A;
  else if (PIN == PWM_PIN5) return TIMER1B;
  else if (PIN == PWM_PIN6) return TIMER2A;
  else return NOT_ON_TIMER;
}


void onPWM(int pin, int val) {
  int timer = pin_Arduino_Nano_3_0_to_timer(pin);
  pMode(pin, OUTPUT);
  switch (timer) {
    case TIMER2B:
     PORTSET(TCCR2A, COM2B1);
     OCR2B = val;
    break;

    case TIMER0B:
     PORTSET(TCCR0A, COM0B1);
//     PORTSET(TCCR0A, COM0B0);  // inverting mode
     OCR0B = val; 
    break;

    case TIMER0A:
     PORTSET(TCCR0A, COM0A1);
     OCR0A = val; 
    break;

    case TIMER1A:
     PORTSET(TCCR1A, COM1A1);
     OCR1A = val;
    break;

    case TIMER1B:
     PORTSET(TCCR1A, COM1B1);
     OCR1B = val; 
    break;

    case TIMER2A:
     PORTSET(TCCR2A, COM2A1);
     OCR2A = val; 
    break;

    case NOT_ON_TIMER:
    default:
      if (val < 128) {
        dWrite(pin, LOW);
      } else {
        dWrite(pin, HIGH);
      }
    break;
  }
}

void offPWM(int pin) {
  int timer = pin_Arduino_Nano_3_0_to_timer(pin);
  switch (timer) {
    case TIMER2B:
     PORTCLEAR(TCCR2A, COM2B1);
    break;

    case TIMER0B:
     PORTCLEAR(TCCR0A, COM0B1);
    break;

    case TIMER0A:
     PORTCLEAR(TCCR0A, COM0A1);
    break;

    case TIMER1A:
     PORTCLEAR(TCCR1A, COM1A1);
    break;

    case TIMER1B:
     PORTCLEAR(TCCR1A, COM1B1);
    break;

    case TIMER2A:
     PORTCLEAR(TCCR2A, COM2A1);
    break;

    case NOT_ON_TIMER:
    default:
    break;
  }
}

unsigned long milliseconds()
{
  unsigned long lm;
  char SREG_tmp = SREG;

  cli();
  lm = milliseconds_timer1;
  //lm = timer1 * 2;
  SREG = SREG_tmp;

  return lm;
}

void delayMillis(unsigned long int mil) {
  unsigned long int a = milliseconds();
  while (milliseconds() - a <= mil);
}

void INT_init() {
  TIMER1_OVF_EN();
 // TIMER1_COMPA_EN();
}

void PWM_init() {
  TCCR0A = 0; 
  TCCR0B = 0; 
  /*init regime 3 for 0 timer (fast PWM to 0xFF)(table 19-9)*/
  PORTSET(TCCR0A, WGM01);
  PORTSET(TCCR0A, WGM00);
  /* init regime 1  for 0 timer (Phase-correct PWM to 0xFF)
  PORTSET(TCCR0A, WGM00);
  */

  /* Iint prescaler(table 19-10) clk/64 */
  PORTSET(TCCR0B, CS00);
  PORTSET(TCCR0B, CS01);
  /*cs210
    000 - stop timer
    001 - 1:1
    010 - 1:8
    011 - 1:64
    100 - 1:256
    101 - 1:1024
    110 - extern.falling
    111 - extern.rising
  */ 
  /*----------------------------------------------*/
  TCCR1A = 0;
  TCCR1B = 0;
  //TCNT1 = 255; // low barrier
  //ICR1 = 255;// up barrier
  /*init regime 1 for 1 timer (Phase-correct PWM 8-bit to 0xFF)(table 20-6)*/
  PORTSET(TCCR1A, WGM10);

  /*init regime 2 for 1 timer (Phase-correct PWM 9-bit to 0x1FF)(table 20-6)*/
  /*
  PORTSET(TCCR1A, WGM11);
  */

  /*init regime 3 for 1 timer (Phase-correct PWM 10-bit to 0x3FF)(table 20-6)*/
  /*
  PORTSET(TCCR1A, WGM10);
  PORTSET(TCCR1A, WGM11);
  */
  /*init regime 10 for 1 timer (Phase-correct PWM 8-bit to 0x3FF)(table 20-6)*/
  /*
  PORTSET(TCCR1A, WGM11);
  PORTSET(TCCR1B, WGM13)
  */
  //PORTSET(TCCR1B, WGM12);
  /* Iint prescaler(table 19-10) clk/64 */
    PORTSET(TCCR1B, CS10);
    PORTSET(TCCR1B, CS11);

  //PORTSET(TCCR1B, CS10);

  /*cs210
    000 - stop timer
    001 - 1:1
    010 - 1:8
    011 - 1:64
    100 - 1:256
    101 - 1:1024
    110 - extern.falling
    111 - extern.rising
  */ 
  /*----------------------------------------------*/
  TCCR2A = 0;
  TCCR2B = 0;
  /*init regime 1 for 2 timer (Phase-correct PWM 8-bit to 0xFF)(table 22-9)*/
  //PORTSET(TCCR2A, WGM20);

  /*init regime 3 for 2 timer (Fast PWM 8-bit to 0xFF(BOTTOM))(table )*/
  /*
  PORTSET(TCCR2A, WGM20);
  PORTSET(TCCR2A, WGM21);
  */
  
  /*init regime 2 for 2 timer */
  PORTSET(TCCR2A, WGM21);
 
  /* Iint prescaler(table 22-10) clk/64 (For time func) */
  //PORTSET(TCCR2B, CS22);
  
  /* Iint prescaler clk/1 */
  PORTSET(TCCR2B, CS20);

  /*cs210
    000 - stop timer
    001 - 1:1
    010 - 1:8
    011 - 1:32
    100 - 1:64
    101 - 1:128
    110 - 1:256
    111 - 1:1024
  */ 
}

ISR(TIMER1_OVF_vect){
  unsigned long lm = milliseconds_timer1;
  unsigned int lf = fmilliseconds_timer1;
  lm += MILLISECONDS_INCREMENT;
  lf += OVERFLOW_FIX_INCREMENT;
  if (lf >= OWF_MAX) {
    lf -= OWF_MAX;
    ++lm;
  }
  fmilliseconds_timer1 = lf;
  milliseconds_timer1 = lm;
}

ISR(INT0_vect) {
  
}

ISR(INT1_vect) {
  
}

ISR(PCINT2_vect) {
    if (PORTCHECK(PIND, PD0)) {/* Pin D0 interrupt*/}
    if (PORTCHECK(PIND, PD1)) {/* Pin D1 interrupt*/}
    if (PORTCHECK(PIND, PD2)) {/* Pin D2 interrupt*/}
    if (PORTCHECK(PIND, PD3)) {/* Pin D3 interrupt*/}
    if (PORTCHECK(PIND, PD4)) {/* Pin D4 interrupt*/}
    if (PORTCHECK(PIND, PD5)) {/* Pin D5 interrupt*/}
    if (PORTCHECK(PIND, PD6)) {/* Pin D6 interrupt*/}
    if (PORTCHECK(PIND, PD7)) {/* Pin D7 interrupt*/}
}
 
ISR(PCINT0_vect) {
    if (PORTCHECK(PINB, PB0)) {/* Pin D8 interrupt*/}
    if (PORTCHECK(PINB, PB1)) {/* Pin D9 interrupt*/}
    if (PORTCHECK(PINB, PB2)) {/* Pin D10 interrupt*/}
    if (PORTCHECK(PINB, PB3)) {/* Pin D12 interrupt*/}
    if (PORTCHECK(PINB, PB4)) {/* Pin D11 interrupt*/}
    if (PORTCHECK(PINB, PB5)) {/* Pin D13 interrupt*/}
}

ISR(PCINT1_vect) {
    if (PORTCHECK(PINC, PC0)) {/* Pin A0 interrupt*/}
    if (PORTCHECK(PINC, PC1)) {/* Pin A1 interrupt*/}
    if (PORTCHECK(PINC, PC2)) {/* Pin A2 interrupt*/}
    if (PORTCHECK(PINC, PC3)) {/* Pin A3 interrupt*/}
    if (PORTCHECK(PINC, PC4)) {/* Pin A4 interrupt*/}
    if (PORTCHECK(PINC, PC5)) {/* Pin A5 interrupt*/}
}
