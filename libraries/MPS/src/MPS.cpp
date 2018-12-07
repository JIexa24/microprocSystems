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
    yesInt();
    return -1;
  }
  yesInt();
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
    yesInt();
    return -1;
  }
  yesInt();
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

void delayMillis(unsigned long int mil) {
  unsigned long int a = millis();
  while (millis() - a <= mil);
}

