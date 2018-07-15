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

#define INT0_PIN P_D2_T
#define INT1_PIN P_D3_T

#define PORTSET(PORT, BIT) (PORT =((PORT) | (1 << (BIT))))
#define PORTCLEAR(PORT, BIT) (PORT = ((PORT) & ~(1 << (BIT))))
#define PORTCHECK(PORT, BIT) (((PORT) >> BIT) & 1)

#define noInt() PORTCLEAR(SREG, 7)
#define yesInt() PORTSET(SREG, 7)

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


#define readPortB() PINB
#define readPortD() PIND
#define readPortC() PINC
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

#define SEC_TO_MILSEC(X) (X * 1000)

void del(unsigned long int time) {
  unsigned long int del = micros();
  while (time > 0) {
    yield();
    while (time > 0) {
      if (micros() - del >= 1000) {
        time--;
        del +=1000;
      }
      check_function();
    }
  }
}
  
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

volatile int u = 0;
volatile int u1 = 0;

ISR(INT0_vect) {
  u1 = ~u1;
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
    if (PORTCHECK(PINB, PB0)) {u = ~u;/* Pin D8 interrupt*/}
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

void INT_init() {
 // TIMER1_OVF_EN();
  TIMER1_COMPA_EN();
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
  //PORTSET(TCCR1A, WGM10);

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
  PORTSET(TCCR1B, WGM13);
  ICR = 639; //MAX VALUE
  */
  PORTSET(TCCR1B, WGM12);
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

volatile unsigned long int trigger_global = 0;
/*Timer 2, pin 11*/
void toneF(unsigned int freq, unsigned long int tm) {
  unsigned char presc = 0x1;
  unsigned long int freqForPresc = F_CPU / freq / 2 - 1;
  unsigned long int trigger = -1;
  if (freqForPresc > 255) { // 1
    freqForPresc = F_CPU / freq / 2 / 8 - 1;
    if (freqForPresc  > 255) { // 8
      presc = 0x2;
      freqForPresc = F_CPU / freq / 2 / 32 - 1;
      if (freqForPresc  > 255) { // 32
        presc = 0x3;
        freqForPresc = F_CPU / freq / 2 / 64 - 1;
        if (freqForPresc  > 255) { // 64
          presc = 0x4;
          freqForPresc = F_CPU / freq / 2 / 128 - 1;
          if (freqForPresc  > 255) { // 128
            presc = 0x5;
            freqForPresc = F_CPU / freq / 2 / 256 - 1;
            if (freqForPresc  > 255) { // 256
              presc = 0x6;
              freqForPresc = F_CPU / freq / 2 / 1024 - 1; 
              if (freqForPresc  > 255) { // 1024
                presc = 0x7;
              }
            }
          }
        }
      }
    }
  }
  if (tm > 0) {
    trigger = 2 * freq * (tm) / 1000; //millis
  }
  TCCR2B = TCCR2B & 0b11111000;
  TCCR2B = TCCR2B | presc;
  OCR2A = freqForPresc ;
  trigger_global = trigger;
  unsigned long int starttime = millis();
  Serial.println("--------------------");
  Serial.println(freq);
  Serial.println(presc);
  Serial.println(freqForPresc);
  Serial.println(trigger_global);
  TIMER2_COMPA_EN();
  while (millis() - starttime < tm + 5);
}
volatile int mask = 0b00001000;
ISR(TIMER2_COMPA_vect) {
  if (trigger_global > 0) {
    // toggle the pin
    PORTB ^= mask;
    trigger_global--;
  } else if (trigger_global-- == 0){
    PORTCLEAR(PORTB, 4);
    TIMER2_COMPA_DIS();
  }
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

#define FREQ_CORE F_CPU
// Count of 1 microseconds ~1/16microsec
#define FREQ_PER_MS() ( FREQ_CORE / 1000000UL )
// Overflow Timing in microseconds
#define FREQ_TO_MS(X) ( (X) / FREQ_PER_MS() )
// Count to overflow
#define COUNT_TIMER1 ( 256 )
#define COUNT_TIMER ( 240 )

// 1.024 milliseconds to overflow
#define MS_TIMER1_OVERFLOW (FREQ_TO_MS(64 * COUNT_TIMER1 )) 

#define MILLISECONDS_INCREMENT (MS_TIMER1_OVERFLOW * 2/ 1000)

// 0.024 - eps. If 1000 microseconds -> 1111101000 >> 3 = 125. 1 byte
#define OVERFLOW_FIX_INCREMENT ((MS_TIMER1_OVERFLOW * 2 % 1000) >> 3)
#define OWF_MAX (1000 >> 3) 

volatile unsigned long milliseconds_timer1 = 0;
volatile unsigned int fmilliseconds_timer1 = 0;
volatile unsigned long timer1 = 0;

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

ISR(TIMER1_COMPA_vect){
  OCR1A=COUNT_TIMER - 1;
  ++timer1;
}

unsigned long milliseconds()
{
  unsigned long lm;
  char SREG_tmp = SREG;

  cli();
  //lm = milliseconds_timer1;
  lm = timer1 * 2;
  SREG = SREG_tmp;

  return lm;
}

void delayMillis(unsigned long int mil) {
  unsigned long int a = milliseconds();
  while (milliseconds() - a <= mil);
}

void test_dWrite_pMode() {
  int i;
  for (i = P_D2_T; i <= P_D13_T; ++i) {
    pMode(i,OUTPUT);
  }
  
  for (i = P_A0_T; i <= P_A5_T; ++i){
    pMode(i,OUTPUT);
  }

  for (i = P_D2_T; i <= P_D13_T; ++i) {
    dWrite(i, HIGH);
    del(200);
  }

  for (i = P_A0_T; i <= P_A5_T; ++i){
    dWrite(i, HIGH);
    del(200);
  }

  for (i = P_D2_T; i <= P_D13_T; ++i) {
    dWrite(i, LOW);
    del(200);
  }
  
  for (i = P_A0_T; i <= P_A5_T; ++i){
    dWrite(i, LOW);
    del(200);
  }
}

void test_ISR_012() {
  int i;
  for (i = P_D2_T; i <= P_D13_T; ++i) {
    pMode(i,OUTPUT);
  }
  
  for (i = P_A0_T; i <= P_A5_T; ++i){
    pMode(i,OUTPUT);
  }

  for (i = P_D2_T; i <= P_D13_T; ++i) {
    dWrite(i, LOW);
  }
  
  for (i = P_A0_T; i <= P_A5_T; ++i){
    dWrite(i, LOW);
  }

  PCINT0_EN(0b000001);
  while(u == 0);
  PCINT0_DIS();
  u = 0;
  dWrite(13, HIGH);
}


void test_PWM() {
 int i = 0;
 while (i <= 255){ onPWM(P_D3_T, i); del(10);++i;}
 i = 0;
 while (i <= 255){ onPWM(P_D5_T, i); del(10);++i;}
 i = 0;
 while (i <= 255){ onPWM(P_D6_T, i); del(10);++i;}
 i = 0;
 while (i <= 255){ onPWM(P_D9_T, i); del(10);++i;}
 i = 0;
 while (i <= 255){ onPWM(P_D10_T, i); del(10);++i;}
 i = 0;
 while (i <= 255){ onPWM(P_D11_T, i); del(10);++i;}
 
 i = 255;
 while (i >= 0){ onPWM(P_D3_T, i); del(10);--i;}
 i = 255;
 while (i >= 0){ onPWM(P_D5_T, i); del(10);--i;}
 i = 255;
 while (i >= 0){ onPWM(P_D6_T, i); del(10);--i;}
 i = 0XFF;
 while (i >= 0){ onPWM(P_D9_T, i); del(10);--i;}
 i = 0XFF;
 while (i >= 0){ onPWM(P_D10_T, i); del(10);--i;}
 i = 255;
 while (i >= 0){ onPWM(P_D11_T, i); del(10);--i;}
}


void test_dRead_portRead() {
  int i = 0;
  for (i = P_D2_T; i <= P_D13_T; ++i) {
    pMode(i, INPUT);
  }
  while(1) {
    Serial.println("2 3 4 5 6 7 8 9 0 1 2 3");
    for (i = P_D2_T; i <= P_D13_T; ++i) {
      Serial.print(dRead(i));
      Serial.print(" ");
    } 
    Serial.print("\n");
    Serial.print(readPortD());
    Serial.print(" ");
    Serial.print(readPortB());
    Serial.print("\n");
    Serial.print(readPortFromPin(0));
    Serial.print(" ");
    Serial.print(readPortFromPin(10));
    Serial.print("\n");
    Serial.print(PIND);
    Serial.print(" ");
    Serial.print(PINB);
    Serial.print("\n");
    Serial.println("-------------------------");
    del(1000);
    if (dRead(P_D13_T) == HIGH) break;
  }
}

void test_ISR_int01() {
  pMode(INT0_PIN, INPUT);
  pMode(P_D13_T, OUTPUT);

  dWrite(P_D13_T, LOW);
  INT0_EN_LOG();    
  while(u1 == 0);
  dWrite(P_D13_T, HIGH);
  u1 = 0;
  del(1000);
  INT0_DIS();
}

#define COUNT_ROW 3
#define COUNT_COL 2
#define STARTKEYW P_D6_T  
#define ENDKEYW P_D8_T  
#define STARTKEYR P_D4_T  
#define ENDKEYR P_D5_T  
void test_matrix_keyboard() {
  int i = 0;
  int j = 0;
  int readval = 0;
  
  for (i = STARTKEYR; i <= ENDKEYR ;++i) {
    pMode(i, INPUT);
  }

  for (i = STARTKEYW; i <= ENDKEYW ;++i) {
    pMode(i, OUTPUT);
  }

  while(1) {
    for (i = 0; i < COUNT_ROW; ++i) {
      for (j = 0; j < COUNT_ROW; ++j) {
        if (i == j) dWrite(STARTKEYW + j, LOW);
        else dWrite(STARTKEYW + j, HIGH);
      }
      for (j = 0; j < COUNT_COL; ++j) {
        readval = dRead(STARTKEYR + j);
        if (readval == LOW) {
          Serial.println(i*(COUNT_ROW - 1) + j);
          //toneF(440, 10000);
        }
      }
    }
  }
}

#define TACT 1000
void setup() {
  int  i = 0;
  yesInt();
  PWM_init();
  INT_init(); 
  pMode(P_D3_T,OUTPUT);
  pMode(P_D5_T,OUTPUT);
  pMode(P_D6_T,OUTPUT);
  pMode(P_D9_T,OUTPUT);
  pMode(P_D10_T,OUTPUT);
  pMode(P_D11_T,OUTPUT);
  
  char buf[80];
  Serial.begin(9600);

  delay(500);
  Serial.println(timer1);
  Serial.println(milliseconds_timer1);
  Serial.println(milliseconds());
  Serial.println(millis());
  /*
  toneF(587, TACT * 3 / 4);
  toneF(659, TACT / 4);
  
  toneF(698, TACT);

  toneF(880, TACT * 3 / 8);
  toneF(784, TACT * 3 / 8);
  toneF(880, TACT / 4);
  
  toneF(523, TACT);

  toneF(587, TACT * 3 / 4);
  toneF(659, TACT / 4);

  toneF(698, TACT / 2);
  toneF(659, TACT / 2);

  toneF(784, TACT / 2);
  toneF(880, TACT / 2);
  
  toneF(784, TACT / 2);
  toneF(698, TACT / 2);

  del(TACT / 4);
  toneF(698, TACT / 4);
  toneF(698, TACT / 4);
  toneF(698, TACT / 4);
  
  toneF(880, TACT / 4);
  toneF(880, TACT / 4);
  toneF(784, TACT / 4);
  toneF(698, TACT / 4);

  del(TACT / 4);
  toneF(880, TACT / 4);
  toneF(880, TACT / 4);
  toneF(880, TACT / 4);
  
  toneF(784, TACT / 4);
  toneF(880, TACT / 4);
  toneF(784, TACT / 4);
  toneF(698, TACT / 4);
  
  del(TACT / 4);
  toneF(698, TACT / 4);
  toneF(698, TACT / 4);
  toneF(698, TACT / 4);
  
  toneF(880, TACT / 4);
  toneF(880, TACT / 4);
  toneF(784, TACT / 4);
  toneF(698, TACT / 4);

  del(TACT / 4);
  toneF(880, TACT / 4);
  toneF(880, TACT / 4);
  toneF(880, TACT / 4);*/
  /*
  del(TACT / 4);
  toneF(1108, TACT / 4);
  toneF(1108, TACT / 4);
  toneF(1108, TACT / 4);
/*
  del(TACT / 4);
  toneF(174, TACT / 4);
  toneF(174, TACT / 4);
  toneF(174, TACT / 4);
  
  toneF(220, TACT / 4);
  toneF(220, TACT / 4);
  toneF(196, TACT / 4);
  toneF(174, TACT / 4);
  */ 

//  delayMillis(500);
//  delayMillis(5000);
//  Serial.println(milliseconds());
//  Serial.println(millis());
/*
  pMode(P_D13_T,OUTPUT);
  dWrite(P_D13_T, HIGH);
  delayMillis(1000);
  dWrite(P_D13_T, LOW);
  delayMillis(1000);
  dWrite(P_D13_T, HIGH);
  delayMillis(1000);
  dWrite(P_D13_T, LOW);
  delayMillis(1000);
*/
//  test_dWrite_pMode();
//  test_ISR_012();;
//  test_PWM();
//  test_dRead_portRead();
 // test_matrix_keyboard();
}

void loop() {  

  test_ISR_int01();
//  Serial.println(trigger_global);
}
