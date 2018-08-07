#include <MPS.h>

#define CLK_PIN_I2C P_D11
#define DATA_PIN_I2C P_D12
#define SIGNAL_PIN_I2C P_D2
#define TRANSMIT_PIN_I2C P_D3
#define E_PIN_LCD P_D10
#define RS_PIN_LCD P_D9
#define DATA_PIN_SH P_D8
#define CLK_PIN_SH P_D7
#define LATCH_PIN_SH P_D6
#define RANGE 50

char master_msg[10] = "Question?";
char slave_msg[8] = "Answer!";

void master_i2c() {
  int transmit_msg = 0;
  int recive_msg = 0;
  int recive = 0;
  int transmit = 0;
  int index_char = 0;
  int index_bit = 0;
  int start_bit = 0;
  int stop_bit = 0;
  int reading = 0;
  char rbuf[256] = "";
  char tmp = 0;
  pMode(CLK_PIN_I2C, OUTPUT);
  
  dWrite(SIGNAL_PIN_I2C, HIGH);
  dWrite(CLK_PIN_I2C, LOW);
  dWrite(DATA_PIN_I2C, HIGH);
  
  while (1) {
    if (dRead(TRANSMIT_PIN_I2C) == HIGH && recive_msg == 0 && transmit_msg == 0) {
      Serial.println("Sending");
      transmit_msg = 1;
      transmit = 1;
      pMode(DATA_PIN_I2C, OUTPUT);
    } else if (dRead(DATA_PIN_I2C) == LOW && recive_msg == 0 && transmit_msg == 0) {
      Serial.println("Reciving");
      tmp = 0;
      recive = 1;
      recive_msg = 1;
      reading = 1;
    } else if (dRead(DATA_PIN_I2C) == LOW && recive_msg == 1 && recive == 0) {
      reading = 1;
      recive = 1;
    }
    
    if (transmit_msg == 1) {
      transmit = 1;
      pMode(DATA_PIN_I2C, OUTPUT);
    }
    if (transmit == 1) {
      if (start_bit == 0) {
        dWrite(DATA_PIN_I2C, LOW);
        start_bit = 1;
      } else {
        if (index_bit < 8) {
          dWrite(DATA_PIN_I2C, (master_msg[index_char] >> index_bit) & 1);
          index_bit++;
          Serial.print(index_bit);
          Serial.print("=");
          Serial.print((master_msg[index_char - 1] >> index_bit) & 1);
          Serial.print(" ");
        } else { 
          index_char = index_char + 1;
          Serial.print(master_msg[index_char - 1]);
          Serial.print("(");
          Serial.print((int)master_msg[index_char - 1]);
          Serial.println(")");
          if (master_msg[index_char - 1] == '\0') {
            index_char = 0;
            transmit_msg = 0;
          }
          index_bit = 0;
          //Send stop-bit
          dWrite(DATA_PIN_I2C, HIGH);
          pMode(DATA_PIN_I2C, INPUT_PULLUP);
          stop_bit = 0;
          start_bit = 0;
          transmit = 0;
        }
      }
    }
    
    dWrite(CLK_PIN_I2C, HIGH);

    if (recive == 1) {
      if (reading == 0) {
        reading = 1;
        if (index_bit < 8) {
          tmp = tmp | (dRead(DATA_PIN_I2C) << index_bit);
          index_bit++;
          Serial.print(index_bit);
          Serial.print("=");
          Serial.print(dRead(DATA_PIN_I2C));
          Serial.print(" ");
        } else {
          stop_bit = dRead(DATA_PIN_I2C);
          if (stop_bit == HIGH) {
            rbuf[index_char] = tmp;
            index_char = index_char + 1;
            if (tmp == 0 || tmp == 255) {
              index_char = 0;
              recive_msg = 0;
            }
            Serial.print(tmp);
            Serial.print("(");
            Serial.print((int)tmp);
            Serial.println(")");
            tmp = 0;
            index_bit = 0;
            recive = 0;
          } else {
            Serial.println("Error stop-bit recive. Rewrite last bit");
            index_bit--;
          }
        }
      }
    }
    
    delay(RANGE);
    dWrite(CLK_PIN_I2C, LOW);
    if (reading == 1) reading = 0;
    delay(RANGE);
  }
}

void slave_i2c() {
  int transmit_msg = 0;
  int recive_msg = 0;
  int recive = 0;
  int transmit = 0;
  int index_char = 0;
  int index_bit = 0;
  int start_bit = 0;
  int stop_bit = 0;
  int reading = 0;
  int writing = 0;
  char rbuf[256] = "";
  char tmp = '\0';
  
  dWrite(SIGNAL_PIN_I2C, LOW);

  while (1) {
    if (dRead(TRANSMIT_PIN_I2C) == HIGH && recive_msg == 0 && transmit_msg == 0) {
      Serial.println("Sending");
      lcd1602_sendstr("Sending", 7, 1, 0);
      transmit = 1;
      transmit_msg = 1;
    } else if (dRead(DATA_PIN_I2C) == LOW && dRead(CLK_PIN_I2C) == HIGH && recive_msg == 0 && transmit_msg == 0) {
      Serial.println("Reciving");
      lcd1602_sendstr("Reciving", 8, 1, 0);
      tmp = 0;
      recive = 1;
      recive_msg = 1;
      reading = 1;
    } else if (dRead(DATA_PIN_I2C) == LOW && dRead(CLK_PIN_I2C) == HIGH && recive_msg == 1 && recive == 0) {
      reading = 1;
      recive = 1;
    }
    
    if (transmit_msg == 1) {
      transmit = 1;
      pMode(DATA_PIN_I2C, OUTPUT);
    }
    if (dRead(CLK_PIN_I2C) == LOW && transmit == 1 && writing == 0) {
      writing = 1;
      if (start_bit == 0) {
        dWrite(DATA_PIN_I2C, LOW);
        start_bit = 1;
      } else {
        if (index_bit < 8) {
          dWrite(DATA_PIN_I2C, (slave_msg[index_char] >> index_bit) & 1);
          index_bit++;
          Serial.print(index_bit);
          Serial.print("=");
          Serial.print((slave_msg[index_char] >> (index_bit - 1)) & 1);
          Serial.print(" ");
        } else { 
          index_char = index_char + 1;
          Serial.print(slave_msg[index_char - 1]);
          Serial.print("(");
          Serial.print((int)slave_msg[index_char - 1]);
          Serial.println(")");
          if (slave_msg[index_char - 1] == '\0') {
            index_char = 0;
            transmit_msg = 0;
          } else 
            lcd1602_sendstr(&slave_msg[index_char - 1], 1, 2, index_char);
          index_bit = 0;
          dWrite(DATA_PIN_I2C, HIGH);
          pMode(DATA_PIN_I2C, INPUT_PULLUP);
          stop_bit = 0;
          start_bit = 0;
          transmit = 0;
        }
      }
    } 
    if (dRead(CLK_PIN_I2C) == HIGH && writing == 1) writing = 0;
    
    if (recive == 1) {
      if (dRead(CLK_PIN_I2C) == HIGH && reading == 0) {
        reading = 1;
        if (index_bit < 8) {
          tmp = tmp | (dRead(DATA_PIN_I2C) << index_bit);
          index_bit++;
          Serial.print(index_bit);
          Serial.print("=");
          Serial.print(dRead(DATA_PIN_I2C));
          Serial.print(" ");
        } else {
          stop_bit = dRead(DATA_PIN_I2C);
          if (stop_bit == HIGH) {
            rbuf[index_char] = tmp;
            index_char = index_char + 1;
            if (tmp == 0 || tmp == 255) {
              index_char = 0;
              recive_msg = 0;
            }
            Serial.print(tmp);
            Serial.print("(");
            Serial.print((int)tmp);
            Serial.println(")"); 
            if (tmp != '\0')
              lcd1602_sendstr(&rbuf[index_char - 1], 1, 2, index_char);
            tmp = 0;
            index_bit = 0;
            recive = 0;
          } else {
            Serial.println("Error stop-bit recive. Rewrite last bit");
            index_bit--;
          }
        }
      }
      if (dRead(CLK_PIN_I2C) == LOW && reading == 1) reading = 0;
    }
    
  }
}

void data_shift595(byte data) {
  shiftOut(DATA_PIN_SH, CLK_PIN_SH, MSBFIRST, data);

  digitalWrite(LATCH_PIN_SH, LOW);
  digitalWrite(LATCH_PIN_SH, HIGH);
  digitalWrite(LATCH_PIN_SH, LOW);
}

void lcd1602_sendcommand(byte command) {
  dWrite(RS_PIN_LCD, LOW);
  data_shift595(command);
  lcd1602_pulse();
  delayMicroseconds(100);
}

void lcd1602_senddata(byte data) {
  dWrite(RS_PIN_LCD, HIGH);
  data_shift595(data);
  lcd1602_pulse();
  delayMicroseconds(100);
}

void lcd1602_sendstr(char* str, int len, int strn, int pos) {
  int i = 0;
  if (pos + len >= 16) return;
  if (strn == 1)
    lcd1602_sendcommand(LCD_COMMAND_SET_CURSOR(0 + pos));
  else
    lcd1602_sendcommand(LCD_COMMAND_SET_CURSOR(63 + pos));

  for (i = 0; i < len; ++i) {
    lcd1602_senddata(str[i]);
  }
  
  for (i = len; i <16; ++i) lcd1602_senddata(32);
}

void lcd1602_pulse() {
  dWrite(E_PIN_LCD, LOW);
  dWrite(E_PIN_LCD, HIGH);
  dWrite(E_PIN_LCD, LOW);
}

void lcd1602_init() {
  delay(15);
  lcd1602_sendcommand(LCD_COMMAND_WORK_MODE(WM_8BIT, WM_2LINE,WM_5X8));
  delay(10);
  lcd1602_sendcommand(LCD_COMMAND_WORK_MODE(WM_8BIT, WM_2LINE,WM_5X8));
  delay(10);
  lcd1602_sendcommand(LCD_COMMAND_WORK_MODE(WM_8BIT, WM_2LINE,WM_5X8));
  delay(10);
  lcd1602_sendcommand(LCD_COMMAND_DISPLAY(DS_ON,DS_VISIBLE,DS_NOBLINK));
  delay(10);
  lcd1602_sendcommand(LCD_COMMAND_SHIFT_MODE(RIGHT,SH_SHIFT_NO));
  delay(10);
  lcd1602_sendcommand(LCD_COMMAND_SHIFT(SH_CURSOR,RIGHT));
  delay(10);
  lcd1602_sendcommand(LCD_COMMAND_CLR());
  delay(10); 
}

void setup() {
  //SLAVE I2C
  pMode(SIGNAL_PIN_I2C, OUTPUT);
  dWrite(SIGNAL_PIN_I2C, LOW);
  pMode(CLK_PIN_I2C, INPUT);
  pMode(DATA_PIN_I2C, INPUT_PULLUP);
  //
  
  pMode(E_PIN_LCD,OUTPUT);
  pMode(RS_PIN_LCD,OUTPUT);
  pMode(DATA_PIN_SH,OUTPUT);
  pMode(CLK_PIN_SH,OUTPUT);
  pMode(LATCH_PIN_SH,OUTPUT);
  
  dWrite(E_PIN_LCD, LOW);
  dWrite(RS_PIN_LCD, LOW);
  dWrite(DATA_PIN_SH, LOW);
  dWrite(CLK_PIN_SH, LOW);
  dWrite(LATCH_PIN_SH, LOW);
  
  Serial.begin(9600);
  lcd1602_init();
  //master_i2c();
  slave_i2c();
}

void loop() { 
}
