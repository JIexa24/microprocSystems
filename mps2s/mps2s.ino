#include <MPS.h>

#define CLK_PIN_I2C P_D13
#define DATA_PIN_I2C P_D12
#define SIGNAL_PIN_I2C P_D2
#define TRANSMIT_PIN_I2C P_D3
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
  pMode(SIGNAL_PIN_I2C, OUTPUT);
  dWrite(SIGNAL_PIN_I2C, HIGH);
  dWrite(CLK_PIN_I2C, LOW);
  dWrite(DATA_PIN_I2C, HIGH);
  pMode(DATA_PIN_I2C, INPUT_PULLUP);
  pMode(CLK_PIN_I2C, OUTPUT);
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
          Serial.println(master_msg[index_char - 1]);
          if (master_msg[index_char - 1] == '\0') {
            index_char = 0;
            transmit_msg = 0;
          }
          index_bit = 0;
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
          rbuf[index_char] = tmp;
          index_char = index_char + 1;
          if (tmp == '\0') {
            index_char = 0;
            recive_msg = 0;
          }
          Serial.println(tmp);
          tmp = 0;
          index_bit = 0;
          recive = 0;
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
  
  pMode(SIGNAL_PIN_I2C, OUTPUT);
  dWrite(SIGNAL_PIN_I2C, LOW);
  pMode(CLK_PIN_I2C, INPUT);
  pMode(DATA_PIN_I2C, INPUT_PULLUP);
  while (1) {
    if (dRead(TRANSMIT_PIN_I2C) == HIGH && recive_msg == 0 && transmit_msg == 0) {
      Serial.println("Sending");
      transmit = 1;
      transmit_msg = 1;
    } else if (dRead(DATA_PIN_I2C) == LOW && dRead(CLK_PIN_I2C) == HIGH && recive_msg == 0 && transmit_msg == 0) {
      Serial.println("Reciving");
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
          Serial.println(slave_msg[index_char - 1]);
          if (slave_msg[index_char - 1] == '\0') {
            index_char = 0;
            transmit_msg = 0;
          }
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
          rbuf[index_char] = tmp;
          index_char = index_char + 1;
          if (tmp == '\0') {
            index_char = 0;
            recive_msg = 0;
          }
          Serial.println(tmp);
          tmp = 0;
          index_bit = 0;
          recive = 0;
        }
      }
      if (dRead(CLK_PIN_I2C) == LOW && reading == 1) reading = 0;
    }
    
  }
}

void setup() {
  Serial.begin(9600);
  master_i2c();
  //slave_i2c();
}

void loop() { 
}
