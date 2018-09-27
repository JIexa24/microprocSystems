
#define REGISTER_IN 4
#define REGISTER_SIGN 3
#define REGISTER_HEX 2
#define REGISTER_CLK 14
#define SHIFT_DATA 2
#define SHIFT_CLK 3
#define SHIFT_LATCH 4
#define REGISTER_D0 5
#define REGISTER_D7 12
#define WRITE_EN 13
#define OUT_EN 14
#define CH_EN 15
#define ADDR 255
#define RATE 100

#define AS 1
#define BS 2
#define CS 4
#define DS 8
#define ES 16
#define FS 32
#define GS 64

// Bit patterns for the digits 0..F
byte digits[] = { 
  AS|BS|CS|DS|ES|FS,   //0 0x3f
  BS|CS,               //1 0x03
  AS|BS|DS|ES|GS,      //2  
  AS|BS|CS|DS|GS,      //3
  BS|CS|FS|GS,         //4
  AS|CS|DS|FS|GS,      //5
  AS|CS|DS|ES|FS|GS,   //6
  AS|BS|CS,            //7
  AS|BS|CS|DS|ES|FS|GS,//8
  AS|BS|CS|DS|FS|GS,   //9
  AS|BS|CS|ES|FS|GS,   //A
  CS|DS|ES|FS|GS,      //B
  AS|DS|ES|FS,         //C
  BS|CS|DS|ES|GS,      //D
  AS|DS|ES|FS|GS,      //E
  AS|ES|FS|GS          //F
};
  
/*
   Output the address bits and outputEnable signal using shift registers.
*/
// xxxx xxxx xxxx xxxx iiii iiii oooo oooo
void setAddress(int address, bool outputEnable) {
  shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, ((address >> 8) & 0xFF));
  shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, address & 0xFF);
  
  if (outputEnable == true)
  digitalWrite(OUT_EN, LOW);
  else
  digitalWrite(OUT_EN, HIGH);

  pulse(SHIFT_LATCH, 0);
}

void setData(byte data) {
  for (int pin = REGISTER_D0; pin <= REGISTER_D7; pin += 1) {
    pinMode(pin, OUTPUT);
  }
  for (int pin = REGISTER_D0; pin <= REGISTER_D7; pin += 1) {
    digitalWrite(pin, data & 1);
    data = data >> 1;
  }
  delay(10);
}
/*
   Read a byte from the CHIP at the specified address.
*/
byte readCHIP(int address) {
  for (int pin = REGISTER_D0; pin <= REGISTER_D7; pin += 1) {
    pinMode(pin, INPUT);
  }
  setAddress(address, /*outputEnable*/ true);
  byte data = 0;
  for (int pin = REGISTER_D0; pin >= REGISTER_D7; pin -= 1) {
    data = (data << 1) + digitalRead(pin);
  }
  return data;
}

/*
   Write a byte to the CHIP at the specified address.
*/
void writeCHIP(int address, byte data) {
  setAddress(address, /*outputEnable*/ false);
  setData(data);
  delay(20);
  digitalWrite(WRITE_EN, LOW);
  delay(1);
  digitalWrite(WRITE_EN, HIGH);
  delay(20);
}

/*
   Read the contents of the CHIP and print them to the serial monitor.
*/
void printContents() {
  delay(1000);
  Serial.println("Reading CHIP");
  pinMode(CH_EN, OUTPUT);
  digitalWrite(CH_EN, LOW);
  for (int base = 0; base <= ADDR; base += 16) {
    byte data[16];
    for (int offset = 0; offset <= 15; offset += 1) {
      data[offset] = readCHIP(base + offset);
    }
    
    char buf[80];
    sprintf(buf, "%04x:  %02x %02x %02x %02x %02x %02x %02x %02x   %02x %02x %02x %02x %02x %02x %02x %02x",
            base, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
            data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);

    Serial.println(buf);
  }
  digitalWrite(CH_EN, HIGH);
}

void decoder(){
  pinMode(SHIFT_DATA, OUTPUT);
  pinMode(SHIFT_CLK, OUTPUT);
  pinMode(SHIFT_LATCH, OUTPUT);
  pinMode(WRITE_EN, OUTPUT);
  pinMode(OUT_EN, OUTPUT);
  pinMode(CH_EN, OUTPUT);
  digitalWrite(CH_EN, LOW);
  digitalWrite(WRITE_EN, HIGH);
  
  Serial.println("Programming ones place");
  for (int value = 0; value <= 255; value += 1) {
    writeCHIP(value, digits[value % 10]);
  }
  Serial.println("Programming tens place");
  for (int value = 0; value <= 255; value += 1) {
    if (value < 10) {
      writeCHIP(value | (1 << 8), 0);
    } else {
      writeCHIP(value | (1 << 8), digits[(value / 10) % 10]);
    }
  }
  Serial.println("Programming hundreds place");
  for (int value = 0; value <= 255; value += 1) {
    if (value < 100) {
      writeCHIP(value | (1 << 9), 0);
    } else {
      writeCHIP(value | (1 << 9), digits[(value / 100) % 10]);
    }
  }
  Serial.println("Programming sign");
  for (int value = 0; value <= 255; value += 1) {
    if (value < 1000) {
      writeCHIP(value | (1 << 8) | (1 << 9), 0);
    } else {
      writeCHIP(value | (1 << 8) | (1 << 9), digits[(value / 1000) % 10]);
    }
  }
  
  Serial.println("Programming ones place (twos complement)");
  for (int value = -128; value <= 127; value += 1) {
    writeCHIP(value & 0xff | (1 << 10), digits[abs(value) % 10]);
  }
  Serial.println("Programming tens place (twos complement)");
  for (int value = -128; value <= 127; value += 1) {
    if ((value < 10 && value > -10)) {
      writeCHIP(value & 0xff | (1 << 8) | (1 << 10), 0);
    } else {
      writeCHIP(value & 0xff | (1 << 8) | (1 << 10), digits[abs(value / 10) % 10]);
    }  
  }
  Serial.println("Programming hundreds place (twos complement)");
  for (int value = -128; value <= 127; value += 1) {
    if ((value < 100 && value > -100)) {
      writeCHIP(value & 0xff | (1 << 9) | (1 << 10), 0);
    } else {
      writeCHIP(value & 0xff | (1 << 9) | (1 << 10), digits[abs(value / 100) % 10]);
    }
  }
  Serial.println("Programming sign (twos complement)");
  for (int value = -128; value <= 127; value += 1) {
    if (value < 0) {
      writeCHIP(value & 0xff | (1 << 8) | (1 << 9) | (1 << 10), GS);
    } else {
      writeCHIP(value & 0xff | (1 << 8) | (1 << 9) | (1 << 10), 0);
    }
  }
  
  Serial.println("Programming hex first");
  for (int value = 0; value <= 255; value += 1) {
      writeCHIP(value & 0xff | (1 << 11), digits[value & 0xF]);
  }
  Serial.println("Programming hex second");
  for (int value = 0; value <= 255; value += 1) {
      writeCHIP(value & 0xff | (1 << 8) | (1 << 11), digits[(value >> 4) & 0xF]);
  }
  Serial.println("Programming hex third");
  for (int value = 0; value <= 255; value += 1) {
      writeCHIP(value & 0xff | (1 << 9)  | (1 << 11), BS|CS|ES|FS|GS);
  }
  Serial.println("Programming hex four");
  for (int value = 0; value <= 255; value += 1) {
      writeCHIP(value & 0xff | (1 << 8) | (1 << 9)| (1 << 11), digits[0]);
  }
  Serial.println("Programming hex not have sign");
  for (int value = 0; value <= 255; value += 1) {
      writeCHIP(value & 0xff | (1 << 10)  | (1 << 11), digits[value & 0xF]);
  }
  Serial.println("Programming hex not have sign");
  for (int value = 0; value <= 255; value += 1) {
      writeCHIP(value & 0xff | (1 << 10)  | (1 << 8) | (1 << 11), digits[(value >> 4) & 0xF]);
  }
  Serial.println("Programming hex not have sign");
  for (int value = 0; value <= 255; value += 1) {
      writeCHIP(value & 0xff | (1 << 10)  | (1 << 9)  | (1 << 11),  BS|CS|ES|FS|GS);
  }
  Serial.println("Programming hex not have sign");
  for (int value = 0; value <= 255; value += 1) {
      writeCHIP(value & 0xff | (1 << 10)  | (1 << 8) | (1 << 9) | (1 << 11), digits[0]);
  }

  digitalWrite(CH_EN, HIGH);
}

void pulse(int pin, int del) {
  digitalWrite(pin, LOW);
  digitalWrite(pin, HIGH);
  if (del == 1)
    delay(100);
  digitalWrite(pin, LOW);
}

void testDecimalDecoder() {
  pinMode(REGISTER_IN, OUTPUT);
  pinMode(REGISTER_SIGN, OUTPUT);
  pinMode(REGISTER_HEX, OUTPUT);
  pinMode(REGISTER_CLK, OUTPUT);
  digitalWrite(REGISTER_IN, HIGH);
  digitalWrite(REGISTER_HEX, LOW);
  digitalWrite(REGISTER_SIGN, LOW);
  digitalWrite(REGISTER_CLK, LOW);
  
  Serial.println("Testing Decoder...");
  Serial.println("Testing Decimal...");
  for (int value = 0; value <= 255; value += 1) {
//    Serial.println(value);
    setData(value);
    pulse(REGISTER_CLK, 1);
    delay(RATE);
  }
  Serial.println("Testing Sign Decimal...");
  digitalWrite(REGISTER_SIGN, HIGH);
  for (int value = 0; value <= 255; value += 1) {
//    Serial.println(value);
    setData((byte)value);
    pulse(REGISTER_CLK, 1);
    delay(RATE);
  }
  Serial.println("Testing Hex...");
  digitalWrite(REGISTER_SIGN, LOW);
  digitalWrite(REGISTER_HEX, HIGH);
  for (int value = 0; value <= 255; value += 1) {
//    Serial.println(value, HEX);
    setData(value);
    pulse(REGISTER_CLK, 1);
    delay(RATE);
  }
  Serial.println("Testing Sign Hex...");
  digitalWrite(REGISTER_SIGN, HIGH);
  for (int value = 0; value <= 255; value += 1) {
//    Serial.println(value, HEX);
    setData(value);
    pulse(REGISTER_CLK, 1);
    delay(RATE);
  }
}

void setup() {
  Serial.begin(9600);
  testDecimalDecoder();

  //decoder();
  // Read and print out the contents of the EERPROM
  //printContents();
}


void loop() {
}
