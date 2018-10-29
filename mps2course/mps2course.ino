#include <MPS.h>

#define WIRE_SKIPROM 0xCC
#define WIRE_CHOOSEROM 0x55
#define WIRE_SEARCH 0xF0
#define DS18B20_ST 0X44
#define DS18B20_PIN P_D3
#define SOUND_SENSOR_START_PIN P_D4
#define LCD_LATCH_PIN_SH P_D6
#define LCD_CLK_PIN_SH P_D7
#define LCD_DATA_PIN_SH P_D8
#define RS_PIN_LCD P_D9
#define E_PIN_LCD P_D10

#define COUNT_ROW 1
#define COUNT_COL 2
#define STARTKEYW P_D11_T  
#define ENDKEYW P_D12_T  
#define STARTKEYR P_D13_T  
#define ENDKEYR P_D13_T  
#define RANGE 300
volatile unsigned long int time_mcs = 0;
uint8_t LastDiscrepancy;
uint8_t LastFamilyDiscrepancy;
uint8_t LastDeviceFlag;
volatile int ready_int = 0;
unsigned char ROM_NO[8] = {0};

volatile int flag_l = 0;
volatile unsigned long int OVF_counter = 0;
void setup() {
  int var = 0, i;
  float celsius = 0;
  double dist = 0;
  int tmps = 0;
  char lcd_buffer[256];
  sei();
  pMode(LCD_LATCH_PIN_SH, OUTPUT);
  pMode(LCD_CLK_PIN_SH, OUTPUT);
  pMode(LCD_DATA_PIN_SH, OUTPUT);
  pMode(RS_PIN_LCD, OUTPUT);
  pMode(E_PIN_LCD, OUTPUT);
  pMode(DS18B20_PIN, INPUT);
  pMode(INT0_PIN, INPUT);
  pMode(SOUND_SENSOR_START_PIN, OUTPUT);
  pMode(LED_PIN, OUTPUT);
  pMode(P_D5_T, INPUT);
  lcd1602_init();
  lcd1602_sendstr("R9ODT/Alexey R.", 15, 1, 0);
  init_timer1();
  TIMER1_OVF_EN();
  INT0_EN_LOG();
  for (i = 0; i< 8; ++i) ROM_NO[i] = 0;
  delay(RANGE);
  while (1) {
    var = dRead(P_D5_T);//read_matrix_keyboard(COUNT_ROW, COUNT_COL, STARTKEYW, ENDKEYW, STARTKEYR, ENDKEYR);
    if (var == 0) {
      lcd1602_sendstr("R9ODT/Alexey R.", 15, 1, 0);
      celsius = start_temp_cel(DS18B20_PIN);
      dist = start_distance(SOUND_SENSOR_START_PIN);
      dist = (dist * (330 + 0.5 * celsius)) / 20000; 
      sprintf(lcd_buffer, "Tem:%02d Dis:%04d",(int)celsius, (int)dist);
      lcd1602_sendstr(lcd_buffer, 15, 2, 0);
      var = -1;
    } else if (var == 1) {
      for (i = 0; i< 8; ++i) ROM_NO[i] = 0;
      //lcd1602_sendstr("ROM_NO", 6, 1, 0);
      reset_search_1wire();
      tmps = search_1wire(ROM_NO, DS18B20_PIN);
      if(tmps == 1)
        sprintf(lcd_buffer, "%02x%02x%02x%02x%02x%02x%02x%02x",ROM_NO[0],ROM_NO[1],ROM_NO[2],ROM_NO[3],ROM_NO[4],ROM_NO[5],ROM_NO[6],ROM_NO[7]);
      else
        sprintf(lcd_buffer, "----------------");
      lcd1602_sendstr(lcd_buffer, 16, 2, 0);
      var = -1;
    }
    dWrite(LED_PIN, HIGH);
    delay(RANGE);
    dWrite(LED_PIN, LOW);
    delay(RANGE);
  }
}

int read_matrix_keyboard(int countc, int countr, int stw, int endw, int str, int enr) {
  int i = 0;
  int j = 0;
  int readval = 0;
  
  for (i = str; i <= enr ;++i) {
    pMode(i, INPUT);
  }

  for (i = stw; i <= endw ;++i) {
    pMode(i, OUTPUT);
  }

  for (i = 0; i < countr; ++i) {
    for (j = 0; j < countr; ++j) {
      if (i == j) dWrite(stw + j, LOW);
      else dWrite(stw + j, HIGH);
    }
    for (j = 0; j < countc; ++j) {
      readval = dRead(str + j);
      if (readval == LOW) {
        return (i*(countr - 1) + j);
      }
    }
  }  
}

float start_temp_cel(int pin) {
  byte data[12];
  int j,i;
  //calculating temperature
  reset_1wire(DS18B20_PIN);
  //select_rom_1wire(ROM_NO, DS18B20_PIN);
  skip_rom_1wire(DS18B20_PIN);
  write_1wire(DS18B20_ST, 1, DS18B20_PIN);
  delay(1000);
  reset_1wire(DS18B20_PIN);
  //select_rom_1wire(ROM_NO, DS18B20_PIN);
  skip_rom_1wire(DS18B20_PIN);
  write_1wire(0xBE, 1, DS18B20_PIN);
  
  for ( i = 0; i < 9; i++) {              
    data[i] = read_1wire(DS18B20_PIN);
  }  
  int16_t raw = (data[1] << 8) | data[0]; 
  if (data[7] == 0x10) {
    raw = (raw & 0xFFF0) + 12 - data[6];
  } else {
    byte cfg =  (data[4] & 0x60);
    if (cfg == 0x00) 
      raw = raw << 3; 
    else if  (cfg == 0x20) 
      raw = raw << 2; 
    else if  (cfg == 0x40) 
      raw = raw << 1;
  }  
  return (float)raw / 16.0;
}

unsigned int start_distance(int pin) {
  int timing = 0;
  flag_l = 0;
  ready_int = 0;
  dWrite(pin, HIGH);
  delayMicroseconds(10);
  dWrite(pin, LOW);
  while(ready_int == 0 && timing++ < 10000);
  return time_mcs;
}
volatile int tcnt1 = 0;
volatile unsigned long int ovf_c = 0;
ISR(INT0_vect)
{  
  if (flag_l == 0) {
    tcnt1 = TCNT1;
    ovf_c = OVF_counter;
    ready_int = 0;
    flag_l = 1;
  } else {
    int t = TCNT1;
    int o = OVF_counter;
    int e = ((t - tcnt1 + 0xFFFF) % 0xFFFF) / 2;
	  if (t - tcnt1 < 0) {
	    e = e + (o - ovf_c - 1) / 2;
    } else
	    e = e + (o - ovf_c) / 2;
    time_mcs = e;
    ready_int = 1;
    flag_l = 0;
  }
}

ISR(TIMER1_OVF_vect)
{
  ++OVF_counter;// Увеличиваем счетчик переполнений
}

void init_timer1 () {
  TCCR1A = 0;// 1 << WGM10;
  TCCR1B = 1 << CS11;
}

void loop() {}


void reset_search_1wire() {
  // reset the search state
  LastDiscrepancy = 0;
  LastDeviceFlag = 0;
  LastFamilyDiscrepancy = 0;
}

uint8_t search_1wire(uint8_t *newAddr, int pin)
{ 
  unsigned char rom_no[8];
  uint8_t id_bit_number;
  uint8_t last_zero, rom_byte_number, search_result;
  uint8_t id_bit, cmp_id_bit;
  unsigned char rom_byte_mask, search_direction;
  id_bit_number = 1;
  last_zero = 0;
  rom_byte_number = 0;
  rom_byte_mask = 1;
  search_result = 0;

  if (!LastDeviceFlag) {  
    if (!reset_1wire(pin)) {

      LastDiscrepancy = 0;
      LastDeviceFlag = 0;
      LastFamilyDiscrepancy = 0;
      return 0;
    }

    // issue the search command
    write_1wire(0xF0, 1, pin);

    // loop to do the search
    do {
      // read a bit and its complement
      id_bit = read_bit_1wire(pin);
      cmp_id_bit = read_bit_1wire(pin);
      // check for no devices on 1-wire
      if ((id_bit == 1) && (cmp_id_bit == 1))
        break;
      else {
        // all devices coupled have 0 or 1
        if (id_bit != cmp_id_bit)
          search_direction = id_bit;  // bit write value for search
        else {
          // if this discrepancy if before the Last Discrepancy
          // on a previous next then pick the same as last time
          if (id_bit_number < LastDiscrepancy)
            search_direction = ((rom_no[rom_byte_number] & rom_byte_mask) > 0);
          else
            // if equal to last pick 1, if not then pick 0
            search_direction = (id_bit_number == LastDiscrepancy);
            // if 0 was picked then record its position in LastZero
          if (search_direction == 0) {
            last_zero = id_bit_number;
            // check for Last discrepancy in family
            if (last_zero < 9)
              LastFamilyDiscrepancy = last_zero;
          }
        }

        // set or clear the bit in the ROM byte rom_byte_number
        // with mask rom_byte_mask
        if (search_direction == 1)
          rom_no[rom_byte_number] |= rom_byte_mask;
        else
          rom_no[rom_byte_number] &= ~rom_byte_mask;

        // serial number search direction write bit
        write_bit_1wire(search_direction, pin);

        // increment the byte counter id_bit_number
        // and shift the mask rom_byte_mask
        id_bit_number++;
        rom_byte_mask <<= 1;
        // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
        if (rom_byte_mask == 0) {
          rom_byte_number++;
          rom_byte_mask = 1;
        }
      }
    } while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7
    // if the search was successful then
    if (!(id_bit_number < 65)) {
      // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
      LastDiscrepancy = last_zero;

      // check for last device
      if (LastDiscrepancy == 0)
        LastDeviceFlag = 1;
      search_result = 1;
    }
 }

   // if no device found then reset counters so next 'search' will be like a first
 if (!search_result || !rom_no[0]) {
    LastDiscrepancy = 0;
    LastDeviceFlag = 0;
    LastFamilyDiscrepancy = 0;
    search_result = 0;
 }
 for (int i = 0; i < 8; i++) newAddr[i] = rom_no[i];
 return search_result;
}

uint8_t reset_1wire(int pin)
{
  uint8_t r;
  uint8_t retries = 125;

  cli();
  dWrite(pin, LOW);
  pMode(pin, OUTPUT);  // drive output low
  sei();
  delayMicroseconds(480);
  cli();
  pMode(pin, INPUT); // allow it to float
  delayMicroseconds(70);
  r = !(dRead(pin));
  sei();
  delayMicroseconds(410);
  return r;
}

uint8_t read_bit_1wire(int pin) {
  uint8_t r;
  cli();
  pMode(pin, OUTPUT);
  dWrite(pin, LOW);
  delayMicroseconds(3);
  pMode(pin, INPUT); // let pin float, pull up will raise
  delayMicroseconds(10);
  r = dRead(pin);
  sei();
  delayMicroseconds(53);
  return r;
}

uint8_t read_1wire(int pin) {
  uint8_t bitMask;
  uint8_t r = 0;

  for (bitMask = 0; bitMask < 8; ++bitMask) {
    if (read_bit_1wire(pin)) r |= 1 << bitMask;
  }
  return r;
}

void write_bit_1wire(byte v, int pin) {
  if (v & 0x1) {
    cli();
    dWrite(pin, LOW);
    pMode(pin, OUTPUT);  // drive output low
    delayMicroseconds(10);
    dWrite(pin, HIGH); // drive output high
    sei();
    delayMicroseconds(55);
  } else {
    cli();
    dWrite(pin, LOW);
    pMode(pin, OUTPUT);  // drive output low
    delayMicroseconds(65);
    dWrite(pin, HIGH); // drive output high
    sei();
    delayMicroseconds(5);
  }
}

void read_bytes_1wire(uint8_t *buf, uint16_t count, int pin) {
  for (uint16_t i = 0 ; i < count ; i++)
    buf[i] = read_1wire(pin);
}

void write_1wire(uint8_t v, byte power, int pin) {
  uint8_t bitMask;

  for (bitMask = 0; bitMask < 8; ++bitMask) {
    write_bit_1wire((v >> bitMask) & 0x1, pin);
  }
  if (!power) {
    cli();
    pMode(pin, INPUT);
    dWrite(pin, LOW);
    sei();
  }
}

void select_rom_1wire(const uint8_t rom[8], int pin)
{
  uint8_t i;
  write_1wire(WIRE_CHOOSEROM, 0, pin);// Choose ROM
  for (i = 0; i < 8; i++)
    write_1wire(rom[i], 0, pin);
}

void skip_rom_1wire(int pin) {
  write_1wire(WIRE_SKIPROM, 0, pin);// Skip ROM
}

void data_shift595(byte data, int data_pin, int clk_pin,int latch_pin) {
  shiftOut(data_pin, clk_pin, MSBFIRST, data);
  
  dWrite(latch_pin, LOW);
  dWrite(latch_pin, HIGH);
  dWrite(latch_pin, LOW);
}

void lcd1602_sendcommand(byte command) {
  dWrite(RS_PIN_LCD, LOW);
  data_shift595(command, LCD_DATA_PIN_SH, LCD_CLK_PIN_SH, LCD_LATCH_PIN_SH);
  pulse(E_PIN_LCD);
  delayMicroseconds(100);
}

void lcd1602_senddata(byte data) {
  dWrite(RS_PIN_LCD, HIGH);
  data_shift595(data, LCD_DATA_PIN_SH, LCD_CLK_PIN_SH, LCD_LATCH_PIN_SH);
  pulse(E_PIN_LCD);
  delayMicroseconds(100);
}

void lcd1602_sendstr(char* str, int len, int strn, int pos) {
  int i = 0;
  if (pos + len > 16) return;
  if (strn == 1)
    lcd1602_sendcommand(LCD_COMMAND_SET_CURSOR(0 + pos));
  else
    lcd1602_sendcommand(LCD_COMMAND_SET_CURSOR(63 + pos));

  for (i = 0; i < len; ++i) {
    lcd1602_senddata(str[i]);
  }
  
  for (i = len; i <16; ++i) lcd1602_senddata(32);
}

void pulse(int pin) {
  dWrite(pin, LOW);
  dWrite(pin, HIGH);
  dWrite(pin, LOW);
}

void lcd1602_init() {
  pMode(E_PIN_LCD,OUTPUT);
  pMode(RS_PIN_LCD,OUTPUT);
  
  dWrite(E_PIN_LCD, LOW);
  dWrite(RS_PIN_LCD, LOW);
  
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
