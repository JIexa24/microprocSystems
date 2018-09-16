#include <MPS.h>

#define DS18B20_PIN P_D5
#define DS18B20_MOSFET_PIN P_D6
#define WIRE_SKIPROM 0xCC
#define WIRE_CHOOSEROM 0x55
#define WIRE_SEARCH 0xF0
#define SIGNAL_PIN_I2C P_D2
#define TRANSMIT_READ_PIN_I2C P_D3
#define TRANSMIT_WRITE_PIN_I2C P_D4
#define E_PIN_LCD P_D10
#define RS_PIN_LCD P_D9
#define DATA_PIN_SH P_D8
#define CLK_PIN_SH P_D7
#define LATCH_PIN_SH P_D6
#define RANGE 50
#define I2C_FREQ 100000L
#define I2C_BUFFER_LENGTH 256
#define UART_BUFFER_LENGTH 256

volatile uint8_t i2c_state;
volatile uint8_t i2c_slarw;
volatile uint8_t i2c_sendStop;      // should the transaction end with a stop
volatile uint8_t i2c_inRepStart;     // in the middle of a repeated start

void (*i2c_onSlaveTransmit)(void);
void (*i2c_onSlaveReceive)(uint8_t*, int);

uint8_t i2c_masterBuffer[I2C_BUFFER_LENGTH];
volatile uint8_t i2c_masterBufferIndex;
volatile uint8_t i2c_masterBufferLength;

uint8_t i2c_txBuffer[I2C_BUFFER_LENGTH];
volatile uint8_t i2c_txBufferIndex;
volatile uint8_t i2c_txBufferLength;

uint8_t i2c_rxBuffer[I2C_BUFFER_LENGTH];
volatile uint8_t i2c_rxBufferIndex;

uint8_t uart_rxBuffer[UART_BUFFER_LENGTH];
volatile uint8_t uart_rxBufferIndex;

volatile uint8_t i2c_error;

volatile uint8_t slave_mode;
volatile uint8_t slave_addr;

int aref = 1;

uint8_t slave_data[11][4] = {{0x1, 0xA, 0xA1, 0xBA},
                       {0x2, 0x9, 0xB2, 0xFF},
                       {0x3, 0x8, 0xC3, 0xBB},
                       {0x4, 0x7, 0xD5, 0x31},
                       {0x5, 0x6, 0xE6, 0x15},
                       {0x6, 0x5, 0xF7, 0x24},
                       {0x7, 0x4, 0xE8, 0x3A},
                       {0x8, 0x3, 0xE9, 0x4C},
                       {0x9, 0x2, 0xEA, 0x1E},
                       {0xA, 0x1, 0xEB, 0xBF},
                       {0x0, 0x0, 0x0, 0x0}};
ISR(USART_RX_vect) {
  byte tmp;
  if (!PORTCHECK(UCSR0A, UPE0))
  if (uart_rxBufferIndex < UART_BUFFER_LENGTH - 1) {
  uart_rxBuffer[uart_rxBufferIndex++ % UART_BUFFER_LENGTH] = usart_rx();
  uart_rxBuffer[uart_rxBufferIndex] = '\0';
  }
  else {
    tmp = usart_rx();
  }
}

int usart_available() {
  return uart_rxBufferIndex;
}

int is_d(char a) {
  if ('0' <= a && a <= '9') return 1;
  return 0;
}

char usart_read() {
  char c = uart_rxBuffer[0];
  int j = 0;
  for (j = 1; j < UART_BUFFER_LENGTH - 1; ++j) {
    uart_rxBuffer[j - 1] = uart_rxBuffer[j];
  }
  uart_rxBufferIndex = --uart_rxBufferIndex % 255;
  return c;
}

long usart_parseInt() {
  delay(10);
  int isNegative = 2;
  long value = 0;
  char c;
  c = usart_read();
  do{
    if(c == ' ')
      ; // ignore this character
    else if(c == '-' && isNegative == 2)
      isNegative = 1;
    else if(c >= '0' && c <= '9'){        // is c a digit?
      value = value * 10 + (c - '0');
      isNegative = 0;
    }
    c = usart_read(); 
  } while( (c >= '0' && c <= '9') || c == ' ' );
  
  return value;
}

void usart_init(unsigned long int sp) {
  int i;
  uart_rxBufferIndex = 0;
  for (i = 0; i < UART_BUFFER_LENGTH; ++i)uart_rxBuffer[i] = 0;
  sp = 0xFFFF & (F_CPU / (16 * sp) - 1);
  UBRR0H = (sp >> 8);
  UBRR0L = (sp & 0xff);
  UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);

  // 2stopbits && 8bit  
  UCSR0C = _BV(USBS0) | _BV(UCSZ00) | _BV(UCSZ01);
}

void usart_tx(byte data) {
  while (!PORTCHECK(UCSR0A, UDRE0));
  cli();
  UDR0 = data;
  sei();
  while (!PORTCHECK(UCSR0A, TXC0));
}

int usart_sends(char* str) {
  int i = 0;
  for (i = 0; str[i] != '\0'; ++i) {
    usart_tx(str[i]);
  }
  return i;
}

int usart_send(int num) {
  char str[256];
  sprintf(str, "%d", num);
  int i = 0;
  for (i = 0; str[i] != '\0'; ++i) {
    usart_tx(str[i]);
  }
  return i;
}

int usart_sendln(int num) {
  char str[256];
  sprintf(str, "%d", num);
  int i = 0;
  for (i = 0; str[i] != '\0'; ++i) {
    usart_tx(str[i]);
  }
  usart_tx('\n');
  return i;
}

int usart_sendsln(char* str) {
  int i = 0;
  for (i = 0; str[i] != '\0'; ++i) {
    usart_tx(str[i]);
  }
  usart_tx('\n');
  return i;
}

byte usart_rx() {
  byte data;
  while (!PORTCHECK(UCSR0A, RXC0));
  data = UDR0;
  while (!PORTCHECK(UCSR0A, UDRE0));
  return data;  
}

uint8_t search_1wire(uint8_t *newAddr)
{
  uint8_t id_bit_number;
  uint8_t rom_byte_number;
  uint8_t id_bit, cmp_id_bit;
  uint8_t rom_no[8];
  id_bit_number = 0;
  rom_byte_number = 0;
  
  if (reset_1wire() == 0) return 0;
  // issue the search command
  write_1wire(WIRE_SEARCH, 0);
  // loop to do the search
  while (id_bit_number <= 63) {
    id_bit = read_bit_1wire();
    cmp_id_bit = read_bit_1wire();
    if (id_bit == 1 && cmp_id_bit == 1) return 0;
    else if (id_bit == 1 && cmp_id_bit == 0)
    rom_no[rom_byte_number] |= 1 << (id_bit_number % 8);
    else if (id_bit == 0 && cmp_id_bit == 1)
    rom_no[rom_byte_number] &= ~(1 << (id_bit_number % 8));
    else if (id_bit == 0 && cmp_id_bit == 0) {
      
    }
    if (id_bit_number % 8 == 7) ++rom_byte_number;
  }
  for (int i = 0; i < 8; i++)
    newAddr[i] = rom_no[i];
  return 1;
}

uint8_t reset_1wire()
{
  uint8_t r;
  cli();
  dWrite(DS18B20_PIN, LOW);
  pMode(DS18B20_PIN, OUTPUT);  // drive output low
  sei();
  delayMicroseconds(480);
  cli();
  pMode(DS18B20_PIN, INPUT); // allow it to float
  delayMicroseconds(70);
  r = dRead(DS18B20_PIN);
  sei();
  delayMicroseconds(410);
  return r;
}

uint8_t read_bit_1wire() {
  uint8_t r;
  cli();
  pMode(DS18B20_PIN, OUTPUT);
  dWrite(DS18B20_PIN, LOW);
  delayMicroseconds(3);
  pMode(DS18B20_PIN, INPUT); // let pin float, pull up will raise
  delayMicroseconds(10);
  r = dRead(DS18B20_PIN);
  sei();
  delayMicroseconds(53);
  return r;
}

uint8_t read_1wire() {
  uint8_t bitMask;
  uint8_t r = 0;

  for (bitMask = 0; bitMask < 8; ++bitMask) {
    if (read_bit_1wire()) r |= 1 << bitMask;
  }
  return r;
}

void write_bit_1wire(byte v) {
  if (v & 0x1) {
    cli();
    dWrite(DS18B20_PIN, LOW);
    pMode(DS18B20_PIN, OUTPUT);  // drive output low
    delayMicroseconds(10);
    dWrite(DS18B20_PIN, HIGH); // drive output high
    sei();
    delayMicroseconds(55);
  } else {
    cli();
    dWrite(DS18B20_PIN, LOW);
    pMode(DS18B20_PIN, OUTPUT);  // drive output low
    delayMicroseconds(65);
    dWrite(DS18B20_PIN, HIGH); // drive output high
    sei();
    delayMicroseconds(5);
  }
}

void read_bytes_1wire(uint8_t *buf, uint16_t count) {
  for (uint16_t i = 0 ; i < count ; i++)
    buf[i] = read_1wire();
}

void write_1wire(uint8_t v, byte power) {
  uint8_t bitMask;

  for (bitMask = 0; bitMask < 8; ++bitMask) {
    write_bit_1wire((v >> bitMask) & 0x1);
  }
  if (!power) {
    cli();
    pMode(DS18B20_PIN, INPUT);
    dWrite(DS18B20_PIN, LOW);
    sei();
  }
}

void select_rom_1wire(const uint8_t rom[8])
{
  uint8_t i;
  write_1wire(WIRE_CHOOSEROM, 0);// Choose ROM
  for (i = 0; i < 8; i++)
    write_1wire(rom[i], 0);
}

void skip_rom_1wire() {
  write_1wire(WIRE_SKIPROM, 0);// Skip ROM
}

int aRead(int pin) {
  int lowbits = 0;
  int highbits = 0;
  if (7 < pin && pin < 0 && 21 < pin && pin < 14) return -1;
  if (pin > 7) pin = pin - 14;

  ADCSRA |= _BV(ADEN);
  // (1000 - temperature, 1110 - 1.1V, 1111- GND)
  ADMUX = (aref << 6) | (pin & 0x7);
  // 00 00000000 (set - 00000000 00)
  PORTCLEAR(ADMUX, ADLAR);

  PORTSET(ADCSRA, ADSC);

  while (PORTCHECK(ADCSRA, ADSC));

  lowbits  = ADCL;
  highbits = ADCH;
  
  return (highbits << 8) | lowbits;
}

void master() {
  uint8_t data[6] = {0x2, 0x1, 0xf1, 0x55, 0x16, 0xA4};
  char buf[256];
  int addr = 0;
  usart_init(9600);
  pMode(SCL_PIN, OUTPUT);
  pMode(SDA_PIN, INPUT);
  
  pMode(TRANSMIT_READ_PIN_I2C, INPUT);
  pMode(TRANSMIT_WRITE_PIN_I2C, INPUT);
  pMode(SIGNAL_PIN_I2C, OUTPUT);
  dWrite(SIGNAL_PIN_I2C, HIGH);
  i2c_init(0);
  usart_sendsln("Ready");
  usart_sends("AnalogLevel A0 - ");
  usart_sendln(aRead(P_A0_T));
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
  while (1) {
    if (dRead(TRANSMIT_READ_PIN_I2C) == HIGH) { 
      dWrite(SIGNAL_PIN_I2C, LOW);
      usart_sendsln("Please, input adress");
      while (!usart_available());
        data[0] = usart_parseInt();
        if (0 <= data[0] && data[0] <= 10){
          data[1] = 0x0;
          i2c_requestFrom(1, data, 4, 1);
          if (data[0] == 10) {
            sprintf(buf,"addr: %02d | %02x %02x %02x %02x",data[0], data[2], data[3], data[4], data[5]);
            usart_sendsln(buf);
            sprintf(buf,"addr: %02d | cel - %d | far - %d",data[0], (data[2] << 8) | data[3], (data[4] << 8) | data[5]);
            usart_sendsln(buf);
          } else {
          sprintf(buf,"addr: %02d | %02x %02x %02x %02x",data[0], data[2], data[3], data[4], data[5]);
          usart_sendsln(buf);
          }
        }
        else {
          usart_sendsln("Not Reading Address");
        }
      delay(500);
      usart_sendsln("Ready");
      usart_sends("AnalogLevel A0 - ");
      usart_sendln(aRead(P_A0_T));
      dWrite(SIGNAL_PIN_I2C, HIGH);
    }
    if (dRead(TRANSMIT_WRITE_PIN_I2C) == HIGH) { 
      dWrite(SIGNAL_PIN_I2C, LOW);
      usart_sendsln("Please, input adress");
      while (!usart_available());
      data[0] = usart_parseInt();
      data[1] = 0x1;
      usart_sendsln("Please, input write data[0]");
      while (!usart_available());
      data[2] = usart_parseInt();
      usart_sendsln("Please, input write data[1]");
      while (!usart_available());
      data[3] = usart_parseInt();
      usart_sendsln("Please, input write data[2]");
      while (!usart_available());
      data[4] = usart_parseInt();
      usart_sendsln("Please, input write data[3]");
      while (!usart_available());
      data[5] = usart_parseInt();
      sprintf(buf,"addr: %02d | %02x %02x %02x %02x", data[0], data[2], data[3], data[4], data[5]);
      if (0 <= data[0] && data[0] <= 9){
        i2c_writeTo(1, data, 6, 1, 1);
        sprintf(buf,"addr: %02d | %02x %02x %02x %02x", data[0], data[2], data[3], data[4], data[5]);
        usart_sendsln(buf);
      } else {
        usart_sendsln("Not Writing Data");
      }
      delay(500);
      usart_sendsln("Ready");
      usart_sends("AnalogLevel A0 - ");
      usart_sendln(aRead(P_A0_T));
      dWrite(SIGNAL_PIN_I2C, HIGH);
    }
  }
}

void slave() {
  pMode(SCL_PIN, INPUT); 
  pMode(SDA_PIN, INPUT);
 
  pMode(SIGNAL_PIN_I2C, OUTPUT);
  dWrite(SIGNAL_PIN_I2C, HIGH);
  lcd1602_init();
  i2c_init(1);
  
  byte i;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;

  while(1){
    reset_1wire();
    skip_rom_1wire();
    write_1wire(0x44, 1);
    delay(1000);
    reset_1wire();
    skip_rom_1wire();
    write_1wire(0xBE, 1);
    for ( i = 0; i < 9; i++) {              
      data[i] = read_1wire(); 
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
    celsius =  (float)raw / 16.0;
    fahrenheit = celsius * 1.8 + 32.0; 
    slave_data[10][0] = (unsigned int)celsius >> 8 & 0xFF;
    slave_data[10][1] = (unsigned int)celsius & 0xFF;
    slave_data[10][2] = (unsigned int)fahrenheit >> 8 & 0xFF;
    slave_data[10][3] = (unsigned int)fahrenheit & 0xFF;
    delay(5000);
  }

}

void setup() {
  pMode(DATA_PIN_SH,OUTPUT);
  pMode(CLK_PIN_SH,OUTPUT);
  pMode(LATCH_PIN_SH,OUTPUT);
  dWrite(DATA_PIN_SH, LOW);
  dWrite(CLK_PIN_SH, LOW);
  dWrite(LATCH_PIN_SH, LOW);
  sei();  

  master();
  //slave();  
}

void i2c_requestFrom(uint8_t address, uint8_t* data, int bytes, int sendStop){
  if (data[1] == 1) return;
  uint8_t c[2] = {data[0], data[1]};
  i2c_writeTo(address, c, 2, 0, 0);
  i2c_readFrom(address, &data[2], bytes, sendStop); 
}

void onRequestService(){
  char buff[128];
  sprintf(buff, "Request: %02d", slave_addr);
  lcd1602_sendstr(buff, 11, 1, 0);
  if (0 <= slave_addr && slave_addr < 10) {
    sprintf(buff, "An: %02x %02x %02x %02x", slave_data[slave_addr][0], slave_data[slave_addr][1], slave_data[slave_addr][2], slave_data[slave_addr][3]);
    lcd1602_sendstr(buff, 15, 2, 0);
    int res = i2c_transmit(slave_data[slave_addr], 4);
  } else {
    sprintf(buff, "An: %02x %02x %02x %02x", slave_data[slave_addr][0], slave_data[slave_addr][1], slave_data[slave_addr][2], slave_data[slave_addr][3]);
    lcd1602_sendstr(buff, 15, 2, 0);
    int res = i2c_transmit(slave_data[10], 4);
  }
}

void onReceiveService(uint8_t* data, int len){
  char buff[128];
  slave_addr = data[0];
  slave_mode = data[1];
  if (slave_mode == 1 && slave_addr < 10 && slave_addr >= 0) {
    slave_data[slave_addr][0] = data[2];
    slave_data[slave_addr][1] = data[3];
    slave_data[slave_addr][2] = data[4];
    slave_data[slave_addr][3] = data[5];    
    sprintf(buff, "Receive: %02d", slave_addr);
    lcd1602_sendstr(buff, 11, 1, 0);
    sprintf(buff, "Da: %02x %02x %02x %02x", slave_data[slave_addr][0], slave_data[slave_addr][1], slave_data[slave_addr][2], slave_data[slave_addr][3]);
    lcd1602_sendstr(buff, 15, 2, 0);
  }
}

void i2c_init(int address) {
  i2c_state = I2C_READY;
  i2c_sendStop = 1; //default
  dWrite(SDA_PIN, HIGH);
  dWrite(SCL_PIN, HIGH);
  //set clock, presc = 1
  PORTSET(TWSR, 0);
  PORTSET(TWSR, 1);

  TWBR = ((F_CPU / I2C_FREQ) - 16) / 2;

  // enable twi module, acks, and twi interrupt
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
  /*
  PORTSET(TWCR, 0);
  PORTSET(TWCR, 2);
  PORTSET(TWCR, 6);
  */
  if (address == 0) return;
  //set slave-address (master = 0)
  TWAR = address << 1;
  i2c_attachSlaveTxEvent(onRequestService);
  i2c_attachSlaveRxEvent(onReceiveService);
}


void i2c_attachSlaveRxEvent( void (*function)(uint8_t*, int) )
{
  i2c_onSlaveReceive = function;
}
void i2c_attachSlaveTxEvent( void (*function)(void) )
{
  i2c_onSlaveTransmit = function;
}

void i2c_releaseBus() {
  // release bus
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);
  i2c_state = I2C_READY;
}

void i2c_stopCond() {
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);
  
  while(TWCR & _BV(TWSTO)){
    continue;
  }
  
  i2c_state = I2C_READY;
}

void i2c_replyack(uint8_t ack) {
  // transmit master read ready signal, with or without ack
  if(ack){
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
  } else { 
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);
  }
}

uint8_t i2c_readFrom(uint8_t address, uint8_t* data, uint8_t length, uint8_t sendStop)
{
  uint8_t i;

  // ensure data will fit into buffer
  if(I2C_BUFFER_LENGTH < length){
    return 0;
  }

  // wait until twi is ready, become master receiver
  while(I2C_READY != i2c_state){
    continue;
  }
  i2c_state = I2C_MRX;
  i2c_sendStop = sendStop;
  // reset error state (0xFF.. no error occured)
  i2c_error = 0xFF;

  // initialize buffer iteration vars
  i2c_masterBufferIndex = 0;
  i2c_masterBufferLength = length-1;
  
  // build sla+w, slave device address + w bit
  i2c_slarw = TW_READ;
  i2c_slarw |= address << 1;

  if (true == i2c_inRepStart) {
    i2c_inRepStart = false; 
    do {
      TWDR = i2c_slarw;
    } while(TWCR & _BV(TWWC));
    TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE);  // enable INTs, but not START
  }
  else
    // send start condition
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);

  // wait for read operation to complete
  while(I2C_MRX == i2c_state);

  if (i2c_masterBufferIndex < length)
    length = i2c_masterBufferIndex;

  // copy twi buffer to data
  for(i = 0; i < length; ++i){
    data[i] = i2c_masterBuffer[i];
  }
  
  return length;
}

uint8_t i2c_writeTo(uint8_t address, uint8_t* data, uint8_t length, uint8_t wait, uint8_t sendStop)
{
  uint8_t i;

  // ensure data will fit into buffer
  if(I2C_BUFFER_LENGTH < length){
    return 1;
  }

  // wait until twi is ready, become master transmitter
  while(I2C_READY != i2c_state){
    continue;
  }
  i2c_state = I2C_MTX;
  i2c_sendStop = sendStop;
  i2c_error = 0xFF;

  // initialize buffer iteration vars
  i2c_masterBufferIndex = 0;
  i2c_masterBufferLength = length;
  
  // copy data to twi buffer
  for(i = 0; i < length; ++i){
    i2c_masterBuffer[i] = data[i];
  }
  
  // build sla+w, slave device address + w bit
  i2c_slarw = TW_WRITE;
  i2c_slarw |= address << 1;
  
  // if we're in a repeated start, then we've already sent the START
  // in the ISR. Don't do it again.
  //
  if (true == i2c_inRepStart) {
    i2c_inRepStart = false;   
    do {
      TWDR = i2c_slarw;       
    } while(TWCR & _BV(TWWC));
    TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE);  // enable INTs, but not START
  }
  else
    // send start condition
    TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE) | _BV(TWSTA); // enable INTs

  // wait for write operation to complete
  while(wait && (I2C_MTX == i2c_state)){
    continue;
  }
  
  if (i2c_error == 0xFF)
    return 0; // success
  else if (i2c_error == TW_MT_SLA_NACK)
    return 2; // error: address send, nack received
  else if (i2c_error == TW_MT_DATA_NACK)
    return 3; // error: data send, nack received
  else
    return 4; // other error
}

uint8_t i2c_transmit(const uint8_t* data, uint8_t length) {
  uint8_t i;

  if(I2C_BUFFER_LENGTH < (i2c_txBufferLength+length)){
    return 1;
  }
  
  if(I2C_STX != i2c_state){
    return 2;
  }
  
  // set length and copy data into tx buffer
  for(i = 0; i < length; ++i){
    i2c_txBuffer[i2c_txBufferLength+i] = data[i];
  }
  i2c_txBufferLength += length;
  
  return 0;
}

ISR(TWI_vect) {
  switch(I2C_STATUS) { 
    // All Master
    case TW_START:     // sent start condition
    case TW_REP_START: // sent repeated start condition
    // copy device address and r/w bit to output register and ack
      TWDR = i2c_slarw;
      i2c_replyack(1);
    break; 
    // Master Transmitter
    case TW_MT_SLA_ACK:  // slave receiver acked address
    case TW_MT_DATA_ACK: // slave receiver acked data
      if(i2c_masterBufferIndex < i2c_masterBufferLength){
        TWDR = i2c_masterBuffer[i2c_masterBufferIndex++];
        i2c_replyack(1);
      }else{
        if (i2c_sendStop)
          i2c_stopCond();
        else {
          i2c_inRepStart = true; 
          TWCR = _BV(TWINT) | _BV(TWSTA)| _BV(TWEN) ;
          i2c_state = I2C_READY;
        }
      }
    break;
    case TW_MT_SLA_NACK:  // address sent, nack received
      i2c_error = TW_MT_SLA_NACK;
      i2c_stopCond();
    break;
    case TW_MT_DATA_NACK: // data sent, nack received
      i2c_error = TW_MT_DATA_NACK;
      i2c_stopCond();
    break;
    case TW_MT_ARB_LOST: // lost bus arbitration
      i2c_error = TW_MT_ARB_LOST;
      i2c_releaseBus();
      break;
      
    // Master Receiver
    case TW_MR_DATA_ACK: // data received, ack sent
      i2c_masterBuffer[i2c_masterBufferIndex++] = TWDR;
    case TW_MR_SLA_ACK:  // address sent, ack received
      if(i2c_masterBufferIndex < i2c_masterBufferLength){
        i2c_replyack(1);
      }else{
        i2c_replyack(0);
      }
      break;
    case TW_MR_DATA_NACK: // data received, nack sent
      i2c_masterBuffer[i2c_masterBufferIndex++] = TWDR;
      if (i2c_sendStop)
        i2c_stopCond();
      else {
        i2c_inRepStart = true;  
        TWCR = _BV(TWINT) | _BV(TWSTA)| _BV(TWEN) ;
        i2c_state = I2C_READY;
      }    
    break;
    case TW_MR_SLA_NACK: // address sent, nack received
      i2c_stopCond();
      break;
    // TW_MR_ARB_LOST handled by TW_MT_ARB_LOST case

    // Slave Receiver
    case TW_SR_SLA_ACK:   // addressed, returned ack
    case TW_SR_GCALL_ACK: // addressed generally, returned ack
    case TW_SR_ARB_LOST_SLA_ACK:   // lost arbitration, returned ack
    case TW_SR_ARB_LOST_GCALL_ACK: // lost arbitration, returned ack
      i2c_state = I2C_SRX;
      i2c_rxBufferIndex = 0;
      i2c_replyack(1);
      break;
    case TW_SR_DATA_ACK:       // data received, returned ack
    case TW_SR_GCALL_DATA_ACK: // data received generally, returned ack

      if(i2c_rxBufferIndex < I2C_BUFFER_LENGTH){
        // put byte in buffer and ack
        i2c_rxBuffer[i2c_rxBufferIndex++] = TWDR;
        i2c_replyack(1);
      }else{
        i2c_replyack(0);
      }
      break;
    case TW_SR_STOP: // stop or repeated start condition received
      i2c_releaseBus();

      if(i2c_rxBufferIndex < I2C_BUFFER_LENGTH){
        i2c_rxBuffer[i2c_rxBufferIndex] = '\0';
      }
      // callback to user defined callback
      i2c_onSlaveReceive(i2c_rxBuffer, i2c_rxBufferIndex);

      i2c_rxBufferIndex = 0;
      break;
    case TW_SR_DATA_NACK:       // data received, returned nack
    case TW_SR_GCALL_DATA_NACK: // data received generally, returned nack
      i2c_replyack(0);
      break;
    
    // Slave Transmitter
    case TW_ST_SLA_ACK:          // addressed, returned ack
    case TW_ST_ARB_LOST_SLA_ACK: // arbitration lost, returned ack
      i2c_state = I2C_STX;
      i2c_txBufferIndex = 0;
      i2c_txBufferLength = 0;
      i2c_onSlaveTransmit();

      if(0 == i2c_txBufferLength){
        i2c_txBufferLength = 1;
        i2c_txBuffer[0] = 0x00;
      }

    case TW_ST_DATA_ACK: // byte sent, ack returned

      TWDR = i2c_txBuffer[i2c_txBufferIndex++];

      if(i2c_txBufferIndex < i2c_txBufferLength){
        i2c_replyack(1);
      }else{
        i2c_replyack(0);
      }
      break;
    case TW_ST_DATA_NACK: // received nack, we are done 
    case TW_ST_LAST_DATA: // received ack, but we are done already!
      // ack future responses
      i2c_replyack(1);
      // leave slave receiver state
      i2c_state = I2C_READY;
      break;

    // All
    case TW_NO_INFO:   // no state information
      break;
    case TW_BUS_ERROR: // bus error, illegal stop/start
      i2c_error = TW_BUS_ERROR;
      i2c_stopCond();
      break;
  }
}

void loop() { 
 //Serial.println(dRead(SCL_PIN));
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
