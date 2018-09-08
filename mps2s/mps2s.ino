#include <MPS.h>

#define I2C_READY 0
#define I2C_MRX   1
#define I2C_MTX   2
#define I2C_SRX   3
#define I2C_STX   4
#define I2C_STATUS  (TWSR & 0xF8)

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
#define TWI_BUFFER_LENGTH 256

volatile uint8_t i2c_state;
volatile uint8_t i2c_slarw;
volatile uint8_t i2c_sendStop;      // should the transaction end with a stop
volatile uint8_t i2c_inRepStart;     // in the middle of a repeated start

void (*i2c_onSlaveTransmit)(void);
void (*i2c_onSlaveReceive)(uint8_t*, int);

uint8_t i2c_masterBuffer[TWI_BUFFER_LENGTH];
volatile uint8_t i2c_masterBufferIndex;
volatile uint8_t i2c_masterBufferLength;

uint8_t i2c_txBuffer[TWI_BUFFER_LENGTH];
volatile uint8_t i2c_txBufferIndex;
volatile uint8_t i2c_txBufferLength;

uint8_t i2c_rxBuffer[TWI_BUFFER_LENGTH];
volatile uint8_t i2c_rxBufferIndex;

volatile uint8_t i2c_error;

volatile uint8_t slave_mode;
volatile uint8_t slave_addr;

char master_msg[256] = "Question?";
char slave_msg[256] = "Answer!";

int aref = 1;

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

void master_i2c() {
  uint8_t data[6] = {0x2, 0x1, 0xf1, 0x55, 0x16, 0xA4};
  char buf[256];
  int addr = 0;
  pMode(SCL_PIN, OUTPUT);
  pMode(SDA_PIN, INPUT);
  
  pMode(TRANSMIT_READ_PIN_I2C, INPUT);
  pMode(TRANSMIT_WRITE_PIN_I2C, INPUT);
  pMode(SIGNAL_PIN_I2C, OUTPUT);
  dWrite(SIGNAL_PIN_I2C, HIGH);
  i2c_init(0);
  Serial.println("Ready");
  Serial.print("AnalogLevel A1 - ");
  Serial.println(aRead(P_A1_T));
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
  while (1) {
    if (dRead(TRANSMIT_READ_PIN_I2C) == HIGH) { 
      dWrite(SIGNAL_PIN_I2C, LOW);
      if (Serial.available() >= 1) {
        data[0] = Serial.parseInt();
        if (0 <= data[0] && data[0] < 10){
          data[1] = 0x0;
          i2c_requestFrom(1, data, 4, 1);
        }
        sprintf(buf,"addr: %02d | %02x %02x %02x %02x",data[0], data[2], data[3], data[4], data[5]);
        Serial.println(buf);
      } else {
        Serial.println("Not Reading Address");
      }
      delay(500);
      Serial.println("Ready");
      Serial.print("AnalogLevel A1 - ");
      Serial.println(aRead(P_A1_T));
      dWrite(SIGNAL_PIN_I2C, HIGH);
    }
    if (dRead(TRANSMIT_WRITE_PIN_I2C) == HIGH) { 
      dWrite(SIGNAL_PIN_I2C, LOW);
      if (Serial.available() >= 5) {
        data[0] = Serial.parseInt();
        data[1] = 0x1;
        data[2] = Serial.parseInt();
        data[3] = Serial.parseInt();
        data[4] = Serial.parseInt();
        data[5] = Serial.parseInt();
        sprintf(buf,"addr: %02d | %02x %02x %02x %02x", data[0], data[2], data[3], data[4], data[5]);
        if (0 <= data[0] && data[0] < 10){
          i2c_writeTo(1, data, 6, 1, 1);
        }
        sprintf(buf,"addr: %02d | %02x %02x %02x %02x", data[0], data[2], data[3], data[4], data[5]);
        Serial.println(buf);
      } else {
        Serial.println("Not Writing Data");
      }
      delay(500);
      Serial.println("Ready");
      Serial.print("AnalogLevel A1 - ");
      Serial.println(aRead(P_A1_T));
      dWrite(SIGNAL_PIN_I2C, HIGH);
    }
  }
}

void slave_i2c() {
  pMode(SCL_PIN, INPUT); 
  pMode(SDA_PIN, INPUT);
 
  pMode(SIGNAL_PIN_I2C, OUTPUT);
  pMode(TRANSMIT_READ_PIN_I2C, INPUT);
  dWrite(SIGNAL_PIN_I2C, HIGH);
  lcd1602_init();
  i2c_init(1);
}

void usart_init(int sp) {
  UBRR0H = (sp>>8);
  UBRR0L = (sp & 0xff);
  UCSR0B = _BV(RXEN0) | _BV(TXEN0);

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

byte usart_rx() {
  
}

void setup() {
  pMode(DATA_PIN_SH,OUTPUT);
  pMode(CLK_PIN_SH,OUTPUT);
  pMode(LATCH_PIN_SH,OUTPUT);
  dWrite(DATA_PIN_SH, LOW);
  dWrite(CLK_PIN_SH, LOW);
  dWrite(LATCH_PIN_SH, LOW);
  
  Serial.begin(9600);
  sei();
  master_i2c();
  //slave_i2c();  
}

void i2c_requestFrom(uint8_t address, uint8_t* data, int bytes, int sendStop){
  if (data[1] == 1) return;
  uint8_t c[2] = {data[0], data[1]};
  i2c_writeTo(address, c, 2, 0, 0);
  i2c_readFrom(address, &data[2], bytes, sendStop); 
}

uint8_t slave_data[10][4] = {{0x1, 0xA, 0xA1, 0xBA},
                       {0x2, 0x9, 0xB2, 0xFF},
                       {0x3, 0x8, 0xC3, 0xBB},
                       {0x4, 0x7, 0xD5, 0x31},
                       {0x5, 0x6, 0xE6, 0x15},
                       {0x6, 0x5, 0xF7, 0x24},
                       {0x7, 0x4, 0xE8, 0x3A},
                       {0x8, 0x3, 0xE9, 0x4C},
                       {0x9, 0x2, 0xEA, 0x1E},
                       {0xA, 0x1, 0xEB, 0xBF}};

void onRequestService(){
  char buff[128];
  sprintf(buff, "Request: %02d", slave_addr);
  lcd1602_sendstr(buff, 11, 1, 0);
  sprintf(buff, "An: %02x %02x %02x %02x", slave_data[slave_addr][0], slave_data[slave_addr][1], slave_data[slave_addr][2], slave_data[slave_addr][3]);
  lcd1602_sendstr(buff, 15, 2, 0);
  int res = i2c_transmit(slave_data[slave_addr], 4);
}

void onReceiveService(uint8_t* data, int len){
  char buff[128];
  slave_addr = data[0];
  slave_mode = data[1];
  if (slave_mode == 1) {
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
  if(TWI_BUFFER_LENGTH < length){
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
  while(I2C_MRX == i2c_state){
    continue;
  }

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
  if(TWI_BUFFER_LENGTH < length){
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

  if(TWI_BUFFER_LENGTH < (i2c_txBufferLength+length)){
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

      if(i2c_rxBufferIndex < TWI_BUFFER_LENGTH){
        // put byte in buffer and ack
        i2c_rxBuffer[i2c_rxBufferIndex++] = TWDR;
        i2c_replyack(1);
      }else{
        i2c_replyack(0);
      }
      break;
    case TW_SR_STOP: // stop or repeated start condition received
      i2c_releaseBus();

      if(i2c_rxBufferIndex < TWI_BUFFER_LENGTH){
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
