#include <SPI.h>

// pin connected to CSN
#define CSN 8



/* Memory Map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define NRF_STATUS  0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RPD         0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	    0x1C
#define FEATURE	    0x1D

#define STR(x) #x

#define R_REGISTER 0
#define REGISTER_MASK 0x1f

void csn(bool mode) {
  if (mode == 0) 
    digitalWrite(CSN, LOW);
  else
    digitalWrite(CSN, HIGH);

}

uint8_t read_register(uint8_t reg, uint8_t* buf, uint8_t len)
{
  uint8_t status;

  digitalWrite(CSN, LOW);
  status = SPI.transfer( R_REGISTER | ( REGISTER_MASK & reg ) );
  while ( len-- ){
    *buf++ = SPI.transfer(0xff);
  }
  digitalWrite(CSN, HIGH);

  return status;
}

/****************************************************************************/

uint8_t read_register(uint8_t reg)
{
  uint8_t result;

  digitalWrite(CSN, LOW);
  SPI.transfer( R_REGISTER | ( REGISTER_MASK & reg ) );
  result = SPI.transfer(0xff);
  digitalWrite(CSN, HIGH);

  return result;
}

#define CHECK_REG(name, val) case name: check_reg(name, #name, val); break;

void check_reg(uint8_t reg, char* reg_name, uint8_t def) {
  uint8_t temp;
  Serial.print(reg_name);
  Serial.print(": ");
  temp = read_register(reg);
  Serial.print(temp, HEX);
  Serial.print(" ");
  Serial.println(def, HEX);
  if (temp != def) Serial.println("== ERROR ==");
}

void check_addr(uint8_t reg, char* reg_name, uint8_t def) {
  uint8_t buf[5];
  bool error = false;
  Serial.print(reg_name);
  Serial.print(": ");
  read_register(reg, buf, 5);
  for (int i=0; i < 5; i++)
    Serial.print(buf[i], HEX);
  Serial.print(" ");
  for (int i=0; i < 5; i++) {
    Serial.print(def, HEX);
    if (buf[i] != def) error = true;
  }
  Serial.println();
  if (error) Serial.println("== ERROR ==");
}

void handle_reg(uint8_t reg) {
  uint8_t buf[5];
  switch(reg) {
    CHECK_REG( CONFIG,     0x08 );
    CHECK_REG( EN_AA,      0x3f );
    CHECK_REG( EN_RXADDR,  0x03 );
    CHECK_REG( SETUP_AW,   0x03 );
    CHECK_REG( SETUP_RETR, 0x03 );
    CHECK_REG( RF_CH,      0x02 );
    CHECK_REG( RF_SETUP,   0x0f );
    CHECK_REG( NRF_STATUS, 0x0e );
    CHECK_REG( OBSERVE_TX, 0x00 );
    CHECK_REG( RPD,        0x00 );
    CHECK_REG( RX_PW_P0,   0x00 );
    CHECK_REG( RX_PW_P1,   0x00 );
    CHECK_REG( RX_PW_P2,   0x00 );
    CHECK_REG( RX_PW_P3,   0x00 );
    CHECK_REG( RX_PW_P4,   0x00 );
    CHECK_REG( RX_PW_P5,   0x00 );
    CHECK_REG( DYNPD,      0x00 );
    CHECK_REG( FEATURE,    0x00 );

    case RX_ADDR_P0:
    check_addr(RX_ADDR_P0, STR(RX_ADDR_P0), 0xE7);
    break;

    case RX_ADDR_P1:
    check_addr(RX_ADDR_P1, STR(RX_ADDR_P1), 0xC2);
    break;

    CHECK_REG( RX_ADDR_P2, 0xC3 );
    CHECK_REG( RX_ADDR_P3, 0xC4 );
    CHECK_REG( RX_ADDR_P4, 0xC5 );
    CHECK_REG( RX_ADDR_P5, 0xC6 );

    case TX_ADDR:
    check_addr(TX_ADDR, STR(TX_ADDR), 0xE7);
    break;
  }
}

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(CSN, OUTPUT);
  digitalWrite(CSN, HIGH);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV2);


  delay(10);

  for (int i=0; i <= FEATURE; ++i) {
    handle_reg(i);
  }
}

// the loop routine runs over and over again forever:
void loop() {
}
