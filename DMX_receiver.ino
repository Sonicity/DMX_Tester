/*
   See http://www.maxpierson.me/2009/03/20/receive-dmx-512-with-an-arduino/
   See https://github.com/mathertel/DMXSerial/blob/master/src/DMXSerial.cpp
*/

#include <LiquidCrystal.h>

// Miscelaeous pins
#define RX_STATUS_LED_PIN 8
#define PWM_LED_PIN 9

// Pins connected to the SN75176AP
#define RX_TX_MODE_PIN 7  // Toggles Driver Enable / Receiver Enable

// Pins connected to the LCD
#define LCD_RS 12
#define LCD_EN 11
#define LCD_D4 5
#define LCD_D5 4
#define LCD_D6 3
#define LCD_D7 2

// DMX
#define DMX_BAUDRATE 250000
#define DMX_SERIAL_MAX 512

// State of receiving DMX Bytes
typedef enum {
  STARTUP = 1,  // wait for any interrupt BEFORE starting anylyzig the DMX protocoll.
  IDLE    = 2,  // wait for a BREAK condition.
  BREAK   = 3,  // BREAK was detected.
  DATA    = 4,  // DMX data.
  DONE    = 5   // All channels received.
} __attribute__((packed)) DMXReceivingState;


LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

#define NUMBER_OF_CHANNELS 100

uint8_t _dmxRecvState;  // Current State of receiving DMX Bytes
int     _dmxChannel;  // the next channel byte to be sent.

volatile unsigned int  _dmxMaxChannel = 32; // the last channel used for sending (1..32).
volatile unsigned long _dmxLastPacket = 0; // the last time (using the millis function) a packet was received.

bool _dmxUpdated = true; // is set to true when new data arrived.

// Array of DMX values (raw).
// Entry 0 will never be used for DMX data but will store the startbyte (0 for DMX mode).
uint8_t  _dmxData[DMX_SERIAL_MAX + 1];

// This pointer will point to the next byte in _dmxData;
uint8_t *_dmxDataPtr;

// This pointer will point to the last byte in _dmxData;
uint8_t *_dmxDataLastPtr;

/******************************* Timer2 variable declarations *****************************/

volatile byte zerocounter = 0;

void setup() {
  pinMode(RX_TX_MODE_PIN, OUTPUT);
  pinMode(RX_STATUS_LED_PIN, OUTPUT); //RX STATUS LED pin, blinks on incoming DMX data
  pinMode(PWM_LED_PIN, OUTPUT);

  digitalWrite(RX_STATUS_LED_PIN, LOW);
  analogWrite(PWM_LED_PIN, 0);
  digitalWrite(RX_TX_MODE_PIN, LOW); // Receive

  lcd.begin(16, 2);

  lcd.print("DMX Tester");

  // Disable interrupts
  cli();

  configure_serial();

  // Enable interrupts
  sei();

}

void loop()  {
  // the processor gets parked here while the ISRs are doing their thing.

  if (_dmxUpdated) {    //when a new set of values are received, jump to action loop...
    action();
  }
} //end loop()

void action() {
  analogWrite(PWM_LED_PIN, _dmxData[1]);

  lcd.setCursor(0, 1);
  for (int k = 1; k < 5; k++) {
    if (_dmxData[k] < 100) {
      lcd.print("0");
    }
    if (_dmxData[k] < 10) {
      lcd.print("0");
    }
    lcd.print(_dmxData[k]);
    lcd.print(" ");
  }
}

ISR(USART_RX_vect) {
  /* The receive buffer (UDR0) must be read during the reception ISR, or the ISR will just
     execute again immediately upon exiting.
  */

  digitalWrite(RX_STATUS_LED_PIN, HIGH);

  uint8_t  USARTstate = UCSR0A;    // get state before data!
  uint8_t  DmxByte    = UDR0;     // get data
  uint8_t  DmxState   = _dmxRecvState;  //just load once from SRAM to increase speed

  if (DmxState == STARTUP) {
    // just ignore any first frame comming in
    _dmxRecvState = IDLE;
    return;
  }

  if (USARTstate & (1 << FE0)) {  //check for break
    // break condition detected.
    _dmxRecvState = BREAK;
    _dmxDataPtr = _dmxData;
    _dmxDataLastPtr = _dmxData + 4;

  } else if (DmxState == BREAK) {
    // first byte after a break was read.
    if (DmxByte == 0) {
      // normal DMX start code (0) detected
      _dmxRecvState = DATA;
      _dmxLastPacket = millis(); // remember current (relative) time in msecs.
      _dmxDataPtr++; // start saving data with channel # 1

    } else {
      // This might be a RDM or customer DMX command -> not implemented so wait for next BREAK !
      _dmxRecvState = DONE;
    } // if

  } else if (DmxState == DATA) {
    // check for new data
    if (*_dmxDataPtr != DmxByte) {
      _dmxUpdated = true;
      // store received data into dmx data buffer.
      *_dmxDataPtr = DmxByte;
    } // if
    _dmxDataPtr++;

    if (_dmxDataPtr > _dmxDataLastPtr) {
      // all channels received.
      _dmxRecvState = DONE;
    } // if
  } // if

  if (_dmxRecvState == DONE) {
    // continue on DMXReceiver mode.
    _dmxRecvState = IDLE; // wait for next break
  }
  
  digitalWrite(RX_STATUS_LED_PIN, LOW);
}

void configure_serial() {
  // Set Baudrate
  uint16_t baud_setting = (((F_CPU / 8) / DMX_BAUDRATE) - 1) / 2;

  UCSR0A = 0;
  UBRR0H = baud_setting >> 8;
  UBRR0L = baud_setting;

  UCSR0B = 0;
  UCSR0C = ((1 << USBS0) | (2 << UPM00) | (3 << UCSZ00));

  bitSet(UCSR0B, RXEN0);
  bitSet(UCSR0B, RXCIE0);

  bitClear(UCSR0B, TXEN0);
  bitClear(UCSR0B, TXCIE0);
  bitClear(UCSR0B, UDRIE0);

  // Clear out any pending data
  uint8_t  voiddata;
  while (UCSR0A & (1 << RXC0)) {
    voiddata = UDR0; // get data
  }
}
