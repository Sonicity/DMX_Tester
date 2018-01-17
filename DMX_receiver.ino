/*
 * Dmx Tester
 * 
   See http://www.maxpierson.me/2009/03/20/receive-dmx-512-with-an-arduino/
   See https://github.com/mathertel/DMXSerial/blob/master/src/DMXSerial.cpp
   See https://github.com/mathertel/DmxSerial2/blob/master/src/DMXSerial2.cpp
   See http://wormfood.net/avrbaudcalc.php
*/

#include <LiquidCrystal.h>

// Miscelaeous pins
#define RX_STATUS_LED_PIN 8
#define PWM_LED_PIN 9
#define MAIN_LOOP_LED 13

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
#define DMX_CHANNEL_OFFSET 1  // Start at channel 1
#define DMX_SERIAL_MAX 512    // Read 512 values

// State of receiving DMX Bytes
typedef enum {
  STARTUP = 1,  // wait for any interrupt BEFORE starting anylyzig the DMX protocoll.
  IDLE    = 2,  // wait for a BREAK condition.
  BREAK   = 3,  // BREAK was detected.
  DATA    = 4,  // DMX data.
  DONE    = 5   // All channels received.
} __attribute__((packed)) DMXReceivingState;


LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

uint8_t _dmxRecvState;  // Current State of receiving DMX Bytes

volatile unsigned long _dmxLastPacket = 0; // the last time (using the millis function) a packet was received.

bool _dmxUpdated = true; // is set to true when new data arrived.
bool _dmxNoSignal = false;

// Array of DMX values (raw).
// Entry 0 will never be used for DMX data but will store the startbyte (0 for DMX mode).
uint8_t  _dmxData[DMX_SERIAL_MAX + 1];

// This pointer will point to the next byte in _dmxData;
uint8_t *_dmxDataPtr;

// This pointer will point to the last byte in _dmxData;
uint8_t *_dmxDataLastPtr;

// Current channel offset
int channelOffset = DMX_CHANNEL_OFFSET;

void setup() {
  pinMode(RX_TX_MODE_PIN, OUTPUT);
  pinMode(RX_STATUS_LED_PIN, OUTPUT); //RX STATUS LED pin, blinks on incoming DMX data
  pinMode(PWM_LED_PIN, OUTPUT);
  pinMode(MAIN_LOOP_LED, OUTPUT);

  digitalWrite(RX_STATUS_LED_PIN, LOW);
  analogWrite(PWM_LED_PIN, 0);
  digitalWrite(RX_TX_MODE_PIN, LOW); // Receive
  digitalWrite(MAIN_LOOP_LED, LOW);

  lcd.begin(16, 2);
  
  lcd.print("DMX Tester  C");
  if (channelOffset < 100) {
      lcd.print("0");
    }
    if (channelOffset < 10) {
      lcd.print("0");
    }
    lcd.print(channelOffset);
    
  // Disable interrupts
  cli();

  configure_serial();

  // Enable interrupts
  sei();

  _dmxDataLastPtr = _dmxData + DMX_SERIAL_MAX;
}

void loop()  {
  /* Based on DMX speed we have roughly 22ms to complete the tasks in 
   * the _dmxupdated loop. That is the time it takes the entire DMX sequence
   * to be sent. Of course we need to substract the time take by the ISR, but 
   * it gives a rough indication about the amount of work we can do here without
   * worrying about missed frames.
   */
  if (_dmxUpdated) {    //when a new set of values are received, jump to action loop...
    digitalWrite(MAIN_LOOP_LED, HIGH);    
    _dmxNoSignal = false;
    _dmxUpdated = false;
    action();
    digitalWrite(MAIN_LOOP_LED, LOW);  
  } else if(_dmxLastPacket < (millis() - 100) && !_dmxNoSignal) {
    lcd.setCursor(0, 1);
    lcd.print("No Signal       ");
    _dmxNoSignal = true;
  }
} //end loop()

void action() {
  /* Currently timed at rougly 4.7ms 
  */
  analogWrite(PWM_LED_PIN, _dmxData[channelOffset]);

  lcd.setCursor(0, 1);
  for (int k = channelOffset; k < channelOffset + 4; k++) {
    lcd.print((char)((unsigned)_dmxData[k] / 100      + '0'));
    lcd.print((char)((unsigned)_dmxData[k] / 10  % 10 + '0'));
    lcd.print((char)((unsigned)_dmxData[k]       % 10 + '0'));
    if (k < channelOffset + 3) {
      lcd.print(' ');
    }
  }
}

ISR(USART_RX_vect) {
  /* The receive buffer (UDR0) must be read during the reception ISR, or the ISR will just
     execute again immediately upon exiting.

     This ISR triggers on every byte received, given the baud rate of DMX this
     method should complete within 44us to make sure the next byte can be received
     in time.
  */
  uint8_t  USARTstate = UCSR0A;    // get state before data!
  uint8_t  DmxByte    = UDR0;     // get data
  uint8_t  DmxState   = _dmxRecvState;  //just load once from SRAM to increase speed

  if (DmxState == STARTUP) {
    // just ignore any first frame comming in
    _dmxRecvState = IDLE;
    return;
  }

  if (USARTstate & (1 << FE0)) { 
    /* Check for frame error indicating a break
     * This indicates a start of the sequence in DMX 
     */
    _dmxRecvState = BREAK;
    _dmxDataPtr = _dmxData;

  } else if (DmxState == BREAK) {
    // first byte after a break was read.
    if (DmxByte == 0) {
      // normal DMX start code (0) detected
      digitalWrite(RX_STATUS_LED_PIN, HIGH);
      
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
      
      // store received data into dmx data buffer.
      *_dmxDataPtr = DmxByte;
    } // if
    _dmxDataPtr++;

    if (_dmxDataPtr > _dmxDataLastPtr) {
      // all channels received.
      _dmxRecvState = DONE;
      _dmxUpdated = true;
    } // if
  } // if

  if (_dmxRecvState == DONE) {
    // continue on DMXReceiver mode.
    digitalWrite(RX_STATUS_LED_PIN, LOW);
    
    _dmxRecvState = IDLE; // wait for next break
  }
}

void configure_serial() {
  // Set Baudrate
  uint16_t baud_setting = 0x03; // DMX 250000 baud
  UBRR0H = baud_setting >> 8;
  UBRR0L = baud_setting;

  // Clear out current config / status
  UCSR0A = 0;
  UCSR0B = 0;
  UCSR0C = 0;

  bitSet(UCSR0B, RXEN0);    // Enable receiver
  bitSet(UCSR0B, RXCIE0);   // Enable receive complete interrupt

  bitClear(UCSR0B, TXEN0);  // Disable transmitter
  bitClear(UCSR0B, TXCIE0); // Disable transmit complete interrupt
  bitClear(UCSR0B, UDRIE0); // Disable data register empty interrupt

  bitSet(UCSR0C, USBS0);    // 2 stop bits
  bitSet(UCSR0C, UCSZ01);   // 8 bit characters 
  bitSet(UCSR0C, UCSZ00);   // combined with above

  // Clear out any pending data
  uint8_t  voiddata;
  while (UCSR0A & (1 << RXC0)) {
    voiddata = UDR0; // get data
  }
}
