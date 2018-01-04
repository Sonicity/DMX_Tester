/*
   See http://www.maxpierson.me/2009/03/20/receive-dmx-512-with-an-arduino/
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

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

#define NUMBER_OF_CHANNELS 100

volatile byte i = 0;              //dummy variable for dmxvalue[]
volatile byte dmxreceived = 0;    //the latest received value
volatile unsigned int dmxcurrent = 0;     //counter variable that is incremented every time we receive a value.
volatile byte dmxvalue[NUMBER_OF_CHANNELS];
/*  stores the DMX values we're interested in using--
    keep in mind that this is 0-indexed. */
volatile boolean dmxnewvalue = false;
/*  set to 1 when updated dmx values are received
    (even if they are the same values as the last time). */
unsigned int dmxaddress = 1;

unsigned short num_received = 0;
unsigned short frame_errors = 0;
unsigned short frame_complete = 0;
unsigned short data_overrun = 0;

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

  cli(); //disable interrupts while we're setting bits in registers

  configure_serial();
  configure_timer_2();

  sei();

}

void loop()  {
  // the processor gets parked here while the ISRs are doing their thing.
  if (dmxnewvalue == 1) {    //when a new set of values are received, jump to action loop...
    action();
    dmxnewvalue = 0;
    dmxcurrent = 0;
    zerocounter = 0;      //and then when finished reset variables and enable timer2 interrupt
    i = 0;
    bitSet(TIMSK2, OCIE2A);    //Enable Timer/Counter2 Output Compare Match A Interrupt
  }
} //end loop()

void action() {
  //analogWrite(PWM_LED_PIN, dmxvalue[0]);

  lcd.setCursor(0, 1);
  for (int k = 0; k < 3; k++) {
    if (dmxvalue[k] < 100) {
      lcd.print("0");
    }
    if (dmxvalue[k] < 10) {
      lcd.print("0");
    }
    lcd.print(dmxvalue[k]);
    lcd.print(" ");
  }

  lcd.setCursor(4, 0);
  lcd.print(frame_errors);
  lcd.print(" ");
  lcd.print(frame_complete);
  lcd.print(" ");
  lcd.print(data_overrun);
  lcd.print(" ");
  
}

ISR(TIMER2_COMPA_vect) {
  digitalWrite(PWM_LED_PIN, HIGH);
  if (bitRead(PIND, PIND0)) {  // if a one is detected, we're not in a break, reset zerocounter.
    zerocounter = 0;
  }
  else {
    zerocounter++;             // increment zerocounter if a zero is received.
    if (zerocounter == 20)     // if 20 0's are received in a row (80uS break)
    {
      bitClear(TIMSK2, OCIE2A);    //disable this interrupt and enable reception interrupt now that we're in a break.
      bitSet(UCSR0B, RXCIE0);
    }
  }
  digitalWrite(PWM_LED_PIN, LOW);
}

ISR(USART_RX_vect) {
  /* The receive buffer (UDR0) must be read during the reception ISR, or the ISR will just
   * execute again immediately upon exiting. 
   */
   
  digitalWrite(RX_STATUS_LED_PIN, HIGH);
  frame_errors += bitRead(UCSR0A, FE0);
  frame_complete += bitRead(UCSR0A, RXC0);
  data_overrun += bitRead(UCSR0A, DOR0);
  
  dmxreceived = UDR0;
  
  dmxcurrent++;                        //increment address counter
  
  if (dmxcurrent > dmxaddress) {        //check if the current address is the one we want.
    dmxvalue[i] = dmxreceived;
    i++;
    if (i == NUMBER_OF_CHANNELS) {
      bitClear(UCSR0B, RXCIE0);
      dmxnewvalue = 1;                        //set newvalue, so that the main code can be executed.
    }
  }
  
  digitalWrite(RX_STATUS_LED_PIN, LOW);
}

void configure_serial() {
  // Set Baudrate
  uint16_t baud_setting = (F_CPU / 4 / DMX_BAUDRATE - 1) / 2;
  UCSR0A = 1 << U2X0;
  
  UBRR0H = baud_setting >> 8;
  UBRR0L = baud_setting;
 
  UCSR0B = 0x0;
  UCSR0C = SERIAL_8N2;

  bitSet(UCSR0B, RXEN0);
  bitSet(UCSR0B, TXEN0);
  bitClear(UCSR0B, TXCIE0);
  bitClear(UCSR0B, RXCIE0);
  bitClear(UCSR0B, UDRIE0);
}

void configure_timer_2() {
  bitClear(TCCR2A, COM2A1);
  bitClear(TCCR2A, COM2A0); //disable compare match output A mode
  bitClear(TCCR2A, COM2B1);
  bitClear(TCCR2A, COM2B0); //disable compare match output B mode
  bitSet(TCCR2A, WGM21);
  bitClear(TCCR2A, WGM20);  //set mode 2, CTC.  TOP will be set by OCRA.

  bitClear(TCCR2B, FOC2A);
  bitClear(TCCR2B, FOC2B);  //disable Force Output Compare A and B.
  bitClear(TCCR2B, WGM22);  //set mode 2, CTC.  TOP will be set by OCRA.
  bitClear(TCCR2B, CS22);
  bitClear(TCCR2B, CS21);
  bitSet(TCCR2B, CS20);   // no prescaler means the clock will increment every 62.5ns (assuming 16Mhz clock speed).

  OCR2A = 64;
  /* Set output compare register to 64, so that the Output Compare Interrupt will fire
     every 4uS.  */

  bitClear(TIMSK2, OCIE2B);  //Disable Timer/Counter2 Output Compare Match B Interrupt
  bitSet(TIMSK2, OCIE2A);    //Enable Timer/Counter2 Output Compare Match A Interrupt
  bitClear(TIMSK2, TOIE2);   //Disable Timer/Counter2 Overflow Interrupt Enable
}

