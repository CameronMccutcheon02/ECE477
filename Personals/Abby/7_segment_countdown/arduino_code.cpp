#include <SPI.h>
#include <digitalWriteFast.h>

// Define the HC595 pins
const int latchPin = 10;   // Connect to the HC595's RCLK (ST_CP) pin 
const int clockPin = 13;   // Connect to the HC595's SRCLK (SH_CP) pin PB3
const int dataPin = 11;    // Connect to the HC595's SER (DS) pin
byte segmentSelect = 0x00; 
byte displaySelect = 0x01;

void setup() {
  // Set the SPI mode and initialize SPI
  SPI.setDataMode(SPI_MODE0); // You may need to change the mode depending on your HC595 settings
//  SPI.setClockDivider(SPI_CLOCK_DIV2); // You may need to change the divider based on your HC595 datasheet
  SPI.begin();
  //Timer Setup ********
  TCCR1A = B00000000;//Register A all 0's since we're not toggling any pins
  TCCR1B = B00001011;//bit 3 set to place in CTC mode, will call an interrupt on a counter match
  //bits 0 and 1 are set to divide the clock by 64, so 16MHz/64=250kHz
  TIMSK1 = B00000010;//bit 1 set to call the interrupt on an OCR1A match
  OCR1A = 700;
  interrupts();               //Enable interrupt
  //TIMER SETUP END
  // Initialize the HC595 pins
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  // Set the initial state of the latch pin (low)
  digitalWrite(latchPin, LOW);
}
void loop() {
}
ISR(TIMER1_COMPA_vect) {
  // Send data (0xFF) to the HC595 
  digitalWriteFast(latchPin, LOW); // Pull the latch pin LOW to start sending data
  // Send the data out using SPI
  SPI.transfer(segmentSelect);
  SPI.transfer(displaySelect);
  // Pull the latch pin HIGH to update the outputs on the HC595
  digitalWriteFast(latchPin, HIGH);
  displaySelect = displaySelect << 1;// Delay for one second before sending new data
  if (displaySelect == 0x00) {
    displaySelect = 0x01;
  }
  delay(5);
}