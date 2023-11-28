#include <SPI.h>
#include <digitalWriteFast.h>
#define LATCH_PIN 10
#define LEDS_PER_LEVEL 64
#define LED_RESOLUTION 8

#define HOLDING_CYCLES 4

uint8_t segvalue[8] = {0,1,2,3,4,5,6,7};
uint8_t display_select = 0;
uint8_t hold = 0;

void setup() {
  pinMode(LATCH_PIN, OUTPUT); //Pin 10 is Storage Clock pin

  SPI.setBitOrder(MSBFIRST);//Most Significant Bit First
  SPI.setDataMode(SPI_MODE0);// Mode 0 Rising edge of data, keep clock low
  SPI.setClockDivider(SPI_CLOCK_DIV2);//Run the data in at 16MHz/2 - 8MHz
  SPI.begin();
  Serial.begin(9600);

  //Timer Setup ********
  TCCR1A = B00000000;//Register A all 0's since we're not toggling any pins
  TCCR1B = B00001011;//bit 3 set to place in CTC mode, will call an interrupt on a counter match
  //bits 0 and 1 are set to divide the clock by 64, so 16MHz/64=250kHz
  TIMSK1 = B00000010;//bit 1 set to call the interrupt on an OCR1A match
  OCR1A = 30;
  interrupts();               //Enable interrupt
  //TIMER SETUP END

  //setFirstLayer();
}

void loop () {

  // display_select = display_select + 1;
  // if (display_select > 9) {
  //   display_select = 0;
  // }
  // delay(1000);
  // Serial.println(display_select);

}

ISR(TIMER1_COMPA_vect) {
  digitalWrite(LATCH_PIN, HIGH);
  
  uint8_t k;
  int i;
  byte segment;
  byte anode_translate;

  segment = TranslateDigit(segvalue[display_select]);
  SPI.transfer(~segment);

  SPI.transfer((0x01 << display_select));

  if ((display_select > 3) && (hold < HOLDING_CYCLES)) {
    hold++;
  }
  else {
    display_select++;
    if (display_select == 8) {
      display_select = 0;
    }
    hold = 0;
  }
  digitalWrite(LATCH_PIN, LOW); //Pull our latch pin high to store the data
}


byte TranslateDigit(int digit) {
    const byte segments[10] = {
        B11111100, // 0
        B01100000, // 1
        B11011010, // 2
        B11110010, // 3
        B01100110, // 4
        B10110110, // 5
        B10111110, // 6
        B11100000, // 7
        B11111110, // 8
        B11110110  // 9
    };

    if (digit >= 0 && digit <= 9) {
        return segments[digit];
    } else {
        // Return 0xFF for an invalid digit (all segments off)
        return B11111111;
    }
}