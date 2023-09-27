#include <SPI.h>

#define DM13A_CLK 9
#define DM13A_LAT 7
#define DM13A_DAI 8

byte segment_select = 0x00;

void setup() {
  // put your setup code here, to run once:
  pinMode(DM13A_LAT, OUTPUT);
  pinMode(DM13A_CLK, OUTPUT);
  pinMode(DM13A_DAI, OUTPUT);
  
  
  digitalWrite(DM13A_LAT, LOW);
  digitalWrite(DM13A_DAI, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(DM13A_LAT,HIGH);
  shiftOut(DM13A_DAI,DM13A_CLK,MSBFIRST,0xFF);
  shiftOut(DM13A_DAI,DM13A_CLK,MSBFIRST,0xFF);
  digitalWrite(DM13A_LAT,LOW);

}
