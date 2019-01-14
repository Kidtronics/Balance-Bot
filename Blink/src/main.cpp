#include <Arduino.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // Set output mode for PA16/D13.
  PORT->Group[PORTA].DIRSET.reg = PORT_PA16;

  // Set input mode for PB17/D2.
  PORT->Group[PORTB].DIRCLR.reg = PORT_PB17;
  PORT->Group[PORTB].WRCONFIG.reg = 
    PORT_WRCONFIG_HWSEL 
    | PORT_WRCONFIG_INEN 
    | (PORT_PB17 >> 16);
}

void loop() {
  PORT->Group[PORTA].OUTTGL.reg = PORT_PA16;
  Serial.print("Input D2: "); 
  Serial.println((PORT->Group[1].IN.reg & PORT_PB17) != 0);
  delay(1000);
}