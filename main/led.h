//#Pragma once
#include <CodeCell.h>
//#include <Arduino.h>

#define rainbow_brig 10u
void LED(uint8_t r, uint8_t g, uint8_t b) {neopixelWrite(LED_PIN, r, g, b); /*RMT ESP32 function for addressable LEDs*/}
void led_init()
{
  pinMode(LED_PIN, OUTPUT);   /*Set LED pin as output*/
  digitalWrite(LED_PIN, LOW); /*Init Set up to output low*/
  delay(1);
  rmtInit(LED_PIN, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000); /*Configure RMT to run the onboard addressable LED*/

  LED(0, 0, 0);
  delay(1);
  LED(0, 0, LED_SLEEP_BRIGHTNESS);
  delay(80);
  LED(LED_SLEEP_BRIGHTNESS, 0, 0);
  delay(80);
  LED(0, LED_SLEEP_BRIGHTNESS, 0);
  delay(80);
  LED(0, 0, 0);
  delay(80);
}

void rainbow()
{
  LED(0, 0, 0);
  delay(1);
  LED(0, 0, rainbow_brig);
  delay(80);
  LED(rainbow_brig, 0, 0);
  delay(80);
  LED(0, rainbow_brig, 0);
  delay(80);
  LED(0, 0, 0);
  delay(80);
}

