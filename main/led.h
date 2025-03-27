//#Pragma once
#include <CodeCell.h>
//#include <Arduino.h>

#define rainbow_brig 10u
#define LED_PIN 10U
#define LED_DEFAULT_BRIGHTNESS 7U
#define LED_SLEEP_BRIGHTNESS 3U

#define LED_COLOR_RED 0XFF0000U
#define LED_COLOR_ORANGE 0XC04000U
#define LED_COLOR_YELLOW 0XA06000U
#define LED_COLOR_GREEN 0X00FF00U
#define LED_COLOR_AQUA 0X00A030U
#define LED_COLOR_PINK 0XC00020U
#define LED_COLOR_BLUE 0X0000FFU
#define LED_COLOR_WHITE 0XFFFFFFU



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

// change for not blocking
void rainbow()
{
  LED(0, 0, 0);
  vTaskDelay(pdMS_TO_TICKS(1));
  LED(0, 0, rainbow_brig);
  vTaskDelay(pdMS_TO_TICKS(80));
  LED(rainbow_brig, 0, 0);
  vTaskDelay(pdMS_TO_TICKS(80));
  LED(0, rainbow_brig, 0);
  vTaskDelay(pdMS_TO_TICKS(80));
  LED(0, 0, 0);
  vTaskDelay(pdMS_TO_TICKS(80));
}

