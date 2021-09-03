#include <Arduino.h>

void setup() {
  // put your setup code here, to run once:
  pinMode (LED_BUILTIN,OUTPUT);
  Serial.begin(115200);
}

#pragma GCC push_options
#pragma GCC optimize ("O3")
void delayUS_DWT(uint32_t us) {
volatile uint32_t cycles = (SystemCoreClock/1000000L)*us;
volatile uint32_t start = DWT->CYCCNT;
do {
} while(DWT->CYCCNT - start < cycles);
}
#pragma GCC pop_options
/*
void testDWT() {
  while (1) {
    digitalWrite(LED_BUILTIN, HIGH);
    DWT->CYCCNT = 0;
    while(DWT->CYCCNT<72000);
    digitalWrite(LED_BUILTIN, LOW);
    DWT->CYCCNT = 0;
    while(DWT->CYCCNT<72000);
  }
}
*/

int i=0;
void loop() {
/*  
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);
  Serial.print("blink "); Serial.println(i); 
  i+= 1;
*/
  // testDWT();
  // test2 DWT
  //digitalWrite(LED_BUILTIN, HIGH);
  GPIOC->BRR = 0x2000; // GPIO_PIN_13
  delayUS_DWT(1);
  GPIOC->BSRR = 0x2000; // GPIO_PIN_13
  //digitalWrite(LED_BUILTIN, LOW);
  delayUS_DWT(100);
}