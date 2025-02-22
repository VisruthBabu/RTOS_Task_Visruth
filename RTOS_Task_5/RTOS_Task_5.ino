/*
  ESP32 FreeRTOS LED Toggle using Interrupts and Binary Semaphore

  - A push button (GPIO23) triggers an interrupt on FALLING edge.
  - The ISR gives a binary semaphore to signal the LED task.
  - The task waits for the semaphore and toggles the LED (GPIO2).
*/

#include <Arduino.h>

#if CONFIG_FREERTOS_UICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

SemaphoreHandle_t BinarySemaphore;

#define LED_PIN 2
#define PUSHBUTTON 23

void interruptHandler() {
  xSemaphoreGiveFromISR(BinarySemaphore, NULL);
}

//Task1 to toggle the LED
void ToggleLED(void *parameter) {
  while (1) {
    if (xSemaphoreTake(BinarySemaphore, portMAX_DELAY)) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
  pinMode(PUSHBUTTON, INPUT);

  BinarySemaphore = xSemaphoreCreateBinary();

  xTaskCreatePinnedToCore(ToggleLED, "LEDToggle", 2048, NULL, 1, NULL, app_cpu);

  if (BinarySemaphore != NULL) {
    attachInterrupt(digitalPinToInterrupt(PUSHBUTTON), interruptHandler, FALLING);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}
