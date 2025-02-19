/*
  This program reads the light intensity using an LDR (Light Dependent Resistor) connected to GPIO34 of the ESP32.
  The LDR value is read by the 'LDR' FreeRTOS task, and the brightness of an LED is adjusted 
  based on the LDR value using PWM in the 'LEDglow' FreeRTOS task.

  - Two tasks run concurrently: 
    1. 'LDR' task: Reads the LDR value and prints it to the Serial Monitor.
    2. 'LEDglow' task: Adjusts the LED brightness based on the LDR value using PWM.
*/

#include <Arduino.h>

#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

SemaphoreHandle_t Mutex;

uint8_t ldrvalue;

static const int ledpin = 2;
static const int ldrpin = 34;

//Task1 : To Read the LDR value
void LDR(void *parameter) {
  while (1) {
    if (xSemaphoreTake(Mutex, portMAX_DELAY)) {
      ldrvalue = analogRead(ldrpin);
      Serial.print("LDR value is : ");
      Serial.println(ldrvalue);
      xSemaphoreGive(Mutex);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void LEDglow(void *parameter) {
  while (1) {
    analogWrite(ledpin, ldrvalue);
  }
  vTaskDelay(50 / portTICK_PERIOD_MS);  // Delay to avoid continuous executionU
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(ledpin, OUTPUT);
  pinMode(ldrpin, INPUT);

  Mutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(LDR, "LDR", 1024, NULL, 2, NULL, app_cpu);
  xTaskCreatePinnedToCore(LEDglow, "LER", 1024, NULL, 1, NULL, app_cpu);

  vTaskDelete(NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
}
