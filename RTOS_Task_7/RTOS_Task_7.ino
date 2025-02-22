/* 
 * This program demonstrates the use of a counting semaphore to synchronize 
 * multiple consumer tasks with a single producer task in an ESP32 environment.
 * 
 * - The ProducerTask reads an analog value from an LDR sensor and updates a shared variable.
 * - It then releases the semaphore multiple times to allow consumer tasks to read the value.
 * - Four ConsumerTasks wait for the semaphore and print the sensor value when acquired.
 * - The counting semaphore ensures that each consumer gets a turn to process the produced data.
 */

#include <Arduino.h>

#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

#define SENSOR_PIN 34

SemaphoreHandle_t CountingSemaphore;

int sensorvalue;

//Producer Task to read data from a LDR sensor
void ProducerTask(void *paramter) {
  while (1) {
    sensorvalue = analogRead(SENSOR_PIN);
    Serial.print("Produced: ");
    Serial.println(sensorvalue);
    xSemaphoreGive(CountingSemaphore);
    xSemaphoreGive(CountingSemaphore);
    xSemaphoreGive(CountingSemaphore);
    xSemaphoreGive(CountingSemaphore);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void ConsumerTask1(void *parameter) {
  while (1) {
    if (xSemaphoreTake(CountingSemaphore, portMAX_DELAY)) {
      Serial.print("Consumed1: ");
      Serial.println(sensorvalue);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void ConsumerTask2(void *parameter) {
  while (1) {
    if (xSemaphoreTake(CountingSemaphore, portMAX_DELAY)) {
      Serial.print("Consumed2: ");
      Serial.println(sensorvalue);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void ConsumerTask3(void *parameter) {
  while (1) {
    if (xSemaphoreTake(CountingSemaphore, portMAX_DELAY)) {
      Serial.print("Consumed3: ");
      Serial.println(sensorvalue);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void ConsumerTask4(void *parameter) {
  while (1) {
    if (xSemaphoreTake(CountingSemaphore, portMAX_DELAY)) {
      Serial.print("Consumed4: ");
      Serial.println(sensorvalue);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(SENSOR_PIN, INPUT);

  CountingSemaphore = xSemaphoreCreateCounting(4, 0);

  xTaskCreatePinnedToCore(ProducerTask, "ProducerTask", 2048, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(ConsumerTask1, "ConsumerTask", 2048, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(ConsumerTask2, "ConsumerTask", 2048, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(ConsumerTask3, "ConsumerTask", 2048, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(ConsumerTask4, "ConsumerTask", 2048, NULL, 1, NULL, app_cpu);
}

void loop() {
  // put your main code here, to run repeatedly:
}
