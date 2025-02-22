/*
  ESP32 FreeRTOS Binary Semaphore Example

  - Two tasks (Task1 & Task2) increment a shared variable.
  - A Binary Semaphore ensures mutual exclusion, allowing only one task to modify the variable at a time.
  - Tasks release the semaphore after working for 1000ms.
*/

#include <Arduino.h>

SemaphoreHandle_t BinarySemaphore;

uint8_t variable = 0;


void Task1(void *pvParameters) {
  while (1) {
      Serial.println("Task1 incrementing variable..");
      variable++;
      Serial.println(variable);
      vTaskDelay(1000 / portTICK_PERIOD_MS);  // Work for 1000ms
      xSemaphoreGive(BinarySemaphore);
    vTaskDelay(100 / portTICK_PERIOD_MS);  
  }
}

void Task2(void *pvParameters) {
  while (1) {
    if (xSemaphoreTake(BinarySemaphore, portMAX_DELAY)) {
      Serial.println("Task2 incrementing variable..");
      variable++;
      Serial.println(variable);
      vTaskDelay(1000 / portTICK_PERIOD_MS);  // Work for 1000ms
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); //for the next task to take the semaphore
  }
}

void setup() {
  Serial.begin(115200);

  BinarySemaphore = xSemaphoreCreateBinary();

  xTaskCreate(Task2, "Task2", 2048, NULL, 1, NULL);
  xTaskCreate(Task1, "Task1", 2048, NULL, 1, NULL);

  xSemaphoreGive(BinarySemaphore);
}

void loop() {
  // Not used in FreeRTOS applications
}
