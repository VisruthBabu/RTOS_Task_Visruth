/* 
 * This program demonstrates the use of a binary semaphore to control task execution 
 * in an ESP32 environment. The Master task alternates between two LED blinking 
 * patterns and synchronizes their execution using a binary semaphore.
 * 
 * - The Master task releases the semaphore periodically and switches between patterns.
 * - Pattern1 and Pattern2 tasks wait for the semaphore and execute their respective 
 *   LED blinking sequences when allowed.
 * - The binary semaphore ensures that only one pattern executes at a time.
 */

#include <Arduino.h>

#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

SemaphoreHandle_t BinarySemaphore;
volatile int patternselector = 1;

//Master Task
void Master(void *parameter) {
  while (1) {
    xSemaphoreGive(BinarySemaphore);
    vTaskDelay(3000 / portTICK_PERIOD_MS);  // Delay before switching patterns
    if (patternselector == 1) {
      patternselector = 2;
    } else {
      patternselector = 1;
    }
  }
}

//Task1
void Pattern1(void *parameter) {
  while (1) {
    if (xSemaphoreTake(BinarySemaphore, portMAX_DELAY)) {
      if (patternselector == 1) {
        Serial.println("Executing Pattern 1...");
        digitalWrite(18, HIGH);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        digitalWrite(18, LOW);
        digitalWrite(19, HIGH);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        digitalWrite(19, LOW);
        digitalWrite(21, HIGH);
        vTaskDelay(1300 / portTICK_PERIOD_MS);
        digitalWrite(21, LOW);
      }
    }
  }
}
//Task2
void Pattern2(void *parameter) {
  while (1) {
    if (xSemaphoreTake(BinarySemaphore, portMAX_DELAY)) {
      if (patternselector == 2) {
        Serial.println("Executing Pattern 2...");
        digitalWrite(18, HIGH);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        digitalWrite(18, LOW);
        digitalWrite(19, HIGH);
        vTaskDelay(150 / portTICK_PERIOD_MS);
        digitalWrite(19, LOW);
        digitalWrite(21, HIGH);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        digitalWrite(21, LOW);
      }
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);
  pinMode(21, OUTPUT);

  BinarySemaphore = xSemaphoreCreateBinary();

  xTaskCreatePinnedToCore(Pattern1, "Pattern1", 2048, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(Pattern2, "Pattern2", 2048, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(Master, "Master", 2048, NULL, 1, NULL, app_cpu);
  xSemaphoreGive(BinarySemaphore);
}

void loop() {
  // put your main code here, to run repeatedly:
}
