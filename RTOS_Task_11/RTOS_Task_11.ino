/*
  This program reads temperature and humidity from a DHT11 sensor and light intensity from an LDR sensor using separate FreeRTOS tasks. 
  The sensor data is transmitted via a queue and printed with a timestamp.

  - Uses FreeRTOS tasks for concurrent sensor readings.
  - Implements a queue for inter-task communication.
  - Formats and displays timestamps in HH:MM:SS format.
*/

#include <Arduino.h>
#include <DHT.h>

#define DHT_PIN 23
#define DHTTYPE DHT11
#define LDR_PIN 15

#define DHT_SENSOR 0
#define LDR_SENSOR 1

DHT DHT_Sensor(DHT_PIN, DHTTYPE);
QueueHandle_t myQueue;

struct SensorValue {
  int type;
  float temp;
  float humidity;
  int ldr;
  uint32_t timestamp;  // Timestamp in seconds
};

// Function to format and print the timestamp as HH:MM:SS
void printTimestamp(uint32_t seconds) {
  uint32_t hours = (seconds / 3600) % 24;
  uint32_t minutes = (seconds / 60) % 60;
  uint32_t secs = seconds % 60;
  Serial.printf("[%02d:%02d:%02d] ", hours, minutes, secs);
}

void DHTSensor(void *pvParameters) {
  while (1) {
    SensorValue data;
    data.type = DHT_SENSOR;
    data.temp = DHT_Sensor.readTemperature();
    data.humidity = DHT_Sensor.readHumidity();
    data.ldr = -1;                                    // Placeholder for LDR value
    data.timestamp = esp_timer_get_time() / 1000000;  // Convert microseconds to seconds

    xQueueSend(myQueue, &data, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1000));  // Delay 1 second
  }
}

void LDRSensor(void *parameter) {
  while (1) {
    SensorValue data;
    data.type = LDR_SENSOR;
    data.temp = -1;  // Placeholder for temperature
    data.humidity = -1;
    data.ldr = analogRead(LDR_PIN);                   // Read LDR value
    data.timestamp = esp_timer_get_time() / 1000000;  // Convert microseconds to seconds

    xQueueSend(myQueue, &data, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1000));  // Delay 1 second
  }
}

void Print(void *pvParameters) {
  SensorValue receivedValue;
  while (1) {
    if (xQueueReceive(myQueue, &receivedValue, portMAX_DELAY) == pdPASS) {
      printTimestamp(receivedValue.timestamp); 

      if (receivedValue.type == DHT_SENSOR) {
        Serial.print("Temperature = ");
        Serial.print(receivedValue.temp);
        Serial.println(" °C");

        Serial.print("Humidity = ");
        Serial.println(receivedValue.humidity);
      } 
      else if (receivedValue.type == LDR_SENSOR) {
        Serial.print("LDR reading = ");
        Serial.println(receivedValue.ldr);
      }
      Serial.println("----------------------");
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LDR_PIN, INPUT);
  DHT_Sensor.begin();

  myQueue = xQueueCreate(5, sizeof(SensorValue));

  xTaskCreate(DHTSensor, "DHTSender", 2048, NULL, 1, NULL);
  xTaskCreate(LDRSensor, "LDRSender", 2048, NULL, 1, NULL);
  xTaskCreate(Print, "Print", 2048, NULL, 1, NULL);
}

void loop() {
  // No code needed in loop, as tasks handle everything
}
