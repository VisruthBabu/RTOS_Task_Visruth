/*
 * ESP32 FreeRTOS DHT11 Sensor Reader
 * 
 * This program reads temperature and humidity data from a DHT11 sensor 
 * using FreeRTOS tasks on an ESP32. The sensor data is sent to a queue 
 * when a button is pressed, and another task retrieves and prints it 
 * to the Serial Monitor.
 */

#include <Arduino.h>
#include <DHT.h>

QueueHandle_t myQueue;

struct SensorValue {
  float temp;
  float humidity;
};

#define DHT_PIN 23
#define DHTTYPE DHT11
DHT DHT_Sensor(DHT_PIN, DHTTYPE);

#define BUTTON 5

void DHTSensor(void *pvParameters) {
  SensorValue data;
  while (1) {
    if (digitalRead(BUTTON)) {
      data.temp = DHT_Sensor.readTemperature();
      data.humidity = DHT_Sensor.readHumidity();

      xQueueSend(myQueue, &data, portMAX_DELAY);
      vTaskDelay(pdMS_TO_TICKS(200));  // Debounce delay
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // Delay 1 second
  }
}

void Print(void *pvParameters) {
  SensorValue receivedValue;
  while (1) {
    if (xQueueReceive(myQueue, &receivedValue, portMAX_DELAY) == pdPASS) {
      Serial.print("Temperature = ");
      Serial.print(receivedValue.temp);
      Serial.println(" °C");

      Serial.print("Humidity = ");
      Serial.println(receivedValue.humidity);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON, INPUT);
  DHT_Sensor.begin();  // Ensure sensor initialization

  myQueue = xQueueCreate(5, sizeof(SensorValue));

  xTaskCreate(DHTSensor, "DHTSender", 2048, NULL, 1, NULL);
  xTaskCreate(Print, "Print", 2048, NULL, 1, NULL);
}

void loop() {
  // Not used in FreeRTOS applications
}
