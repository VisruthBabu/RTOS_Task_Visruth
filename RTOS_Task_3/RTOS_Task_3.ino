/*
  ESP32 FreeRTOS Multi-Sensor Data Acquisition using Mutex

  - Reads temperature from a DHT11 sensor (GPIO23).
  - Measures distance using an ultrasonic sensor (Trig: GPIO5, Echo: GPIO18).
  - Detects motion using an IR sensor (GPIO4).
  - Uses a Mutex to ensure synchronized access to shared data.
  - Sensor values are printed to Serial Monitor every 500ms.
*/

#include <Arduino.h>
#include <DHT.h>

#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

#define DHT_PIN 23
#define DHTTYPE DHT11
DHT DHT_Sensor(DHT_PIN, DHTTYPE);

#define TRIG_PIN 5
#define ECHO_PIN 18

SemaphoreHandle_t Mutex;

static const int ledpin = 2;
static const int IR_pin = 4;
static const int UltraSonic = 34;

uint8_t value;

struct values {
  float temp;
  int distance;
  int object;
  float duration;
} s;

//Task1 to read the DHT sensor values
void DHTsensor(void *parameter) {
  while (1) {
    if (xSemaphoreTake(Mutex, portMAX_DELAY)) {
      s.temp = DHT_Sensor.readTemperature();  // Read Temperature(°C)
      xSemaphoreGive(Mutex);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

//Task2 to read the IR sensor values
void IRSensor(void *parameter) {
  while (1) {
    if (xSemaphoreTake(Mutex, portMAX_DELAY)) {
      s.object = digitalRead(IR_pin);
      xSemaphoreGive(Mutex);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

//Task3 to read the UltraSonic sensor values
void UltraSonicSensor(void *parameter) {
  while (1) {
    if (xSemaphoreTake(Mutex, portMAX_DELAY)) {
      // Send trigger pulse
      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);

      // Measure echo time
      s.duration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30ms timeout

      // Convert time to distance (Speed of sound = 343 m/s)
      s.distance = (s.duration * 0.0343) / 2;
      xSemaphoreGive(Mutex);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

//Task4 to print the values
void PrintValues(void *parameter) {
  while (1) {
    Serial.print("Temperature : ");
    Serial.print(s.temp);
    Serial.println(" °C ");

    Serial.print("Distance: ");
    Serial.print(s.distance);
    Serial.println(" cm");

    if (s.object == 0)
      Serial.println("Motion detected");
    else
      Serial.println("No Motion detected");
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  DHT_Sensor.begin();

  pinMode(IR_pin, INPUT);  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);


  Mutex = xSemaphoreCreateMutex();  // Create Mutex

  xTaskCreatePinnedToCore(DHTsensor, "DHTsensor", 2048, NULL, 2, NULL, app_cpu);
  xTaskCreatePinnedToCore(PrintValues, "Print", 2048, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(IRSensor, "IRsensor", 2048, NULL, 2, NULL, app_cpu);
  xTaskCreatePinnedToCore(UltraSonicSensor, "UltraSonicsensor", 2048, NULL, 2, NULL, app_cpu);
}

void loop() {
  // put your main code here, to run repeatedly:
}
