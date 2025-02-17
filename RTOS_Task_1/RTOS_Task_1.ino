 #include <Arduino.h>

#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

SemaphoreHandle_t Mutex;  // Mutex handle

uint8_t ledstate = 0;

static const int led_pin = 2;
static const int push_button = 23;

// Task1: Read the push button
void PushButton(void *parameter) {
  while (1) {
    if (xSemaphoreTake(Mutex, portMAX_DELAY)) {
      if (digitalRead(push_button)) {
        ledstate = !ledstate;
        Serial.print("LED State: ");
        Serial.println(ledstate);
      }
      xSemaphoreGive(Mutex);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Task2: Blink the LED based on ledstate
void BlinkLed(void *parameter) {
  while (1) {
    if (ledstate == 1)
      digitalWrite(led_pin, HIGH);
    else
      digitalWrite(led_pin, LOW);
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void setup() {
  pinMode(led_pin, OUTPUT);
  pinMode(push_button, INPUT);

  Mutex = xSemaphoreCreateMutex();  // Create Mutex

  Serial.begin(115200);
  Serial.println("LED Blinking Code with FreeRTOS");

  xTaskCreatePinnedToCore(PushButton, "ReadButton", 1024, NULL, 2, NULL, app_cpu);
  xTaskCreatePinnedToCore(BlinkLed, "BlinkLed", 1024, NULL, 1, NULL, app_cpu);

  vTaskDelete(NULL);  // Delete the setup() task
}

void loop() {
  // Empty loop as FreeRTOS tasks handle everything
}
