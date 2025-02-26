/*
  ESP32 FreeRTOS LED Control using Queue

  This program uses FreeRTOS on an ESP32 to control LEDs based on push-button inputs 
  or serial commands. A queue is used for communication between tasks.

  - `ButtonTask`: Reads push-button inputs and sends corresponding LED commands to the queue.
  - `ReadCommand`: Reads user input from the Serial Monitor (commands '1', '2', '3') and sends them to the queue.
  - `LED`: Listens for commands from the queue and turns on the corresponding LED for 1 second.
*/

#include <Arduino.h>

QueueHandle_t myQueue;

void ButtonTask(void *paramater) {
  int ledstate;
  while (1) {
    if (digitalRead(23) == LOW) {  
      ledstate = 1;
      xQueueSend(myQueue, &ledstate, portMAX_DELAY);
      vTaskDelay(pdMS_TO_TICKS(200));  // Debounce delay
    }
    if (digitalRead(21) == LOW) {
      ledstate = 2;
      xQueueSend(myQueue, &ledstate, portMAX_DELAY);
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    if (digitalRead(19) == LOW) {
      ledstate = 3;
      xQueueSend(myQueue, &ledstate, portMAX_DELAY);
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    vTaskDelay(pdMS_TO_TICKS(50));  // General polling delay
  }
}

void ReadCommand(void *paramter) {
  int command;
  while (1) {
    if (Serial.available()) {
      char receivedChar = Serial.read();
      while (Serial.available()) Serial.read();  //Clearing the buffer so that it doesnt read the same value again and again causing more delay to led

      if (receivedChar == '1') {
        command = 1;
      } else if (receivedChar == '2') {
        command = 2;
      } else if (receivedChar == '3') {
        command = 3;
      } 
      else {
        continue; // Ignore invalid characters
      }

      xQueueSend(myQueue, &command, portMAX_DELAY);
    }
    vTaskDelay(pdMS_TO_TICKS(50));  // Reduce CPU usage
  }
}

void LED(void *parameter) {
  int receivedState;
  TickType_t ledOnTime = pdMS_TO_TICKS(1000);
  
  while (1) {
    if (xQueueReceive(myQueue, &receivedState, portMAX_DELAY) == pdPASS) {
      int ledPin;
      if (receivedState == 1) {
        ledPin = 4;
      } else if (receivedState == 2) {
        ledPin = 5;
      } else if (receivedState == 3) {
        ledPin = 18;
      } 
      digitalWrite(ledPin, HIGH);
      vTaskDelay(ledOnTime); // Delay LED on time
      digitalWrite(ledPin, LOW);
    }
  }
}


void setup() {
  Serial.begin(115200);
  pinMode(23, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);

  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(18, OUTPUT);

  myQueue = xQueueCreate(5, sizeof(int));

  xTaskCreate(ButtonTask, "ButtonTask", 2048, NULL, 1, NULL);
  xTaskCreate(ReadCommand, "ReadCommand", 2048, NULL, 1, NULL);
  xTaskCreate(LED, "LED", 2048, NULL, 1, NULL);
}

void loop() {
}
