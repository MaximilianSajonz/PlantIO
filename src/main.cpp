#include <Arduino.h>
#include <Wire.h>
#include <BH1750.h>
#include <DHT.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Define sensor pins
#define SOIL_MOISTURE_PIN 34  // GPIO for soil moisture sensor
#define DHT_PIN 27            // GPIO for DHT sensor (connected to S pin)
#define DHTTYPE DHT22         // DHT22 sensor type

// Define I2C pins for ESP32 (21 and 21 are default could also be left out empty)
#define SDA_PIN 21
#define SCL_PIN 22

DHT dht(DHT_PIN, DHTTYPE);   // Initialize DHT sensor
BH1750 lightMeter;           // Initialize light sensor

// Declare Mutexes that protect the shared ressources
SemaphoreHandle_t xSerialSemaphore;
SemaphoreHandle_t xSensorDataSemaphore;

// declare our variables to store sensor data
// Voltaile ensures that the compiler does not optimize the variables since they are modified by multiple tasks.
volatile int soilMoistureValue = 0;
volatile int soilMoisturePercent = 0;
volatile float humidity = 0.0;
volatile float temperature = 0.0;
volatile float lux = 0.0;

// Declare the tasks for reading and displaying the sensor data
void readSoilMoistureTask(void *pvParameters);
void readDHTTask(void *pvParameters);
void readLightTask(void *pvParameters);
void displayDataTask(void *pvParameters);

int dryValue = 3500;         // Calibration value for dry soil
int wetValue = 500;          // Calibration value for wet soil

void setup() {
  // Set Serial Monitor Bit Rate
  Serial.begin(115200);
  Serial.println("\n\nStarting sensor interface...");
  Serial.println("----------------------------");

  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN); //If the default pins are used those parameters could stay empty

  // Start DHT sensor
  dht.begin();
  delay(2000); // Delay to allow the DHT sensor to stabilize

  // Start BH1750 light sensor
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire)) {
    Serial.println("BH1750 sensor initialized successfully.");
  } else {
    Serial.println("Failed to initialize BH1750 sensor.");
    while (1);  // Halt execution if sensor fails to initialize
  }

  // Set the soil moisture sensor pin as input
  pinMode(SOIL_MOISTURE_PIN, INPUT);

  // Create the mutexes before starting the tasks
  xSerialSemaphore = xSemaphoreCreateMutex();
  xSensorDataSemaphore = xSemaphoreCreateMutex();

  // Create tasks and allocate memory
  xTaskCreate(readSoilMoistureTask, "ReadSoilMoisture", 2048, NULL, 1, NULL);
  xTaskCreate(readDHTTask, "ReadDHT", 2048, NULL, 1, NULL);
  xTaskCreate(readLightTask, "ReadLight", 2048, NULL, 1, NULL);
  xTaskCreate(displayDataTask, "DisplayData", 2048, NULL, 1, NULL);
}

void readSoilMoistureTask(void *pvParameters) {
  // while(1) ensures that the tasks continues to run as long as the system is running
  while (1) {
    int value = analogRead(SOIL_MOISTURE_PIN);
    int percent = map(value, dryValue, wetValue, 0, 100);
    percent = constrain(percent, 0, 100);  // Limit to 0-100%

    // wait until the semphre is availble to take the data
    if (xSemaphoreTake(xSensorDataSemaphore, portMAX_DELAY) == pdTRUE) {
      soilMoistureValue = value;
      soilMoisturePercent = percent;
      xSemaphoreGive(xSensorDataSemaphore); // Release the sensor data
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS); // Block the task for 2 seconds to ensure preiodical reading
  }
}

void readDHTTask(void *pvParameters) {
  while (1) {
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (xSemaphoreTake(xSensorDataSemaphore, portMAX_DELAY) == pdTRUE) {
      humidity = h;
      temperature = t;
      xSemaphoreGive(xSensorDataSemaphore);
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void readLightTask(void *pvParameters) {
  while (1) {
    float l = lightMeter.readLightLevel();

    if (xSemaphoreTake(xSensorDataSemaphore, portMAX_DELAY) == pdTRUE) {
      lux = l;
      xSemaphoreGive(xSensorDataSemaphore);
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void displayDataTask(void *pvParameters) {
  while (1) {
    // Attempt to take the mutex, waiting indefinitely
    if (xSemaphoreTake(xSerialSemaphore, portMAX_DELAY) == pdTRUE) {
      // Print header
      Serial.println("=================================");
      Serial.println("       Sensor Data Readings");
      Serial.println("=================================\n");

      // Print humidity and temperature
      if (isnan(humidity) || isnan(temperature)) {
        Serial.println("Failed to read from DHT sensor!");
      } else {
        Serial.printf("Humidity:       %.2f %%\n", humidity);
        Serial.printf("Temperature:    %.2f *C\n\n", temperature);
      }

      // Print light intensity
      if (isnan(lux)) {
        Serial.println("Failed to read from BH1750 sensor!");
      } else {
        Serial.printf("Light Intensity: %.2f lux\n\n", lux);
      }

      // Print soil moisture
      Serial.printf("Soil Moisture:\n");
      Serial.printf("  Raw Value:     %d\n", soilMoistureValue);
      Serial.printf("  Percentage:    %d %%\n\n", soilMoisturePercent);

      // Print footer
      Serial.println("=================================");

      // Provide advice based on sensor readings
      Serial.println("\n       Plant Care Advice");
      Serial.println("=================================");

      // Humidity advice
      if (humidity > 60) {
        Serial.println("Humidity is high! Aloe Vera prefers low to moderate humidity.");
      } else {
        Serial.println("Humidity is fine.");
      }

      // Temperature advice
      if (temperature < 13) {
        Serial.println("Temperature is too low! Move the plant to a warmer location.");
      } else if (temperature > 27) {
        Serial.println("Temperature is too high! Move the plant to a cooler location.");
      } else {
        Serial.println("Temperature is ideal.");
      }

      // Light intensity advice
      if (lux < 1000) {
        Serial.println("Light intensity is low! Move the plant to a brighter location.");
      } else if (lux > 10000) {
        Serial.println("Light intensity is too high! Avoid direct sunlight.");
      } else {
        Serial.println("Light intensity is ideal.");
      }

      // Soil moisture advice
      if (soilMoisturePercent < 10) {
        Serial.println("Soil is too dry! Water the plant.");
      } else if (soilMoisturePercent > 50) {
        Serial.println("Soil is too wet! Allow the soil to dry out before watering again.");
      } else {
        Serial.println("Soil moisture is ideal.");
      }

      // Release the mutex
      xSemaphoreGive(xSerialSemaphore);

      // Wait for five seconds before displaying data again
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
  }
}

void loop() {
  // Nothing to do here as tasks handle the work
}
