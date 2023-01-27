#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>


#define CONFIG_DISABLE_HAL_LOCKS 0

#define LED_BUILTIN 33
#define FLASH_BUILTIN 4

#define I2C_SDA 14
#define I2C_SCL 15

TwoWire I2CSensors = TwoWire(0);

// BMP280 (Using I2C)
Adafruit_BMP280 bmp(&I2CSensors);

void ReadSensors( void *pvParameters );


#define AP_SSID "Altair"
#define AP_PASS "PtS4LzLxjpepE4w67N"

SemaphoreHandle_t  i2c_lock; 

void setup() {
  Serial.begin(115200);
  delay(1000);

  i2c_lock = xSemaphoreCreateBinary();

  I2CSensors.begin(I2C_SDA, I2C_SCL, 100000);

  unsigned status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
    Serial.printf("SensorID was: 0x%x\n", bmp.sensorID());
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("        ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  xTaskCreatePinnedToCore(
    ReadSensors
    , "ReadSensors"
    , 1024
    , NULL
    , 1
    , NULL
    , ARDUINO_RUNNING_CORE);
}

void loop() {
}

void ReadSensors(void *pvParameters) // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
    xSemaphoreTake(i2c_lock, portMAX_DELAY);
    Serial.printf("Temperature = %.0f °C\n", bmp.readTemperature());
    Serial.printf("Pressure = %.2f hPa\n", bmp.readPressure()/100);
    Serial.printf("Approx altitude = %.2f m\n\n", bmp.readAltitude(1021)); /* Adjusted to local forecast! */
    xSemaphoreGive(i2c_lock);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 1000ms delay
  }
}
