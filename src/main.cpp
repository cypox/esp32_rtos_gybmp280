#include <esp32cam.h>
#include <WebServer.h>
#include <WiFi.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>


#define LED_BUILTIN 33
#define FLASH_BUILTIN 4

#define I2C_SDA 14
#define I2C_SCL 15

TwoWire I2CSensors = TwoWire(0);

// BMP280 (Using I2C)
Adafruit_BMP280 bmp(&I2CSensors);

// define two tasks for Blink & AnalogRead
void TaskBlink( void *pvParameters );
void ReadSensors( void *pvParameters );


#define AP_SSID "Altair"
#define AP_PASS "PtS4LzLxjpepE4w67N"

WebServer server(80);

void handle_capture() {
  auto img = esp32cam::capture();
  if (img == nullptr) {
    server.send(500, "", "");
    return;
  }
  server.setContentLength(img->size());
  server.send(200, "image/jpeg");
  WiFiClient client = server.client();
  img->writeTo(client);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

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

  auto res = esp32cam::Resolution::find(1024, 768);
  esp32cam::Config cfg;
  cfg.setPins(esp32cam::pins::AiThinker);
  cfg.setResolution(res);
  cfg.setJpeg(80);
  esp32cam::Camera.begin(cfg);

  if(1) { // create own wifi network
    WiFi.softAP("ESP32", "ESP1234567890");
  } else {
    WiFi.mode(WIFI_STA); //Optional
    WiFi.begin(AP_SSID, AP_PASS);
    Serial.println("\nConnecting to wifi network");

    while(WiFi.status() != WL_CONNECTED){
      Serial.print(".");
      delay(100);
    }

    Serial.println("\nConnected to the WiFi network");
    Serial.print("Local ESP32 IP: ");
    Serial.println(WiFi.localIP());
  }

  server.on("/capture.jpg", handle_capture);
  server.begin();

  xTaskCreatePinnedToCore(
    TaskBlink
    , "TaskBlink"
    , 1024
    , NULL
    , 2
    , NULL
    , ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    ReadSensors
    , "ReadSensors"
    , 4096
    , NULL
    , 1
    , NULL
    , ARDUINO_RUNNING_CORE);
}

void loop() {
  server.handleClient();
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBlink(void *pvParameters)
{
  (void) pvParameters;

  pinMode(LED_BUILTIN, OUTPUT);

  for (;;)
  {
    digitalWrite(LED_BUILTIN, LOW); // turn the LED on (HIGH is the voltage level)
    // arduino-esp32 has FreeRTOS configured to have a tick-rate of 1000Hz and portTICK_PERIOD_MS
    // refers to how many milliseconds the period between each ticks is, ie. 1ms.
    vTaskDelay(100 / portTICK_PERIOD_MS ); // vTaskDelay wants ticks, not milliseconds
    digitalWrite(LED_BUILTIN, HIGH); // turn the LED off by making the voltage LOW
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 1 second delay
  }
}

void ReadSensors(void *pvParameters) // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
    Serial.printf("Temperature = %.0f °C\n", bmp.readTemperature());
    Serial.printf("Pressure = %.2f hPa\n", bmp.readPressure()/100);
    Serial.printf("Approx altitude = %.2f m\n\n", bmp.readAltitude(1021)); /* Adjusted to local forecast! */

    Serial.println();
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 1000ms delay
  }
}
