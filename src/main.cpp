#include <esp32cam.h>
#include <WebServer.h>
#include <WiFi.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>

#include <ESP32Servo.h>


#define MAIN_CORE ARDUINO_RUNNING_CORE
#define SECONDARY_CORE (1-ARDUINO_RUNNING_CORE)

#define LED_BUILTIN 33
#define FLASH_BUILTIN 4

#define I2C_SDA 14
#define I2C_SCL 15

// Possible PWM GPIO pins on the ESP32: 0(used by on-board button),2,4,5(used by on-board LED),12-19,21-23,25-27,32-33
#define AIRLON_PWM_PIN 12
// Possible ADC pins on the ESP32: 0,2,4,12-15,32-39; 34-39 are recommended for analog input
#define AIRLON_TEST_INPUT_PIN 34
#define ADC_MAX 4096    // This is the default ADC max value on the ESP32 (12 bit ADC width);
                        // this width can be set (in low-level oode) from 9-12 bits, for a
                        // a range of max values of 512-4096

#define AP_SSID "Altair"
#define AP_PASS "PtS4LzLxjpepE4w67N"


TwoWire I2CWire = TwoWire(0);

// BMP280 (Using I2C)
Adafruit_BMP280 bmp(&I2CWire);
Adafruit_MPU6050 mpu;

WebServer server(80);

Servo airlons;


// define two tasks for Blink & AnalogRead
void ServerHandler( void *pvParameters );
void TaskBlink( void *pvParameters );
void ReadSensors( void *pvParameters );
void AirlonController( void *pvParameters );


void handle_capture() {
  digitalWrite(FLASH_BUILTIN, HIGH);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  auto img = esp32cam::capture();
  digitalWrite(FLASH_BUILTIN, LOW);
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

  I2CWire.begin(I2C_SDA, I2C_SCL, 100000);

  // Try to initialize!
  if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &I2CWire)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);

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

  // Default settings from datasheet.
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode.
                  Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,      // Filtering.
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time.


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

  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  airlons.setPeriodHertz(50);                // Standard 50hz servo
  airlons.attach(AIRLON_PWM_PIN, 750, 3000); // attaches the servo on pin (definded) to the servo object for 180 degrees rotation

  xTaskCreatePinnedToCore(
    TaskBlink
    , "TaskBlink"
    , 1024
    , NULL
    , 2
    , NULL
    , MAIN_CORE);

  xTaskCreatePinnedToCore(
    ReadSensors
    , "ReadSensors"
    , 4096
    , NULL
    , 1
    , NULL
    , MAIN_CORE);

  xTaskCreatePinnedToCore(
    ServerHandler
    , "ServerHandler"
    , 8192
    , NULL
    , 1
    , NULL
    , MAIN_CORE);

  xTaskCreatePinnedToCore(
    AirlonController
    , "AirlonController"
    , 4096
    , NULL
    , 1
    , NULL
    , SECONDARY_CORE);
}

void loop() {

}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void ServerHandler(void *pvParameters)
{
  (void) pvParameters;

  pinMode(FLASH_BUILTIN, OUTPUT);

  for (;;)
  {
    server.handleClient();
  }
}

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
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    /* Print out the values */
    Serial.printf("Acceleration X: %.2f, Y: %.2f, Z: %.2f m/s^2\n", a.acceleration.x, a.acceleration.y, a.acceleration.z);
    Serial.printf("Rotation X: %.2f, Y: %.2f, Z: %.2f rad/s\n", g.gyro.x, g.gyro.y, g.gyro.z);
    Serial.printf("Temperature = %.1f °C\n\n", temp.temperature);

    Serial.printf("Temperature = %.1f °C\n", bmp.readTemperature());
    Serial.printf("Pressure = %.2f hPa\n", bmp.readPressure()/100);
    Serial.printf("Approx altitude = %.2f m\n\n", bmp.readAltitude(1021)); /* Adjusted to local forecast! */
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 1000ms delay
  }
}

void AirlonController( void *pvParameters )
{
  (void) pvParameters;

  // go to balanced angle, used for calibration
  airlons.write(90);
  vTaskDelay(6000 / portTICK_PERIOD_MS); // during this time, the fin should be set level

  for (;;)
  {
    int pos = 0;
    for (pos = 90; pos >= 45; pos -= 1) {  // sweep from 90 degrees to 45 degrees
      airlons.write(pos);
      vTaskDelay(15 / portTICK_PERIOD_MS);
    }
    for (pos = 45; pos <= 90; pos += 1) {  // sweep back from 45 degrees to 90 degrees
      // in steps of 1 degree
      airlons.write(pos);
      vTaskDelay(15 / portTICK_PERIOD_MS); // waits 20ms for the servo to reach the position
    }
    vTaskDelay(3000 / portTICK_PERIOD_MS); // wait for calibration if needed
    for (pos = 90; pos <= 135; pos += 1) {  // sweep from 90 degrees to 135 degrees
      // in steps of 1 degree
      airlons.write(pos);
      vTaskDelay(15 / portTICK_PERIOD_MS); // waits 20ms for the servo to reach the position
    }
    for (pos = 135; pos >= 90; pos -= 1) {  // sweep back from 135 degrees to 90 degrees
      airlons.write(pos);
      vTaskDelay(15 / portTICK_PERIOD_MS);
    }
    vTaskDelay(3000 / portTICK_PERIOD_MS); // wait for calibration if needed
  }
}
