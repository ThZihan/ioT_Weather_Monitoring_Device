#include <WiFi.h>
#include <WiFiClient.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <GP2YDustSensor.h>

// -------------------- Wi-Fi Configuration --------------------
const char* ssid = "Xihan";             // Your Wi-Fi SSID
const char* password = "121121121";     // Your Wi-Fi Password

// -------------------- ThingSpeak Configuration --------------------
const char* apiKey = "5HZYOU8CAAOKB8J3";  // ThingSpeak Write API Key
const char* server = "api.thingspeak.com";
const unsigned long sendInterval = 15000; // Data sending interval in milliseconds

WiFiClient client;

// -------------------- DHT11 Configuration --------------------
#define DHTPIN 5          // GPIO5 (D5)
#define DHTTYPE DHT11     // DHT 11
DHT dht(DHTPIN, DHTTYPE);

// -------------------- BMP180 Configuration --------------------
Adafruit_BMP085 bmp;

// -------------------- LDR Configuration --------------------
#define LDR_PIN1 34       // GPIO34
#define LDR_PIN2 35       // GPIO35
const int LDR_MAX_VALUE = 4095; // ESP32 ADC resolution for 12-bit ADC

// -------------------- GP2Y1010AU0F Dust Sensor Configuration --------------------
#define DUST_LED_PIN 25   // GPIO25 - Controls the LED
#define DUST_VO_PIN 33    // GPIO33 - Analog Output
GP2YDustSensor dustSensor(GP2YDustSensorType::GP2Y1010AU0F, DUST_LED_PIN, DUST_VO_PIN);

// -------------------- Anemometer Configuration --------------------
#define ANEMOMETER_PIN 32 // GPIO32 for Anemometer Analog Input
int sensorValue = 0;     // Variable stores the analog value
float sensorVoltage = 0.0; // Voltage from anemometer
float windSpeed = 0.0;   // Wind speed in m/s

const float voltageConversionConstant = 3.3 / 4095.0; // ESP32 12-bit ADC: 3.3V / 4095
const float voltageMin = 0.4;    // Minimum voltage output from anemometer in V
const float windSpeedMin = 0.0;  // Wind speed in m/s corresponding to voltageMin
const float voltageMax = 2.0;    // Maximum voltage output from anemometer in V
const float windSpeedMax = 32.0; // Wind speed in m/s corresponding to voltageMax

// -------------------- Timing Variables --------------------
unsigned long previousSendTime = 0;

// -------------------- Function Prototypes --------------------
void connectToWiFi();
void sendDataToThingSpeak(float temperature, int humidity, float air_pressure, float dust_density, int light_percentage, float wind_speed);

// -------------------- Setup Function --------------------
void setup() {
  Serial.begin(9600);
  Serial.println("ESP32 is starting...");

  // Initialize DHT11 sensor
  dht.begin();
  Serial.println("DHT11 sensor initialized.");

  // Initialize BMP180 sensor
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    while (1) {} // Halt if BMP180 is not found
  }
  Serial.println("BMP180 sensor initialized.");

  // Initialize LDR pins as inputs
  pinMode(LDR_PIN1, INPUT);
  pinMode(LDR_PIN2, INPUT);
  Serial.println("LDRs initialized.");

  // Initialize Dust Sensor
  dustSensor.begin();
  // Optional: Set baseline and calibration factor based on your experiments
  // dustSensor.setBaseline(0.4); // Example value
  // dustSensor.setCalibrationFactor(1.1); // Example value
  Serial.println("GP2Y1010AU0F Dust Sensor initialized.");

  // Initialize Anemometer pin as input
  pinMode(ANEMOMETER_PIN, INPUT);
  Serial.println("Anemometer initialized.");

  // Connect to Wi-Fi
  connectToWiFi();
}

// -------------------- Loop Function --------------------
void loop() {
  unsigned long currentMillis = millis();

  // -------------------- Read DHT11 Sensor --------------------
  float temperature = dht.readTemperature(); // Celsius
  int humidity = dht.readHumidity();

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    // Optionally, you can skip sending data or handle the error
  }

  // -------------------- Read BMP180 Sensor --------------------
  float air_pressure = bmp.readPressure() / 100.0; // Convert Pa to hPa

  // -------------------- Read LDRs and Calculate Light Percentage --------------------
  int ldr1 = analogRead(LDR_PIN1);
  int ldr2 = analogRead(LDR_PIN2);

  // Average the two LDR readings
  float ldr_avg = (ldr1 + ldr2) / 2.0;

  // Map the average LDR value to a percentage (0-100%)
  // Adjust the mapping based on your specific environment and LDR characteristics
  int light_percentage = map(ldr_avg, 0, LDR_MAX_VALUE, 0, 100);

  // Constrain the percentage to 0-100%
  light_percentage = constrain(light_percentage, 0, 100);

  // -------------------- Read GP2Y1010AU0F Dust Sensor --------------------
  float dust_density = dustSensor.getDustDensity(); // ug/m3
  float running_average = dustSensor.getRunningAverage(); // ug/m3

  // -------------------- Read Anemometer and Calculate Wind Speed --------------------
  sensorValue = analogRead(ANEMOMETER_PIN); // Read analog value from anemometer
  sensorVoltage = sensorValue * voltageConversionConstant; // Convert to voltage

  // Convert voltage to wind speed based on linear relationship
  if (sensorVoltage <= voltageMin) {
    windSpeed = windSpeedMin; // Wind speed is zero or minimum
  } else {
    windSpeed = (sensorVoltage - voltageMin) * windSpeedMax / (voltageMax - voltageMin);
    // Ensure wind speed does not exceed maximum
    if (windSpeed > windSpeedMax) {
      windSpeed = windSpeedMax;
    }
  }

  // -------------------- Display Sensor Data --------------------
  Serial.println("----- Sensor Readings -----");
  Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" °C");
  Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");
  Serial.print("Air Pressure: "); Serial.print(air_pressure); Serial.println(" hPa");
  Serial.print("Dust Density: "); Serial.print(dust_density); Serial.println(" ug/m3");
  Serial.print("Running Average: "); Serial.print(running_average); Serial.println(" ug/m3");
  Serial.print("Light Percentage: "); Serial.print(light_percentage); Serial.println(" %");
  Serial.print("Wind Speed: "); Serial.print(windSpeed); Serial.println(" m/s");
  Serial.println("----------------------------");

  // -------------------- Send Data to ThingSpeak --------------------
  if (currentMillis - previousSendTime >= sendInterval) {
    previousSendTime = currentMillis;
    sendDataToThingSpeak(temperature, humidity, air_pressure, running_average, light_percentage, windSpeed);
  }

  // Short delay to prevent overwhelming the loop
  delay(100); // Adjust as needed
}

// -------------------- Wi-Fi Connection Function --------------------
void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  int attempt = 0;

  // Wait for connection with a maximum of 20 attempts (10 seconds)
  while (WiFi.status() != WL_CONNECTED && attempt < 20) {
    delay(500);
    Serial.print(".");
    attempt++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to Wi-Fi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to Wi-Fi. Retrying...");
    delay(5000);  // Wait before retrying
    connectToWiFi();
  }
}

// -------------------- ThingSpeak Data Transmission Function --------------------
void sendDataToThingSpeak(float temperature, int humidity, float air_pressure, float dust_density, int light_percentage, float wind_speed) {
  if (client.connect(server, 80)) {
    // Construct the POST data string
    String postStr = "api_key=";
    postStr += String(apiKey);
    postStr += "&field1="; postStr += String(temperature, 2);       // Temperature
    postStr += "&field2="; postStr += String(humidity);          // Humidity
    postStr += "&field3="; postStr += String(air_pressure, 2);      // Air Pressure
    postStr += "&field4="; postStr += String(dust_density, 2);       // Dust Density
    postStr += "&field5="; postStr += String(light_percentage);      // Light Percentage
    postStr += "&field6="; postStr += String(wind_speed, 2);        // Wind Speed

    Serial.println("Sending data to ThingSpeak:");
    Serial.println(postStr);

    // Create the HTTP POST request
    client.print("POST /update HTTP/1.1\r\n");
    client.print("Host: " + String(server) + "\r\n");
    client.print("Connection: close\r\n");
    client.print("Content-Type: application/x-www-form-urlencoded\r\n");
    client.print("Content-Length: " + String(postStr.length()) + "\r\n\r\n");
    client.print(postStr);

    // Wait for server response with a timeout
    unsigned long timeout = millis();
    while (client.available() == 0) {
      if (millis() - timeout > 5000) { // 5 seconds timeout
        Serial.println(">>> Client Timeout !");
        client.stop();
        return;
      }
    }

    // Read and print the server response
    while (client.available()) {
      String response = client.readStringUntil('\n');
      Serial.println(response);
    }

    client.stop();  // Close the connection
    Serial.println("Data sent to ThingSpeak successfully.\n");
  } else {
    Serial.println("Connection to ThingSpeak failed.");
  }
}
