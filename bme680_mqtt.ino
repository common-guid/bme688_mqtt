/***************************************************************************
  This is a library for the BME680 gas, humidity, temperature & pressure sensor
    to be used on a wifi enabled microcontroller
    

 --!> if using with bme688 make sure to change the i2c addr in the 680.h file
***************************************************************************/

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include "Adafruit_BME680.h"

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

// WiFi Network Credentials
const char* ssid     = "<2.4GHZ_WIFI_SSID>";
const char* password = "<2.4GHZ_WIFI_PW>";

// Home Assistant Credentials
const char *HA_USER = <"MOSQUITTO_U">;
const char *HA_PASS = "<MOSQUITTO_PW>";

// MQTT Network
IPAddress broker(<MOSQUITTO_IP_ADDR_[USE_,_NOT_.]>); // IP address of your MQTT broker eg. 192.168.1.50
const byte SWITCH_PIN = 0;           // Pin to control the light with
const char *ID = "bme688";  // Name of our device, must be unique
const char *TOPIC = "office/bme688";  // Topic to subcribe to
WiFiClient wclient;
Adafruit_BME680 bme; // I2C

PubSubClient client(wclient); // Setup MQTT client
bool state=0;

// Connect to WiFi network
void setup_wifi() {

  WiFi.begin(ssid, password); // Connect to network

  while (WiFi.status() != WL_CONNECTED) { // Wait for connection
    delay(500);
    Serial.print(".");
  }

  Serial.println(WiFi.localIP());
}

// Reconnect to client
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(ID,HA_USER,HA_PASS)) {
      Serial.println("connected");
      Serial.print("Publishing to: ");
    //  Serial.println(TOPIC);
      Serial.println('\n');

    } else {
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200); // Start serial communication at 115200 baud
  delay(100);
  setup_wifi(); // Connect to network
  client.setServer(broker, 1883);
  //while (!Serial);    // from original bme script
  Serial.println(F("BME680 test"));

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void loop() {
    if (!client.connected())  // Reconnect if connection is lost
  {
    reconnect();
  }
  client.loop();

  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  StaticJsonDocument<200> doc;
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");
  float Temperature = bme.temperature;
  //sprintf(tmp, "%f", Temperature);
  doc["temp_C"] = Temperature;

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");
  float Pressure = bme.pressure / 100.0;
  //sprintf(prs, "%f", Pressure);
  doc["pressure_hpa"] = Pressure;

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");
  float Humidity = bme.humidity;
  //sprintf(hum, "%f", Humidity);
  doc["humidity_%"] = Humidity;

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");
  float Gas = bme.gas_resistance / 1000.0;
  //sprintf(giaq, "%f", Gas);
  doc["gas_index_kohms"] = Gas;

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
  float Altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  //sprintf(alt, "%f", Altitude);
  doc["altitude_m"] = Altitude;

  char buffer[256];
  serializeJson(doc, buffer);
  client.publish(TOPIC, buffer);

  Serial.println();
  delay(5000);
}
