// Water Metering
// Libraries
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <ArduinoJson.h>
#include <FlowSensor.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

//Wi-Fi Setup
#define WLAN_SSID    "Galaxybobs"
#define WLAN_PASS    "12345667"

//WiFi LED
const int BlueLed = 13; //GPIO 13 which is D7

// WiFiClient 
WiFiClient client;

// MQTT Broker
#define AIO_SERVER      "156.0.232.201"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "iotlab"
#define AIO_KEY         "80d18b0d"

//NTP Server
const char* NTP_SERVER = "pool.ntp.org";
const int NTP_TIMEZONE = 10800; // conversion from GMT to EAT, 10800 seconds which represents 3 hours
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_SERVER, NTP_TIMEZONE);
String StartTime;
String EndTime;
String StartDay;
String EndDay;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// WaterFlow sensor
#define type YFS201
const int sensorInterruptPin = 14; // GPIO 14 which is D5 on the board
FlowSensor Sensor(type, sensorInterruptPin);

// Variables
unsigned long Pulses = 0;
const unsigned long pulsesPerLiter = 450; // The YF-S201 has a specification of 450pulses per litre
float Volume = 0.0; // Initialize the volume.
boolean waterFlow = false; // Flag to track water flow
float totalVolume = 0.0; // Total volume when water is flowing
boolean prevWaterFlow = false; // Previous water flow status

// WaterFlow Sensor LED's
//Red LED indicates no water flow
const int RedLed = 15; //GPIO 15 which is D6
//Green LED indicates water flow
const int GreenLed = 4; //GPIO 4 which is D2

void IRAM_ATTR countPulse() 
{
  // This ISR function is called each time the sensor pulses
  Pulses++;
  waterFlow = true; // When a pulse is detected the flag is set to true to indicate wateflow.
}

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT,"IOTLAB",AIO_USERNAME, AIO_KEY);

//Topics
//Publishing
Adafruit_MQTT_Publish WATERMETERING = Adafruit_MQTT_Publish(&mqtt,"WATERMETERING");

//Bug Workaround
void MQTT_connect();

void setup() {
  //Baud Rate to be set on the serial monitor.
  Serial.begin(115200); 

  //Configuring the Sensor Interrupt pin
  pinMode(sensorInterruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensorInterruptPin), countPulse, FALLING);

  Serial.println(F("WATER METERING"));

  //Setting the LED's as Output.
  pinMode(BlueLed, OUTPUT);
  pinMode(RedLed, OUTPUT);
  pinMode(GreenLed, OUTPUT);

  //Connecting to WiFi access point.
  WiFi.begin(WLAN_SSID, WLAN_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    // Wait for successful connection
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  // WiFi is connected, turn on the LED
  digitalWrite(BlueLed, HIGH);
  Serial.println("Connected to WiFi");

  // Initialize a NTPClient to get time
  timeClient.begin();

  //Set the Waterflow sensor's LED
  digitalWrite(RedLed, HIGH);
  digitalWrite(GreenLed, LOW);

  Serial.println("WAITING FOR WATERFLOW");
}

uint32_t x=0;

void loop() {
  
  //Wifi and MQTT Functions
  MQTT_connect(); 
  WIFI_connect();

  // Get the current time from the NTP server
  timeClient.update();

  if (waterFlow) {
    // Check if water has just started flowing. If it has started flowing the if block is executed.
    digitalWrite(GreenLed, HIGH);
    digitalWrite(RedLed, LOW);
    if (!prevWaterFlow) {
      // It checks if the previous water flow status has changed 
      // Checks if there was no water flow in the previous cycle.
      // If there was no waterflow it means water has just started flowing hence the start time and date are recorded.

      //Getting the time and day the water started flowing.
      StartTime = timeClient.getFormattedTime();
      Serial.print("Water started flowing at: ");
      Serial.println(StartTime);
      StartDay = daysOfTheWeek[timeClient.getDay()];
      Serial.print("Water started flowing on: ");
      Serial.println(StartDay);
    }
    // Water is flowing, calculate and print the volume
    Volume = Pulses / float(pulsesPerLiter);
    Serial.print("The Total Pulses is : ");
    Serial.println(Pulses);
    Serial.print("Total Volume (L) : ");
    Serial.println(Volume);

    // Getting the sum of the total volume that flowed.
    totalVolume += Volume;
  }
  if (prevWaterFlow && !waterFlow) {
    // Checks if there was water in the previous cycle and no water in the current cycle
    // If this is the case it means water has stopped flowing and the if block will be executed.
    digitalWrite(GreenLed, LOW);
    digitalWrite(RedLed, HIGH);

    // Getting the time and day the water stopped flowing.
    EndTime = timeClient.getFormattedTime();
    Serial.print("Water stopped flowing at: ");
    Serial.println(EndTime);
    EndDay = daysOfTheWeek[timeClient.getDay()];
    Serial.print("Water stopped flowing on: ");
    Serial.println(EndDay);

    Serial.println("PUBLISHING");

    //Publishing the parameters
    StaticJsonDocument<256> jsonDoc;
    jsonDoc["Start Day"] = String (StartDay);
    jsonDoc["Start Time"] = String (StartTime);
    jsonDoc["Volume"] = float(totalVolume);
    jsonDoc["End Day"] = String (EndDay);
    jsonDoc["End Time"] = String (EndTime);
    String jsonString;
    serializeJson(jsonDoc, jsonString);
    WATERMETERING.publish(jsonString.c_str());

    Serial.println("WAITING FOR WATERFLOW");

    // Reset the total volume to 0 in readiness for the next iteration.
    totalVolume = 0.0;
  }

  // Update the previous water flow status
  prevWaterFlow = waterFlow;

  // Reset the flag to false and pulses to 0 in readiness for the nesxt iteration
  waterFlow = false;  
  Pulses = 0;

  //When water is flowing the Volume and pulses will be published on the serial monitor each second.
  delay(1000); 
}

//FUNCTIONS
// Check if Wi-Fi is still connected
void WIFI_connect(){
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi disconnected");
    // Turn off the Wifi and no flow LED's  
    digitalWrite(BlueLed, LOW); 
    digitalWrite(RedLed, LOW); 

    // Attempt to reconnect to Wi-Fi
    WiFi.begin(WLAN_SSID, WLAN_PASS);

    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Reconnecting to WiFi...");
    }
    Serial.println("Reconnected to WiFi");  
    // Turn on the wifi and no flow LED's
    digitalWrite(BlueLed, HIGH);
    digitalWrite(RedLed, HIGH); 
  }
}

// Connecting and reconnecting to MQTT.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;

    if (retries == 0) {
      // Retry three times if not connected start wifi-reconnection
      Serial.println("Max retries reached. Unable to connect to MQTT.");
      return;  // Exit the function without resetting the device
    }
  }

  Serial.println("MQTT Connected!");
}