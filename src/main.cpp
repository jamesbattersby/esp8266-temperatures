//-----------------------------------------------------------------------------
// Temperatures
// By James Battersby
//-----------------------------------------------------------------------------

#include <Arduino.h>

// For encryption
#include <xxtea-lib.h>

// For OTA
#include <ArduinoOTA.h>

// Config for WiFi
#include "wifiConfig.h"

// For MQTT messaging
#include <PubSubClient.h>

// For sensor reading
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 (D4) on the Wemos
#define ONE_WIRE_BUS 2
// Seconds between temperature readings
#define SAMPLE_DELAY 15 

// Setup a oneWire instance
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
// Max number of sensors
#define MAX_NUMBER_OF_SENSORS 10
// arrays to hold device addresses
DeviceAddress sensorAddresses[MAX_NUMBER_OF_SENSORS];
// Number of sensors detected
int sensorsInUse = 0;
// Wifi client
WiFiClient espClient;
// MQTT client
PubSubClient mqttClient(espClient);

// Local functions
void notify(int, float);
void setUpWifi();
void callback(char*, byte*, unsigned int);
void connectToMqtt();
void notifyTemp(const int deviceNo);

//-----------------------------------------------------------------------------
// notifyTemp
//
// Read the temperature from a device and trigger mqtt notification.
//-----------------------------------------------------------------------------
void notifyTemp(const int deviceNo)
{
  float tempC = sensors.getTempC(sensorAddresses[deviceNo]);
  if (tempC != DEVICE_DISCONNECTED_C)
  {
    notify(deviceNo, tempC);
  }
  else 
  {
    Serial.print("DEVICE DISCONNECTED");
  }
  Serial.print(" ");
}

//-----------------------------------------------------------------------------
// setup
//
// Set-up wifi, serial and enumerate all the temperature sensors.
//-----------------------------------------------------------------------------
void setup(void)
{
  // start serial port
  Serial.begin(115200);
  setUpWifi();

  // Start up the library
  sensors.begin();
  
  // locate devices on the bus
  Serial.print("Found ");
  int numberOfSensors = sensors.getDeviceCount();
  Serial.print(numberOfSensors, DEC);
  Serial.println(" devices.");

  int sensorId = 0;
  int loop = 0;
  while (loop < numberOfSensors && sensorId < MAX_NUMBER_OF_SENSORS)
  {
    if (!sensors.getAddress(sensorAddresses[sensorId], loop)) 
    {
      Serial.printf("Unable to find address for Device %d", loop);   
    }
    else
    {
      sensorId++;
    }
    loop++;
  }
  
  sensorsInUse = sensorId;
}

//-----------------------------------------------------------------------------
// loop
//
// Main loop.
//
// Check for mqtt connection and OTA commands.  Update temperature readings
// every SAMPLE_DELAY seconds.
//-----------------------------------------------------------------------------
void loop(void)
{ 
  static int currentDelay = SAMPLE_DELAY;

  if (!mqttClient.connected())
  {
    connectToMqtt();
  }
  mqttClient.loop();
  ArduinoOTA.handle();
  
  if (--currentDelay == 0)
  {
    sensors.requestTemperatures();
    for (int i = 0; i < sensorsInUse; ++i)
    {
      notifyTemp(i);
    }
    currentDelay = SAMPLE_DELAY;
  }

  delay(1000);
}

//-----------------------------------------------------------------------------
// setUpWifi
//
// Responsible for connecting to Wifi, initialising the over-air-download
// handlers.
//
// If GENERATE_ENCRYPTED_WIFI_CONFIG is set to true, will also generate
// the encrypted wifi configuration data.
//-----------------------------------------------------------------------------
void setUpWifi()
{
  String ssid = SSID;
  String password = WIFI_PASSWORD;

  // Set the key
  xxtea.setKey(ENCRYPTION_KEY);

  // Perform Encryption on the Data
#if GENERATE_ENCRYPTED_WIFI_CONFIG
  Serial.printf("--Encrypted Wifi SSID: %s\n", xxtea.encrypt(SSID).c_str());
  Serial.printf("--Encrypted Wifi password: %s\n", xxtea.encrypt(WIFI_PASSWORD).c_str());
  Serial.printf("--Encrypted MQTT username: %s\n", xxtea.encrypt(MQTT_USERNAME).c_str());
  Serial.printf("--Encrypted MQTT password: %s\n", xxtea.encrypt(MQTT_PASSWORD).c_str());
#endif // GENERATE_ENCRYPTED_WIFI_CONFIG

  unsigned char pw[MAX_PW_LEN];
  unsigned char ss[MAX_PW_LEN];
  // Connect to Wifi
  WiFi.mode(WIFI_STA);
  xxtea.decrypt(password).getBytes(pw, MAX_PW_LEN);
  xxtea.decrypt(ssid).getBytes(ss, MAX_PW_LEN);

  WiFi.begin((const char*)(ss), (const char*)(pw));
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(callback);

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

//-----------------------------------------------------------------------------
// connectToMqtt
//
// Connect to the MQTT server
//-----------------------------------------------------------------------------
void connectToMqtt()
{
  unsigned char mqttUser[MAX_PW_LEN];
  unsigned char mqttPassword[MAX_PW_LEN];
  String username = MQTT_USERNAME;
  String passwordmqtt = MQTT_PASSWORD;
  xxtea.decrypt(username).getBytes(mqttUser, MAX_PW_LEN);
  xxtea.decrypt(passwordmqtt).getBytes(mqttPassword, MAX_PW_LEN);
  int retry = 20;

  while (!mqttClient.connected() && --retry)
  {
    Serial.println("Connecting to MQTT...");

    if (mqttClient.connect("WaterTemperature", reinterpret_cast<const char *>(mqttUser), reinterpret_cast<const char *>(mqttPassword)))
    {
      Serial.println("connected");
      mqttClient.subscribe("waterTemperatureQuery");
    }
    else
    {
      Serial.print("failed with state ");
      Serial.println(mqttClient.state());
      delay(2000);
    }
  }

  if (retry == 0)
  {
    printf("Failed to connect to MQTT server on %s:%d", MQTT_SERVER, MQTT_PORT);
  }
}

//-----------------------------------------------------------------------------
// callback
//
// Process a received mesage.
//-----------------------------------------------------------------------------
void callback(char* topic, byte* payload, unsigned int length)
{
  if (!strcmp(topic, "waterTemperature"))
  {
    // Someone wants to know the current water temperature
  }
}

//-----------------------------------------------------------------------------
// notify
//
// Send notification of the current water temperatures to the MQTT server, and 
// to the serial port.
//-----------------------------------------------------------------------------
void notify(int sensor, float temp)
{
  char message[20];
  sprintf(message, "%d:%f", sensor, temp);
  printf("%s\n", message);
  mqttClient.publish("waterTemperature", message);
}
