#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT11.h>

// Put the credentials of the wifi used here
const char* ssid     = "RRZPC";       // Nom de la xarxa WiFi
const char* password = "ProvesLab";    // Contrasenya de la xarxa WiFi

// IP of the device that host Mosquitto MQTT broker
const char* mqtt_server = "192.168.137.1";
const int mqtt_port = 1883;  // default port for MQTT

//Create the instance of Wifi and MQTT Clkient
WiFiClient espClient;
PubSubClient client(espClient);

// Create an instance of the DHT11 class i PIN 4
DHT11 dht11(4);

// Define the variables and interval to generate periodic readings.
unsigned long previousMillis = 0;
const long interval = 1000; // 1 seconds in milliseconds

void setup() {
  Serial.begin(115200);  // Millor velocitat per ESP32

  Serial.println();
  Serial.println("Connecting to the WiFi...");

  WiFi.begin(ssid, password);

  // Espera fins que es connecti
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n✅ Connected to WIFI!");
  Serial.print("📡 SSID: ");
  Serial.println(WiFi.SSID());

  Serial.print("🌐Local IP: ");
  Serial.println(WiFi.localIP());

  Serial.print("🆔 MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.print("📶 Signal power (RSSI): ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");

  client.setServer(mqtt_server, mqtt_port);

}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connectant al servidor MQTT...");
    if (client.connect("esp32-client"))
    {
      Serial.println("connected!");
      //client.publish("test/topic", "Connected");
    }
    else
    {
      Serial.print("Error, rc=");
      Serial.print(client.state());
      Serial.println(" trying to connect in 5 seconds");
      delay(5000);
    }
  }
}

void MesureAndSendTemperatureAndHumidity()
{
  int temperature = 0;
  int humidity = 0;
  char value_string[100];
  
  //Do the measure every interval (5s)
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;


    // Attempt to read the temperature and humidity values from the DHT11 sensor.
    int result = dht11.readTemperatureHumidity(temperature, humidity);

    // Check the results of the readings.
    // If the reading is successful, print the temperature and humidity values.
    // If there are errors, print the appropriate error messages.
    if (result == 0) {
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print(" °C\tHumidity: ");
        Serial.print(humidity);
        Serial.println(" %");

        client.publish("test/temperature", String(temperature).c_str());
        client.publish("test/humidity", String(humidity).c_str());
    } else {
        // Print error message based on the error code.
        Serial.println(DHT11::getErrorString(result));
    }
  }
}


void loop() {
  if (!client.connected()) {
    reconnect();
  }
  else
  {
    MesureAndSendTemperatureAndHumidity();
  }
  client.loop();
}