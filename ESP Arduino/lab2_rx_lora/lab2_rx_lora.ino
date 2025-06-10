#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT11.h>
#include <LoRa.h>
#include <ArduinoJson.h>




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

// Pins LoRa per Heltec WiFi LoRa 32 V2
#define LORA_SCK  5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS   18
#define LORA_RST  14
#define LORA_DIO0 26

#define PIN_LED   25

void setup() {
  Serial.begin(115200);  // Millor velocitat per ESP32

  pinMode(PIN_LED, OUTPUT);  // Configura el pin 25 com a sortida
  digitalWrite(PIN_LED, LOW); // Apaga el LED inicialment (opcional)

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

  // Configura SPI amb els pins personalitzats
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  connect_lora();

  LoRa.setSignalBandwidth(125000); // BW de 500 kHz
}

void toggleLED() {
  // Llegeix l'estat actual i posa el contrari
  int state = digitalRead(PIN_LED);
  digitalWrite(PIN_LED, !state);
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

void connect_lora() {
  Serial.println("connect_lora - start");
  if (!LoRa.begin(915E6))
  //if (!LoRa.begin(868E6))
  {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("connect_lora - end");
}

void ReadLORAAndSendTemperatureAndHumidity()
{
  int temperature = 0;
  int humidity = 0;
  static char jsonBuffer[200];  // buffer per guardar el JSON rebut
  int index = 0;
  
  //Do the measure every interval (5s)
  unsigned long currentMillis = millis();

  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");
    toggleLED();

    // llegim el missatge i el guardem al buffer
    while (LoRa.available() && index < sizeof(jsonBuffer) - 1) {
      char c = (char)LoRa.read();
      Serial.print(c);
      jsonBuffer[index++] = c;
    }
    jsonBuffer[index] = '\0';  // tancar la cadena
    Serial.println("");

    StaticJsonDocument<128> doc;

    DeserializationError error = deserializeJson(doc, jsonBuffer);
    if (error) {
      Serial.print("Error de deserialització: ");
      Serial.println(error.c_str());
      return;
    }

    // Extreure valors
    int ownAdr = doc["ownAdr"];
    int destAdr = doc["destAdr"];
    float temperature = doc["temperature"];
    float humidity = doc["humidity"];

    if(destAdr!=2000)
    {
      Serial.println("Packet is not for me");
      return;
    }

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" °C\tHumidity: ");
    Serial.println(humidity);

    client.publish("test/temperature", String(temperature).c_str());
    client.publish("test/humidity", String(humidity).c_str());

  }

}


void loop() {  
  if (!client.connected()) {
    reconnect();
  }
    else
  {
    ReadLORAAndSendTemperatureAndHumidity();
  }
  client.loop();
}