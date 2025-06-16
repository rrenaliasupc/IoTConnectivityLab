#include <DHT11.h>
#include <LoRa.h>
#include <ArduinoJson.h>


// Create an instance of the DHT11 class i PIN 4
DHT11 dht11(4);

// Define the variables and interval to generate periodic readings.
unsigned long previousMillis = 0;
const long interval = 1000; // 1 seconds in milliseconds

unsigned int LoraTxPower=2;

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

  // Configura SPI amb els pins personalitzats
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  connect_lora();

  LoRa.setSignalBandwidth(125000); // BW de 125 kHz
}

void toggleLED() {
  // Llegeix l'estat actual i posa el contrari
  int state = digitalRead(PIN_LED);
  digitalWrite(PIN_LED, !state);
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


        LoraTxPower++;
        if(LoraTxPower>20)
          LoraTxPower=2;

        //Empaquetem utilitzant JSON
        // Crear objecte JSON
        StaticJsonDocument<128> doc;
        doc["ownAdr"] = 1000;
        doc["destAdr"] = 2000;
        doc["txPower"] = LoraTxPower;
        doc["temperature"] = temperature;
        doc["humidity"] = humidity;

        // Serialitza a una cadena JSON
        char buffer[128];
        size_t n = serializeJson(doc, buffer);


        LoRa.setTxPower(LoraTxPower);
        Serial.print("LoraTxPower: ");
        Serial.println(LoraTxPower);

        // Enviar per LoRa
        LoRa.beginPacket();
        LoRa.write((uint8_t*)buffer, n);
        LoRa.endPacket();

        Serial.print("JSON enviat: ");
        Serial.println(buffer);

        toggleLED();


    } else {
        // Print error message based on the error code.
        Serial.println(DHT11::getErrorString(result));
    }
  }
}




void loop() {  
  MesureAndSendTemperatureAndHumidity();
}
