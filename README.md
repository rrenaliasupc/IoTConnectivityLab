# IoTConnectivityLab
Merit IoTConnectivity Project: Lab

## Abstract

This Lab is composed in two parts:
- **LAB 1**: Comprension of the MQTT protocol by using a ESP32 (MQTT publisher), an MQTT broker and a client that represents the values (MQTT subscriber).
- **LAB 2**: Introduce a new interface (LORA) to connect some sensors(+ ESP32) to a gateway that send the information using MQTT. The rest of the LAB uses the resources from LAB1.

## Organization

This repository is organized in three main folders:
- **ESP Arduino**: Contains the code for the ESP32. This code is done using Arduino IDE.
- **docker-mqtt broker**: Contains the docker-compose file to run the MQTT broker.
- **MQTT subscriber**: Contains the code for the MQTT subscriber. This code is done using Python.


## ESP Arduino
This folder contains the code for the ESP32. The code is done using Arduino IDE. 
The code is divided in two parts:
- **LAB1:** In this case we use only one ESP32:
    - **lab1*: The code for the ESP32 that publishes the values to the MQTT broker using WIFI.
- **LAB2**: In this case there are two different codes:
    - **lab2_tx_lora**: The code for the ESP32 that sends the values to the gateway using LORA.
    - **lab2_rx_lora**: The code for the ESP32 that receives the values from the sensors using LORA and sends them to the MQTT broker using WIFI.

Note: This code requires the installation of some libraries in the Arduino IDE:
- **LoRa** (LoRa by Sandeep Mistry): For LORA communication.
- **ArduinoJson** (ArduinoJson by Benoit): For handling JSON data.
- **DHT11** (DHT11 by DhrubaSaha): For reading DHT11 sensor data.
- **PubSubClient** (PubSubClient by Nick O'Leary): For MQTT communication.
- **WIFI** (ArduinoBLE by Arduino): For WIFI communication.

### Hardware requirements
- ESP32 development board: In this case, we use Heltec ESP32 LoRa board V2. https://heltec.org/project/wifi-lora-32v2/
- DHT11 sensor

## docker-mqtt broker

We have implemented a docker-compose file to run the MQTT broker. The broker used is Mosquitto, which is a popular open-source MQTT broker.

To deploy the MQTT broker, you need to have Docker and Docker Compose installed on your machine. Once you have them installed, you can run the following command in the terminal:

```bash
docker-compose up -d
```
This command will start the MQTT broker in detached mode. The broker will be accessible on port 1883.


## MQTT subscriber
This folder contains the code for the MQTT subscriber. The code is done using Python.
The code is divided in two parts:

- **mqttSubscriber.py**: The code for the MQTT subscriber that receives the values from the MQTT broker and prints them to the console.
- **mqttSubscriberGraph.py**: The code for the MQTT subscriber that receives the values from the MQTT broker and plots them in a graph using Matplotlib.

Install the required libraries using the following command:

```bash
pip install -r requirements.txt
```
Run the MQTT subscriber using the following command:

```bash
python3 mqttSubscriber.py
```
or

```bash
python3 mqttSubscriberGraph.py
```
