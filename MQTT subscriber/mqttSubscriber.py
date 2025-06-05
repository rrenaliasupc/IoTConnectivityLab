import paho.mqtt.client as mqtt

# Callback quan es connecta al broker
def on_connect(client, userdata, flags, rc):
    print("Connectat amb codi de resultat " + str(rc))
    client.subscribe("#")  # Subscriu-te a TOTS els temes

# Callback quan arriba un missatge
def on_message(client, userdata, msg):
    print(f"Tema: {msg.topic} | Missatge: {msg.payload.decode()}")

# Crea el client
client = mqtt.Client()

# Assigna callbacks
client.on_connect = on_connect
client.on_message = on_message

# Connecta't al broker
client.connect("192.168.137.1", 1883, 60)  # Canvia "nom.del.broker" per l’adreça IP o domini

# Bucle per mantenir la connexió i escoltar missatges
client.loop_forever()