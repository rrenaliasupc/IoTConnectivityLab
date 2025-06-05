import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# Configurate the MQTT broker and topic
TOPIC = "test/temperature"
BROKER = "192.168.137.1"  # o l'adreça del teu broker local

# We will use deques to store the last 100 temperature values
temps = deque(maxlen=100)
valors = deque(maxlen=100)

# Callback when connected.
def on_connect(client, userdata, flags, rc):
    print("Connected with code ", rc)
    client.subscribe(TOPIC)

# Callback when a message is received.
def on_message(client, userdata, msg):
    try:
        valor = float(msg.payload.decode())
        # temps.append(len(1))  # Pending to update with time
 
        valors.append(valor)
        print(f"{msg.topic} → {valor}")
    except ValueError:
        print("Message received is not a number:", msg.payload)

# Set up the MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, 1883, 60)
client.loop_start()  # Important to start the loop in a separate thread

# Configure the plot
fig, ax = plt.subplots()
linea, = ax.plot([], [], lw=2)
ax.set_ylim(0, 50)  # Set the y-axis limits for temperature
ax.set_xlim(0, 100)
ax.set_xlabel("Time")
ax.set_ylabel("Temperature (°C)")
ax.set_title("Real time temperature data")

# Function that updates the plot
def update(frame):
    linea.set_data(range(len(valors)), valors)
    ax.set_xlim(max(0, len(valors)-100), len(valors))
    return linea,

ani = animation.FuncAnimation(fig, update, interval=1000)
plt.tight_layout()
plt.show()