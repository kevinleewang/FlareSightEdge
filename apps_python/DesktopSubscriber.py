import paho.mqtt.client as mqtt

# ===========================
# Configuration Parameters
# ===========================

# MQTT Broker Settings
BROKER_ADDRESS = "192.168.1.92"  # Replace with your laptop's IP address
BROKER_PORT = 1883                 # Default MQTT port
KEEPALIVE = 60                     # Keepalive interval in seconds

# MQTT Topics to Subscribe To
TOPICS = [
    ("sensor/co2", 0),   # QoS level 0
    ("sensor/smoke", 0)  # QoS level 0
]

# Optional: MQTT Authentication
USERNAME = "your_username"  # Replace with your MQTT username if authentication is enabled
PASSWORD = "your_password"  # Replace with your MQTT password if authentication is enabled

# ===========================
# Callback Functions
# ===========================

def on_connect(client, userdata, flags, rc):
    """
    Callback when the client receives a CONNACK response from the server.
    """
    if rc == 0:
        print("Connected to MQTT Broker successfully.")
        # Subscribe to the specified topics
        for topic, qos in TOPICS:
            client.subscribe(topic, qos)
            print(f"Subscribed to topic: {topic} with QoS: {qos}")
    else:
        print(f"Failed to connect to MQTT Broker. Return code: {rc}")

def on_message(client, userdata, msg):
    """
    Callback when a PUBLISH message is received from the server.
    """
    topic = msg.topic
    payload = msg.payload.decode('utf-8')
    print(f"Received message from topic '{topic}': {payload}")

def on_disconnect(client, userdata, rc):
    """
    Callback when the client disconnects from the broker.
    """
    if rc != 0:
        print("Unexpected disconnection from MQTT Broker.")
    else:
        print("Disconnected from MQTT Broker.")

# ===========================
# Main Function
# ===========================

def main():
    # Initialize the MQTT Client
    client = mqtt.Client()

    # Set Username and Password if Authentication is Enabled
    if USERNAME and PASSWORD:
        client.username_pw_set(USERNAME, PASSWORD)

    # Assign Callback Functions
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect

    try:
        # Connect to the MQTT Broker
        client.connect(BROKER_ADDRESS, BROKER_PORT, KEEPALIVE)

        # Start the Network Loop
        client.loop_forever()

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()
