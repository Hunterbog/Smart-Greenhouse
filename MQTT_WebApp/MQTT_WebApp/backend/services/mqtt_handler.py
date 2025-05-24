import os
import paho.mqtt.client as mqtt
from dotenv import load_dotenv

load_dotenv()

MQTT_USER = os.getenv("MQTT_USER")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD")
MQTT_BROKER = os.getenv("MQTT_BROKER")
MQTT_PORT = int(os.getenv("MQTT_PORT"))

def create_mqtt_client():
    client = mqtt.Client()
    client.username_pw_set(MQTT_USER, MQTT_PASSWORD)
    client.tls_set()
    try:
        client.connect(MQTT_BROKER, MQTT_PORT)
        print(f"Connected to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
    except Exception as e:
        print(f"Failed to connect to MQTT broker: {e}")

    return client
mqtt_client = create_mqtt_client()