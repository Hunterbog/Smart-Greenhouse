from flask import Flask, request, jsonify, render_template
from pymongo import MongoClient
from flask_cors import CORS
from plants_history import plants_history_bp
from utils.status_helper import load_all_statuses, load_status, save_status
from jinja2 import TemplateNotFound
import paho.mqtt.client as mqtt
import logging
from random import random
from services.mqtt_handler import mqtt_client
import json
from datetime import datetime
from graphs import graphs_bp
from datetime import datetime
from bson import ObjectId

STATUS_FILE = 'status.json'
#logging.basicConfig(level=logging.INFO)
app = Flask(__name__, template_folder='../frontend/templates', static_folder='../frontend/static')
CORS(app)

client = MongoClient('localhost', 27017)
db = client.flask_database
senzori = db.senzori

app.register_blueprint(plants_history_bp, )
app.register_blueprint(graphs_bp)

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        timestamp = datetime.now()
       
        active_crop = db.plants_history.find_one({"active": True})
        if msg.topic == "myhome/esp8266/data":
            formatted_timestamp = timestamp.strftime('%Y-%m-%d %H:%M:%S')
            sensor_doc = {
                "timestamp": formatted_timestamp,
                "active_crop_id": active_crop["_id"] if active_crop else None,
                "soil1": payload.get("soil1"),
                "soil2": payload.get("soil2"),
                "soil3": payload.get("soil3"),
                "waterLevel": payload.get("waterLevel"),
                "waterVolume": payload.get("waterVolume"),
                "temperature": payload.get("temperature"),
                "humidity": payload.get("humidity"),
                "light": payload.get("light")
            }

            senzori.insert_one(sensor_doc)
            logging.info(f"Sensor data inserted: {sensor_doc}")

        elif msg.topic == "myhome/esp8266/actuators/all":
            data = load_all_statuses()
            for actuator_name, status in payload.items():
                data[actuator_name] = status
            with open(STATUS_FILE, 'w') as f:
                json.dump(data, f, indent=2)

    except json.JSONDecodeError:
        logging.error("Failed to parse MQTT payload (invalid JSON).")
    except Exception as e:
        logging.exception("Unexpected error in on_message handler.")



mqtt_client.on_message = on_message
mqtt_client.subscribe("myhome/esp8266/actuators/all")
mqtt_client.subscribe("myhome/esp8266/data")
mqtt_client.loop_start()

@app.after_request
def add_header(response):
    if request.path.startswith('/static/'):
        return response
    response.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
    response.headers['Pragma'] = 'no-cache'
    response.headers['Expires'] = '0'
    return response

@app.route('/')
def index():
    active_crop = db.plants_history.find_one({"active": True})

    if active_crop:
        statuses = load_all_statuses()
        return render_template('index.html', statuses=statuses, active_crop=active_crop, random=random)
    else:
        return render_template('plants.html')


@app.route('/data', methods=['GET'])
def handle_data():
    try:
        active_crop = db.plants_history.find_one({"active": True})
        if not active_crop and request.headers.get("X-Requested-With") == "XMLHttpRequest":
            return jsonify({"error": "no_crop"}), 200

        latest_sensor_data = None
        if active_crop:
            latest_sensor_data = senzori.find_one(
                {"active_crop_id": active_crop["_id"]},
                sort=[('_id', -1)]
            )

        if not latest_sensor_data and request.headers.get("X-Requested-With") == "XMLHttpRequest":
            return jsonify({"error": "no_sensor"}), 200

        if latest_sensor_data:
            latest_sensor_data["_id"] = str(latest_sensor_data["_id"])
            if isinstance(latest_sensor_data.get("active_crop_id"), ObjectId):
                latest_sensor_data["active_crop_id"] = str(latest_sensor_data["active_crop_id"])

        if active_crop:
            active_crop["_id"] = str(active_crop["_id"])

        if request.headers.get("X-Requested-With") == "XMLHttpRequest":
            return jsonify({
                "sensor_data": latest_sensor_data,
                "active_crop": True
            })

        return render_template('data.html',
                               sensor_data=latest_sensor_data,
                               active_crop=active_crop)

    except TemplateNotFound as e:
        logging.error(f"Template not found: {e}")
        return jsonify({"error": "Missing template"}), 500

    except Exception as e:
        logging.exception("Unknown error loading data:")
        return jsonify({"error": "Internal server error"}), 500

@app.route('/actuator/<name>', methods=['GET', 'POST'])
def actuator(name):
    if request.method == 'GET':
        return jsonify({"status": load_status(name)})

    elif request.method == 'POST':
        try:
            data = request.get_json()
            if not data:
                return jsonify({"error": "No data provided"}), 400

            topic = f"myhome/esp8266/actuator/{name}"

            if "status" in data:
                save_status(name, data["status"])
                mqtt_client.publish(topic, str(data["status"]))
            elif "value" in data:
                value = data["value"]
                save_status(name, value)
                mqtt_client.publish(topic, str(value))
            else:
                return jsonify({"error": "Missing status or value"}), 400

            return jsonify({"status": "OK"}), 200

        except mqtt.MQTTException as e:
            return jsonify({"error": "MQTT error"}), 500
        except Exception as e:
            return jsonify({"error": "Server error"}), 500
        
@app.route('/supraveghere')
def supraveghere():
    return render_template('supraveghere.html')

@app.route('/api/latest-waterlevel')
def latest_water_level():
    try:
        sensor_data = db.senzori.find_one(sort=[("_id", -1)])
        if not sensor_data:
            return jsonify({"error": "No sensor data found"}), 404
        return jsonify({"waterLevel": sensor_data.get("waterLevel", 0)}), 200
    except Exception as e:
        print("Exception:", str(e))  
        return jsonify({"error": str(e)}), 500


if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)