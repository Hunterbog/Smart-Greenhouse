from flask import Blueprint, request, jsonify, render_template
from datetime import datetime
from pymongo import MongoClient
import json
import logging
from utils.status_helper import save_status
from services.mqtt_handler import mqtt_client
from utils.normalize import normalize_interval 

client = MongoClient('localhost', 27017)
db = client.flask_database
plants_history = db.plants_history

plants_history_bp = Blueprint('plants_history', __name__, template_folder='../frontend/templates')


@plants_history_bp.route('/plants_history', methods=['GET', 'POST'])
def handle_plants():
    if request.method == 'POST':
        data = request.get_json()

        try:

            temp_day = normalize_interval(data.get("optimal_temp_day"))
            humidity_day = normalize_interval(data.get("optimal_humidity_day"))
            temp_night = normalize_interval(data.get("optimal_temp_night"))
            humidity_night = normalize_interval(data.get("optimal_humidity_night"))

            planting_date = datetime.now()
            plants_history.update_many({}, {"$set": {"active": False}})
            plant_doc = {
                "planting_date": planting_date.strftime("%Y-%m-%d"),
                "optimal_temp_day": temp_day,
                "optimal_humidity_day": humidity_day,
                "optimal_temp_night": temp_night,
                "optimal_humidity_night": humidity_night,
                "active": True
            }
            plants_history.insert_one(plant_doc)
            timestamp = planting_date.strftime("%Y-%m-%d %H:%M:%S")
            mqtt_payload = f"True,{temp_day},{humidity_day},{temp_night},{humidity_night},{timestamp}"
            mqtt_client.publish("myhome/greenhouse/status", mqtt_payload)
            print("Payload trimis MQTT:", mqtt_payload)

            return jsonify({"status": "success"}), 201

        except Exception as e:
            logging.exception(e)
            return jsonify({"error": str(e)}), 400

    elif request.method == 'GET':
        try:
            active_crop = plants_history.find_one({"active": True})
            return render_template("plants.html", active_crop=active_crop)
        except Exception as e:
            logging.exception(e)
            return jsonify({"error": str(e)}), 404
        
@plants_history_bp.route('/plants_update', methods=['GET', 'PUT'])
def send_commands():
    if request.method == 'PUT':
        data = request.get_json()

        try:
            temp_day = normalize_interval(data.get("optimal_temp_day"))
            humidity_day = normalize_interval(data.get("optimal_humidity_day"))
            temp_night = normalize_interval(data.get("optimal_temp_night"))
            humidity_night = normalize_interval(data.get("optimal_humidity_night"))
            command_date = datetime.now()

            plant_doc = {
                "optimal_temp_day": temp_day,
                "optimal_humidity_day": humidity_day,
                "optimal_temp_night": temp_night,
                "optimal_humidity_night": humidity_night,
            }

            plants_history.update_one({'active': True}, {'$set': plant_doc})

            timestamp = command_date.strftime("%Y-%m-%d %H:%M:%S")
            mqtt_payload = f"Running,{temp_day},{humidity_day},{temp_night},{humidity_night},{timestamp}"
            mqtt_client.publish("myhome/greenhouse/status", mqtt_payload)
            print("Payload trimis MQTT:", mqtt_payload)
            return jsonify({"status": "success"}), 201

        except Exception as e:
            logging.exception(e)
            return jsonify({"error": str(e)}), 400

    elif request.method == 'GET':
        try:
            active_crop = plants_history.find_one({"active": True})
            if not active_crop:
                return jsonify({"error": "Nicio cultură activă"}), 404

            return jsonify({
                "optimal_temp_day": active_crop.get("optimal_temp_day", ""),
                "optimal_humidity_day": active_crop.get("optimal_humidity_day", ""),
                "optimal_temp_night": active_crop.get("optimal_temp_night", ""),
                "optimal_humidity_night": active_crop.get("optimal_humidity_night", "")
            })

        except Exception as e:
            logging.exception(e)
            return jsonify({"error": str(e)}), 500

@plants_history_bp.route('/plants_history/harvest', methods=['POST'])
def harvest_active_crop():
    try:
        command_date = datetime.now()

        result = plants_history.update_one({"active": True},{"$set": {"turn_off_date": command_date, "active": False}})

        if result.modified_count > 0:
            mqtt_client.publish("myhome/greenhouse/status", json.dumps({"has_crop": False}))

            actuator_defaults = {
                "mod": "OFF",
                "pompa1": "OFF",
                "geam": "OFF",
                "umidificator": "OFF",
                "pompa3": "OFF",
                "pompa2": "OFF",
                "incalzire1": "0",
                "incalzire2": "0",
                "ventilator1": "0",
                "ventilator2": "0",
                "lumina": "OFF"
            }

            for actuator, value in actuator_defaults.items():
                save_status(actuator, value)

            return jsonify({"status": "success"})
        else:
            return jsonify({"error": "Nu există o recoltă activă."}), 404

    except Exception as e:
        logging.exception(e)
        return jsonify({"error": str(e)}), 500



@plants_history_bp.route('/plants_update_form')
def show_plant_update_form():
    return render_template("plants_update_form.html")
