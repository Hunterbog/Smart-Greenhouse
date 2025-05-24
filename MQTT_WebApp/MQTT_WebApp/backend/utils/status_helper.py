import os
import json

STATUS_FILE = 'status.json'

def load_all_statuses():
    if os.path.exists(STATUS_FILE):
        with open(STATUS_FILE, 'r') as f:
            try:
                return json.load(f)
            except json.JSONDecodeError:
                pass
    return {}

def load_status(actuator_name):
    data = load_all_statuses()
    return data.get(actuator_name.lower(), "OFF")

def save_status(actuator_name, status):
    data = load_all_statuses()
    normalized_name = actuator_name.lower()
    data[normalized_name] = status
    with open(STATUS_FILE, 'w') as f:
        json.dump(data, f, indent=2)
