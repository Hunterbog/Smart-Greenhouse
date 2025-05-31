from flask import Blueprint, render_template, request, jsonify
from datetime import datetime, timedelta
from pymongo import MongoClient
import locale
locale.setlocale(locale.LC_TIME, 'ro_RO.UTF-8')

graphs_bp = Blueprint('graphs_bp', __name__)

client = MongoClient('localhost', 27017)
db = client.flask_database
senzori = db.senzori
plants_history = db.plants_history

@graphs_bp.route('/temperature-graph')
def temperature_graph():
    return render_template('temperature-graph.html')

@graphs_bp.route('/sensor-data')
def sensor_data():
    sensor_field = request.args.get('sensor')
    period = request.args.get('period', '1zi')
    page = int(request.args.get('page', 0))
    now = datetime.now()

    field_map = {
        'temperature': ['temperature'],
        'soil_moisture': ['soil1', 'soil2', 'soil3'],
        'humidity': ['humidity'],
        'all': ['temperature', 'soil1', 'soil2', 'soil3', 'humidity']
    }

    if sensor_field not in field_map:
        return jsonify({'error': 'Sensor invalid'}), 400

    selected_fields = field_map[sensor_field]
    active_crop = plants_history.find_one({"active": True})
    crop_filter = {"active_crop_id": active_crop["_id"]} if active_crop else {}

    if period == '1zi':
        end_time = now - timedelta(minutes=page * 60)
        start_time = end_time - timedelta(hours=1)
        slot_duration = timedelta(minutes=5)
        label_format = '%H:%M'

        query_filter = {"timestamp": {"$exists": True}, **crop_filter}
        docs = senzori.find(query_filter).sort("timestamp", 1)

        slot_buckets = {}
        for doc in docs:
            try:
                ts = datetime.strptime(doc['timestamp'], '%Y-%m-%d %H:%M:%S')
                if not (start_time <= ts <= end_time):
                    continue

                slot_time = ts - timedelta(
                    minutes=ts.minute % int(slot_duration.total_seconds() / 60),
                    seconds=ts.second,
                    microseconds=ts.microsecond
                )

                if slot_time not in slot_buckets:
                    slot_buckets[slot_time] = {f: [] for f in selected_fields}

                for field in selected_fields:
                    if field in doc and doc[field] is not None:
                        slot_buckets[slot_time][field].append(doc[field])
            except Exception:
                continue
        sorted_slots = sorted(slot_buckets.keys())
        last_timestamp = sorted_slots[-1].strftime('%Y-%m-%d %H:%M:%S') if sorted_slots else None

        results = {f: [] for f in selected_fields}
        labels = []

        for slot in sorted_slots:
            labels.append(slot.strftime(label_format))
            for field in selected_fields:
                values = slot_buckets.get(slot, {}).get(field, [])
                avg = round(sum(values) / len(values), 2) if values else None
                results[field].append(avg)

        if len(selected_fields) == 1:
            results = results[selected_fields[0]]

        return jsonify({'labels': labels, 'data': results, 'last_timestamp': last_timestamp})

    elif period == 'maxim':
        query_filter = {"timestamp": {"$exists": True}, **crop_filter}
        first_doc = senzori.find_one(query_filter, sort=[("timestamp", 1)])
        if not first_doc:
            return jsonify({'labels': [], 'data': {}})

        first_date = datetime.strptime(first_doc['timestamp'], '%Y-%m-%d %H:%M:%S').date()
        today = now.date()

        day_buckets = {}
        for doc in senzori.find(query_filter):
            try:
                ts = datetime.strptime(doc['timestamp'], '%Y-%m-%d %H:%M:%S')
                doc_date = ts.date()
                if doc_date not in day_buckets:
                    day_buckets[doc_date] = {f: [] for f in selected_fields}
                for field in selected_fields:
                    if field in doc and doc[field] is not None:
                        day_buckets[doc_date][field].append(doc[field])
            except Exception:
                continue

        results = {f: [] for f in selected_fields}
        labels = []
        current_day = first_date

        while current_day <= today:
            labels.append(current_day.strftime('%d %B'))  
            for field in selected_fields:
                values = day_buckets.get(current_day, {}).get(field, [])
                avg = round(sum(values) / len(values), 2) if values else None
                results[field].append(avg)
            current_day += timedelta(days=1)

        if len(selected_fields) == 1:
            results = results[selected_fields[0]]

        return jsonify({'labels': labels, 'data': results})

    return jsonify({'error': 'Perioadă invalidă'}), 400

@graphs_bp.route('/soil-graph')
def soil_graph():
    return render_template('soil-graph.html')

@graphs_bp.route('/humidity-graph')
def humidity_graph():
    return render_template('humidity-graph.html')
