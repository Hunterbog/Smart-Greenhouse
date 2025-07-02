from datetime import datetime, timedelta
from pymongo import MongoClient
import random
from bson import ObjectId

client = MongoClient('localhost', 27017)
db = client['flask_database']
senzori = db['senzori']

# Calculează timpul de start: 5 zile în urmă, la ora 00:00
end_time = datetime.now().replace(hour=23, minute=0, second=0, microsecond=0)
start_time = end_time - timedelta(days=5)

# Număr total de minute (5 zile * 24 ore * 60 minute)
total_minutes = int((end_time - start_time).total_seconds() / 60)

for i in range(total_minutes):
    timestamp = start_time + timedelta(minutes=i)

    temperature = round(random.uniform(18, 28), 2)
    humidity = random.randint(50, 60)
    light = random.randint(0, 100)  # modificat
    soil1 = random.randint(60, 80)  # modificat
    soil2 = random.randint(60, 80)  # modificat
    soil3 = random.randint(60, 80)  # modificat
    waterLevel = random.randint(50, 100)
    waterVolume = round(random.uniform(0, 3), 2)

    doc = {
        "timestamp": timestamp.strftime('%Y-%m-%d %H:%M:%S'),
        "active_crop_id": ObjectId("6864246d73a382b0f095794f"),
        "soil1": soil1,
        "soil2": soil2,
        "soil3": soil3,
        "waterLevel": waterLevel,
        "waterVolume": waterVolume,
        "temperature": temperature,
        "humidity": humidity,
        "light": light
    }

    senzori.insert_one(doc)

print(f"{total_minutes} date simulate ({total_minutes//1440} zile) au fost inserate cu succes.")
