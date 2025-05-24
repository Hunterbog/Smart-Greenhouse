from datetime import datetime, timedelta
from pymongo import MongoClient
import random
from bson import ObjectId

client = MongoClient('localhost', 27017)
db = client['flask_database']
senzori = db['senzori']


start_time = datetime(2025, 5, 15, 0, 0)

for i in range(1440 * 3):
    timestamp = start_time + timedelta(minutes=i)

    temperature = round(random.uniform(18, 28), 2)
    humidity = random.randint(30, 60)
    light = random.randint(200, 600)
    soil1 = random.randint(400, 1000)
    soil2 = random.randint(400, 1000)
    soil3 = random.randint(400, 1000)
    waterLevel = random.randint(50, 100)
    waterVolume = round(random.uniform(5, 15), 2)

    doc = {
        "timestamp": timestamp.strftime('%Y-%m-%d %H:%M:%S'),
        "active_crop_id": ObjectId("6828f20fa17f771e187714f8") if i > 1441 else ObjectId("690000000000000000000000"),
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

print("4320 date simulate (3 zile) au fost inserate cu succes.")
