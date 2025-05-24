document.addEventListener('DOMContentLoaded', () => {
    const container = document.getElementById('sensor-data');

    if (!container) {
        console.log("Nu ai recoltă activă.");
        return;
    }

    async function refreshSensorData() {
        try {
            const response = await fetch(`/data?_ts=${Date.now()}`, {
                headers: { "X-Requested-With": "XMLHttpRequest" }
            });

            const result = await response.json();

            if (result.error === "no_crop") {
                container.innerHTML = '<p><strong>Nu ai nicio recoltă plantată!</strong></p>';
                return;
            }

            if (result.error === "no_sensor") {
                container.innerHTML = '<p><strong>Nu sunt date de la senzori</strong></p>';
                return;
            }

            const sensor = result.sensor_data;

            if (!sensor) {
                container.innerHTML = '<p><strong>Nu sunt date de la senzori</strong></p>';
                return;
            }

            document.getElementById('temperature').textContent = `${sensor.temperature.toFixed(1)} °C`;
            document.getElementById('humidity').textContent = `${sensor.humidity.toFixed(1)}%`;
            document.getElementById('lux').textContent = `${sensor.light}`;
            document.getElementById('waterLevel').textContent = `${sensor.waterLevel}%`;
            document.getElementById('waterVolume').textContent = `${sensor.waterVolume} L`;
            document.getElementById('soil1').textContent = `${sensor.soil1} `;
            document.getElementById('soil2').textContent = `${sensor.soil2}`;
            document.getElementById('soil3').textContent = `${sensor.soil3}`;
            document.getElementById('timestamp').textContent = `${sensor.timestamp}`;

            const fill = document.getElementById('waterLevelFill');
            if (fill && !isNaN(sensor.waterLevel)) {
                const angle = (sensor.waterLevel / 100) * 180;
                fill.style.transform = `rotate(${angle}deg)`;
            }

        } catch (error) {
            console.error('Error fetching sensor data:', error);
            container.innerHTML = '<p><strong>Failed to load sensor data!</strong></p>';
        }
    }

    refreshSensorData();

    const mqttOptions = {
        username: "hunterbog",
        password: "Ashford1875",
        connectTimeout: 4000
    };

    const mqttClient = mqtt.connect('wss://5852f7b9b4c348afb716480914b3ea19.s1.eu.hivemq.cloud:8884/mqtt', mqttOptions);

    mqttClient.on('connect', () => {
        console.log("MQTT Connected");
        mqttClient.subscribe("myhome/esp8266/data");
    });

    mqttClient.on('message', () => {
        const requiredElements = [
            'temperature', 'humidity', 'lux',
            'waterLevel', 'waterVolume', 'soil1', 'soil2', 'soil3', 'timestamp'
        ];

        const missing = requiredElements.some(id => !document.getElementById(id));
        if (missing) {
            location.reload();
        } else {
            refreshSensorData();
        }
    });
});
