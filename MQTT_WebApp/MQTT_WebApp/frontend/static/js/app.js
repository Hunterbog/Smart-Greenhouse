async function initActuatorPage() {
  const GrennhouseStatusElement = document.getElementById('mod-functionare');
  const actuatorControls = document.getElementById('actuator-controls');
  const btnAuto = document.getElementById('btn-automat');
  const btnManual = document.getElementById('btn-manual');

  const logicActuators = ['pompa1', 'pompa2', 'pompa3', 'umidificator', 'geam', 'lumina'];
  const analogActuators = ['ventilator1', 'ventilator2', 'incalzire1', 'incalzire2'];

  function updateActuatorVisibility(state) {
    if (actuatorControls) {
      actuatorControls.classList.toggle('opacity', state !== 'ON');
    }
  }

  async function fetchModStateAndInit() {
    try {
      const res = await fetch('/actuator/mod?t=' + new Date().getTime(), {
        method: 'GET',
        headers: { 'Content-Type': 'application/json' }
      });
      const data = await res.json();
      const state = data.status;

      GrennhouseStatusElement.textContent = state === 'ON' ? 'MANUAL' : 'AUTOMAT';
      updateActuatorVisibility(state);
      btnManual.classList.toggle('active', state === 'ON');
      btnAuto.classList.toggle('active', state !== 'ON');
    } catch (err) {
      console.error('Failed to load MOD state', err);
    }
  }

  async function setModState(newState) {
    try {
      await fetch('/actuator/mod', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ status: newState })
      });
      GrennhouseStatusElement.textContent = newState === 'ON' ? 'MANUAL' : 'AUTOMAT';
      updateActuatorVisibility(newState);
      btnManual.classList.toggle('active', newState === 'ON');
      btnAuto.classList.toggle('active', newState !== 'ON');
    } catch (err) {
      console.error('Failed to set mod state:', err);
    }
  }

  btnAuto?.addEventListener('click', () => setModState('OFF'));
  btnManual?.addEventListener('click', () => setModState('ON'));
  await fetchModStateAndInit();

  logicActuators.forEach(async name => {
    const el = document.getElementById(name);
    if (!el) return;

    try {
      const res = await fetch('/actuator/' + name + '?t=' + new Date().getTime(), {
        method: 'GET',
        headers: { 'Content-Type': 'application/json' }
      });
      const data = await res.json();
      const currentState = data.status;
      el.checked = (currentState === 'ON');
    } catch (err) {
      console.error('Failed to load state for ' + name, err);
    }

    el.addEventListener('change', async function (e) {
      const newState = e.target.checked ? 'ON' : 'OFF';

      try {
        await fetch('/actuator/' + name, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ status: newState })
        });
      } catch (err) {
        console.error('Failed to set ' + name, err);
      }
    });
  });

  function publishAnalog(id) {
    const value = document.getElementById(id).value;
    fetch('/actuator/' + id, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ value: value })
    }).then(res => {
      if (!res.ok) throw new Error('Failed to publish');
      localStorage.setItem(id, value);
    }).catch(err => console.error(err));
  }

analogActuators.forEach(id => {
  const slider = document.getElementById(id);
  const output = document.getElementById(id + '-value');

  if (slider && output) {
    slider.addEventListener('input', () => {
      output.textContent = slider.value;
    });
    slider.addEventListener('change', () => {
      publishAnalog(id);
    });
  }
});





  const mqttOptions = {
    username: 'hunterbog',
    password: 'Ashford1875',
    connectTimeout: 4000
  };
  const mqttClient = mqtt.connect('wss://5852f7b9b4c348afb716480914b3ea19.s1.eu.hivemq.cloud:8884/mqtt', mqttOptions);

  mqttClient.on('connect', () => {
    mqttClient.subscribe('myhome/esp8266/actuators/all');
  });

  mqttClient.on('message', (topic, message) => {
    if (topic === 'myhome/esp8266/actuators/all') {
      try {
        const payload = JSON.parse(message.toString());
        for (const name in payload) {
          const el = document.getElementById(name);
          if (!el) continue;

          if (el.type === 'checkbox') {
            el.checked = (payload[name] === 'ON');
          } else if (el.type === 'range') {
            el.value = payload[name];
            const span = document.getElementById(name + '-value');
            if (span) span.textContent = payload[name];
          }
        }
      } catch (e) {
        console.error('MQTT JSON parsing error:', e);
      }
    }
  });
}

document.addEventListener('DOMContentLoaded', initActuatorPage);
