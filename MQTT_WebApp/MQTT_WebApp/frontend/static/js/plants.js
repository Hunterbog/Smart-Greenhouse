
// async function plantCrop(plant_type_id) {
//   const response = await fetch('/plants_history', {
//     method: 'POST',
//     headers: { 'Content-Type': 'application/json' },
//     body: JSON.stringify({ plant_type_id })
//   });

//   if (response.ok) {
//     location.reload();
//   } else {
//     const data = await response.json();
//     alert(data.error || "Eroare la salvare!");
//   }
// }

async function markAsHarvested() {
  const confirmare = confirm("Ești sigur că vrei să oprești sera și să marchezi cultura ca recoltată?");
  if (!confirmare) return;

  const response = await fetch('/plants_history/harvest', { method: 'POST' });

  if (response.ok) {
    location.reload();
  } else {
    alert("Eroare la marcare ca recoltată.");
  }
}

document.getElementById('startForm')?.addEventListener('submit', async function (e) {
  e.preventDefault();
  const form = e.target;

  const tempDay = form.optimal_temp_day.value;
  const humDay = form.optimal_humidity_day.value;
  const tempNight = form.optimal_temp_night.value;
  const humNight = form.optimal_humidity_night.value;
  
  const data = {
    optimal_temp_day: tempDay,
    optimal_humidity_day: humDay,
    optimal_temp_night: tempNight,
    optimal_humidity_night: humNight 
  };

  const response = await fetch('/plants_history', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data)
  });

  const result = await response.json();
  if (response.ok) {
    location.reload();
  } else {
    alert('Eroare: ' + (result.error || 'Nu s-a putut porni sera'));
  }
});

function openCommandPopup() {
  document.getElementById("commandForm").style.display = "block";
  document.getElementById("overlay").style.display = "block";
  document.body.style.overflow = "hidden";

  fetch('/plants_update')
    .then(res => res.json())
    .then(data => {
      if (!window.slidersInitialized) {
        initSliders(data); 
        window.slidersInitialized = true;
      } else {
        setSlidersFromServer(data);
      }
    })
    .catch(err => console.error('Eroare la preluare:', err));
}


function closeCommandPopup() {
  document.getElementById("commandForm").style.display = "none";
  document.getElementById("overlay").style.display = "none";
  document.body.style.overflow = "";
}

function submitForm(event) {
  event.preventDefault();

  const data = {
    optimal_temp_day: document.getElementById("temp_day").value,
    optimal_humidity_day: document.getElementById("humidity_day").value,
    optimal_temp_night: document.getElementById("temp_night").value,
    optimal_humidity_night: document.getElementById("humidity_night").value
  };

  fetch('/plants_update', {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data)
  })
    .then(res => res.json())
    .then(response => {
      if (response.status === "success") {
        closeCommandPopup();
        document.getElementById("temp_day").value = "";
        document.getElementById("humidity_day").value = "";
        document.getElementById("temp_night").value = "";
        document.getElementById("humidity_night").value = "";
      } else {
        alert("Eroare: " + (response.message || "necunoscută"));
      }
    })
    .catch(error => {
      console.error(error);
      alert("Eroare!");
    });
}

function initSliders(savedData = null) {
  const sliders = [
    {
      id: 'slider-temp-day',
      inputId: 'temp_day',
      displayId: 'temp_day_value',
      min: 0,
      max: 50,
      defaultStart: [24, 30],
      key: 'optimal_temp_day',
      step: 1,
      suffix: '°C'
    },
    {
      id: 'slider-temp-night',
      inputId: 'temp_night',
      displayId: 'temp_night_value',
      min: 0,
      max: 50,
      defaultStart: [18, 26],
      key: 'optimal_temp_night',
      step: 1,
      suffix: '°C'
    },
    {
      id: 'slider-humidity-day',
      inputId: 'humidity_day',
      displayId: 'humidity_day_value',
      min: 0,
      max: 100,
      defaultStart: [65, 70],
      key: 'optimal_humidity_day',
      step: 1,
      suffix: '%'
    },
    {
      id: 'slider-humidity-night',
      inputId: 'humidity_night',
      displayId: 'humidity_night_value',
      min: 0,
      max: 100,
      defaultStart: [70, 80],
      key: 'optimal_humidity_night',
      step: 1,
      suffix: '%'
    }
  ];

  sliders.forEach(s => {
    const slider = document.getElementById(s.id);
    if (!slider || slider.classList.contains("noUi-target")) return;

    let start = s.defaultStart;

    if (savedData && savedData[s.key]) {
      const parts = savedData[s.key].split('-').map(Number);
        start = parts;
    }
    noUiSlider.create(slider, {
      start,
      connect: true,
      step: s.step,
      range: {
        min: s.min,
        max: s.max
      }
    });

    const input = document.getElementById(s.inputId);
    const output = document.getElementById(s.displayId);

    slider.noUiSlider.on('update', function (values) {
      const roundedValues = values.map(v => Math.round(v));
      input.value = roundedValues.join('-');
      output.innerText = values.map(v => `${parseFloat(v).toFixed(2)}${s.suffix}`).join(' – ');
    });
  });
}

function setSlidersFromServer(data) {
  const fields = [
    { key: 'optimal_temp_day', sliderId: 'slider-temp-day', inputId: 'temp_day', displayId: 'temp_day_value', suffix: '°C' },
    { key: 'optimal_humidity_day', sliderId: 'slider-humidity-day', inputId: 'humidity_day', displayId: 'humidity_day_value', suffix: '%' },
    { key: 'optimal_temp_night', sliderId: 'slider-temp-night', inputId: 'temp_night', displayId: 'temp_night_value', suffix: '°C' },
    { key: 'optimal_humidity_night', sliderId: 'slider-humidity-night', inputId: 'humidity_night', displayId: 'humidity_night_value', suffix: '%' }
  ];

  fields.forEach(f => {
    const val = data[f.key];
    if (!val || !val.includes('-')) return;

    const [min, max] = val.split('-').map(Number);
    const slider = document.getElementById(f.sliderId);
    const input = document.getElementById(f.inputId);
    const display = document.getElementById(f.displayId);

    if (slider && slider.noUiSlider) {
      slider.noUiSlider.set([min, max]);
      input.value = `${min}-${max}`;
      display.innerText = `${min.toFixed(2)}${f.suffix} – ${max.toFixed(2)}${f.suffix}`;
    }
  });
}


window.addEventListener('DOMContentLoaded', () => {
  if (document.getElementById('slider-temp-day')) {
    if (!window.slidersInitialized) {
      initSliders(null);
      window.slidersInitialized = true;
    }
  }
});

