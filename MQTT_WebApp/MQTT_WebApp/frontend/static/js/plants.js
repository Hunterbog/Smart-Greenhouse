function isValidInterval(input) {
  if (!input) return false;
  const cleaned = input.toString().trim().replace(/\s+/g, ' ').replace(/-/g, ' ').trim();
  const parts = cleaned.split(' ');

  if (parts.length === 1) {
    return /^\d+(\.\d+)?$/.test(parts[0]);
  } else if (parts.length === 2) {
    return /^\d+(\.\d+)?$/.test(parts[0]) && /^\d+(\.\d+)?$/.test(parts[1]);
  }

  return false;
}

async function plantCrop(plant_type_id) {
  const response = await fetch('/plants_history', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ plant_type_id })
  });

  if (response.ok) {
    location.reload();
  } else {
    const data = await response.json();
    alert(data.error || "Eroare la salvare!");
  }
}

async function markAsHarvested() {
  const confirmare = confirm("Ești sigur că vrei să oprești sera și să marchezi cultura ca recoltată?");

  if (!confirmare) {
    return;
  }

  const response = await fetch('/plants_history/harvest', {
    method: 'POST'
  });

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

  if (tempDay && !isValidInterval(tempDay)) {
    alert("Valoare incorectă pentru temperatura optimă zi.");
    return;
  }

  if (humDay && !isValidInterval(humDay)) {
    alert("Valoare incorectă pentru umiditatea optimă zi.");
    return;
  }

  if (tempNight && !isValidInterval(tempNight)) {
    alert("Valoare incorectă pentru temperatura optimă noapte.");
    return;
  }

  if (humNight && !isValidInterval(humNight)) {
    alert("Valoare incorectă pentru umiditatea optimă noapte.");
    return;
  }

  const data = {
    optimal_temp_day: tempDay || "24",
    optimal_humidity_day: humDay || "65",
    optimal_temp_night: tempNight || "18",
    optimal_humidity_night: humNight || "75"
  };

  const response = await fetch('/plants_history', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json'
    },
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
    .catch(err => {
      console.error('Eroare la preluare:', err);
    });
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
    method: 'POST',
    headers: {
      'Content-Type': 'application/json'
    },
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
      //  alert("Trimis cu Succes!");

      } else {
        alert("Eroare: " + (response.message || "necunoscută"));
      }
    })
    .catch(error => {
      console.error(error);
      alert("Eroare!");
    });
}