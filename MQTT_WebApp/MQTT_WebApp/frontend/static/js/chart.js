const ctx = document.getElementById(chartConfig.canvasId).getContext('2d');
let chart;
let currentPeriod = '1zi';
let currentOffset = 0;

function setPeriod(period) {
  currentPeriod = period;
  currentOffset = 0;

  document.querySelectorAll('.chart-button').forEach(btn => btn.classList.remove('active'));
  document.querySelector(`[onclick="setPeriod('${period}')"]`).classList.add('active');

  const showNav = (period !== 'maxim');
  document.getElementById('back-button').style.display = showNav ? 'inline-block' : 'none';
  document.getElementById('forward-button').style.display = showNav ? 'inline-block' : 'none';

  loadData();
  updateNavButtons();
}

function goBack() {
  if (currentOffset < 24) {
    currentOffset++;
    loadData();
    updateNavButtons();
  }
}

function goForward() {
  if (currentOffset > 0) {
    currentOffset--;
    loadData();
    updateNavButtons();
  }
}

async function loadData() {
  const url = `/sensor-data?sensor=${chartConfig.sensor}&period=${currentPeriod}&page=${currentOffset}`;
  try {
    const response = await fetch(url);
    const json = await response.json();
    renderChart(json.labels, json.data);
  } catch (err) {
    console.error('Eroare la fetch');
  }
}

function renderChart(labels, data) {
  if (chart) chart.destroy();

  const datasets = [];
  const isMulti = typeof data === 'object' && !Array.isArray(data);
  const keys = isMulti ? Object.keys(data) : [''];

  const values = isMulti
    ? Object.values(data).flat().filter(n => n !== null)
    : data.filter(n => n !== null);

  const yMin = Math.min(...values);
  const yMax = Math.max(...values);

  keys.forEach((key, i) => {
    const color = chartConfig.colorSet[i % chartConfig.colorSet.length];

    datasets.push({
      label: isMulti ? `${chartConfig.labelPrefix} ${key.replace(/[^\d]/g, '')}` : chartConfig.labelPrefix,
      data: isMulti ? data[key] : data,
      borderColor: color,
      fill: false,
      tension: 0.1,
      pointRadius: 4,
      pointBackgroundColor: color,
      spanGaps: true,
      borderWidth: 2
    });
  });

  chart = new Chart(ctx, {
    type: 'line',
    data: {
      labels: labels,
      datasets: datasets
    },
    options: {
      animation: false,
      responsive: true,
      plugins: {
        legend: {
          display: chartConfig.showLegend !== false
        },
        tooltip: {
          enabled: true,
          titleFont: { size: 16 },
          bodyFont: { size: 14 }
        }
      },
      scales: {
        y: {
          min: Math.floor(yMin - yMin * 0.1),
          max: Math.ceil(yMax + yMax * 0.1),
          title: {
            display: true,
            text: `${chartConfig.labelPrefix} (${chartConfig.unit})`,
            font: { size: 16 }
          },
          ticks: { font: { size: 14 } },
          grid: { display: true }
        },
        x: {
          title: {
            display: true,
            text: 'Perioadă',
            font: { size: 16 }
          },
          ticks: { font: { size: 14 } },
          grid: { display: false }
        }
      }
    }
  });
}

function updateNavButtons() {
  const backBtn = document.getElementById('back-button');
  const forwardBtn = document.getElementById('forward-button');
  forwardBtn.disabled = currentOffset <= 0;
  backBtn.disabled = currentOffset >= 24;
}


setInterval(() => {
  const now = new Date();
  if (now.getMinutes() % 5 === 0) {
    if (currentOffset === 0) {
      loadData();
    }
  }
}, 1000);

setPeriod('1zi');
