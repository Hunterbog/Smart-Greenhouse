const ctx = document.getElementById(chartConfig.canvasId).getContext('2d');
let chart;
let currentPeriod = '1zi';
let currentOffset = 0;
let lastTimestamp = null;

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
    loadData(true);
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

async function loadData(force = false) {
  const url = `/sensor-data?sensor=${chartConfig.sensor}&period=${currentPeriod}&page=${currentOffset}`;
  try {
    const response = await fetch(url);
    const json = await response.json();

    const newTimestamp = json.last_timestamp;

    const tsDate = new Date(newTimestamp);
    const minute = tsDate.getMinutes();

    if (currentPeriod !== 'maxim' && !force && minute % 5 !== 0) return;

    renderChart(json.labels, json.data);
    lastTimestamp = newTimestamp;

  } catch (err) {
    console.error('Eroare la fetch:', err);
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
  if (currentOffset === 0) {
    loadData();
  }
}, 10000);

setPeriod('1zi');
