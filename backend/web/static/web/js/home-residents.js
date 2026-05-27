const residents = [
  { name: 'Alex', type: 'Person', position: 'Living Room', activity: 'Reading on sofa', color: '#2563eb' },
  { name: 'Jordan', type: 'Person', position: 'Kitchen', activity: 'Preparing dinner', color: '#16a34a' },
  { name: 'Taylor', type: 'Person', position: 'Bedroom', activity: 'Working at desk', color: '#d97706' },
  { name: 'Milo', type: 'Pet (Cat)', position: 'Living Room', activity: 'Napping near window', color: '#9333ea' },
  { name: 'Luna', type: 'Pet (Dog)', position: 'Hallway', activity: 'Walking between rooms', color: '#dc2626' },
];

function renderResidents() {
  const container = document.querySelector('[data-resident-list]');
  const count = document.querySelector('[data-resident-count]');
  if (!container) {
    return;
  }

  container.innerHTML = '';
  residents.forEach((resident) => {
    const item = document.createElement('article');
    item.className = 'resident-item';
    item.innerHTML = `
      <div class="resident-item__name-row">
        <span class="resident-item__color" style="background:${resident.color};" title="${resident.color}"></span>
        <div>
          <h3 class="resident-item__name">${resident.name}</h3>
          <p class="resident-item__type">${resident.type}</p>
        </div>
      </div>
      <dl class="resident-item__details">
        <div><dt>Position</dt><dd>${resident.position}</dd></div>
        <div><dt>Activity</dt><dd>${resident.activity}</dd></div>
        <div><dt>Color code</dt><dd><code>${resident.color}</code></dd></div>
      </dl>
    `;
    container.appendChild(item);
  });

  if (count) {
    count.textContent = `${residents.length} residents detected`;
  }
}

renderResidents();
