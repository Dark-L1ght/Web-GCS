// --- CONFIGURATION ---
const COMPANION_COMPUTER_IP = '192.168.10.100'; // <-- 192.168.10.100 for Drone and 127.0.0.1 for SITL

// --- MAP INITIALIZATION ---
const map = L.map('map').setView([0, 0], 2);
L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
    maxZoom: 22, // Corrected from 19 to a higher value for better zoom
}).addTo(map);

const droneIcon = L.icon({
    iconUrl: 'assets/navigation.png',
    iconSize: [32, 32],
    iconAnchor: [16, 16]
});
let vehicleMarker = null;
let mapInitialized = false;

// --- UI ELEMENT REFERENCES ---
const cameraFeedElem = document.getElementById('camera-feed'); // FIX: Added this missing variable
const connectionStatusElem = document.getElementById('connection-status');
const flightModeElem = document.getElementById('flight-mode');
const batteryLevelElem = document.getElementById('battery-level');
const logContainer = document.getElementById('log-container');
const armStatusElem = document.getElementById('arm-status');
const pitchElem = document.getElementById('data-pitch');
const rollElem = document.getElementById('data-roll');
const headingElem = document.getElementById('data-heading');
const altElem = document.getElementById('data-alt');
const longitudeElem = document.getElementById('data-longitude');
const latitudeElem = document.getElementById('data-latitude');
const groundspeedElem = document.getElementById('data-groundspeed');
const climbElem = document.getElementById('data-climb');

// --- WEBSOCKET LOGIC ---
function connectWebSocket() {
    const ws = new WebSocket(`ws://${window.location.hostname || 'localhost'}:8765`);
    ws.onopen = () => {
        console.log('Connected to MAVLink WebSocket server!');
        connectionStatusElem.textContent = 'CONNECTED';
        connectionStatusElem.style.backgroundColor = 'var(--secondary-color)';
    };
    ws.onmessage = (event) => {
        const data = JSON.parse(event.data);
        if (data.type === 'log') {
            updateLog(data);
        } else {
            updateAll(data);
        }
    };
    ws.onclose = () => {
        console.log('Disconnected. Reconnecting in 3s...');
        connectionStatusElem.textContent = 'DISCONNECTED';
        connectionStatusElem.style.backgroundColor = 'var(--error-color)';
        setTimeout(connectWebSocket, 3000);
    };
    ws.onerror = (error) => {
        console.error('WebSocket Error:', error);
        ws.close();
    };
}

// --- UPDATE FUNCTIONS ---
function updateAll(state) {
    if (state.pitch != null) pitchElem.textContent = `${state.pitch.toFixed(2)} °`;
    if (state.roll != null) rollElem.textContent = `${state.roll.toFixed(2)} °`;
    if (state.heading != null) headingElem.textContent = `${state.heading.toFixed(2)} °`;
    if (state.alt_rel != null) altElem.textContent = `${state.alt_rel.toFixed(2)} m`;
    if (typeof state.lat === 'number') latitudeElem.textContent = `${state.lat.toFixed(7)}`;
    if (typeof state.lon === 'number') longitudeElem.textContent = `${state.lon.toFixed(7)}`;
    if (state.flight_mode !== undefined) flightModeElem.textContent = state.flight_mode;
    if (state.level !== undefined) batteryLevelElem.textContent = `${state.level}%`;
    if (state.groundspeed != null) groundspeedElem.textContent = `${state.groundspeed.toFixed(2)} m/s`;
    if (state.climb != null) climbElem.textContent = `${state.climb.toFixed(2)} m/s`;

    if (state.armed !== undefined) {
        if (state.armed) {
            armStatusElem.textContent = 'ARMED';
            armStatusElem.classList.remove('disarmed');
            armStatusElem.classList.add('armed');
            armStatusElem.style.backgroundColor = 'var(--secondary-color)';
        } else {
            armStatusElem.textContent = 'DISARMED';
            armStatusElem.classList.remove('armed');
            armStatusElem.classList.add('disarmed');
            armStatusElem.style.backgroundColor = 'var(--error-color)';
        }
    }

    if (typeof state.lat === 'number' && typeof state.lon === 'number') {
        const latlng = [state.lat, state.lon];
        if (!mapInitialized) {
            map.setView(latlng, 19); // FIX: Changed invalid zoom of 25 to 19
            vehicleMarker = L.marker(latlng, { icon: droneIcon, rotationAngle: 0 }).addTo(map);
            mapInitialized = true;
        } else {
            vehicleMarker.setLatLng(latlng);
        }
        if (typeof state.heading === 'number') {
            vehicleMarker.setRotationAngle(state.heading);
        }
    }
}

function updateLog(log) {
    const logMessage = document.createElement('div');
    logMessage.className = `log-message severity-${log.severity}`;
    const timestamp = new Date().toLocaleTimeString('id-ID', { hour12: false });
    logMessage.textContent = `[${timestamp}] ${log.text}`;
    logContainer.appendChild(logMessage);
    logContainer.scrollTop = logContainer.scrollHeight;
}

// --- START THE APP ---
cameraFeedElem.src = `http://${COMPANION_COMPUTER_IP}:5001/video_feed`;
connectWebSocket();