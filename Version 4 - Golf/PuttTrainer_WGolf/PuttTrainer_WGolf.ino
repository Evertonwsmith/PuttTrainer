#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include <stdint.h>

const char* AP_SSID = "IR-Sensors";
const char* AP_PASS = "12345678";

// Pins:
// Index 0 -> Sensor 1 (GPIO 25, backstroke line)
// Index 1 -> Sensor 2 (GPIO 26, heel)
// Index 2 -> Sensor 3 (GPIO 27, toe)
const int SENSOR_PINS[3] = {25, 26, 27};

// Sensors read HIGH when clear (unbroken) and LOW when broken
const bool BROKEN_IS_LOW = true;

// Setup hold time in milliseconds (sensors 2 & 3 both broken)
const unsigned long SETUP_HOLD_MS = 2000;

// === PHYSICAL GEOMETRY & COLLISION PARAMETERS ===
const float DIST_S1_TO_S2_M = 0.20f; // Distance from Sensor 1 to Sensor 2 (m)
const float DIST_S1_TO_S3_M = 0.20f; // Distance from Sensor 1 to Sensor 3 (m)
const float SPACING_S2_S3_M = 0.12f; // Spacing between Sensor 2 and Sensor 3 (m)

const float PUTTER_MASS_KG = 0.350f;   // Effective putter head mass (kg)
const float BALL_MASS_KG = 0.04593f;   // Standard USGA golf ball mass (kg)
const float COR_PUTTER_BALL = 0.80f;   // Coefficient of restitution (0.0 to 1.0)

// Guardrails against sensor bounce/glitches causing extreme speed/roll values
const float MIN_STROKE_MS = 25.0f;     // Minimum plausible S1 -> S2/S3 time (ms)
const float MAX_ROLL_FEET = 120.0f;    // Maximum displayed roll cap (ft)

// Reference ball speed for Stimpmeter roll scaling (~6.0 ft/s = 1.8288 m/s)
const float STIMP_REF_BALL_SPEED_MPS = 1.8288f;
const float FEET_PER_METER = 3.28084f;
const float METERS_PER_FOOT = 0.3048f;

enum TrainerState {
  STATE_SETUP,
  STATE_READY,
  STATE_PUTT_CAPTURED
};

WebServer server(80);

TrainerState state = STATE_SETUP;
bool rawBroken[3] = {false, false, false};

// Stimpmeter setting: 5 (Rough), 8 (Beginner), 10 (Intermediate), 12 (Expert), 14 (Pro)
int stimpRating = 10;

// Target putt distance in feet (6 to 50 ft)
int puttLengthFeet = 18;

// Setup tracking
bool both23BrokenPrev = false;
unsigned long both23StartMs = 0;

// Stroke timing state
bool timingArmed = false;
bool s1Recorded = false;
bool s2Recorded = false;
bool s3Recorded = false;

unsigned long tS1BreakUs = 0;
unsigned long tS2BreakUs = 0;
unsigned long tS3BreakUs = 0;

bool allThreeAlertSent = false;

// Captured putt metrics
bool hasPuttData = false;
float dtS1ToS2Ms = 0.0f;
float dtS1ToS3Ms = 0.0f;
float dtS3MinusS2Ms = 0.0f;

float putterSpeedMps = 0.0f;
float ballSpeedMps = 0.0f;
float ballSpeedFps = 0.0f;
float ballEnergyMj = 0.0f;

// Signed angle in degrees: positive => Left (Sensor 3 before Sensor 2), negative => Right
float signedAngleDeg = 0.0f;
float angleMagDeg = 0.0f;
String angleDir = "SQUARE";

float rollMeters = 0.0f;
float rollFeet = 0.0f;

// Event log ring buffer
const int MAX_LOGS = 80;
String logs[MAX_LOGS];
int logCount = 0;
int logHead = 0;

void pushLog(const String& msg) {
  Serial.println(msg);
  int idx = (logHead + logCount) % MAX_LOGS;
  if (logCount < MAX_LOGS) {
    logs[idx] = msg;
    logCount++;
  } else {
    logs[logHead] = msg;
    logHead = (logHead + 1) % MAX_LOGS;
  }
}

bool readSensorBroken(int index) {
  int val = digitalRead(SENSOR_PINS[index]);
  return BROKEN_IS_LOW ? (val == LOW) : (val == HIGH);
}

float getTransferGain() {
  float denom = PUTTER_MASS_KG + BALL_MASS_KG;
  if (denom <= 0.0f) return 1.0f;
  return (PUTTER_MASS_KG * (1.0f + COR_PUTTER_BALL)) / denom;
}

void recomputeRollFromBallSpeed() {
  if (!hasPuttData || ballSpeedMps <= 0.0f) {
    rollMeters = 0.0f;
    rollFeet = 0.0f;
    return;
  }
  float baseRollMeters = (float)stimpRating * METERS_PER_FOOT;
  float speedRatio = ballSpeedMps / STIMP_REF_BALL_SPEED_MPS;
  float rawFeet = (baseRollMeters * speedRatio * speedRatio) * FEET_PER_METER;
  if (rawFeet > MAX_ROLL_FEET) rawFeet = MAX_ROLL_FEET;
  if (rawFeet < 0.0f) rawFeet = 0.0f;
  rollFeet = rawFeet;
  rollMeters = rollFeet * METERS_PER_FOOT;
}

void prepareStrokeCapture() {
  timingArmed = true;
  s1Recorded = false;
  s2Recorded = false;
  s3Recorded = false;
  tS1BreakUs = 0;
  tS2BreakUs = 0;
  tS3BreakUs = 0;
}

void enterReadyState() {
  state = STATE_READY;
  allThreeAlertSent = false;
  prepareStrokeCapture();

  pushLog("Setup complete - sensors 2 and 3 held broken");

  if (rawBroken[0]) {
    pushLog("Sensor 1 is broken at setup completion; waiting until Sensor 1 is unbroken.");
  } else {
    pushLog("Sensor 1 is unbroken; ready to time Sensor 1 break -> Sensors 2 & 3.");
  }
}

void resetMachine() {
  state = STATE_SETUP;
  both23BrokenPrev = false;
  both23StartMs = 0;

  timingArmed = false;
  s1Recorded = false;
  s2Recorded = false;
  s3Recorded = false;
  tS1BreakUs = 0;
  tS2BreakUs = 0;
  tS3BreakUs = 0;
  allThreeAlertSent = false;

  hasPuttData = false;
  dtS1ToS2Ms = 0.0f;
  dtS1ToS3Ms = 0.0f;
  dtS3MinusS2Ms = 0.0f;
  putterSpeedMps = 0.0f;
  ballSpeedMps = 0.0f;
  ballSpeedFps = 0.0f;
  ballEnergyMj = 0.0f;
  signedAngleDeg = 0.0f;
  angleMagDeg = 0.0f;
  angleDir = "SQUARE";
  rollMeters = 0.0f;
  rollFeet = 0.0f;

  pushLog("Machine reset -> SETUP mode");
}

String stateName(TrainerState s) {
  switch (s) {
    case STATE_SETUP: return "SETUP";
    case STATE_READY: return "READY";
    case STATE_PUTT_CAPTURED: return "PUTT_CAPTURED";
    default: return "UNKNOWN";
  }
}

String escapeJson(const String& input) {
  String out = input;
  out.replace("\\", "\\\\");
  out.replace("\"", "\\\"");
  out.replace("\n", "\\n");
  return out;
}

String buildStatusJson() {
  String json = "{";
  json += "\"state\":\"" + stateName(state) + "\",";
  json += "\"stimp\":" + String(stimpRating) + ",";
  json += "\"puttLengthFt\":" + String(puttLengthFeet) + ",";
  json += "\"hasPutt\":" + String(hasPuttData ? "true" : "false") + ",";
  json += "\"putterSpeedMps\":" + String(putterSpeedMps, 3) + ",";
  json += "\"ballSpeedMps\":" + String(ballSpeedMps, 3) + ",";
  json += "\"ballSpeedFps\":" + String(ballSpeedFps, 3) + ",";
  json += "\"ballEnergyMj\":" + String(ballEnergyMj, 2) + ",";
  json += "\"signedAngleDeg\":" + String(signedAngleDeg, 2) + ",";
  json += "\"angleMagDeg\":" + String(angleMagDeg, 2) + ",";
  json += "\"angleDir\":\"" + angleDir + "\",";
  json += "\"rollMeters\":" + String(rollMeters, 2) + ",";
  json += "\"rollFeet\":" + String(rollFeet, 2) + ",";
  json += "\"logs\":[";
  for (int i = 0; i < logCount; i++) {
    int idx = (logHead + i) % MAX_LOGS;
    if (i > 0) json += ",";
    json += "\"" + escapeJson(logs[idx]) + "\"";
  }
  json += "]}";
  return json;
}

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>PuttTrainer</title>
  <style>
    :root {
      --bg-light-green: #40916c;
      --panel-green: #52b788;
      --turf-bright: #74c69d;
      --turf-deep: #2d6a4f;
      --cream: #f4f9f4;
      --cream-soft: #d8f3dc;
      --gold: #f2c94c;
      --state-orange: #e68a1e;
      --state-green: #2ecc71;
      --state-red: #d93829;
    }
    * { box-sizing: border-box; }
    body {
      font-family: "Courier New", Courier, monospace;
      background: var(--bg-light-green);
      color: var(--cream);
      margin: 0;
      padding: 8px 10px;
    }
    .app-shell {
      max-width: 860px;
      margin: 0 auto;
      display: flex;
      flex-direction: column;
      gap: 8px;
    }
    .top-block {
      background: var(--turf-deep);
      border: 3px solid var(--cream);
      padding: 8px 10px;
      display: flex;
      flex-direction: column;
      gap: 7px;
    }
    .header-box {
      border-bottom: 2px solid var(--cream-soft);
      padding-bottom: 4px;
      text-align: center;
    }
    h1 {
      margin: 0;
      font-size: 1.45rem;
      letter-spacing: 2px;
      color: var(--gold);
    }
    .reset-btn {
      display: block;
      width: 100%;
      background: var(--state-orange);
      color: #102418;
      border: 3px solid var(--cream);
      padding: 9px;
      font-family: inherit;
      font-size: 0.98rem;
      font-weight: bold;
      letter-spacing: 2px;
      text-transform: uppercase;
      cursor: pointer;
    }
    .reset-btn.state-orange {
      background: var(--state-orange);
      color: #102418;
    }
    .reset-btn.state-green {
      background: var(--state-green);
      color: #102418;
    }
    .reset-btn.state-red {
      background: var(--state-red);
      color: var(--cream);
    }
    .stimp-select {
      display: block;
      width: 100%;
      background: var(--panel-green);
      color: #102418;
      border: 3px solid var(--cream);
      padding: 8px 10px;
      font-family: inherit;
      font-size: 0.88rem;
      font-weight: bold;
      cursor: pointer;
    }
    .length-box {
      background: var(--panel-green);
      border: 3px solid var(--cream);
      padding: 7px 10px;
      color: #102418;
      font-weight: bold;
      font-size: 0.86rem;
    }
    .length-top {
      display: flex;
      justify-content: space-between;
      margin-bottom: 4px;
    }
    input[type="range"] {
      width: 100%;
      accent-color: var(--gold);
      cursor: pointer;
      margin: 0;
    }
    .green-stage {
      background: var(--turf-deep);
      border: 3px solid var(--cream);
      padding: 8px;
      display: flex;
      flex-direction: column;
      gap: 6px;
    }
    .hud-bar {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 8px;
      background: var(--panel-green);
      color: #102418;
      border: 2px solid var(--cream);
      padding: 6px 10px;
      font-weight: bold;
      font-size: 0.9rem;
    }
    canvas {
      display: block;
      width: 100%;
      height: 270px;
      background: var(--turf-bright);
      border: 3px solid var(--cream);
      image-rendering: pixelated;
    }
    .log-panel {
      background: var(--turf-deep);
      border: 3px solid var(--cream);
    }
    .log-toggle {
      display: flex;
      justify-content: space-between;
      align-items: center;
      width: 100%;
      background: var(--panel-green);
      color: #102418;
      border: none;
      padding: 8px 10px;
      font-family: inherit;
      font-size: 0.85rem;
      font-weight: bold;
      letter-spacing: 1px;
      text-transform: uppercase;
      cursor: pointer;
    }
    .log-toggle:hover {
      background: var(--turf-bright);
    }
    .log-body {
      display: none;
      padding: 8px;
      border-top: 3px solid var(--cream);
    }
    .log-body.open {
      display: block;
    }
    #log {
      background: #1b4332;
      border: 2px solid var(--cream-soft);
      padding: 8px;
      height: 150px;
      overflow-y: auto;
      white-space: pre-wrap;
      font-size: 0.78rem;
      color: var(--cream);
    }
  </style>
</head>
<body>
  <div class="app-shell">
    <div class="top-block">
      <div class="header-box">
        <h1>PuttTrainer</h1>
      </div>

      <button id="resetBtn" class="reset-btn state-orange" onclick="resetMachine()">Reset Machine</button>

      <select id="stimpSelect" class="stimp-select" aria-label="Stimpmeter Selection" onchange="setStimp(this.value)">
        <option value="5">Rough (Stimp 5)</option>
        <option value="8">Beginner Course (Stimp 8)</option>
        <option value="10" selected>Intermediate Course (Stimp 10)</option>
        <option value="12">Expert Course (Stimp 12)</option>
        <option value="14">Pro Course (Stimp 14)</option>
      </select>

      <div class="length-box">
        <div class="length-top">
          <span>PUTT LENGTH</span>
          <span id="lengthVal">18 FT</span>
        </div>
        <input id="lengthSlider" type="range" min="6" max="50" step="1" value="18" oninput="onLengthInput(this.value)" onchange="sendLength(this.value)">
      </div>
    </div>

    <div class="green-stage">
      <div class="hud-bar">
        <span id="distToHole">DIST TO HOLE: 18.0 FT</span>
        <span id="puttSummary">READY</span>
      </div>
      <canvas id="greenCanvas" width="800" height="270"></canvas>
    </div>

    <div class="log-panel">
      <button id="logToggleBtn" class="log-toggle" type="button" aria-expanded="false" onclick="toggleLog()">
        <span>SYSTEM LOG</span>
        <span id="logToggleIcon">[+]</span>
      </button>
      <div id="logBody" class="log-body">
        <div id="log">Waiting...</div>
      </div>
    </div>
  </div>

  <script>
    let userSelectingStimp = false;
    let userDraggingLength = false;
    let logOpen = false;

    let currentState = {
      state: 'SETUP',
      stimp: 10,
      puttLengthFt: 18,
      hasPutt: false,
      putterSpeedMps: 0,
      ballSpeedMps: 0,
      ballSpeedFps: 0,
      ballEnergyMj: 0,
      signedAngleDeg: 0,
      angleMagDeg: 0,
      angleDir: 'SQUARE',
      rollFeet: 0
    };

    function toggleLog() {
      logOpen = !logOpen;
      const body = document.getElementById('logBody');
      const icon = document.getElementById('logToggleIcon');
      const btn = document.getElementById('logToggleBtn');
      const logBox = document.getElementById('log');

      body.classList.toggle('open', logOpen);
      icon.textContent = logOpen ? '[-]' : '[+]';
      btn.setAttribute('aria-expanded', logOpen ? 'true' : 'false');

      if (logOpen) {
        logBox.scrollTop = logBox.scrollHeight;
      }
    }

    function onLengthInput(val) {
      userDraggingLength = true;
      currentState.puttLengthFt = Number(val);
      document.getElementById('lengthVal').textContent = val + ' FT';
      drawGreen();
    }

    async function sendLength(val) {
      document.getElementById('lengthVal').textContent = val + ' FT';
      await fetch('/length?value=' + encodeURIComponent(val), { method: 'POST' });
      userDraggingLength = false;
      pollState();
    }

    async function setStimp(val) {
      userSelectingStimp = true;
      await fetch('/stimp?value=' + encodeURIComponent(val), { method: 'POST' });
      userSelectingStimp = false;
      pollState();
    }

    async function resetMachine() {
      await fetch('/reset', { method: 'POST' });
      pollState();
    }

    function updateResetButtonColor(stateStr) {
      const btn = document.getElementById('resetBtn');
      btn.classList.remove('state-orange', 'state-green', 'state-red');

      if (stateStr === 'PUTT_CAPTURED') {
        btn.classList.add('state-red');
      } else if (stateStr === 'READY') {
        btn.classList.add('state-green');
      } else {
        btn.classList.add('state-orange');
      }
    }

    function drawGreen() {
      const canvas = document.getElementById('greenCanvas');
      const ctx = canvas.getContext('2d');
      const w = canvas.width;
      const h = canvas.height;

      const stripeCount = 8;
      const stripeW = w / stripeCount;
      for (let i = 0; i < stripeCount; i++) {
        ctx.fillStyle = (i % 2 === 0) ? '#74c69d' : '#65b891';
        ctx.fillRect(Math.floor(i * stripeW), 0, Math.ceil(stripeW), h);
      }

      ctx.strokeStyle = '#f4f9f4';
      ctx.lineWidth = 4;
      ctx.strokeRect(4, 4, w - 8, h - 8);

      const targetFt = Math.max(1, Number(currentState.puttLengthFt) || 18);
      const rollFt = (currentState.hasPutt && currentState.rollFeet > 0) ? Number(currentState.rollFeet) : 0;

      const viewMaxFeet = Math.max(targetFt * 1.15, rollFt * 1.15, 10);
      const topMarginPx = 42;
      const bottomMarginPx = 28;
      const usableHeightPx = h - topMarginPx - bottomMarginPx;
      const pxPerFoot = usableHeightPx / viewMaxFeet;

      const startY = h - bottomMarginPx;
      const holeX = Math.floor(w * 0.5);
      const holeY = startY - targetFt * pxPerFoot;

      ctx.save();
      ctx.setLineDash([8, 8]);
      ctx.strokeStyle = 'rgba(244, 249, 244, 0.6)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(holeX, startY);
      ctx.lineTo(holeX, holeY);
      ctx.stroke();
      ctx.restore();

      ctx.fillStyle = '#1b4332';
      ctx.beginPath();
      ctx.arc(holeX, holeY, 9, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#f4f9f4';
      ctx.lineWidth = 2;
      ctx.stroke();

      ctx.strokeStyle = '#f4f9f4';
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(holeX, holeY);
      ctx.lineTo(holeX, holeY - 30);
      ctx.stroke();

      ctx.fillStyle = '#f2c94c';
      ctx.beginPath();
      ctx.moveTo(holeX + 1, holeY - 30);
      ctx.lineTo(holeX + 22, holeY - 24);
      ctx.lineTo(holeX + 1, holeY - 18);
      ctx.closePath();
      ctx.fill();
      ctx.strokeStyle = '#1b4332';
      ctx.lineWidth = 1.5;
      ctx.stroke();

      let ballX = holeX;
      let ballY = startY;
      let distRemainingFt = targetFt;

      if (currentState.hasPutt && rollFt > 0) {
        const angleRad = (Number(currentState.signedAngleDeg) * Math.PI) / 180;
        const forwardFt = rollFt * Math.cos(angleRad);
        const lateralFt = rollFt * Math.sin(angleRad);

        ballX = holeX - lateralFt * pxPerFoot;
        ballY = startY - forwardFt * pxPerFoot;

        ctx.strokeStyle = '#f2c94c';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(holeX, startY);
        ctx.lineTo(ballX, ballY);
        ctx.stroke();

        const dForwardFt = targetFt - forwardFt;
        distRemainingFt = Math.sqrt(dForwardFt * dForwardFt + lateralFt * lateralFt);
      }

      ctx.fillStyle = '#f4f9f4';
      ctx.beginPath();
      ctx.arc(ballX, ballY, 8, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#1b4332';
      ctx.lineWidth = 2.5;
      ctx.stroke();

      document.getElementById('distToHole').textContent = 'DIST TO HOLE: ' + distRemainingFt.toFixed(1) + ' FT';

      if (currentState.hasPutt) {
        const angTxt = currentState.angleDir === 'SQUARE'
          ? '0.0° SQ'
          : currentState.angleMagDeg.toFixed(1) + '° ' + currentState.angleDir.toUpperCase();
        document.getElementById('puttSummary').textContent =
          'ROLL: ' + rollFt.toFixed(1) + ' FT | ' + angTxt;
      } else {
        document.getElementById('puttSummary').textContent =
          'TARGET: ' + targetFt.toFixed(0) + ' FT';
      }
    }

    async function pollState() {
      try {
        const res = await fetch('/status');
        const d = await res.json();

        currentState = d;
        updateResetButtonColor(d.state);

        const sel = document.getElementById('stimpSelect');
        if (!userSelectingStimp && sel.value !== String(d.stimp)) {
          sel.value = String(d.stimp);
        }

        const slider = document.getElementById('lengthSlider');
        if (!userDraggingLength) {
          slider.value = String(d.puttLengthFt);
          document.getElementById('lengthVal').textContent = d.puttLengthFt + ' FT';
        }

        drawGreen();

        const logBox = document.getElementById('log');
        logBox.textContent = d.logs.join('\n');
        if (logOpen) {
          logBox.scrollTop = logBox.scrollHeight;
        }
      } catch (e) {}
    }

    drawGreen();
    setInterval(pollState, 150);
    pollState();
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  server.send(200, "application/json", buildStatusJson());
}

void handleReset() {
  resetMachine();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleStimp() {
  if (server.hasArg("value")) {
    int val = server.arg("value").toInt();
    const int presets[5] = {5, 8, 10, 12, 14};
    int best = presets[0];
    int bestDiff = abs(val - presets[0]);
    for (int i = 1; i < 5; i++) {
      int diff = abs(val - presets[i]);
      if (diff < bestDiff) {
        bestDiff = diff;
        best = presets[i];
      }
    }
    stimpRating = best;
    recomputeRollFromBallSpeed();
    pushLog("Stimpmeter set to " + String(stimpRating));
  }
  server.send(200, "application/json", "{\"stimp\":" + String(stimpRating) + "}");
}

void handleLength() {
  if (server.hasArg("value")) {
    int val = server.arg("value").toInt();
    if (val < 6) val = 6;
    if (val > 50) val = 50;
    puttLengthFeet = val;
    pushLog("Putt length set to " + String(puttLengthFeet) + " ft");
  }
  server.send(200, "application/json", "{\"puttLengthFt\":" + String(puttLengthFeet) + "}");
}

void finalizePutt() {
  if (!s1Recorded || (!s2Recorded && !s3Recorded)) return;

  dtS1ToS2Ms = s2Recorded ? ((tS2BreakUs - tS1BreakUs) / 1000.0f) : 0.0f;
  dtS1ToS3Ms = s3Recorded ? ((tS3BreakUs - tS1BreakUs) / 1000.0f) : 0.0f;

  if (s2Recorded && s3Recorded) {
    int64_t diffS3MinusS2Us = (int64_t)tS3BreakUs - (int64_t)tS2BreakUs;
    dtS3MinusS2Ms = (float)(diffS3MinusS2Us / 1000.0);
  } else {
    dtS3MinusS2Ms = 0.0f;
  }

  float v12 = (s2Recorded && dtS1ToS2Ms >= MIN_STROKE_MS && DIST_S1_TO_S2_M > 0.0f)
                ? (DIST_S1_TO_S2_M / (dtS1ToS2Ms / 1000.0f))
                : 0.0f;
  float v13 = (s3Recorded && dtS1ToS3Ms >= MIN_STROKE_MS && DIST_S1_TO_S3_M > 0.0f)
                ? (DIST_S1_TO_S3_M / (dtS1ToS3Ms / 1000.0f))
                : 0.0f;

  if (v12 > 0.0f && v13 > 0.0f) {
    putterSpeedMps = 0.5f * (v12 + v13);
  } else if (v12 > 0.0f) {
    putterSpeedMps = v12;
  } else if (v13 > 0.0f) {
    putterSpeedMps = v13;
  } else {
    putterSpeedMps = 0.0f;
  }

  float transferGain = getTransferGain();
  ballSpeedMps = putterSpeedMps * transferGain;
  ballSpeedFps = ballSpeedMps * FEET_PER_METER;
  ballEnergyMj = 0.5f * BALL_MASS_KG * ballSpeedMps * ballSpeedMps * 1000.0f;

  if (s2Recorded && s3Recorded && putterSpeedMps > 0.0f && SPACING_S2_S3_M > 0.0f) {
    float lateralOffsetM = putterSpeedMps * (-dtS3MinusS2Ms / 1000.0f);
    signedAngleDeg = atan2f(lateralOffsetM, SPACING_S2_S3_M) * (180.0f / (float)M_PI);
    angleMagDeg = fabsf(signedAngleDeg);

    if (angleMagDeg < 0.25f) {
      angleDir = "SQUARE";
    } else if (signedAngleDeg > 0.0f) {
      angleDir = "Left";
    } else {
      angleDir = "Right";
    }
  } else {
    signedAngleDeg = 0.0f;
    angleMagDeg = 0.0f;
    angleDir = "SQUARE";
  }

  hasPuttData = true;
  recomputeRollFromBallSpeed();

  state = STATE_PUTT_CAPTURED;

  String summary = "Putt captured | Putter: " + String(putterSpeedMps, 2) + " m/s | Ball: " +
                   String(ballSpeedFps, 2) + " ft/s (" + String(ballEnergyMj, 1) +
                   " mJ) | Angle: " + String(angleMagDeg, 1) + " deg " + angleDir +
                   " | Roll: " + String(rollFeet, 1) + " ft (Stimp " + String(stimpRating) + ")";
  pushLog(summary);
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 3; i++) {
    pinMode(SENSOR_PINS[i], BROKEN_IS_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
    rawBroken[i] = readSensorBroken(i);
  }

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/reset", HTTP_POST, handleReset);
  server.on("/stimp", HTTP_POST, handleStimp);
  server.on("/length", HTTP_POST, handleLength);
  server.begin();

  resetMachine();
}

void loop() {
  server.handleClient();

  unsigned long nowMs = millis();
  unsigned long nowUs = micros();

  bool prevBroken[3] = {rawBroken[0], rawBroken[1], rawBroken[2]};

  for (int i = 0; i < 3; i++) {
    bool current = readSensorBroken(i);
    if (current != rawBroken[i]) {
      rawBroken[i] = current;
      pushLog("signal " + String(i + 1) + " " + (rawBroken[i] ? "broken/low" : "clear/high"));
    }
  }

  if (state == STATE_SETUP) {
    bool both23 = rawBroken[1] && rawBroken[2];
    if (both23) {
      if (!both23BrokenPrev) {
        both23StartMs = nowMs;
        both23BrokenPrev = true;
      } else if ((nowMs - both23StartMs) >= SETUP_HOLD_MS) {
        enterReadyState();
      }
    } else {
      both23BrokenPrev = false;
      both23StartMs = 0;
    }
  } else if (state == STATE_READY) {
    if (rawBroken[0] && rawBroken[1] && rawBroken[2]) {
      if (!allThreeAlertSent) {
        allThreeAlertSent = true;
        pushLog("All three sensors broken during backswing.");
      }
    } else {
      allThreeAlertSent = false;
    }

    if (!timingArmed && !rawBroken[0]) {
      prepareStrokeCapture();
    }

    if (timingArmed && !s1Recorded) {
      bool s1JustBecameBroken = !prevBroken[0] && rawBroken[0];
      if (s1JustBecameBroken && !rawBroken[1] && !rawBroken[2]) {
        s1Recorded = true;
        tS1BreakUs = nowUs;
      }
    } else if (s1Recorded) {
      bool s2JustBecameBroken = !prevBroken[1] && rawBroken[1];
      bool s3JustBecameBroken = !prevBroken[2] && rawBroken[2];

      if (s2JustBecameBroken && !s2Recorded) {
        s2Recorded = true;
        tS2BreakUs = nowUs;
      }
      if (s3JustBecameBroken && !s3Recorded) {
        s3Recorded = true;
        tS3BreakUs = nowUs;
      }

      if (s2Recorded && s3Recorded) {
        finalizePutt();
      } else if ((s2Recorded || s3Recorded) && ((nowUs - tS1BreakUs) > 2000000UL)) {
        finalizePutt();
      }
    }
  }
}