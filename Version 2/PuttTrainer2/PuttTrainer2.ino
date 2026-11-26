#include <WiFi.h>
#include <WebServer.h>

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

// Optional physical distances in meters (set > 0 to compute speed in m/s)
const float DIST_S1_TO_S2_M = 0.0f;
const float DIST_S1_TO_S3_M = 0.0f;

enum TrainerState {
  STATE_SETUP,
  STATE_READY,
  STATE_PUTT_CAPTURED
};

WebServer server(80);

TrainerState state = STATE_SETUP;
bool rawBroken[3] = {false, false, false};

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
float dtS2And3AbsMs = 0.0f;
float speedS1ToS2Mps = 0.0f;
float speedS1ToS3Mps = 0.0f;

// Event log ring buffer
const int MAX_LOGS = 80;
String logs[MAX_LOGS];
int logCount = 0;
int logHead = 0;

String statusMessage = "Place putter across sensors 2 and 3 for 2s to complete setup.";

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

  statusMessage = "Setup complete. Stroke putter to capture putt.";
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
  dtS2And3AbsMs = 0.0f;
  speedS1ToS2Mps = 0.0f;
  speedS1ToS3Mps = 0.0f;

  statusMessage = "Machine reset. Hold sensors 2 and 3 broken for 2s to complete setup.";
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
  json += "\"message\":\"" + escapeJson(statusMessage) + "\",";
  json += "\"sensors\":[" + String(rawBroken[0] ? "true" : "false") + "," +
          String(rawBroken[1] ? "true" : "false") + "," +
          String(rawBroken[2] ? "true" : "false") + "],";
  json += "\"hasPutt\":" + String(hasPuttData ? "true" : "false") + ",";
  json += "\"dtS1ToS2Ms\":" + String(dtS1ToS2Ms, 3) + ",";
  json += "\"dtS1ToS3Ms\":" + String(dtS1ToS3Ms, 3) + ",";
  json += "\"dtS2And3AbsMs\":" + String(dtS2And3AbsMs, 3) + ",";
  json += "\"speedS1ToS2Mps\":" + String(speedS1ToS2Mps, 3) + ",";
  json += "\"speedS1ToS3Mps\":" + String(speedS1ToS3Mps, 3) + ",";
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
  <title>Putting Trainer</title>
  <style>
    body { font-family: system-ui, sans-serif; background: #121212; color: #f0f0f0; margin: 0; padding: 20px; max-width: 720px; }
    .card { background: #1e1e1e; border: 1px solid #333; border-radius: 8px; padding: 16px; margin-bottom: 16px; }
    .banner { font-size: 1.1rem; font-weight: 600; padding: 12px; border-radius: 6px; background: #262b33; margin-bottom: 12px; }
    .grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; margin-bottom: 12px; }
    .metric { background: #161616; border: 1px solid #2c2c2c; border-radius: 6px; padding: 10px; }
    .metric-label { font-size: 0.8rem; color: #aaa; }
    .metric-val { font-size: 1.15rem; font-weight: 700; margin-top: 4px; }
    button { background: #2d7ff9; color: #fff; border: none; border-radius: 6px; padding: 10px 16px; font-size: 1rem; font-weight: 600; cursor: pointer; }
    button:hover { background: #1c68dc; }
    #log { font-family: monospace; background: #161616; border: 1px solid #2c2c2c; border-radius: 6px; padding: 10px; height: 220px; overflow-y: auto; white-space: pre-wrap; font-size: 0.85rem; }
  </style>
</head>
<body>
  <h2>Right-Handed Putting Trainer</h2>

  <div class="card">
    <div id="banner" class="banner">Connecting...</div>
    <div>State: <strong id="state">-</strong> | Sensors [1, 2, 3] broken: <strong id="sensors">-</strong></div>
    <div style="margin-top: 12px;">
      <button id="resetBtn" onclick="resetMachine()">Reset Machine</button>
    </div>
  </div>

  <div class="card">
    <h3>Captured Putt Timing</h3>
    <div class="grid">
      <div class="metric">
        <div class="metric-label">Sensor 1 -> Sensor 2</div>
        <div class="metric-val" id="dt12">-</div>
      </div>
      <div class="metric">
        <div class="metric-label">Sensor 1 -> Sensor 3</div>
        <div class="metric-val" id="dt13">-</div>
      </div>
      <div class="metric">
        <div class="metric-label">|Sensor 2 - Sensor 3|</div>
        <div class="metric-val" id="dt23">-</div>
      </div>
    </div>
    <div id="speeds"></div>
  </div>

  <div class="card">
    <h3>Messages / Log</h3>
    <div id="log">Waiting...</div>
  </div>

  <script>
    async function resetMachine() {
      await fetch('/reset', { method: 'POST' });
      pollState();
    }

    async function pollState() {
      try {
        const res = await fetch('/status');
        const d = await res.json();

        document.getElementById('banner').textContent = d.message;
        document.getElementById('state').textContent = d.state;
        document.getElementById('sensors').textContent = JSON.stringify(d.sensors);

        if (d.hasPutt) {
          document.getElementById('dt12').textContent = d.dtS1ToS2Ms.toFixed(2) + ' ms';
          document.getElementById('dt13').textContent = d.dtS1ToS3Ms.toFixed(2) + ' ms';
          document.getElementById('dt23').textContent = d.dtS2And3AbsMs.toFixed(2) + ' ms';

          let sp = [];
          if (d.speedS1ToS2Mps > 0) sp.push('Speed (S1->S2): ' + d.speedS1ToS2Mps.toFixed(2) + ' m/s');
          if (d.speedS1ToS3Mps > 0) sp.push('Speed (S1->S3): ' + d.speedS1ToS3Mps.toFixed(2) + ' m/s');
          document.getElementById('speeds').textContent = sp.join(' | ');
        } else {
          document.getElementById('dt12').textContent = '-';
          document.getElementById('dt13').textContent = '-';
          document.getElementById('dt23').textContent = '-';
          document.getElementById('speeds').textContent = '';
        }

        const logBox = document.getElementById('log');
        logBox.textContent = d.logs.join('\n');
        logBox.scrollTop = logBox.scrollHeight;
      } catch (e) {}
    }

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

void finalizePutt() {
  if (!s1Recorded || (!s2Recorded && !s3Recorded)) return;

  dtS1ToS2Ms = s2Recorded ? ((tS2BreakUs - tS1BreakUs) / 1000.0f) : 0.0f;
  dtS1ToS3Ms = s3Recorded ? ((tS3BreakUs - tS1BreakUs) / 1000.0f) : 0.0f;

  if (s2Recorded && s3Recorded) {
    long diffUs = (long)tS3BreakUs - (long)tS2BreakUs;
    dtS2And3AbsMs = labs(diffUs) / 1000.0f;
  } else {
    dtS2And3AbsMs = 0.0f;
  }

  if (DIST_S1_TO_S2_M > 0.0f && dtS1ToS2Ms > 0.0f) {
    speedS1ToS2Mps = DIST_S1_TO_S2_M / (dtS1ToS2Ms / 1000.0f);
  }
  if (DIST_S1_TO_S3_M > 0.0f && dtS1ToS3Ms > 0.0f) {
    speedS1ToS3Mps = DIST_S1_TO_S3_M / (dtS1ToS3Ms / 1000.0f);
  }

  hasPuttData = true;
  state = STATE_PUTT_CAPTURED;
  statusMessage = "Putt captured. Click Reset Machine to run another putt.";

  String summary = "Putt captured | S1->S2: " + String(dtS1ToS2Ms, 2) +
                   " ms | S1->S3: " + String(dtS1ToS3Ms, 2) +
                   " ms | |S2-S3|: " + String(dtS2And3AbsMs, 2) + " ms";
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
    // Alert when all three sensors are broken together on the backswing
    if (rawBroken[0] && rawBroken[1] && rawBroken[2]) {
      if (!allThreeAlertSent) {
        allThreeAlertSent = true;
        statusMessage = "All three sensors broken during backswing.";
        pushLog("All three sensors broken during backswing.");
      }
    } else {
      allThreeAlertSent = false;
    }

    // Ensure timing is armed once Sensor 1 is unbroken (either already unbroken or just became unbroken)
    if (!timingArmed && !rawBroken[0]) {
      prepareStrokeCapture();
    }

    // Wait until Sensor 1 is unbroken and Sensors 2 & 3 are unbroken before starting a stroke
    if (timingArmed && !s1Recorded) {
      bool s1JustBecameBroken = !prevBroken[0] && rawBroken[0];
      if (s1JustBecameBroken && !rawBroken[1] && !rawBroken[2]) {
        s1Recorded = true;
        tS1BreakUs = nowUs;
      }
    } else if (s1Recorded) {
      // Sensor 1 has broken; record Sensor 2 and Sensor 3 as the putter comes through
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