#include <WiFi.h>
#include <WebServer.h>

const char* AP_SSID = "IR-Sensors";
const char* AP_PASS = "12345678"; // At least 8 characters

// Safe general-purpose GPIOs on standard ESP32 boards
const int SIGNAL_PINS[] = {25, 26, 27};
const int NUM_SIGNALS = 3;
const int MAX_LOGS = 80;

WebServer server(80);

int lastState[NUM_SIGNALS];
String logs[MAX_LOGS];
int logCount = 0;
int logHead = 0;

void addLog(const String& msg) {
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

String buildLogsJson() {
  String json = "[";
  for (int i = 0; i < logCount; i++) {
    int idx = (logHead + i) % MAX_LOGS;
    String escaped = logs[idx];
    escaped.replace("\\", "\\\\");
    escaped.replace("\"", "\\\"");
    if (i > 0) json += ",";
    json += "\"" + escaped + "\"";
  }
  json += "]";
  return json;
}

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32 IR Signals</title>
  <style>
    body { font-family: monospace; background: #111; color: #eee; padding: 20px; }
    h2 { margin-top: 0; }
    #log { background: #1c1c1c; border: 1px solid #333; padding: 12px; height: 70vh; overflow-y: auto; white-space: pre-wrap; }
  </style>
</head>
<body>
  <h2>ESP32 Signal Monitor</h2>
  <div id="log">Waiting for events...</div>
  <script>
    async function pollLogs() {
      try {
        const res = await fetch('/logs');
        const lines = await res.json();
        const box = document.getElementById('log');
        box.textContent = lines.length ? lines.join('\n') : 'No signal changes yet.';
        box.scrollTop = box.scrollHeight;
      } catch (e) {}
    }
    setInterval(pollLogs, 250);
    pollLogs();
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

void handleLogs() {
  server.send(200, "application/json", buildLogsJson());
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_SIGNALS; i++) {
    pinMode(SIGNAL_PINS[i], INPUT_PULLUP);
    lastState[i] = digitalRead(SIGNAL_PINS[i]);
  }

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/logs", handleLogs);
  server.begin();

  addLog("Ready (Pins 25, 26, 27) - AP: " + String(AP_SSID) + " | URL: http://" + WiFi.softAPIP().toString());
}

void loop() {
  server.handleClient();

  for (int i = 0; i < NUM_SIGNALS; i++) {
    int state = digitalRead(SIGNAL_PINS[i]);
    if (state != lastState[i]) {
      String msg = "signal " + String(i + 1) + " " + (state == HIGH ? "high" : "low");
      addLog(msg);
      lastState[i] = state;
    }
  }
}