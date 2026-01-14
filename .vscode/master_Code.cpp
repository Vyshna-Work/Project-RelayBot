#include <WiFiS3.h>
#include <ArduinoHttpClient.h>

String incoming;

//-------------Mission Sequencing-----------
String currentBot = "Unknown";
bool waitingForDone = false;

// ---- WiFi settings ----
char ssid[] = "Slman";
char pass[] = "00000000";

// ---- Flask server settings ----
char serverAddress[] = "172.20.10.5";
int port = 5000;

WiFiClient wifi;
HttpClient client = HttpClient(wifi, serverAddress, port);

// --------------------------------------------------
// STRONG INPUT SANITIZER — removes all HC‑12 noise
// --------------------------------------------------
String cleanInput(String s) {
  String out = "";
  for (int i = 0; i < s.length(); i++) {
    char c = s[i];

    // Accept only valid characters
    if (isAlphaNumeric(c) || c == ',' || c == '-' || c == '_') {
      out += c;
    }
  }
  return out;
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {

  // ---- Manual START from Serial Monitor ----
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() > 0) {
      sendStartCommand(cmd);
      waitingForDone = true;
      currentBot = cmd;
    }
  }

  // ---- HC‑12 incoming telemetry ----
  if (Serial1.available()) {

    // Read raw line
    incoming = Serial1.readStringUntil('\n');

    // Clean EVERYTHING
    incoming = cleanInput(incoming);

    // -----------------------------
    //  Skip corrupted lines
    // -----------------------------
    int c1 = incoming.indexOf(',');
    int c2 = incoming.indexOf(',', c1 + 1);
    int c3 = incoming.indexOf(',', c2 + 1);

    if (c1 < 0 || c2 < 0 || c3 < 0) {
      return;
    }
    // -----------------------------

    // Extract fields
    String botID_raw   = incoming.substring(0, c1);
    String leftStr_raw = incoming.substring(c1 + 1, c2);
    String rightStr_raw = incoming.substring(c2 + 1, c3);
    String state_raw   = incoming.substring(c3 + 1);

    // Clean fields again (safety)
    botID_raw = cleanInput(botID_raw);
    leftStr_raw = cleanInput(leftStr_raw);
    rightStr_raw = cleanInput(rightStr_raw);
    state_raw = cleanInput(state_raw);

    // Convert speeds
    int leftSpeed  = leftStr_raw.toInt();
    int rightSpeed = rightStr_raw.toInt();

    String botID = botID_raw;
    String state = state_raw;

    // ------------ DONE detection ------------
    if (waitingForDone && state == "DONE" && botID == currentBot) {
      Serial.print("Bot finished: ");
      Serial.println(botID);

      waitingForDone = false;
      Serial.println("Waiting for next manual START...");
    }

    // Print locally
    Serial.print("Bot: "); Serial.print(botID);
    Serial.print(" | L: "); Serial.print(leftSpeed);
    Serial.print(" | R: "); Serial.print(rightSpeed);
    Serial.print(" | State: "); Serial.println(state);

    // ---- Build JSON ----
    String json = "{";
    json += "\"bot\":\"" + botID + "\",";
    json += "\"left\":" + String(leftSpeed) + ",";
    json += "\"right\":" + String(rightSpeed) + ",";
    json += "\"state\":\"" + state + "\"";
    json += "}";

    // ---- Send to Flask ----
    client.beginRequest();
    client.post("/relaybot-data");
    client.sendHeader("Content-Type", "application/json");
    client.sendHeader("Content-Length", json.length());
    client.beginBody();
    client.print(json);
    client.endRequest();

    int status = client.responseStatusCode();
    String response = client.responseBody();

    Serial.print("Flask status: ");
    Serial.println(status);
    Serial.print("Response: ");
    Serial.println(response);
  }
}

void sendStartCommand(String botName) {
  Serial1.print(botName);
  Serial1.println(",START");

  Serial.print("Sent START to: ");
  Serial.println(botName);
}
