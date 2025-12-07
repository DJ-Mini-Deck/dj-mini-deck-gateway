/******************************************************
 * ESP8266 NodeMCU - UART <-> NETPIE MQTT Gateway
 * Language: C++ (Arduino)
 * Library: PubSubClient
 *
 * Features:
 * - Read text lines from STM32 via UART (newline-terminated)
 * - Parse JSON or simple "CMD:..." text
 * - Publish to NETPIE MQTT
 * - Subscribe to cloud topic and forward messages to STM32
 * - Reconnect logic + simple retry queue for failed publishes
 *
 * Notes:
 * - Uses Hardware Serial (Serial) for UART with STM32 in this sketch.
 *   On NodeMCU Serial is shared with USB debug; for production prefer ESP32
 *   or hardware with extra UART.
 ******************************************************/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// ----------------- CONFIG -----------------
const char* WIFI_SSID = "Flook";
const char* WIFI_PASS = "Flook3333";

const char* MQTT_HOST = "mqtt.netpie.io";
const uint16_t MQTT_PORT = 1883;

// NETPIE credentials (Token only, no secret)
const char* NETPIE_CLIENT_ID = "2b6d6f68-9379-4d77-a48b-43a837bd3ffa"; // Device client id
const char* NETPIE_TOKEN     = "bPFBcnDbEFFiAc3QDXwnY2pM9rTXuxFK";     // Device token (username)

// Topics
// Publish from STM32 -> Cloud: PUBLISH_TOPIC
// Subscribe from Cloud -> Gateway -> STM32: SUBSCRIBE_TOPIC
const char* PUBLISH_TOPIC   = "@msg/commands";   // change as needed
const char* SUBSCRIBE_TOPIC = "@msg/commands";

// MQTT keepalive
const uint16_t MY_KEEPALIVE = 60;
// ------------------------------------------

// Networking
WiFiClient espClient;
PubSubClient mqtt(espClient);

// Simple publish retry queue (circular)
#define QUEUE_CAPACITY 12       // adjust small for RAM limitations
String pubQueue[QUEUE_CAPACITY];
uint8_t qHead = 0;
uint8_t qTail = 0;
uint8_t qCount = 0;

// UART buffer
String uartBuf = "";
const size_t MAX_UART_LINE = 512; // safety limit

// reconnect
unsigned long lastMqttReconnect = 0;
const unsigned long RECONNECT_INTERVAL_MS = 6000;

// Utility: enqueue
bool enqueuePublish(const String &s) {
  if (qCount >= QUEUE_CAPACITY) {
    // queue full - drop oldest (or drop new). Here we drop oldest to keep recent
    qHead = (qHead + 1) % QUEUE_CAPACITY;
    qCount--;
  }
  pubQueue[qTail] = s;
  qTail = (qTail + 1) % QUEUE_CAPACITY;
  qCount++;
  return true;
}

// Utility: peek/dequeue
bool dequeuePublish(String &out) {
  if (qCount == 0) return false;
  out = pubQueue[qHead];
  qHead = (qHead + 1) % QUEUE_CAPACITY;
  qCount--;
  return true;
}

// ----- Wi-Fi -----
void setupWifi() {
  Serial.print("WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
    if (millis() - start > 15000) {
      Serial.println("\nWiFi connect timeout.");
      return;
    }
  }
  Serial.println();
  Serial.print("WiFi connected, IP: ");
  Serial.println(WiFi.localIP());
}

// ----- MQTT connect -----
bool mqttConnect() {
  Serial.print("Connecting to NETPIE MQTT... ");
  bool ok = mqtt.connect(NETPIE_CLIENT_ID, NETPIE_TOKEN, "");
  if (ok) {
    Serial.println("connected");
    // Subscribe to inbound topic
    if (!mqtt.subscribe(SUBSCRIBE_TOPIC)) {
      Serial.println("Subscribe failed!");
    } else {
      Serial.print("Subscribed to: ");
      Serial.println(SUBSCRIBE_TOPIC);
    }
  } else {
    Serial.print("failed (rc=");
    Serial.print(mqtt.state());
    Serial.println(")");
  }
  return ok;
}

// ----- MQTT callback: forward to STM32 -----
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("\n<MQTT_RX> ");
  Serial.println(topic);

  // Build message string
  String msg = "";
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  Serial.print("Forward to STM32: ");
  Serial.println(msg);

  // Append newline for STM32 to read line-by-line
  Serial.print(msg);
  Serial.print("\n");
  // Note: We use Serial for both debug & UART here. If connecting STM32 directly
  // to Serial pins, be careful with USB serial overlap.
}

// ----- Handle single line from UART (STM32) -----
void handleUartLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  Serial.print("[UART_RX] ");
  Serial.println(line);

  // Decide payload:
  // If starts with '{' treat as JSON payload; otherwise, try to parse simple CMD: format
  String payload = line;
  if (line.charAt(0) != '{') {
    // Try simple format examples:
    // "CMD:LED ON" => {"cmd":"LED","arg":"ON"}
    // "SENSOR:TMP 25.3" => {"type":"SENSOR","name":"TMP","value":"25.3"}
    int colon = line.indexOf(':');
    if (colon > 0) {
      String left = line.substring(0, colon);
      String right = line.substring(colon + 1);
      left.trim();
      right.trim();
      // further split right by space
      int sp = right.indexOf(' ');
      if (sp > 0) {
        String a = right.substring(0, sp);
        String b = right.substring(sp + 1);
        a.trim(); b.trim();
        payload = String("{\"type\":\"") + left + String("\",\"name\":\"") + a + String("\",\"value\":\"") + b + String("\"}");
      } else {
        payload = String("{\"type\":\"") + left + String("\",\"value\":\"") + right + String("\"}");
      }
    } else {
      // fallback: send plain text as message field
      payload = String("{\"text\":\"") + line + String("\"}");
    }
  }

  // Try immediate publish if connected
  if (mqtt.connected()) {
    bool ok = mqtt.publish(PUBLISH_TOPIC, payload.c_str());
    if (!ok) {
      Serial.println("Publish failed -> enqueued");
      enqueuePublish(payload);
    } else {
      Serial.println("Published -> " + payload);
    }
  } else {
    Serial.println("MQTT not connected -> enqueued");
    enqueuePublish(payload);
  }
}

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println();
  Serial.println("=== ESP8266 UART <-> NETPIE Gateway ===");

  setupWifi();

  mqtt.setClient(espClient);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  lastMqttReconnect = 0;
}

void loop() {
  // maintain WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost, trying reconnect...");
    setupWifi();
  }

  // maintain MQTT
  if (!mqtt.connected()) {
    if (millis() - lastMqttReconnect > RECONNECT_INTERVAL_MS) {
      lastMqttReconnect = millis();
      mqttConnect();
    }
  } else {
    mqtt.loop();
    // try to flush queued publishes (one per loop to avoid blocking)
    if (qCount > 0) {
      String next;
      if (dequeuePublish(next)) {
        Serial.println("Retry publish from queue: " + next);
        bool ok = mqtt.publish(PUBLISH_TOPIC, next.c_str());
        if (!ok) {
          Serial.println("Retry failed -> re-enqueue");
          enqueuePublish(next);
          // small delay to avoid busy loop
          delay(50);
        } else {
          Serial.println("Retry succeeded");
        }
      }
    }
  }

  // Read UART from STM32 (line buffered)
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String line = uartBuf;
      uartBuf = "";
      handleUartLine(line);
    } else {
      uartBuf += c;
      if (uartBuf.length() > MAX_UART_LINE) {
        Serial.println("UART line too long, flushed");
        uartBuf = "";
      }
    }
  }

  delay(5);
}
