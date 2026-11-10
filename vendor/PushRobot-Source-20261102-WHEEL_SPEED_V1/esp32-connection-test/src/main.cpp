#include <Arduino.h>

unsigned long lastHeartbeat = 0;
unsigned long heartbeatCount = 0;

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(50);
    delay(1000);
    Serial.println();
    Serial.println("ESP32 serial test started.");
    Serial.println("Send PING with a newline; expect PONG.");
}

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command == "PING") {
            Serial.println("PONG");
        }
    }

    const unsigned long now = millis();
    if (now - lastHeartbeat >= 1000) {
        lastHeartbeat = now;
        Serial.printf("ESP32 OK | count=%lu | uptime_ms=%lu\r\n",
                      ++heartbeatCount, now);
    }
}
