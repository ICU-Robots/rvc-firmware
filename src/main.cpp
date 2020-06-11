#include <Arduino.h>
#include <Servo.h>
#include "Positioner.h"

#define PERIOD 200 // ms

static const int EFFECTOR_PIN = 5;
static const int LED_PIN = 4;

Positioner positioner(A3, A4, A5, A0, A1, A2, 3, 2);
Servo end_effector;
unsigned long lt;

void setup() {
    positioner.start();
    end_effector.attach(EFFECTOR_PIN);
    end_effector.write(10);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    lt = millis();

    Serial.begin(19200);
    while (!Serial);
}

void loop() {
    if (Serial.available()) {
        int t = Serial.read();
        float x, y;

        switch (t) {
            case 'E': // Enable
                positioner.enable();
                break;
            case 'D': // Disable
                positioner.disable();
                break;
            case 'H': // Home
                if (positioner.home())
                    Serial.println("T");
                else
                    Serial.println("F");
                break;
            case 'M': // Move (absolute)
                delay(10);
                x = Serial.parseFloat();
                y = Serial.parseFloat();
                positioner.moveTo(x, y);
                break;
            case 'm': // Move (relative)
                delay(10);
                x = Serial.parseFloat();
                y = Serial.parseFloat();
                positioner.move(x, y);
                break;
            case 'S': // Stop
                positioner.stop();
                break;
            case 'T': // Tap
                for (int i = 10; i < 120; i++) {
                    end_effector.write(i);
                    delay(4);
                }
                delay(50);
                for (int i = 120; i >= 10; i--) {
                    end_effector.write(i);
                    delay(4);
                }
                break;
            case 'P': // Press
                for (int i = 10; i < 120; i++) {
                    end_effector.write(i);
                    delay(4);
                }
                break;
            case 'R': // Release
                for (int i = 120; i >= 10; i--) {
                    end_effector.write(i);
                    delay(4);
                }
                break;
            case 'L': // LED On
                digitalWrite(LED_PIN, HIGH);
                break;
            case 'O': // LED Off
                digitalWrite(LED_PIN, LOW);
                break;
            case 'V':
                x = Serial.parseFloat();
                positioner.set_vel(x);
            default:
                break;
        }
    }

    if (!positioner.run() && !positioner.reported) {
        positioner.report(true);
        lt = millis();
    }

    if (millis() - lt >= PERIOD) {
        positioner.report(false);
        lt = millis();
    }


}