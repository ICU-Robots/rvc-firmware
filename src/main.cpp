#include <Arduino.h>
#include <Servo.h>
#include "Positioner.h"

static const int EFFECTOR_PIN = 5;
static const int MICRO_STEPPING = 16;
static const int LED_PIN = 4;

Positioner positioner(A3, A4, A5, A0, A1, A2, MICRO_STEPPING, 2, 3);
Servo end_effector;

void setup() {
    positioner.start();
    end_effector.attach(EFFECTOR_PIN);
    end_effector.write(10);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.begin(19200);
}

void loop() {
    if (Serial.available()) {
        int t = Serial.read();
        double x, y;

        switch (t) {
            case 'E':
                positioner.enable();
                break;
            case 'D':
                positioner.disable();
                break;
            case 'H':
                positioner.home();
                break;
            case 'M':
                x = Serial.parseFloat();
                y = Serial.parseFloat();
                positioner.moveTo(x, y);
                break;
            case 'T':
                for (int i = 10; i < 120; i++) {
                    end_effector.write(i);
                    delay(10);
                }
                delay(1500);
                for (int i = 120; i >= 10; i--) {
                    end_effector.write(i);
                    delay(10);
                }
            case 'L':
                digitalWrite(LED_PIN, HIGH);
                break;
            case 'O':
                digitalWrite(LED_PIN, LOW);
                break;
            default:
                break;
        }
    }
    delay(10);
}