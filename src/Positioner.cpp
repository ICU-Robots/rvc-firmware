//
// Created by Misha on 4/9/2020.
//

#include <Arduino.h>
#include "Positioner.h"


Positioner::Positioner(int x_enable, int x_step, int x_dir, int y_enable, int y_step, int y_dir, int microstepping,
                       int x_limit_pin, int y_limit_pin, int steps_per_rev, int mm_per_rev) {
    this->x_enable = x_enable;
    this->y_enable = y_enable;
    this->x_limit_pin = x_limit_pin;
    this->y_limit_pin = y_limit_pin;

    micro_stepping = microstepping;
    steps_per_mm = steps_per_rev / mm_per_rev;

    x_stepper = AccelStepper(AccelStepper::DRIVER, x_step, x_dir);
    y_stepper = AccelStepper(AccelStepper::DRIVER, y_step, y_dir);
    steppers = MultiStepper();
}

void Positioner::start() {
    pinMode(x_enable, OUTPUT);
    pinMode(y_enable, OUTPUT);
    digitalWrite(x_enable, HIGH);
    digitalWrite(y_enable, HIGH);

    pinMode(x_limit_pin, INPUT_PULLUP);
    pinMode(y_limit_pin, INPUT_PULLUP);

    steppers.addStepper(x_stepper);
    steppers.addStepper(y_stepper);

    x_stepper.setMaxSpeed(1000.0f * (float) micro_stepping);
    y_stepper.setMaxSpeed(1000.0f * (float) micro_stepping);
}

void Positioner::enable() {
    digitalWrite(x_enable, LOW);
    digitalWrite(y_enable, LOW);
}

void Positioner::disable() {
    digitalWrite(x_enable, HIGH);
    digitalWrite(y_enable, HIGH);
}

void Positioner::moveTo(double x_mm, double y_mm) {
    long pos[] = {(long) (micro_stepping * steps_per_mm * x_mm),
                  (long) (micro_stepping * steps_per_mm * y_mm)};
    steppers.moveTo(pos);
    steppers.runSpeedToPosition();
}

void Positioner::move(double x_mm, double y_mm) {
    long pos[] = {(long) (micro_stepping * steps_per_mm * x_mm + x_stepper.currentPosition()),
                  (long) (micro_stepping * steps_per_mm * y_mm + y_stepper.currentPosition())};
    steppers.moveTo(pos);
    steppers.runSpeedToPosition();
}

void Positioner::home() {
    move(10, -10);
    delay(250);
    x_stepper.setSpeed((float) micro_stepping * -200.0f);
    while (digitalRead(x_limit_pin))
        x_stepper.runSpeed();
    delay(250);

    move(5, 0);
    delay(250);
    x_stepper.setSpeed((float) micro_stepping * -25.0f);
    while (digitalRead(x_limit_pin))
        x_stepper.runSpeed();
    x_stepper.setCurrentPosition(0);

    y_stepper.setSpeed((float) micro_stepping * 200.0f);
    while (digitalRead(y_limit_pin))
        y_stepper.runSpeed();
    delay(250);

    move(0, -5);
    delay(250);
    y_stepper.setSpeed((float) micro_stepping * 25.0f);
    while (digitalRead(y_limit_pin))
        y_stepper.runSpeed();
    y_stepper.setCurrentPosition(0);
}
