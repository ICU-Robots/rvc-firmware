//
// Created by Misha on 8/25/2020.
//

#include <Arduino.h>
#include "MultiStepper.h"

MultiStepper::MultiStepper(float max_speed, float acceleration) {
    c = 0;
    this->max_speed = max_speed;
    this->acceleration = acceleration;
}

bool MultiStepper::addStepper(AccelStepper &stepper) {
    steppers[c] = &stepper;
    c++;
}

void MultiStepper::moveTo(long absolute[]) {
    long dx = absolute[0] - steppers[0]->currentPosition();
    long dy = absolute[1] - steppers[1]->currentPosition();

    if (abs(dx) < abs(dy)) {
        float k = ((float) dx) / dy;

        steppers[0]->setMaxSpeed(max_speed * k);
        steppers[0]->setAcceleration(acceleration * k);
        steppers[1]->setMaxSpeed(max_speed);
        steppers[1]->setAcceleration(acceleration);
    } else if (abs(dx) > abs(dy)) {
        float k = ((float) dy) / dx;

        steppers[0]->setMaxSpeed(max_speed);
        steppers[0]->setAcceleration(acceleration);
        steppers[1]->setMaxSpeed(max_speed * k);
        steppers[1]->setAcceleration(acceleration * k);
    } else {
        steppers[0]->setMaxSpeed(max_speed);
        steppers[0]->setAcceleration(acceleration);
        steppers[1]->setMaxSpeed(max_speed);
        steppers[1]->setAcceleration(acceleration);
    }

    steppers[0]->moveTo(absolute[0]);
    steppers[1]->moveTo(absolute[1]);
}

bool MultiStepper::run() {
    return steppers[0]->run() || steppers[1]->run();
}

void MultiStepper::runSpeedToPosition() {
    long dx = steppers[0]->targetPosition() - steppers[0]->currentPosition();
    long dy = steppers[1]->targetPosition() - steppers[1]->currentPosition();

    if (abs(dx) < abs(dy)) {
        float k = ((float) dx) / dy;

        steppers[0]->setSpeed(max_speed * k);
        steppers[1]->setSpeed(max_speed);
    } else if (abs(dx) > abs(dy)) {
        float k = ((float) dy) / dx;

        steppers[0]->setSpeed(max_speed);
        steppers[1]->setSpeed(max_speed * k);
    } else {
        steppers[0]->setSpeed(max_speed);
        steppers[1]->setSpeed(max_speed);
    }

    while (steppers[0]->isRunning() && steppers[1]->isRunning()) {
        steppers[0]->runSpeed();
        steppers[1]->runSpeed();
    }
}