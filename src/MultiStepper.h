//
// Created by Misha on 8/25/2020.
//

#ifndef RVC_FIRMWARE_MULTISTEPPER_H
#define RVC_FIRMWARE_MULTISTEPPER_H

#include <AccelStepper.h>

class MultiStepper {
private:
    AccelStepper * steppers[2];
    int c;
    float max_speed, acceleration;

public:
    MultiStepper(float max_speed, float acceleration);

    bool addStepper(AccelStepper & stepper);

    void moveTo(long absolute[]);

    bool run();

    void runSpeedToPosition();
};


#endif //RVC_FIRMWARE_MULTISTEPPER_H
