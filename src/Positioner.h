//
// Created by Misha on 4/9/2020.
//

#ifndef RVC_FIRMWARE_POSITIONER_H
#define RVC_FIRMWARE_POSITIONER_H

#include <AccelStepper.h>
#include <MultiStepper.h>

class Positioner {
public:
    Positioner(int x_enable, int x_step, int x_dir, int y_enable, int y_step, int y_dir, int microstepping,
               int x_limit_pin, int y_limit_pin, int steps_per_rev = 200, int mm_per_rev = 40);

    void start();
    void enable();
    void disable();
    void moveTo(double x_mm, double y_mm);
    void move(double x_mm, double y_mm);
    void home();

private:
    AccelStepper x_stepper, y_stepper;
    MultiStepper steppers;
    int micro_stepping;
    int x_enable, y_enable;
    int x_limit_pin, y_limit_pin;
    int steps_per_mm;
};


#endif //RVC_FIRMWARE_POSITIONER_H
