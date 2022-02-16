//
// Created by Misha on 4/9/2020.
//

#ifndef RVC_FIRMWARE_POSITIONER_H
#define RVC_FIRMWARE_POSITIONER_H

#include <AccelStepper.h>
#include "MultiStepper.h"

class Positioner {
public:
    Positioner(int x_enable, int x_step, int x_dir, int y_enable, int y_step, int y_dir, int x_child_select,
               int y_child_select, int steps_per_rev = 200, int mm_per_rev = 40, int microstepping = 1);

    void start();
    void enable();
    void disable();
    void moveTo(double x_mm, double y_mm);
    void move(double x_mm, double y_mm);
    bool home();
    bool run();
    void stop();
    float xpos();
    float ypos();
    void report(bool eom);
    void set_vel(float scale);

    bool reported;

private:
    AccelStepper x_stepper, y_stepper;
    MultiStepper steppers;
    int microstepping;
    int x_enable, y_enable;
    int x_child_select, y_child_select;
    int x_col_count, y_col_count;
    long x_bound, y_bound;
    int steps_per_mm;
    long x_ref, y_ref;
    bool enabled;
    uint32_t data_buffer;
};


#endif //RVC_FIRMWARE_POSITIONER_H
