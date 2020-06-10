//
// Created by Misha on 4/9/2020.
//

#include <Arduino.h>
#include <SPI.h>
#include "Positioner.h"

#define MOSI_PIN 11
#define MISO_PIN 12
#define SCK_PIN  13

//TMC2130 registers
#define WRITE_FLAG     0x80U //write flag (1<<7)
#define READ_FLAG      (0<<7) //read flag
#define REG_GCONF      0x00U
#define REG_GSTAT      0x01U
#define REG_IHOLD_IRUN 0x10U
#define REG_CHOPCONF   0x6CU
#define REG_COOLCONF   0x6DU
#define REG_DCCTRL     0x6EU
#define REG_DRVSTATUS  0x6FU

#define STALL_THRESH   9
#define MAX_SPEED      600.0f

uint8_t tmc_write(uint8_t cmd, uint32_t data, int child_select);

uint8_t tmc_read(uint8_t cmd, uint32_t *data, int child_select);

bool is_stalled(uint32_t *data, int child_select);


Positioner::Positioner(int x_enable, int x_step, int x_dir, int y_enable, int y_step, int y_dir,
                       int x_child_select, int y_child_select, int steps_per_rev, int mm_per_rev) {
    this->x_enable = x_enable;
    this->y_enable = y_enable;
    this->x_child_select = x_child_select;
    this->y_child_select = y_child_select;

    steps_per_mm = steps_per_rev / mm_per_rev;

    x_stepper = AccelStepper(AccelStepper::DRIVER, x_step, x_dir);
    y_stepper = AccelStepper(AccelStepper::DRIVER, y_step, y_dir);
    steppers = MultiStepper();

    data_buffer = 0;
    x_col_count = y_col_count = 0;
    x_ref = y_ref = 0;
    reported = true;
}

void Positioner::start() {
    pinMode(x_enable, OUTPUT);
    pinMode(y_enable, OUTPUT);
    disable();
    enabled = false;

    pinMode(x_child_select, OUTPUT);
    pinMode(y_child_select, OUTPUT);
    digitalWrite(x_child_select, HIGH);
    digitalWrite(y_child_select, HIGH);

    pinMode(MOSI_PIN, OUTPUT);
    digitalWrite(MOSI_PIN, LOW);
    pinMode(MISO_PIN, INPUT);
    digitalWrite(MISO_PIN, HIGH);
    pinMode(SCK_PIN, OUTPUT);
    digitalWrite(SCK_PIN, LOW);

    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));


    uint32_t microstepping_data = 0x18008008UL;

    tmc_write(WRITE_FLAG | REG_GCONF, 0x00000001UL, x_child_select); //voltage on AIN is current reference
    tmc_write(WRITE_FLAG | REG_IHOLD_IRUN, 0x00001010UL, x_child_select); //IHOLD=0x10, IRUN=0x10
    tmc_write(WRITE_FLAG | REG_CHOPCONF, microstepping_data, x_child_select); //MRES=0, TBL=1=24, TOFF=8
    tmc_write(WRITE_FLAG | REG_GCONF, 0x00000001UL, y_child_select); //voltage on AIN is current reference
    tmc_write(WRITE_FLAG | REG_IHOLD_IRUN, 0x00001010UL, y_child_select); //IHOLD=0x10, IRUN=0x10
    tmc_write(WRITE_FLAG | REG_CHOPCONF, microstepping_data, y_child_select); //MRES=0, TBL=1=24, TOFF=8

    tmc_write(WRITE_FLAG | REG_COOLCONF, 0x01050000UL, x_child_select);
    tmc_write(WRITE_FLAG | REG_COOLCONF, 0x01050000UL, y_child_select);

    steppers.addStepper(x_stepper);
    steppers.addStepper(y_stepper);

    set_vel(1.0);
}

void Positioner::enable() {
    digitalWrite(x_enable, LOW);
    digitalWrite(y_enable, LOW);
    enabled = true;
}

void Positioner::disable() {
    digitalWrite(x_enable, HIGH);
    digitalWrite(y_enable, HIGH);
    enabled = false;
}

void Positioner::moveTo(double x_mm, double y_mm) {
    if (enabled) {
        x_ref = x_stepper.currentPosition();
        y_ref = y_stepper.currentPosition();
        long pos[] = {max(x_bound, min(0, (long) (steps_per_mm * x_mm))),
                      max(0, min(y_bound, (long) (steps_per_mm * y_mm)))};
        steppers.moveTo(pos);
        reported = false;
    }
}

void Positioner::move(double x_mm, double y_mm) {
    if (enabled) {
        x_ref = x_stepper.currentPosition();
        y_ref = y_stepper.currentPosition();
        long pos[] = {max(x_bound, min(0, (long) (steps_per_mm * x_mm + x_stepper.currentPosition()))),
                      max(0, min(y_bound, (long) (steps_per_mm * y_mm + y_stepper.currentPosition())))};
        steppers.moveTo(pos);
        reported = false;
    }
}

bool Positioner::home() {
    if (enabled) {
        x_stepper.setCurrentPosition(0);
        y_stepper.setCurrentPosition(0);

        x_stepper.setSpeed(-MAX_SPEED);
        while (x_stepper.currentPosition() > -20)
            x_stepper.runSpeed();
        while (!is_stalled(&data_buffer, x_child_select)) {
            x_stepper.runSpeed();
        }
        delay(250);
        x_stepper.setCurrentPosition(0);

        x_stepper.setSpeed(MAX_SPEED);
        while (x_stepper.currentPosition() < 20)
            x_stepper.runSpeed();
        while (!is_stalled(&data_buffer, x_child_select)) {
            x_stepper.runSpeed();
        }
        delay(250);
        x_bound = -x_stepper.currentPosition() + 20;
        x_stepper.setCurrentPosition(0);

        move(-80, 0);
        steppers.runSpeedToPosition();
        delay(250);

        y_stepper.setSpeed(MAX_SPEED);
        while (y_stepper.currentPosition() < 20)
            y_stepper.runSpeed();
        while (!is_stalled(&data_buffer, y_child_select)) {
            y_stepper.runSpeed();
        }
        delay(250);
        y_stepper.setCurrentPosition(0);

        y_stepper.setSpeed(-MAX_SPEED);
        while (y_stepper.currentPosition() > -20)
            y_stepper.runSpeed();
        while (!is_stalled(&data_buffer, y_child_select)) {
            y_stepper.runSpeed();
        }
        delay(250);
        y_bound = -y_stepper.currentPosition() - 20;
        y_stepper.setCurrentPosition(0);

        moveTo(0, 0);
        return true;
    } else {
        return false;
    }
}

bool Positioner::run() {
    if (abs(x_stepper.currentPosition() - x_ref) > 20 && is_stalled(&data_buffer, x_child_select)) {
        stop();
        x_col_count++;
        if (x_stepper.currentPosition() - x_ref < 0) {
            move(10, 0);
        } else {
            move(-10, 0);
        }
    } else if (abs(y_stepper.currentPosition() - y_ref) > 20 && is_stalled(&data_buffer, y_child_select)) {
        stop();
        y_col_count++;
        if (y_stepper.currentPosition() - y_ref < 0) {
            move(0, 10);
        } else {
            move(0, -10);
        }
    }
    return steppers.run();
}


float Positioner::xpos() {
    return (float) x_stepper.currentPosition() / ((float) steps_per_mm);
}

float Positioner::ypos() {
    return (float) y_stepper.currentPosition() / ((float) steps_per_mm);
}

void Positioner::stop() {
    x_stepper.stop();
    y_stepper.stop();
    x_stepper.stop();
    y_stepper.stop();
    report();
}

void Positioner::report() {
    Serial.print(xpos());
    Serial.print('\t');
    Serial.print(ypos());
    Serial.print('\t');
    Serial.print(x_col_count);
    Serial.print('\t');
    Serial.print(y_col_count);
    Serial.println();
    reported = true;
}

void Positioner::set_vel(float scale) {
    x_stepper.setMaxSpeed(MAX_SPEED * scale);
    y_stepper.setMaxSpeed(MAX_SPEED * scale);
}

uint8_t tmc_write(uint8_t cmd, uint32_t data, int child_select) {
    uint8_t s;

    digitalWrite(child_select, LOW);

    s = SPI.transfer(cmd);
    SPI.transfer((data >> 24UL) & 0xFFUL);
    SPI.transfer((data >> 16UL) & 0xFFUL);
    SPI.transfer((data >> 8UL) & 0xFFUL);
    SPI.transfer((data >> 0UL) & 0xFFUL);

    digitalWrite(child_select, HIGH);

    return s;
}

uint8_t tmc_read(uint8_t cmd, uint32_t *data, int child_select) {
    uint8_t s;

    tmc_write(cmd, 0UL, child_select); //set read address

    digitalWrite(child_select, LOW);

    s = SPI.transfer(cmd);
    *data = SPI.transfer(0x00) & 0xFFU;
    *data <<= 8U;
    *data |= SPI.transfer(0x00) & 0xFFU;
    *data <<= 8U;
    *data |= SPI.transfer(0x00) & 0xFFU;
    *data <<= 8U;
    *data |= SPI.transfer(0x00) & 0xFFU;

    digitalWrite(child_select, HIGH);

    return s;
}

bool is_stalled(uint32_t *data, int child_select) {
    static int stallcount[10];

    tmc_read(REG_DRVSTATUS, data, child_select);
//    Serial.print(*data, HEX);
//    Serial.print(' ');
//    Serial.println(stallcount);

    if ((bool) (((*data) & 0x01000000UL) >> 24U))
        stallcount[child_select]++;
    else
        stallcount[child_select] = 0;

    return stallcount[child_select] >= STALL_THRESH;
}