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
#define WRITE_FLAG     (1<<7) //write flag
#define READ_FLAG      (0<<7) //read flag
#define REG_GCONF      0x00
#define REG_GSTAT      0x01
#define REG_IHOLD_IRUN 0x10
#define REG_CHOPCONF   0x6C
#define REG_COOLCONF   0x6D
#define REG_DCCTRL     0x6E
#define REG_DRVSTATUS  0x6F

uint8_t tmc_write(uint8_t cmd, uint32_t data, int child_select);

uint8_t tmc_read(uint8_t cmd, uint32_t *data, int child_select);

bool is_stalled(uint32_t *data, int child_select);


Positioner::Positioner(int x_enable, int x_step, int x_dir, int y_enable, int y_step, int y_dir, int microstepping,
                       int x_child_select, int y_child_select, int steps_per_rev, int mm_per_rev) {
    this->x_enable = x_enable;
    this->y_enable = y_enable;
    this->x_child_select = x_child_select;
    this->y_child_select = y_child_select;

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

    pinMode(x_child_select, OUTPUT);
    pinMode(y_child_select, OUTPUT);
    digitalWrite(x_child_select, HIGH);
    digitalWrite(y_child_select, HIGH);

    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

    uint32_t p;
    uint32_t cmp = 256;
    for (p = 0; cmp > micro_stepping; cmp >>= 1UL, p++);

    uint32_t microstepping_data = 0x00008008UL | (p << 24UL);

    tmc_write(WRITE_FLAG | REG_GCONF, 0x00000001UL, x_child_select); //voltage on AIN is current reference
    tmc_write(WRITE_FLAG | REG_IHOLD_IRUN, 0x00001010UL, x_child_select); //IHOLD=0x10, IRUN=0x10
    tmc_write(WRITE_FLAG | REG_CHOPCONF, microstepping_data, x_child_select); //MRES=0, TBL=1=24, TOFF=8
    tmc_write(WRITE_FLAG | REG_GCONF, 0x00000001UL, y_child_select); //voltage on AIN is current reference
    tmc_write(WRITE_FLAG | REG_IHOLD_IRUN, 0x00001010UL, y_child_select); //IHOLD=0x10, IRUN=0x10
    tmc_write(WRITE_FLAG | REG_CHOPCONF, microstepping_data, y_child_select); //MRES=0, TBL=1=24, TOFF=8

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
    while (!is_stalled(data_buffer, x_child_select))
        x_stepper.runSpeed();
    delay(250);

    move(5, 0);
    delay(250);
    x_stepper.setSpeed((float) micro_stepping * -25.0f);
    while (!is_stalled(data_buffer, x_child_select))
        x_stepper.runSpeed();
    x_stepper.setCurrentPosition(0);

    y_stepper.setSpeed((float) micro_stepping * 200.0f);
    while (!is_stalled(data_buffer, y_child_select))
        y_stepper.runSpeed();
    delay(250);

    move(0, -5);
    delay(250);
    y_stepper.setSpeed((float) micro_stepping * 25.0f);
    while (!is_stalled(data_buffer, y_child_select))
        y_stepper.runSpeed();
    y_stepper.setCurrentPosition(0);
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
    *data = SPI.transfer(0x00) & 0xFF;
    *data <<= 8;
    *data |= SPI.transfer(0x00) & 0xFF;
    *data <<= 8;
    *data |= SPI.transfer(0x00) & 0xFF;
    *data <<= 8;
    *data |= SPI.transfer(0x00) & 0xFF;

    digitalWrite(child_select, HIGH);

    return s;
}

bool is_stalled(uint32_t *data, int child_select) {
    uint8_t s = tmc_read(REG_DRVSTATUS, data, child_select);
    return s & 0x04U;
}