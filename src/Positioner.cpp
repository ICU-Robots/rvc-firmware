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
#define WRITE_FLAG     0x0E //write flag
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

static int stallcount = 0;


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
    reported = false;
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

    x_stepper.setMaxSpeed(600.0f);
    y_stepper.setMaxSpeed(600.0f);
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
    long pos[] = {(long) (steps_per_mm * x_mm),
                  (long) (steps_per_mm * y_mm)};
    steppers.moveTo(pos);
    reported = false;
}

void Positioner::move(double x_mm, double y_mm) {
    long pos[] = {(long) (steps_per_mm * x_mm + x_stepper.currentPosition()),
                  (long) (steps_per_mm * y_mm + y_stepper.currentPosition())};
    steppers.moveTo(pos);
    reported = false;
}

void Positioner::home() {
    move(-70, 70);
    steppers.runSpeedToPosition();

    x_stepper.setSpeed(600.0f);
    delay(250);
    stallcount = 0;
    for (int i = 0; i < 1600; i++)
        x_stepper.runSpeed();
    while (stallcount < 8) {
        x_stepper.runSpeed();
        is_stalled(&data_buffer, x_child_select);
    }
    delay(250);
    x_stepper.setCurrentPosition(0);

    move(-120, 0);
    steppers.runSpeedToPosition();

    y_stepper.setSpeed(-600.0f);
    delay(250);
    stallcount = 0;
    for (int i = 0; i < 1600; i++)
        y_stepper.runSpeed();
    while (stallcount < 8) {
        y_stepper.runSpeed();
        is_stalled(&data_buffer, y_child_select);
    }
    delay(250);
    y_stepper.setCurrentPosition(0);

    moveTo(0, 0);
    steppers.runSpeedToPosition();

}

void Positioner::run() {
    steppers.run();
}


float Positioner::xpos() {
    return (float) x_stepper.currentPosition() / ((float) steps_per_mm);
}

float Positioner::ypos() {
    return (float) y_stepper.currentPosition() / ((float) steps_per_mm);
}

void Positioner::stop() {
    move(0, 0);
    x_stepper.setSpeed(0);
    y_stepper.setSpeed(0);
}

bool Positioner::done() {
    return !x_stepper.isRunning() && !y_stepper.isRunning();
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
    tmc_read(REG_DRVSTATUS, data, child_select);
//    Serial.print(*data, HEX);
//    Serial.print(' ');
//    Serial.println(stallcount);

    if ((bool) (((*data) & 0x01000000UL) >> 24U))
        stallcount++;
    else
        stallcount = 0;

    return (bool) (((*data) & 0x01000000UL) >> 24U);
}