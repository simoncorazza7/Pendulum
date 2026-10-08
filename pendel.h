#ifndef PENDEL_H
#define PENDEL_H

#include <gpiod.h>
#include <iostream>
#include <stdexcept>
#include <atomic>
#include <string>
#include "encoder.h"
#include "axis.h"


// Start-up options. Defaults = bring-up mode (safe): no homing, no gnuplot, driver off until
// the first motion command. --home restores the original start-up (home at once, driver on).
struct PendelOptions {
    bool home = false;               // home on start (original behaviour)
    bool plot = false;               // gnuplot angle plot (needs gnuplot + X11)
    std::string chip = "gpiochip4";  // on Raspberry Pi OS gpiochip4 -> gpiochip0 (udev symlink)
};

class Pendel {
private:
    Encoder* encoder;
    Axis* axis;
    gpiod_chip* chip;
    std::atomic<bool> plotRunning{false};
        std::thread thread;             // Thread for plot stream

public:
    explicit Pendel(const PendelOptions& opt = PendelOptions());
    ~Pendel();

    int getAngle() const; // in Anzahl Drehgeberimpulse
    int getPos() const;      // in mm
    int getSpeed() const;
    int getAngleVelocity() const;
    int getEndPos() const;
    bool setPos(double pos);
    bool setRelPos(double pos);
    bool setSpeed(int speed);
    bool setAcceleration(int acceleration);
    void calibratePos();
    bool isCalibrating();
    bool setMaxSpeed(int stepsPerSecond);
    bool setMaxAcceleration(int stepsPerSecond2);
    void consoleOut() const;
    void plotLoop() const;
    void startPlot();
};

#endif // PENDEL_H
