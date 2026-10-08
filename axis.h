#ifndef AXIS_H
#define AXIS_H

#include <gpiod.h>
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <atomic>
#include "SafeQueue.h"


class Axis { 
private:
    enum class MotorState {
        POS,                // go to position as fast as possible
        SPEED,              // accelerate until speed is reached
        ACCELERATE,         // accelerate
        BRAKEPOS, 
        BRAKESPEED, 
        CHANGEDIRECTION
        };
    
    struct MotorTask {
        MotorState state;
        int arg; // desired Speed or desired Position otherwise 0
    };
    
    SafeQueue<MotorTask> tasks;

    std::atomic<bool> running{false};   // LÃ¤uft Thread?
    std::thread thread;             // Thread fÃ¼r Motorposition
    // Shared between the step thread and the TCP thread -> atomic (Stage 0.3)
    std::atomic<bool> isHoming{false};
    std::atomic<int> lastRightLimitValue{1}; // start, 1 = not pressed (pull up) 
    std::atomic<int> lastLeftLimitValue{1}; // start, not pressed
    
    std::atomic<int> pos{0};                // current position in steps
    std::atomic<int> endPos{0};             // total amount of steps from one side to the other
    std::atomic<int> direction{-1};         // current direction


    std::atomic<int> curSpeed{0};           // current speed in motor steps per second
    std::atomic<int> maxSpeed{120000};           // max speed in motor steps per second
    std::atomic<int> maxAcceleration{120000};    // maximum acceleration in steps/second*second
    double inverseMaxSpeed;
    int usDelay;
    int waitedUs;
    int minUsDelay;
    double accumulatedAcceleration;

    gpiod_line* step_line;
    gpiod_line* dir_line;
    gpiod_line* en_line;
    gpiod_line* right_limit_line;
    gpiod_line* left_limit_line;
    
    gpiod_line* ms1_line;
    gpiod_line* ms2_line;

    bool enableAtStart = true;              // false in bring-up mode: EN stays 1 until enableDriver()
    std::atomic<bool> driverEnabled{false}; // mirrors EN (true = EN low = driver on)

    // --- internal loop ---
    void eventLoop();
    
    void clearQueue();
    void stopAndReverse(int direction);

    void maxAccelerate(int untilSpeed);
    void maxDecelerate(int untilSpeed = 0);
    void decelerateForPos(int remainingDistance);
    void accelerate(double value);
    void stopOnLimit();
    void checkLimits();
    int getDecelDistance();

public:
    // Konstruktor / Destruktor
    Axis(gpiod_chip* chip,
         unsigned int pinStep,
         unsigned int pinDir,
         unsigned int pinEn,
         unsigned int pinRightLimit,
         unsigned int pinLeftLimit,
         unsigned int pinMs1,
         unsigned int pinMs2,
         bool homeOnStart = true   // false = bring-up mode: no homing, driver off until enableDriver()
         );

    ~Axis();

    // Threadsteuerung
    bool startThread();
    bool isCalibrating();
    
    void stopThread();

    // Bewegungssteuerung
    bool setPos(int newPos);
    bool setSpeed(int stepsPerSecond);
    bool setAcceleration(int stepsPerSecond);
    bool home();
    
    
    
    bool setMaxSpeed(int stepsPerSecond);
    bool setMaxAcceleration(int stepsPerSecond2);

    // Driver enable (EN pin, active low)
    void enableDriver();
    void disableDriver();
    bool isDriverEnabled();

    // Getter
    double getEndPos();
    int getPos();
    int getSpeed();
};




#endif
