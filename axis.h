#ifndef AXIS_H
#define AXIS_H

#include <gpiod.h>
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <atomic>
#include <cstdint>
#include "SafeQueue.h"


class Axis { 
public:
    // Called by the step thread right after each STEP edge (instrumentation, Stage 0.5).
    // Times are CLOCK_MONOTONIC ns. Must be real-time safe (no allocation, locks or I/O).
    using StepObserver = void (*)(void* ctx, std::int64_t scheduledNs, std::int64_t actualNs);

private:
    enum class MotorState {
        POS,                // go to position as fast as possible
        SPEED,              // accelerate until speed is reached
        ACCELERATE,         // accelerate
        BRAKEPOS, 
        BRAKESPEED, 
        CHANGEDIRECTION,
        HALT                // shutdown: ignore the queue, brake to standstill
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
    std::atomic<bool> halted{false};        // set by halt(); the step thread then only brakes
    std::atomic<StepObserver> stepObserver{nullptr};
    void* stepObserverCtx = nullptr;
    std::atomic<std::uint32_t> resyncs{0};  // times the loop fell > 500 us behind and reset its schedule

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

    // Shutdown: brake to standstill at maxAcceleration and ignore all further tasks
    void halt();
    bool isStopped();

    // Instrumentation and set-up for the pendctl runtime (Stage 0.5)
    void setStepObserver(StepObserver fn, void* ctx);   // nullptr to remove
    std::uint32_t getResyncCount();
    bool configureRealtime(int priority, int cpu);      // SCHED_FIFO priority, pin to cpu (-1: no pin)
    void setPositionReference(int newPos, int newEndPos); // declare position and track length, no motion

    // Getter
    double getEndPos();
    int getPos();
    int getSpeed();
};




#endif
