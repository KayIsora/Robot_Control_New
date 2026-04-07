#ifndef ROBOT_MOVE_H
#define ROBOT_MOVE_H
#pragma once
#include <iostream>
#include <string>
// for delay function.
#include <chrono> 
#include <thread>

// for signal handling
#include <signal.h>
#include <math.h>
#include <../include/JetsonGPIO.h>

#define STOP true
#define RUN false

#define FORWARD true
#define BACKWARD false
#define SLAVE_ADDRESS 0x08

using namespace std;
using namespace GPIO;
#include <memory>



extern std::unique_ptr<GPIO::PWM> p_right;
extern std::unique_ptr<GPIO::PWM> p_left;


void signalHandler (int s);
int Distance(int knowWidth, int knowHeight, int frame_width, int frame_height);
std::unique_ptr<GPIO::PWM> CreatePWMObject(int output_pin);
void SetupMotor();
void InitMotor();

void Move(int frameWidth,int frameHeight, int recX, int recWidth, int recHeight);
const std::string I2C_DEVICE = "/dev/i2c-1";

void sendPWM(int leftSpeed, int rightSpeed);
void receiveEncoderData();
void sendFallSignal();
void Cleanup();
#endif
