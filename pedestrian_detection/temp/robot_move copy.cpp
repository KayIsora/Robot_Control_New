#include "robot_move.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <cstring> // Thư viện cho memcpy

#define maxdistance 250
#define mindistance 180
#define MaxRight 200
#define MaxLeft 200
#define Speed 25

bool end_this_program = false;
#define I2C_DEV "/dev/i2c-1" // Đường dẫn I2C
#define ARDUINO_ADDR 0x08
string status= "stop";
// Đường dẫn file ghi dữ liệu encoder
const char* leftFilePath = "/home/jetson/Desktop/person_detection/pedestrian_detection/build/left.txt";
const char* rightFilePath = "/home/jetson/Desktop/person_detection/pedestrian_detection/build/right.txt";
// Hàm chạy giao diện điều khiển I2C
void sendPWM(int leftSpeed, int rightSpeed)
{
    int file = open(I2C_DEV, O_RDWR);
    if (file < 0)
    {
        //std::cerr << "Failed to open the I2C bus." << std::endl;
        return;
    }

    if (ioctl(file, I2C_SLAVE, ARDUINO_ADDR) < 0)
    {
        //std::cerr << "Failed to acquire bus access and/or talk to slave." << std::endl;
        close(file);
        return;
    }

    uint8_t buffer[3];
    buffer[0] = 0x01;
    buffer[1] = leftSpeed;  // PWM cho động cơ trái
    buffer[2] = rightSpeed; // PWM cho động cơ phải
    //std::cout << "Left Speed: " << static_cast<int>(buffer[0]) << std::endl;
    //std::cout << "Right Speed: " << static_cast<int>(buffer[1]) << std::endl;
    if (write(file, buffer, 3) != 3) // Gửi 2 byte dữ liệu (leftSpeed, rightSpeed)
    {
        //std::cerr << "Failed to write to the I2C bus." << std::endl;
    }
    
}
void receiveData() {
   const char *dev = "/dev/i2c-1";  // Cổng I2C của Jetson Nano
    int file;

    // Mở cổng I2C
    if ((file = open(dev, O_RDWR)) < 0) {
        //std::cerr << "Failed to open the I2C bus." << std::endl;
        return ;
    }

    // Địa chỉ của Arduino Nano trên I2C (ví dụ: 0x08)
    int addr = 0x08;
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        //std::cerr << "Failed to connect to the I2C device." << std::endl;
        return ;
    }
    uint8_t buffer[8];
    if (read(file, buffer, 8) != 8) {
        //std::cerr << "Failed to read from the I2C bus." << std::endl;
        return ;
    }

    // Chuyển đổi 4 byte thành float
    float LeftSpeed, RightSpeed;
    memcpy(&LeftSpeed, buffer, sizeof(LeftSpeed));
    memcpy(&RightSpeed, buffer + 4, sizeof(RightSpeed));
    //std::cout << "Left speed: " << LeftSpeed << std::endl;
    std::ofstream leftFile;
    std::ofstream rightFile;

    leftFile.open(leftFilePath, std::ios::app);  // Mở tệp left_speed.txt (append mode)
    if (leftFile.is_open()) {
      leftFile << LeftSpeed << std::endl;  // Ghi giá trị leftSpeed vào tệp
      leftFile.close();
    } 
    rightFile.open(rightFilePath, std::ios::app);  // Mở tệp right_speed.txt (append mode)
    if (rightFile.is_open()) {
      rightFile << RightSpeed << std::endl;  // Ghi giá trị rightSpeed vào tệp
      rightFile.close();
    } 
    close(file);
}
    // Gửi dữ liệu (ví dụ: một giá trị float)
void signalHandler(int s)
{
    end_this_program = true;
}

int Distance(int knowWidth, int knowHeight, int frame_width, int frame_height)
{
    int focalLength = (frame_width / 2)*sqrt(3);
    //cout << "focal length" << focalLength  << endl;
    int realheight = 160 - 42;
    int sensorheight = 270;
    int Distance = (focalLength * realheight * frame_height) / (knowHeight * sensorheight) - 100;
    //std::cout << Distance << std::endl;
    return Distance;
}
void delay(int milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}
void Move(int frameWidth, int frameHeight, int recX, int recWidth, int recHeight)
{

    int distance = Distance(recWidth, recHeight, frameWidth, frameHeight);
    //std::cout<< distance << std::endl;
    int leftSpeed, rightSpeed;
    int objX = recX + recX / 2;
    int errorPan = objX - frameWidth / 2;
    //cout << "error Pan" << errorPan  << endl;
    if (abs(errorPan) > 100)
    {
        if (errorPan > 0)
        {
            leftSpeed = 0;
            rightSpeed = 220;
            sendPWM(leftSpeed, rightSpeed);
        }
        else // Taget in left of frame
        {
            rightSpeed = 0;
            leftSpeed = 220;
            sendPWM(leftSpeed, rightSpeed);
        }
        
    }
    else
    {
        if ((distance > maxdistance))
        {
            leftSpeed = 175;
            rightSpeed = 175;
            sendPWM(leftSpeed, rightSpeed);
            //std::cout << "Left Speed: " << leftSpeed << " Right Speed: " << rightSpeed << std::endl;
        }
        else if (mindistance < distance && distance < maxdistance)
        {
            leftSpeed = 0; // Dừng cả 2 động cơ
            rightSpeed = 0;
            sendPWM(leftSpeed, rightSpeed);
            //std::cout << "Left Speed: " << leftSpeed << " Right Speed: " << rightSpeed << std::endl;
        }
        else
        {
            leftSpeed = 174;
            rightSpeed = 174;
            sendPWM(leftSpeed, rightSpeed);
            //std::cout << "Left Speed: " << leftSpeed << " Right Speed: " << rightSpeed << std::endl;
        }
    }
   //std::cout << status << " Left: " << leftSpeed << " Right: " << rightSpeed << std::endl;
    receiveData();
    //receiveSpeedData();
}
// Hàm gửi tín hiệu té ngã
void sendFallSignal() {
    int file = open(I2C_DEV, O_RDWR);
    if (file < 0) {
        std::cerr << "Failed to open the I2C bus for fall signal." << std::endl;
        return;
    }

    if (ioctl(file, I2C_SLAVE, ARDUINO_ADDR) < 0) {
        std::cerr << "Failed to acquire bus access and/or talk to slave for fall signal." << std::endl;
        close(file);
        return;
    }

    uint8_t buffer[3];
    buffer[0] = 0x02;// Loại gói tin: báo té ngã
    buffer[1] = 255; // Tín hiệu đặc biệt đại diện cho "té ngã"
    buffer[2] = 255; // Tín hiệu bổ sung
    if (write(file, buffer, 3) != 3) {
        std::cerr << "Failed to write fall signal to the I2C bus." << std::endl;
    }

    close(file);
    std::cout << "Fall signal sent!" << std::endl;
}



void Cleanup()
{
    sendPWM(0, 0);
}