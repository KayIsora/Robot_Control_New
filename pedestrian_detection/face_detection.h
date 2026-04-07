#ifndef FACE_DETECTION_H
#define FACE_DETECTION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "net.h" // Include ncnn::Net
#include "/home/jetson/Desktop/person_detection/pedestrian_detection/src/anchor_creator.h"

int init_retinaface(ncnn::Net* retinaface, const int target_size);


void deinit(ncnn::Net* retinaface, ncnn::Net* mbv2facenet);


int init_mbv2facenet(ncnn::Net* mbv2facenet, const int target_size);


std::vector<cv::Rect> detect_retinaface(ncnn::Net* retinaface, cv::Mat img, const int target_size, std::vector<cv::Mat>& face_det);


void run_mbv2facenet(ncnn::Net* mbv2facenet, std::vector<cv::Mat>& img, int target_size, std::vector<std::vector<float>>& res);

#endif
