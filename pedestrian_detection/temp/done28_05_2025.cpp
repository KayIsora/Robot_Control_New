// Code nay chay duoc face recognition, tracking, 
//điều kiển bánh xe, nhưng code xấu, chưa có chia sẻ vùng nhớ

#include "object_detection.h"
#include "camera_tracking.h"
#include "robot_move.h"
#include "face_detection.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <opencv4/opencv2/tracking/tracker.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/face.hpp>
#include <math.h>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <fstream>
#include <sstream>
#include <chrono>
#include <iostream>
#include <thread>
#include <mutex>
// lib for face recognize
#include "src/config.h"
#include "src/anchor_creator.h"
#include "src/utils.h"
#include "benchmark.h"
// lib for tracker
#include "src/kalman_filter.h"
#include <opencv2/core/core.hpp>
#include "src/kcftracker.hpp"
#include <dirent.h>
#include <chrono>
#include <unistd.h>
#include <map>
#include <future>
#include <string>
#include <cstring>
#include <sys/time.h>
using namespace std;
using namespace cv;
using namespace cv::face;
using namespace cv::xfeatures2d;
// move
#define SLAVE_ADDRESS 0x08
// NEW
int ok = 1;
std::mutex videoMutex;
//***************************funtion  face detect and recognize****************************//

//****************************************end func **************************************
int pan = 90;
double minfps = 100, maxfps = 0;
int serial_port;

cv::Rect2d r;

cv::Point CalCenterObject(cv::Rect r)
{
  cv::Point point;
  point.x = r.x + r.width / 2;
  point.y = r.y + r.height / 2;
  return point;
}
std::queue<cv::Mat> frameQueue;        // Hàng đợi chứa các khung hình
std::mutex queueMutex;                 // Mutex để bảo vệ hàng đợi
std::condition_variable queueCondVar;  // Biến điều kiện để đồng bộ hóa luồng

// Hàm ghi video trong luồng riêng biệt
void writeVideo(cv::VideoWriter &writer) {
    while (true) {
        std::unique_lock<std::mutex> lock(queueMutex);
        queueCondVar.wait(lock, [] { return !frameQueue.empty(); });  // Chờ dữ liệu trong hàng đợi
        
        cv::Mat frame = frameQueue.front();
        frameQueue.pop();
        lock.unlock();

        if (frame.empty()) break;  // Dừng ghi nếu gặp frame rỗng (điều kiện kết thúc)

        writer.write(frame);  // Ghi frame vào video
    }
}
int main(int argc, char **argv)
{
  //gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! 'video/x-raw(memory:NVMM),width=1920, height=1080, framerate=30/1' ! nvvidconv flip-method=0 ! 'video/x-raw,width=960, height=540' ! nvvidconv ! nvegltransform ! nveglglessink -e

  //-----------------------------DETECT FACE-------------------------------
  //face recognizer

  int target_size = 300;
  int facenet_size = 112;
  ncnn::Net retinaface;
  ncnn::Net mbv2facenet;
  bool RUNFACE_DETECTION = true;
  int ret = 0;

  cv::Mat img1 = cv::imread("../test_pic/nhi.png", 1);
  ret = init_retinaface(&retinaface, target_size);
  if (ret)
  {
    cout << "loi load model init_retinaface() " << endl;
    return -1;
  }

  ret = init_mbv2facenet(&mbv2facenet, facenet_size);
  if (ret)
  {
    cout << "loi load model nit_mbv2facenet() " << endl;
    return -1;
  }
  std::vector<cv::Mat> face_det1;
  std::vector<std::vector<float>> feature_face1;
  auto box1 = detect_retinaface(&retinaface, img1, target_size, face_det1); //pic1 dect
  //cv::imshow("out put detect",face_det1);
  run_mbv2facenet(&mbv2facenet, face_det1, facenet_size, feature_face1);
  //-----------------------------DETECT FACE-------------------------------END
  //cv::VideoCapture cap(0);
  //cv::VideoCapture cap("/dev/video0");
  //cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  //cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  // cv::VideoCapture cap("/home/jetson/Desktop/NhiThanh/dataset/FO_CROSSING.avi");
  cv::VideoCapture cap("/home/jetson/Desktop/person_detection/pedestrian_detection/build/8-5.avi");
  cv::Mat frame;

  ObjectDetection det("../../pedestrian_detection/");

  auto m_StartTime = std::chrono::system_clock::now();

  bool isInitTracking = false;

  //config tracker
  bool HOG = true;
  bool FIXEDWINDOW = false;
  bool MULTISCALE = true;
  bool LAB = false;
  bool SILENT = true;
  // Create KCFTracker object
  KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

  cv::Point center_face_detect;
  cv::Point center_person_detect;
  cv::Rect BoudBox_Target;
  cv::Rect BoudBox_Tracking;
  cv::Mat cropTarget_1;
  float time_test = 0;
  int nFrame = 0;
  int nFrameTest = 0;
  int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  cv::VideoWriter videoWriter("output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(640, 480));
  if (!videoWriter.isOpened())
  {
    std::cerr << "Error: Cannot open video file for writing!" << std::endl;
    return -1;
  }
  std::thread writerThread(writeVideo, std::ref(videoWriter));
  // Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file.
  //cv::VideoWriter videoWriter("output.avi", cv::VideoWriter::fourcc('M','J','P','G'), 30, cv::Size(frame_width, frame_height));
  //mutex modVideoMutex;
  while (true)
  {
    //Kalman setup//
    Kalman kalman;
    float genome[] = {
        -1.04996, -0.35197, 2.14865, 1.81949, 1.0531,
        0.379274, -0.972355, -4.83614, -0.664763, 1.58524,
        3.81422, 0.125361, -0.805518, 1.51277, -2.40536,
        -3.02137, 2.54712, 0.250593, 0.310221, 0.0344269,
        -0.52078, 2.17617, 0.0965927, 0.910537, 0.0220096,
        0.745542, -1.34083, 0.2846, -1.66556, -1.72201,
        -0.333888, 2.63724, 1.7406, -2.1478, -0.25823,
        0.908437, 3.43149, -0.00610241, -0.767901, 0.0167775,
        1.48063, 0.0126195, 0.167397, -0.0533417, -0.0268063,
        -0.04304, -0.0294877, -2.30189, 0.188723, -0.300192,
        1.17634, 1.12888, 0.0306505, 0.0520604, -0.249356, -3.18886};
    kalman.set_from_genome(genome);
    //End kalman//
    nFrameTest++;
    cap >> frame;

    //cv::resize(frame,frame, cv::Size(640,360));
    if (frame.empty())
    {
      break;
    }

    double fps = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - m_StartTime).count();
    m_StartTime = std::chrono::system_clock::now();
    cv::putText(frame, to_string(static_cast<int>(1000 / fps)) + " FPS", cv::Point(7, 30), cv::FONT_HERSHEY_DUPLEX, 0.7, cv::Scalar(0, 0, 255), 1, false);
    cv::putText(frame, "Frame: " + std::to_string(nFrameTest), cv::Point(7, 90), 
                cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255, 255, 255), 1, false);
    //std::cout << 1000/fps << std::endl;
    //-----------------------------------Detect face -----------------------------
    if (RUNFACE_DETECTION == true)
    {
      //std::cout << "face" << std::endl;
      cv::Mat img2 = frame.clone();
      std::vector<cv::Mat> face_det;
      std::vector<std::vector<float>> feature_face;

      auto box2 = detect_retinaface(&retinaface, img2, target_size, face_det); //pic2 dect
      if (face_det.size() >= 1)
      {
        run_mbv2facenet(&mbv2facenet, face_det, facenet_size, feature_face);
        for (int i = 0; i < int(feature_face.size()); i++)
        {
          float sim = calc_similarity_with_cos(feature_face1[0], feature_face[i]);
          cv::rectangle(frame, box2[i], cv::Scalar(255, 255, 0), 2, 8, 0);
          string text;
          if (sim >= 0.1)
          {
            text = "object";
            cv::putText(frame, text, cv::Point(box2[i].x - 10, box2[i].y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            center_face_detect = CalCenterObject(box2[i]);
            RUNFACE_DETECTION = false;
            break;
          }
          else
          {
            text = "unknow";
            cv::putText(frame, text, cv::Point(box2[i].x - 10, box2[i].y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
          }
          //cout << "Same ratio of Face: " << sim << endl;
        }
      }
    }
    else if (RUNFACE_DETECTION == false && isInitTracking == false)
    {
      
      std::cout << "person" << std::endl;
      auto recs = det.detectObject(frame);
      if (recs.empty())
      {
      }
      else
      {
        float dis_Face_Person = 1000;

        for (auto rec : recs)
        {
          center_person_detect = CalCenterObject(rec);
          if (abs(center_person_detect.x - center_face_detect.x) < dis_Face_Person)
          {
            dis_Face_Person = abs(center_person_detect.x - center_face_detect.x);
            BoudBox_Target = rec;
          }
        }
        if (!BoudBox_Target.empty())
        {
          tracker.init(BoudBox_Target, frame);
        }
        if (0 <= BoudBox_Target.x && 0 <= BoudBox_Target.width && BoudBox_Target.x + BoudBox_Target.width <= frame.cols && 0 <= BoudBox_Target.y && 0 <= BoudBox_Target.height && BoudBox_Target.y + BoudBox_Target.height <= frame.rows)
          cropTarget_1 = frame(BoudBox_Target);

        isInitTracking = true;
      }
    }
    //std::cout << nFrame << std::endl;
    if (isInitTracking == true && nFrame == 10)
    {
      nFrame = 0;
      //std::cout << "person detection" << std::endl;
      bool isUpDate = true;
      auto recs = det.detectObject(frame);
      cv::Rect temp;
      if (recs.empty())
      {
        //std::cout << "error" << std::endl;
      }
      else
      {
        
        //std::cout << "Update target ...................." <<std::endl;
        float dis_person = 300;
        float cen_box_ex = (CalCenterObject(BoudBox_Target)).x;
        int count = 0;
        for (auto rec : recs)
        {
          //cv::rectangle(frame, rec, cv::Scalar(255, 255, 0), 2);
          if (0 <= rec.x && 0 <= rec.width && rec.x + rec.width <= frame.cols && 0 <= rec.y && 0 <= rec.height && rec.y + rec.height <= frame.rows && !rec.empty())
          {

            count++;
            float dis_cal = abs((CalCenterObject(rec)).x - cen_box_ex);
            //std::cout << dis_cal << std::endl;
            if (count != 1)
            {
              if (((rec.x >= temp.x) && (rec.x <= temp.x + temp.width)) || ((rec.x + rec.width >= temp.x) && (rec.x + rec.width <= temp.x + temp.width)))
              {
                cout << "false";
                isUpDate = false;
                break;
              }
            }
            if (dis_cal < dis_person)
            {
              dis_person = dis_cal;
              temp = rec;
              //isUpDate = true;
            }
          }
        }
        
        if (isUpDate == true && count != 0)
        {
          tracker.init(temp, frame);
        }
        else
        {
          std::future<cv::Rect> update_future = std::async(std::launch::async, [&]()
              { return tracker.update(frame, ok); });
          BoudBox_Tracking = update_future.get();
          isUpDate = false;
        }
      }
    }

    else if (isInitTracking == true)
    {
    
      //std::cout << "update"  << std::endl;
      nFrame = nFrame + 1;
      //std::cout << "tracking" << std::endl;
      std::future<cv::Rect> update_future = std::async(std::launch::async, [&]()
                                                       { return tracker.update(frame, ok); });
      BoudBox_Tracking = update_future.get();
      cv::rectangle(frame, BoudBox_Tracking, cv::Scalar(0, 0, 255), 2, 1);
      Move(640, 480, BoudBox_Tracking.x, BoudBox_Tracking.width, BoudBox_Tracking.height);
    }
    if (1000/fps < minfps && 1000/fps >= 2) minfps = 1000/fps;
    if (1000/fps > maxfps && 1000/fps <= 50) maxfps = 1000/fps;
     cv::putText(frame, "Min FPS: " + std::to_string(static_cast<int>(minfps)), cv::Point(7, 50), 
                cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 255), 1, false);
    cv::putText(frame, "Max FPS: " + std::to_string(static_cast<int>(maxfps)), cv::Point(7, 70), 
                cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 255), 1, false);
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now() - m_StartTime);
    time_test += time_used.count();

    cv::imshow("detection", frame);
    // cv::imwrite("output.jpg", frame);

    cv::Mat temp = frame.clone();
    {
      std::lock_guard<std::mutex> lock(queueMutex);
      frameQueue.push(frame.clone());  // Clone frame để tránh xung đột dữ liệu
    }
    queueCondVar.notify_one();
    if (cv::waitKey(1) == 'q') {
      break;
    }
    
  }
 
  cout << "Time average: " << time_test / nFrameTest << endl;
  cout << "Number of Frames: " << nFrameTest << endl;
  Cleanup();

  cap.release();
  videoWriter.release();
  cv::destroyAllWindows();
  deinit(&retinaface, &mbv2facenet);
  return 0;
}
