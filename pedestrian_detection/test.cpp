// Code nay chay duoc face recognition, tracking, 
//điều kiển bánh xe, có chia sẻ vùng nhớ với mỗi lần chạy update tuy nhien van con bi crash

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
// lib for shared memory
#include <sys/mman.h>   // For shm_open, mmap, munmap, shm_unlink
#include <sys/stat.h>   // For mode constants
#include <fcntl.h>      // For O_* constants
#include <unistd.h>     // For ftruncate
#include <semaphore.h>  // For sem_open, sem_close, sem_wait, sem_post  

using namespace std;
using namespace cv;
using namespace cv::face;
using namespace cv::xfeatures2d;


// Cấu trúc này cần được định nghĩa giống nhau trong cả hai tiến trình (C++ và Python)
// Không có mutex bên trong cấu trúc SharedData nữa
struct SharedData {
    int x;
    int y;
    int width;
    int height;
    double fall_probability;
    bool data_ready; // Cờ này sẽ được bảo vệ bởi các semaphores
};
// Tên của vùng nhớ chia sẻ và các semaphores (cần duy nhất)
#define SHM_NAME "/robot_shm"
#define SEM_PRODUCER_NAME "/robot_sem_producer" // Semaphore cho bên ghi (C++)
#define SEM_CONSUMER_NAME "/robot_sem_consumer" // Semaphore cho bên đọc (Python)

// move
#define SLAVE_ADDRESS 0x08
// NEW
int ok = 1;
std::mutex videoMutex;
volatile bool stopWriting = false; // Thêm dòng này

// Khai báo các biến toàn cục để dễ quản lý trong ví dụ này
SharedData* shared_data = nullptr;
sem_t *sem_producer = nullptr;
sem_t *sem_consumer = nullptr;
int shm_fd = -1;

// Hàm dọn dẹp tài nguyên IPC
void CleanupIPC() {
    if (sem_producer != nullptr) {
        sem_close(sem_producer);
        sem_unlink(SEM_PRODUCER_NAME);
    }
    if (sem_consumer != nullptr) {
        sem_close(sem_consumer);
        sem_unlink(SEM_CONSUMER_NAME);
    }
    if (shared_data != nullptr) {
        munmap(shared_data, sizeof(SharedData));
    }
    if (shm_fd != -1) {
        close(shm_fd);
        shm_unlink(SHM_NAME);
    }
    std::cout << "IPC resources cleaned up." << std::endl;
}
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


  atexit(CleanupIPC);
    // 1. Mở hoặc tạo vùng nhớ chia sẻ
  shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
  if (shm_fd == -1) {
      perror("shm_open");
      return 1;
  }

  // 2. Thiết lập kích thước vùng nhớ chia sẻ
  if (ftruncate(shm_fd, sizeof(SharedData)) == -1) {
      perror("ftruncate");
      shm_unlink(SHM_NAME); // Dọn dẹp nếu có lỗi
      return 1;
  }

  // 3. Map vùng nhớ chia sẻ vào không gian địa chỉ của tiến trình
  shared_data = (SharedData*)mmap(NULL, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
  if (shared_data == MAP_FAILED) {
      perror("mmap");
      shm_unlink(SHM_NAME);
      return 1;
  }

  // 4. Mở hoặc tạo semaphores
  // Semaphore producer: ban đầu giá trị 0 (Python đợi)
  sem_producer = sem_open(SEM_PRODUCER_NAME, O_CREAT, 0666, 0);
  if (sem_producer == SEM_FAILED) {
      perror("sem_open producer");
      munmap(shared_data, sizeof(SharedData));
      shm_unlink(SHM_NAME);
      return 1;
  }

  // Semaphore consumer: ban đầu giá trị 1 (C++ có thể ghi ngay)
  sem_consumer = sem_open(SEM_CONSUMER_NAME, O_CREAT, 0666, 5);
  if (sem_consumer == SEM_FAILED) {
      perror("sem_open consumer");
      sem_close(sem_producer); sem_unlink(SEM_PRODUCER_NAME);
      munmap(shared_data, sizeof(SharedData));
      shm_unlink(SHM_NAME);
      return 1;
  }
    // Đảm bảo cờ data_ready được khởi tạo ban đầu là false
    // Điều này giúp Python biết rằng chưa có dữ liệu hợp lệ
    shared_data->data_ready = false;
    shared_data->fall_probability = 0.0; // Khởi tạo giá trị mặc định

    std::cout << "Shared memory and semaphores initialized. Size of SharedData: " << sizeof(SharedData) << " bytes" << std::endl;

  //-----------------------------DETECT FACE-------------------------------
  //face recognizer
  int target_size = 300;
  int facenet_size = 112;
  ncnn::Net retinaface;
  ncnn::Net mbv2facenet;
  bool RUNFACE_DETECTION = true;
  int ret = 0;
  int count_fall = 0;
  bool fall_detected = false;
  cv::Mat img1 = cv::imread("../test_pic/336.jpg", 1);
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
  cv::VideoCapture cap("/dev/video1");
  
  //cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  //cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  // cv::VideoCapture cap("/home/jetson/Desktop/person_detection/pedestrian_detection/build/Bacu.mp4");
  // cv::VideoCapture cap("/home/jetson/Desktop/person_detection/pedestrian_detection/build/8-5.avi");
  // cv::VideoCapture cap("/home/jetson/Desktop/person_detection/pedestrian_detection/build/DanhgiaNangluong.mp4");

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
    // if (nFrameTest == 300) {
    //   std::cout << "==> Simulating fall at frame 300!" << std::endl;
    //   sendFallSignal();  // Gửi tín hiệu té ngã
    // }
    bool object_tracked_or_detected = false; // Cờ để kiểm tra xem có đối tượng hợp lệ

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
    // RUNFACE_DETECTION = false;
    
    if (RUNFACE_DETECTION == true)
    {
      clearFallSignal();  // Gửi tín hiệu té ngã
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
        cout << "Empty in RUNFACE_DETECTION" << endl;
      }
      else
      {
        int dis_Face_Person = 1000;
        int error; 
        for (auto rec : recs)
        {
          center_person_detect = CalCenterObject(rec);
          error = abs(center_person_detect.x - center_face_detect.x);
          if (error < dis_Face_Person)
          {
            dis_Face_Person = error;
            BoudBox_Target = rec;
          }
        }
        if (!BoudBox_Target.empty())
        {
          tracker.init(BoudBox_Target, frame);
          isInitTracking = true;
        }
      }
    }
    //std::cout << nFrame << std::endl;
    if (isInitTracking == true && nFrame > 5)
    {
      nFrame = 0;
      std::cout << "Running person detection" << std::endl;
      bool isUpDate = false;
      auto recs = det.detectObject(frame);
      cv::Rect temp;
      int size = recs.size();
      if (size==0)
      {
        cout << "Empty in RUN PERSON DETECTION" << endl;
      }
      else
      {
        // std::cout << "Update target ...................." <<std::endl;
        float dis_person = 300;
        float cen_box_ex = (CalCenterObject(BoudBox_Target)).x;
        int count = 0;
        for (auto rec : recs)
        {
          if (size == 1)
          {
            if (0 <= rec.x && 0 <= rec.width && rec.x + rec.width <= frame.cols && 0 <= rec.y && 0 <= rec.height && rec.y + rec.height <= frame.rows && !rec.empty())
            {
              temp = rec;
              isUpDate = true;
            }
          }
          else
            isUpDate = false;
          // cv::rectangle(frame, rec, cv::Scalar(255, 255, 0), 2);
          // if (0 <= rec.x && 0 <= rec.width && rec.x + rec.width <= frame.cols && 0 <= rec.y && 0 <= rec.height && rec.y + rec.height <= frame.rows && !rec.empty())
          // {
          //   count++;
          //   float dis_cal = abs((CalCenterObject(rec)).x - cen_box_ex);
          //   std::cout << dis_cal << std::endl;
          //   if (count == 0)
          //   {
          //     if (((rec.x >= temp.x) && (rec.x <= temp.x + temp.width)) || ((rec.x + rec.width >= temp.x) && (rec.x + rec.width <= temp.x + temp.width)))
          //     {
          //       cout << "false";
          //       isUpDate = false;
          //       break;
          //     }
          //   }
          //   if (dis_cal < dis_person)
          //   {
          //     dis_person = dis_cal;
          //     temp = rec;
          //     //isUpDate = true;
          //   }
          // }
        }

        
        if (isUpDate == true)
        {
          tracker.init(temp, frame);
        }
        else
        {
          std::future<cv::Rect> update_future = std::async(std::launch::async, [&]()
              { return tracker.update(frame, ok); });
          BoudBox_Tracking = update_future.get();
          // isUpDate = false;
        }
      }
    }

    else if (isInitTracking == true)
    {
    
      // std::cout << "Running update"  << std::endl;
      nFrame = nFrame + 1;
      //std::cout << "tracking" << std::endl;
      std::future<cv::Rect> update_future = std::async(std::launch::async, [&]()
                                                       { return tracker.update(frame, ok); });
      BoudBox_Tracking = update_future.get();
      object_tracked_or_detected = true; // Đánh dấu là đã có bounding box
      // Kiểm tra xem bounding box có hợp lệ không (tránh lỗi kích thước 0 hoặc ra ngoài ảnh)
      if (BoudBox_Tracking.x < 0 || BoudBox_Tracking.y < 0 ||
          BoudBox_Tracking.width <= 0 || BoudBox_Tracking.height <= 0 ||
          BoudBox_Tracking.x + BoudBox_Tracking.width > frame.cols ||
          BoudBox_Tracking.y + BoudBox_Tracking.height > frame.rows) {
          object_tracked_or_detected = false; // Bounding box không hợp lệ
          // Bạn có thể cân nhắc reset tracker hoặc chuyển sang chế độ dò lại ở đây
      }
      cv::rectangle(frame, BoudBox_Tracking, cv::Scalar(0, 0, 255), 2, 1);
      // Distance(BoudBox_Tracking.width, BoudBox_Tracking.height, 640, 480);
      Move(640, 480, BoudBox_Tracking.x, BoudBox_Tracking.width, BoudBox_Tracking.height);
    }
    // --- Ghi Bounding Box vào Shared Memory ---
    if (object_tracked_or_detected) {
        // 5. Đợi semaphore consumer (đảm bảo Python đã đọc dữ liệu trước đó)
        // Lệnh này sẽ chặn nếu Python chưa đọc xong dữ liệu cũ.
        // sem_wait(sem_consumer);

        // 6. Ghi dữ liệu tọa độ vào shared memory
        shared_data->x = BoudBox_Tracking.x;
        shared_data->y = BoudBox_Tracking.y;
        shared_data->width = BoudBox_Tracking.width;
        shared_data->height = BoudBox_Tracking.height;
        shared_data->data_ready = true; // Báo hiệu dữ liệu mới đã sẵn sàng

        // 7. Post semaphore producer (báo hiệu cho Python có dữ liệu mới để đọc)
        sem_post(sem_producer);

        // Tùy chọn: Đọc fall_probability từ Python (nếu Python ghi lại)
        // Bạn có thể cần một semaphore khác để đồng bộ việc đọc này
        // Hoặc đơn giản là đọc mà không cần đồng bộ quá chặt nếu bạn chấp nhận độ trễ nhỏ
        // và Python đảm bảo ghi giá trị 0.0 khi không có té ngã.
        // Ví dụ:
        if (shared_data->fall_probability > 0.8) {
            // cv::putText(frame, "FALL CONFIRMED!", cv::Point(7, 110), cv::FONT_HERSHEY_DUPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            // cout << "FALL CONFIRMED";
            count_fall++;
            if(count_fall > 5 && fall_detected == false)
            {
              cv::putText(frame, "FALL CONFIRMED!", cv::Point(7, 110), cv::FONT_HERSHEY_DUPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
              cout << "FALL SENT";
              sendFallSignal();  // Gửi tín hiệu té ngã
              count_fall = 0;
              fall_detected = true;
            }
        }
    }
    // if (1000/fps < minfps && 1000/fps >= 2) minfps = 1000/fps;
    // if (1000/fps > maxfps && 1000/fps <= 50) maxfps = 1000/fps;
    //  cv::putText(frame, "Min FPS: " + std::to_string(static_cast<int>(minfps)), cv::Point(7, 50), 
    //             cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 255), 1, false);
    // cv::putText(frame, "Max FPS: " + std::to_string(static_cast<int>(maxfps)), cv::Point(7, 70), 
    //             cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 255), 1, false);
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

    // Báo hiệu cho luồng ghi video dừng lại
  stopWriting = true;
  queueCondVar.notify_one(); // Báo hiệu để luồng ghi thoát khỏi trạng thái chờ
  if (writerThread.joinable()) {
      writerThread.join(); // Chờ luồng ghi video kết thúc
  }

  cap.release();
  videoWriter.release();
  cv::destroyAllWindows();
  deinit(&retinaface, &mbv2facenet);
  return 0;
}
