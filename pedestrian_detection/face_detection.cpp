#include "net.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <string>
#if CV_MAJOR_VERSION >= 3
#include "opencv2/video/video.hpp"
#endif

#include <vector>
#include "/home/jetson/Desktop/person_detection/pedestrian_detection/src/config.h"
#include "/home/jetson/Desktop/person_detection/pedestrian_detection/src/anchor_creator.h"
#include "/home/jetson/Desktop/person_detection/pedestrian_detection/src/utils.h"
#include "benchmark.h"
//gpu 
#include "gpu.h"
using namespace std;
bool face_detect = true;
int init_retinaface(ncnn::Net* retinaface, const int target_size)
{
    int ret = 0;
    // gpu
    
    ncnn::create_gpu_instance();
    retinaface->opt.use_vulkan_compute = 1;
    retinaface->opt.num_threads = 8;
    retinaface->opt.use_winograd_convolution = true;
    retinaface->opt.use_sgemm_convolution = true;

    const char* model_param = "../models/retinaface.param";
    const char* model_model = "../models/retinaface.bin";
    
    ret = retinaface->load_param(model_param);
    if(ret)
    {
        return ret;
    }
    ret = retinaface->load_model(model_model);
    if(ret)
    {
        return ret;
    }

    return 0;
}
void deinit(ncnn::Net* retinaface,ncnn::Net* mbv2facenet){ 
    retinaface->clear();
    ncnn::destroy_gpu_instance();
    mbv2facenet->clear();
}
int init_mbv2facenet(ncnn::Net* mbv2facenet, const int target_size)
{
    int ret = 0;
    // use gpu vulkan
    ncnn::create_gpu_instance();
    mbv2facenet->opt.use_vulkan_compute = 1;
    
    mbv2facenet->opt.num_threads = 8;
    mbv2facenet->opt.use_sgemm_convolution = 1;
    mbv2facenet->opt.use_winograd_convolution = 1;

    const char* model_param = "../models/mbv2facenet.param";
    const char* model_bin = "../models/mbv2facenet.bin";

    ret = mbv2facenet->load_param(model_param);
    if(ret)
    {
        return ret;
    }

    ret = mbv2facenet->load_model(model_bin);
    if(ret)
    {
        return ret;
    }

    return 0;
}

vector<cv::Rect> detect_retinaface(ncnn::Net* retinaface, cv::Mat img, const int target_size, std::vector<cv::Mat>& face_det)
{
    int img_w = img.cols;
    int img_h = img.rows;
    float tempo1= img_w*1.0 /target_size;
    //cout <<tempo1<<endl;
    float tempo2= img_h*1.0 /target_size;
    //cout<<img_w <<" "<<img_h<<endl;
    cv::Mat img1;
    vector<cv::Rect> bor;
    const float mean_vals[3] = {0, 0, 0};
    const float norm_vals[3] = {1, 1, 1};

    //ncnn::Mat input = ncnn::Mat::from_pixels(img.data, ncnn::Mat::PIXEL_BGR2RGB, img_w, img_h);
    ncnn::Mat input = ncnn::Mat::from_pixels_resize(img.data, ncnn::Mat::PIXEL_BGR2RGB,img_w, img_h, target_size, target_size);
    input.substract_mean_normalize(mean_vals, norm_vals);

    ncnn::Extractor ex = retinaface->create_extractor();
    ex.set_vulkan_compute(true);
    std::vector<AnchorCreator> ac(_feat_stride_fpn.size());
    for(size_t i = 0; i < _feat_stride_fpn.size(); i++)
    {
        int stride = _feat_stride_fpn[i];
        ac[i].init(stride, anchor_config[stride], false);
    }
    
    ex.input("data", input);

    std::vector<Anchor> proposals;

    for(size_t i = 0; i < _feat_stride_fpn.size(); i++)
    {
        ncnn::Mat cls;
        ncnn::Mat reg;
        ncnn::Mat pts;
        char cls_name[100];
        char reg_name[100];
        char pts_name[100];
        sprintf(cls_name, "face_rpn_cls_prob_reshape_stride%d", _feat_stride_fpn[i]);
        sprintf(reg_name, "face_rpn_bbox_pred_stride%d", _feat_stride_fpn[i]);
        sprintf(pts_name, "face_rpn_landmark_pred_stride%d", _feat_stride_fpn[i]);

        // cout << _feat_stride_fpn[i]<<endl;
        ex.extract(cls_name,cls);
        ex.extract(reg_name,reg);
        ex.extract(pts_name,pts);

        ac[i].FilterAnchor(cls, reg, pts, proposals);

    }

    std::vector<Anchor> finalres;
    box_nms_cpu(proposals, nms_threshold, finalres, target_size);
    cv::resize(img, img, cv::Size(target_size, target_size));
    for(size_t i = 0; i < finalres.size(); ++i)
    {
        finalres[i].print();//in thong so khuon mat xy-width-height-threadshold 300x300
        cv::Mat face = img(cv::Range((int)finalres[i].finalbox.y, (int)finalres[i].finalbox.height),cv::Range((int)finalres[i].finalbox.x, (int)finalres[i].finalbox.width)).clone();
        face_det.push_back(face);
        
        /*cv::rectangle(img, cv::Point((int)finalres[i].finalbox.x, (int)finalres[i].finalbox.y), cv::Point((int)finalres[i].finalbox.width, (int)finalres[i].finalbox.height),cv::Scalar(255,255,0), 2, 8, 0);*/
        float x=(float)finalres[i].finalbox.x*tempo1;
        float y= (float)finalres[i].finalbox.y*tempo2;
        float width =(float)finalres[i].finalbox.width*tempo1-x;
        float heigh =(float)finalres[i].finalbox.height*tempo2-y;
        //cout << "x,y "<<x<< " "<<y<<"width-height "<<width<<" "<<heigh<<endl;
        bor.push_back(cv::Rect((int)x,(int)y,(int)width,(int)heigh));
        /*for(size_t l = 0; l < finalres[i].pts.size(); ++l)
        {
            cv::circle(img, cv::Point((int)finalres[i].pts[l].x, (int)finalres[i].pts[l].y), 1, cv::Scalar(255, 255, 0), 2, 8, 0);
        }*/
    }
    return bor;
}

void run_mbv2facenet(ncnn::Net* mbv2facenet, std::vector<cv::Mat>& img, int target_size, std::vector<std::vector<float>>& res)
{
    for(size_t i = 0; i < img.size(); ++i)
    {
        ncnn::Extractor ex = mbv2facenet->create_extractor();
        //网络结构中的前两层已经做了归一化和均值处理， 在输入的时候不用处理了
        ex.set_vulkan_compute(true);
        ncnn::Mat input = ncnn::Mat::from_pixels_resize(img[i].data, ncnn::Mat::PIXEL_BGR2RGB, img[i].cols, img[i].rows, target_size, target_size);
        ex.input("data", input);
        
        ncnn::Mat feat;

        ex.extract("fc1", feat);

        //printf("c: %d h: %d w: %d\n", feat.c, feat.h, feat.w);
        std::vector<float> tmp;
        for(int i = 0; i < feat.w; ++i)
        {
            //printf("%f ", feat.channel(0)[i]);
            tmp.push_back(feat.channel(0)[i]);
        }
        res.push_back(tmp);
        //printf("\n");
    }
}   