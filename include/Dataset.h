#ifndef MYSLAM_DATASET_H
#define MYSLAM_DATASET_H
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <mutex>
#include <vector>
#include<opencv2/core/core.hpp>

namespace ORB_SLAM3 {

/**
  * Data set read
  * The configuration file path is passed in during construction, and the dataset_dir of the configuration file is the dataset path
  * Camera and next frame image available after Init
  */
class Dataset {
    public:
  Dataset(const std::string& dataset_path_);
     typedef std::shared_ptr<Dataset> Ptr;
     std::vector<std::string> left_img;
     std::vector<std::string> right_img;
     std::vector<std::string> depth_img;
      std::vector<double> time_stamps;

     std::string dataset_path= "";
     int index = 0;
     int length = 0;
     void read_dataset();
     void read_dataset_realsense();
     bool next_realsense(cv::Mat &rgbimg, cv::Mat &depthimg);
     bool nextrgb_image(cv::Mat &rgbimg, cv::Mat &depthimg);
  

};
} // namespace myslam

#endif