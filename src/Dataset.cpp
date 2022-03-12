
#include "Dataset.h"
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>
using namespace std;
using namespace cv;
namespace ORB_SLAM3 {
    Dataset::Dataset(const std::string& dataset_path_)
    {
        cout << "\nkhoitao";
        dataset_path = dataset_path_;
        cout << "\ndataset :" + dataset_path_;
    }
    void Dataset::read_dataset()
    {
                
                index = 0;
                ifstream fdepth;
                ifstream frgb;
                cout << "\nread 1";
                fdepth.open(dataset_path+ "/depth.txt");
                if (!fdepth) {
                   cout << "\nnot found fdepth: "<< dataset_path+ "/depth.txt";
                }
                cout << "\nread 2";
                frgb.open(dataset_path+ "/rgb.txt");
                 if (!frgb) {
                   cout << "\nnot found frgb: "<< dataset_path+ "/rgb.txt";
                }
                cout << "\nread 3";
                cout<<"\nfilename"<<dataset_path+ "/depth.txt";
                while(!fdepth.eof())
                {
                    string s;
                    getline(fdepth,s);
                    if(!s.empty())
                    {
                        stringstream ss;
                        ss << s;
                    
                        double t;
                        string  sDepth;
                        // UNKNOWN=> cannot understand the >> operator
                        ss >> t;
                        time_stamps.push_back(t);
                        ss >> sDepth;
                    
                        depth_img.push_back(sDepth);
                        length ++;
                    }
                }
                while(!frgb.eof())
                {
                    string s;
                    getline(frgb,s);
                    if(!s.empty())
                    {
                        stringstream ss;
                        ss << s;
                        double t;
                        string sRGB;
                        // UNKNOWN=> cannot understand the >> operator
                        ss >> t;
                        ss >> sRGB;
                        left_img.push_back(sRGB);
                    }
                }
    }
    bool Dataset::next_realsense(cv::Mat &rgbimg, cv::Mat &depthimg)
        {
        //   //  try{
                
        //         boost::format fmt("%s/%s/_%d.png");
        //         cout << "\n-------lala";
               
        //         cout << "\nread image:  "<<left;
        //         // cv::Mat image_left =
        //         //     cv::imread((fmt % dataset_path %"rgb" %(index + 1)).str(),
        //         //             cv::IMREAD_UNCHANGED);
        //         // cv::Mat   image_depth = cv::imread((fmt % dataset_path %"depth" %(index + 1)).str(),
        //         //         cv::IMREAD_UNCHANGED);
        //         index ++;
        //         return true;
        //     // }
        //     // catch(std::exception ex)
        //     // {
        //     //     cout << "\n" << ex.what();
        //     //     return false;
        //     // }
        
    }
     void Dataset::read_dataset_realsense()
    {
                
        index = 0;
        length = 1694;
        std::string left = dataset_path + "/"+ "rgb" +"/" + std::to_string(index+1) +".png";
        std::string depth = dataset_path + "/"+ "depth" +"/" + std::to_string(index+1) +".png";
        while (index < length)
        {
            left = "rgb/_" + std::to_string(index+1) +".png";
         //   cout << "\n"<<left;
            depth = "depth/_" + std::to_string(index+1) +".png";
            depth_img.push_back(depth);
            left_img.push_back(left);
            time_stamps.push_back(0);
            index ++;
        }
        index = 0;
              
    }
    bool Dataset::nextrgb_image(cv::Mat &rgbimg, cv::Mat &depthimg)
    {
        if(index < length)
        {
            try{
                boost::format fmt("%s/%s");
                rgbimg =
                    cv::imread((fmt % dataset_path %left_img[index]).str(),cv::IMREAD_UNCHANGED);
                depthimg =  
                    cv::imread((fmt % dataset_path %depth_img[index]).str(),cv::IMREAD_UNCHANGED);
             //   double mDepthMapFactor = 5000;
            //   if((fabs(mDepthMapFactor-1.0f)>1e-5) || image_depth.type()!=CV_32F)
            //        image_depth.convertTo(image_depth,CV_32F,1.0 / 16.0f);
                    // image_depth.convertTo(image_depth,CV_32F,5000);
            
        
                index++;
              
            }
            catch(std::exception e)
            {
                cout <<"error in reading";
                return false;
            }
            return true;
        }
        else
            return false;
          
    }

}