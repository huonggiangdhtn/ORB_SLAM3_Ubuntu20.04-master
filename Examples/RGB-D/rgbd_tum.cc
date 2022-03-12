/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include <librealsense2/rs.hpp>  
#include <boost/format.hpp>
#include<System.h>
#include "Dataset.h"

using namespace cv;
using namespace std;
// namespace ORB_SLAM3
//{
    void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                    vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

    // int main(int argc, char **argv)
    // {
    //     ORB_SLAM3::Dataset::Ptr dataset_ = nullptr;
    //      rs2::pipeline pipe;
    //     rs2::config cfg;
    //     cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    //     cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
    //     auto pipeProfile = pipe.start(cfg);
    //     auto sensor = pipeProfile.get_device().first<rs2::depth_sensor>();
    //     auto align = new rs2::align(RS2_STREAM_COLOR);
    //     auto depth_stream = pipeProfile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    //     auto color_stream = pipeProfile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    //     // if(argc != 5)
    //     // {
    //     //     cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
    //     //     return 1;
    //     // }
    //     string str4 = "/home/giang/ORB_SLAM3_Ubuntu20.04-master/Examples/RGB-D/associations/fr1_xyz.txt";
    //    // string str1 = "/home/giang/ORB_SLAM3_Ubuntu20.04-master/Examples/RGB-D/TUM1.yaml";
    //    string str1 = "/home/giang/ORB_SLAM3_Ubuntu20.04-master/Examples/RGB-D/realsene.yaml";
    //   //  string str3 = "/media/giang/SSD 1/3d dataset/trum1";
    //     string str3 = "/media/giang/SSD 1/3d dataset/lab5";
    //     string str2= "/home/giang/ORB_SLAM3_Ubuntu20.04-master/Vocabulary/ORBvoc.txt";
    //     // Retrieve paths to images
    //     vector<string> vstrImageFilenamesRGB;
    //     vector<string> vstrImageFilenamesD;
    //     vector<double> vTimestamps;
    //     string strAssociationFilename = string(str4);

    //     dataset_ =
    //         ORB_SLAM3::Dataset::Ptr(new ORB_SLAM3::Dataset(str3));
    //     dataset_->read_dataset_realsense();
    //  //   LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
    //     vTimestamps = dataset_->time_stamps;
    //     // Check consistency in the number of images and depthmaps
    //     int nImages = dataset_->length;
     

    //     // Create SLAM system. It initializes all system threads and gets ready to process frames.
    //     ORB_SLAM3::System SLAM(str2,str1,ORB_SLAM3::System::RGBD,true);

    //     // Vector for tracking time statistics
    
    //     auto depthIntrinsic = depth_stream.get_intrinsics();
    //     SLAM.pointcloud_->setIntrinsic(depthIntrinsic);
    //     auto depthScale = sensor.get_depth_scale();
    //      SLAM.pointcloud_->setDepthscale(depthScale);
    //     // Main loop
    //   //  cv::Mat imRGB, imD;
    //     int ni=0;
    //    // while (dataset_->nextrgb_image(imRGB,imD))
    //     while (ni < 1000)
    //     {
    //         rs2::frameset frames = pipe.wait_for_frames();
    //         auto processedFrames = align->process(frames);
    //         rs2::frame frameDepth = processedFrames.first(RS2_STREAM_DEPTH);
    //         rs2::frame frameRGB = processedFrames.first(RS2_STREAM_COLOR);
    //         const int w = depth_stream.width();
    //         const int h = depth_stream.height();
    //         cv::Mat imRGB(cv::Size(w, h), CV_8UC3, (void*)frameRGB.get_data(), cv::Mat::AUTO_STEP);
    //         cv::Mat imD(cv::Size(w, h), CV_16U, (uchar*)frameDepth.get_data(), cv::Mat::AUTO_STEP);
    //         cv::Mat image_bgr, image_map, image_nrm;

    //         cv::cvtColor(imRGB, image_bgr, cv::COLOR_RGB2BGR);

    //         imD.convertTo(image_nrm, CV_32F);
    //         // double tframe = vTimestamps[ni];
    //         double tframe = 0;
    //         std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    //           // Pass the image to the SLAM system
    //         SLAM.TrackRGBD(image_bgr,image_nrm,tframe);
    //         std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    //         double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
           
    //      //   sleep(1);
    //         ni++;
    //     }
       
    // //     for(int ni=0; ni<nImages; ni++)
    // //     {
    // //         cout << "image at "<< ni;
    // //         // Read image and depthmap from file
    // //         // imRGB = cv::imread(string(str3)+"/"+vstrImageFilenamesRGB[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
    // //         // imD = cv::imread(string(str3)+"/"+vstrImageFilenamesD[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
    // //        dataset_->nextrgb_image(imRGB,imD);
    // //         double tframe = vTimestamps[ni];

    // //         if(imRGB.empty())
    // //         {
    // //             cerr << endl << "Failed to load image at: "
    // //                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
    // //             return 1;
    // //         }

    // // #ifdef COMPILEDWITHC11
    // //         std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    // // #else
    // //         std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    // // #endif

    // //         // Pass the image to the SLAM system
    // //         SLAM.TrackRGBD(imRGB,imD,tframe);

    // // #ifdef COMPILEDWITHC11
    // //         std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    // // #else
    // //         std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    // // #endif

    // //         double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    // //         vTimesTrack[ni]=ttrack;

    // //         // Wait to load the next frame
    // //         double T=0;
    // //         if(ni<nImages-1)
    // //             T = vTimestamps[ni+1]-tframe;
    // //         else if(ni>0)
    // //             T = tframe-vTimestamps[ni-1];

    // //         if(ttrack<T)
    // //             usleep((T-ttrack)*1e6);
    // //    }

    //     // Stop all threads
    //     SLAM.Shutdown();
    //     // nImages = ni;
    //     // if (vTimestamps.size() > 0)
    //     // {
    //     //     // Tracking time statistics
    //     //     sort(vTimesTrack.begin(),vTimesTrack.end());
    //     //     float totaltime = 0;
    //     //     for( ni=0; ni<nImages; ni++)
    //     //     {
    //     //         totaltime+=vTimesTrack[ni];
    //     //     }
    //     //     cout << "-------" << endl << endl;
    //     //     cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    //     //     cout << "mean tracking time: " << totaltime/nImages << endl;
    //     // }
     

    //     // Save camera trajectory
    //     SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    //     SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   
    //     dataset_.reset();
    //     return 0;
    // }


 int main(int argc, char **argv)
    {
        ORB_SLAM3::Dataset::Ptr dataset_ = nullptr;

        
        // if(argc != 5)
        // {
        //     cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        //     return 1;
        // }
        string str4 = "/home/giang/ORB_SLAM3_Ubuntu20.04-master/Examples/RGB-D/associations/fr1_xyz.txt";
        string str1 = "/home/giang/ORB_SLAM3_Ubuntu20.04-master/Examples/RGB-D/TUM1.yaml";
        string str3 = "/home/giang/rgbd_dataset_freiburg1_plant";
        string str2= "/home/giang/ORB_SLAM3_Ubuntu20.04-master/Vocabulary/ORBvoc.txt";
       
        // Retrieve paths to images
        vector<string> vstrImageFilenamesRGB;
        vector<string> vstrImageFilenamesD;
        vector<double> vTimestamps;
        string strAssociationFilename = string(str4);

        dataset_ =
            ORB_SLAM3::Dataset::Ptr(new ORB_SLAM3::Dataset(str3));
        dataset_->read_dataset();
         std::cout << str2;
     //   LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
        vTimestamps = dataset_->time_stamps;
        // Check consistency in the number of images and depthmaps
        int nImages = dataset_->length;
        cout << "\n Number images:" << nImages;

        // Create SLAM system. It initializes all system threads and gets ready to process frames.
        ORB_SLAM3::System SLAM(str2,str1,ORB_SLAM3::System::RGBD,true);

        // Vector for tracking time statistics
        vector<double> vTimesTrack;
        vTimesTrack.resize(nImages);

        // Main loop
        cv::Mat imRGB, imD;
        int ni=0;
        while (dataset_->nextrgb_image(imRGB,imD))
        {
            std::cout << "\n images:" << ni;
             double tframe = vTimestamps[ni];
           // double tframe = 0;
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
              // Pass the image to the SLAM system
            SLAM.TrackRGBD(imRGB,imD,tframe);
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            vTimesTrack[ni]=ttrack;
            // Wait to load the next frame
            double T=0;
            if (vTimestamps.size() > 0)
            {
                if(ni<nImages-1)
                    T = vTimestamps[ni+1]-tframe;
                else if(ni>0)
                    T = tframe-vTimestamps[ni-1];

                if(ttrack<T)
                    usleep((T-ttrack)*1e6);
            }
         //   sleep(1);
            ni++;
        }
       
    //     for(int ni=0; ni<nImages; ni++)
    //     {
    //         cout << "image at "<< ni;
    //         // Read image and depthmap from file
    //         // imRGB = cv::imread(string(str3)+"/"+vstrImageFilenamesRGB[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
    //         // imD = cv::imread(string(str3)+"/"+vstrImageFilenamesD[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
    //        dataset_->nextrgb_image(imRGB,imD);
    //         double tframe = vTimestamps[ni];

    //         if(imRGB.empty())
    //         {
    //             cerr << endl << "Failed to load image at: "
    //                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
    //             return 1;
    //         }

    // #ifdef COMPILEDWITHC11
    //         std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    // #else
    //         std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    // #endif

    //         // Pass the image to the SLAM system
    //         SLAM.TrackRGBD(imRGB,imD,tframe);

    // #ifdef COMPILEDWITHC11
    //         std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    // #else
    //         std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    // #endif

    //         double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    //         vTimesTrack[ni]=ttrack;

    //         // Wait to load the next frame
    //         double T=0;
    //         if(ni<nImages-1)
    //             T = vTimestamps[ni+1]-tframe;
    //         else if(ni>0)
    //             T = tframe-vTimestamps[ni-1];

    //         if(ttrack<T)
    //             usleep((T-ttrack)*1e6);
    //    }

        // Stop all threads
        SLAM.Shutdown();
        // nImages = ni;
        // if (vTimestamps.size() > 0)
        // {
        //     // Tracking time statistics
        //     sort(vTimesTrack.begin(),vTimesTrack.end());
        //     float totaltime = 0;
        //     for( ni=0; ni<nImages; ni++)
        //     {
        //         totaltime+=vTimesTrack[ni];
        //     }
        //     cout << "-------" << endl << endl;
        //     cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
        //     cout << "mean tracking time: " << totaltime/nImages << endl;
        // }
     

        // Save camera trajectory
        SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   
        dataset_.reset();
        return 0;
    }


    void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                    vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
    {
        ifstream fAssociation;
        fAssociation.open(strAssociationFilename.c_str());
        while(!fAssociation.eof())
        {
            string s;
            getline(fAssociation,s);
            if(!s.empty())
            {
                stringstream ss;
                ss << s;
                double t;
                string sRGB, sD;
                ss >> t;
                vTimestamps.push_back(t);
                ss >> sRGB;
                vstrImageFilenamesRGB.push_back(sRGB);
                ss >> t;
                ss >> sD;
                vstrImageFilenamesD.push_back(sD);

            }
        }
    }
//}