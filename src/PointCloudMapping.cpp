#include "PointCloudMapping.h"
#include <fstream>
#include <thread>
#include <boost/format.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/projection_matrix.h>
#include <KeyFrame.h>
#include "Converter.h"

using namespace std;
namespace ORB_SLAM3 {
   
    PointCloudMapping::PointCloudMapping(double resolution_)
    {
        this->resolution = resolution_;
        voxel.setLeafSize( resolution, resolution, resolution);
        globalMap = boost::make_shared< PointCloud >( );
        a = boost::make_shared< PointCloud >( );

        viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
    }
    void PointCloudMapping::save_pointcloud()
    {
         shutDownFlag = true;
        //     globalMap->clear();
        //    int n = keyframes.size();
        //     for ( size_t i=0; i<n ; i++ )
        //     {
        //         PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
        //         *globalMap += *p;
        //     }
        //     PointCloud::Ptr tmp(new PointCloud());
        //     voxel.setInputCloud( globalMap );
        //     voxel.filter( *tmp );
        //     globalMap->swap( *tmp );
            

        //     pcl::io::savePCDFileBinary("/home/giang/ORB_SLAM3_Ubuntu20.04-master/Vocabulary/map1.pcd", *globalMap);
            cout <<" luu file pcd";
    }
    void PointCloudMapping::shutdown()
    {
        {

            unique_lock<mutex> lck(shutDownMutex);
          

            shutDownFlag = true;
            keyFrameUpdated.notify_one();
        }
        viewerThread->join();
    }
    string type2str(int type) {
    string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
    }
    void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
    {
        cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
        unique_lock<mutex> lck(keyframeMutex);
        keyframes.push_back( kf );
        colorImgs.push_back( color.clone() );
        depthImgs.push_back( depth.clone() );
        string datadir = "/home/giang/ORB_SLAM3_Ubuntu20.04-master/Vocabulary/images";
        boost::format fmt("%s/%s_%d.png");
        int n = keyframes.size();
        depth.convertTo(depth, CV_16U);
        cv::imwrite((fmt % datadir %"rgb"%(n)).str(),color);
        cv::imwrite((fmt % datadir %"depth"%(n)).str(),depth);
     //   cout << "\nframe id" << kf->mnId << " " <<kf->mnFrameId;
       //   string ty =  type2str( depth.type() );
       // cout <<"Matrix: "<< ty.c_str() << depth.cols << "x "<< depth.rows;
        /////////
    //    a = generatePointCloud(kf,color,depth);
    //    listLocalpointcloud.push_back();
        ////////////
        key_modified.push_back(false);
        keyFrameUpdated.notify_one();
    }

    pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
    {
        
        cout <<"\n lala 1";
        PointCloud::Ptr tmp( new PointCloud() );
        // for each pixel...
        // cout <<"\n depthIntrinsic: "<<depthIntrinsic.fx << " " <<depthIntrinsic.fy << " "<<depthIntrinsic.ppx <<" "
        //     <<depthIntrinsic.ppy << " " ;;
        // point cloud is null ptr
        for ( int m=0; m<depth.rows; m+=2 )
        {
            for ( int n=0; n<depth.cols; n+=2 )
            {
        //        uint16_t depth_value =  depth.at<ushort>(m, n);
         //  float d = depth_value * depthScale;
            float d = depth.ptr<float>(m)[n] ;
            float d1 = depth.ptr<float>(m + 1)[n] ; 
            float d2 = depth.ptr<float>(m )[n + 1] ;
            float d3 = depth.ptr<float>(m )[n - 1] ; 
            float d4 = depth.ptr<float>(m + 1)[n] ; 
			
                 if (d <= 0 || d >3 || d1 ==0 || d2 ==0 || d3 ==0 || d4 ==0 )
                     continue;
            //    cout << "\ndepth: " <<d;

            //    float depth_pixel[2];
            //     depth_pixel[0] = n;

            //     depth_pixel[1] = m;
                PointT p;
                // float depth_point[3];
                // rs2_deproject_pixel_to_point(depth_point, &depthIntrinsic, depth_pixel, d);
                // p.x = depth_point[0];
                // p.y = depth_point[1];
                // p.z = depth_point[2];

                // p.b = color.data[m * color.step + n * color.channels()];
                // p.g = color.data[m * color.step + n * color.channels() + 1];
                // p.r = color.data[m* color.step + n * color.channels() + 2];


              
                p.z = d;
                p.x = ( n - kf->cx) * p.z / kf->fx;
                p.y = ( m - kf->cy) * p.z / kf->fy;

                p.b = color.ptr<uchar>(m)[n*3];
                p.g = color.ptr<uchar>(m)[n*3+1];
                p.r = color.ptr<uchar>(m)[n*3+2];

                tmp->points.push_back(p);
            }
        }

        //  for (int v = 0; v < color.rows; v++)
        //     for (int u = 0; u < color.cols; u++) {
        //         unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
        //       //  cout << "d: " << d;
        //         if (d == 0) continue; // 为0表示没有测量到
        //         Eigen::Vector3d point;
        //         point[2] = double(d) / depthScale;
        //         point[0] = (u - kf->cx) * point[2] / kf->fx;
        //         point[1] = (v - kf->cy) * point[2] / kf->fy;
        //         Eigen::Vector3d pointWorld =  point;

        //         PointT p;
        //         p.x = pointWorld[0];
        //         p.y = pointWorld[1];
        //         p.z = pointWorld[2];
        //         p.b = color.data[v * color.step + u * color.channels()];
        //         p.g = color.data[v * color.step + u * color.channels() + 1];
        //         p.r = color.data[v * color.step + u * color.channels() + 2];
        //         tmp->points.push_back(p);
        //     }

       
       
        Eigen::Isometry3d T = ORB_SLAM3::Converter::toSE3Quat( kf->GetPose() );
        PointCloud::Ptr cloud(new PointCloud());
        pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
        cloud->is_dense = false;

      //  cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
   
        return cloud;
    }


    void PointCloudMapping::viewer()
    {
        PointCloud::Ptr globalMap1(new PointCloud());
        pcl::visualization::CloudViewer viewer("viewer");
        while(1)
        {
             
            {
                unique_lock<mutex> lck_shutdown( shutDownMutex );
                if (shutDownFlag)
                {


                    break;
                }
            }
            {
                unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
                keyFrameUpdated.wait( lck_keyframeUpdated );
            }
        //    globalMap->clear();
            // keyframe is updated
             cout << "\n 1 ------show global map, size=" << globalMap->points.size() << endl;
            size_t N=0;
            {
                unique_lock<mutex> lck( keyframeMutex );
                N = keyframes.size();
            }
            globalMap1->clear();
            int i = 0;
       
        //    tao keyframe moi
            for ( i=lastKeyframeSize; i<N ; i++ )
            {
                PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
          
                listLocalpointcloud.push_back(p);
            }
            
            //cap nhat keyframe cu
            for (  i=0 ; i<lastKeyframeSize ; i++ )
            {
                
                if(key_modified[i] ==true)
                {
                    cout << "\n keyframe " << i << " modified";
                    listLocalpointcloud[i]->points.clear();
                //    *listLocalpointcloud[i] = *generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
                   pcl::copyPointCloud(*generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] ),*listLocalpointcloud[i]);
                   
                    key_modified[i] = false;
                }
            }
            for ( size_t i=0; i<N ; i++ )
            {
              
                *globalMap1 += *listLocalpointcloud[i];
             // *globalMap += *p;
            }
         
            PointCloud::Ptr tmp(new PointCloud());
            voxel.setInputCloud( globalMap1 );
            voxel.filter( *tmp );
            globalMap1->swap( *tmp );
            
            viewer.showCloud( globalMap1 );
            cout << "show global map, size=" << globalMap1->points.size() << endl;
            lastKeyframeSize = N;
        }
        // PointCloud::Ptr globalMap1(new PointCloud());
        // for ( size_t i=0; i<keyframes.size() ; i++ )
        // {
        //     *globalMap1 += *listLocalpointcloud[i];
        // }
        
        // PointCloud::Ptr tmp(new PointCloud());
        // voxel.setInputCloud( globalMap1 );
        // voxel.filter( *tmp );
        // globalMap1->swap( *tmp );
        pcl::io::savePCDFileBinary("/home/giang/ORB_SLAM3_Ubuntu20.04-master/Vocabulary/map3.pcd", *globalMap1); 
    }

}