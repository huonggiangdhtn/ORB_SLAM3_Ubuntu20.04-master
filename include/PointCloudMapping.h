#ifndef POINTCLOUD_H
#define POINTCLOUD_H
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "System.h"
#include "KeyFrame.h"
#include "IntefaceCallback.h"
#include <condition_variable>
#include <librealsense2/rs.hpp>  
namespace ORB_SLAM3
{
    

    class PointCloudMapping : public myinterface
    {
    public:
        void doSomething(long unsigned int keyframeId)
        {
            if (shutDownFlag == true)
                return;
            cout << "\ndo some thing" <<keyframeId <<" size: "<<key_modified.size();
            if (key_modified.size() > keyframeId)
             {
                    key_modified[keyframeId] = true;
                    cout << "\n da set";
             }   
        }
        typedef std::shared_ptr<PointCloudMapping> Ptr;

        typedef pcl::PointXYZRGBA PointT;
        typedef pcl::PointCloud<PointT> PointCloud;

        PointCloudMapping( double resolution_ );
        void save_pointcloud();
    // Insert a keyframe, which will update the map once
        void insertKeyFrame( KeyFrame* kf, cv::Mat& color, cv::Mat& depth );
        void shutdown();
        void viewer();
        void setIntrinsic(rs2_intrinsics &x)
        {
            depthIntrinsic = x;
        }
        void setDepthscale(float depthscale_)
        {
            depthScale = depthscale_;
        }

    protected:
        PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);
    PointCloud::Ptr a;
        PointCloud::Ptr globalMap;
        std::shared_ptr<std::thread>  viewerThread;

        bool    shutDownFlag    =false;
        std::mutex   shutDownMutex;

        std::condition_variable  keyFrameUpdated;
        std::mutex               keyFrameUpdateMutex;

        // data to generate point clouds
        std::vector<KeyFrame*>       keyframes;
        std::vector<cv::Mat>         colorImgs;
        std::vector<cv::Mat>         depthImgs;
        std::vector<bool>            key_modified;
        std::vector<PointCloud::Ptr>      listLocalpointcloud;
        std::mutex                   keyframeMutex;
        uint16_t                lastKeyframeSize =0;
        rs2_intrinsics depthIntrinsic;
        double resolution = 0.04;
        pcl::VoxelGrid<PointT>  voxel;
        float depthScale = 0;
    };
}
#endif // MAP_H