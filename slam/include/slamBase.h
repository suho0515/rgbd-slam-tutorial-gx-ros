  
/* ************************************************ ************************
	> File Name: rgbd-slam-tutorial-gx/part III/code/include/slamBase.h
	> Author: xiang gao
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
	> Created Time: Saturday, July 18, 2015 15:14:22
    > Description: Basic functions used in the rgbd-slam tutorial (C style)
************************************************** ********************* */
# pragma once

// Various header files
// C ++ standard library
#include <fstream>
#include <vector>
#include <map>
using namespace std;

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/nonfree/features2d.hpp> // use this if you want to use SIFT or SURF

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>






// type definition
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// Camera internal reference structure
struct CAMERA_INTRINSIC_PARAMETERS 
{ 
    double cx, cy, fx, fy, scale;
};

// Frame structure
struct FRAME
{
    int frameID; 
    cv :: Mat rgb, depth; // Color and depth map corresponding to this frame
    cv :: Mat desp;        // feature descriptor
    vector <cv :: KeyPoint> kp; // Key point
};

// PnP result
struct RESULT_OF_PNP
{
    cv :: Mat rvec, tvec;
    int inliers;
    bool error_flag;
};

// function interface
// image2PonitCloud converts rgb image to point cloud
PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera );

// point2dTo3d converts a single point from image coordinates to spatial coordinates
// input: 3-dimensional point Point3f (u, v, d)
cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera );

// computeKeyPointsAndDesp extracts both key points and feature descriptors
void computeKeyPointsAndDesp( FRAME& frame );

// estimateMotion calculates the motion between two frames
// input: frame 1 and frame 2, camera internal parameters
RESULT_OF_PNP estimateMotion( FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera );


// own cvMat2
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec );


// joinPointCloud 
PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera ) ;

