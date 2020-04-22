  
/* ************************************************ ************************
	> File Name: rgbd-slam-tutorial-gx/part V/src/visualOdometry.cpp
	> Author: xiang gao
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
	> Created Time: Saturday, August 01, 2015 15:35:42
************************************************** ********************* */

#include <iostream>

// for header
#include "slamBase.h"

// for ros
#include <ros/ros.h>

// for multiple subscriber
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// for image processing
#include <cv_bridge/cv_bridge.h>

// for pcl
#include <pcl_ros/point_cloud.h>

// for opencv
#include <opencv2/core/eigen.hpp>



// Note: for colorized point cloud, use PointXYZRGBA
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher pointcloud_pub;

// Declare two frames, see include / slamBase.h for FRAME structure
FRAME frame1, frame2;

int cnt = 0;

PointCloud::Ptr output (new PointCloud());

// Measure the size of the movement
double  normofTransform (cv :: Mat rvec, cv :: Mat tvec);




void callback(const ImageConstPtr& image_color_msg,
		const ImageConstPtr& image_depth_msg,
		const CameraInfoConstPtr& info_color_msg,
		const CameraInfoConstPtr& info_depth_msg) {

	// Solve all of perception here...
	cv::Mat image_color = cv_bridge::toCvCopy(image_color_msg)->image;
	cv::Mat image_depth = cv_bridge::toCvCopy(image_depth_msg)->image;

	cvtColor(image_color,image_color, CV_RGB2BGR);

 	if(cnt==0) 
	{
		frame2.rgb = image_color;
		frame2.depth = image_depth;
		cnt++;
		return;
	}
	else
	{
		frame1 = frame2;

		frame2.rgb = image_color;
		frame2.depth = image_depth;
        cnt++;
        if(cnt>3) cnt=3;
	}
	
    // Extract features and calculate descriptors
    cout<<"extracting features"<<endl;

    if(cnt==1)computeKeyPointsAndDesp (frame1);
    computeKeyPointsAndDesp (frame2);

    // Camera internal parameters
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.cx = info_depth_msg->K[0];
    camera.cy = info_depth_msg->K[4];
    camera.fx = info_depth_msg->K[2];
    camera.fy = info_depth_msg->K[2];
    camera.scale = 1000.0 ;
	// cout << "fx="<< camera.fx << " fy="<<camera.fy<<" cx="<<camera.cx<<" cy="<<camera.cy<<endl;



    cout<<"solving pnp"<<endl;
    // solve pnp
    RESULT_OF_PNP result = estimateMotion( frame1, frame2, camera );
    
    if(result.error_flag) return;

    cout<<result.rvec<<endl<<result.tvec<<endl;

    // Calculate whether the range of motion is too large
    double norm = normofTransform(result.rvec, result.tvec);
    cout<<"norm = "<<norm<<endl;
    if ( norm >= 0.3 ) return; 


    Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );
    cout<<"T="<<T.matrix()<<endl;





    // Merge point clouds
    cout<<"combining clouds"<<endl;

    if(cnt==1){
        PointCloud::Ptr cloud1 = image2PointCloud( frame1.rgb, frame1.depth, camera );
        output = joinPointCloud( cloud1, frame2, T, camera );
    }
    else{
        output = joinPointCloud( output, frame2, T, camera );
    }

    // produce a point cloud
    output->header = pcl_conversions::toPCL(image_depth_msg->header);
    output->height = 1;
    output->width = output->points.size();
    pointcloud_pub.publish (output);


    cv::waitKey( 100 );


}

int main(int argc, char** argv) {
	ros::init(argc, argv, "vision_node");

	ros::NodeHandle nh;

	message_filters::Subscriber<Image> image_color_sub(nh,"/camera/color/image_raw", 1);
	message_filters::Subscriber<Image> image_depth_sub(nh,"/camera/depth/image_rect_raw", 1);
	message_filters::Subscriber<CameraInfo> info_color_sub(nh,"/camera/color/camera_info", 1);
	message_filters::Subscriber<CameraInfo> info_depth_sub(nh,"/camera/depth/camera_info", 1);
	pointcloud_pub = nh.advertise<PointCloud> ("/output", 100);

	typedef sync_policies::ApproximateTime<Image, Image, CameraInfo, CameraInfo> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_color_sub, image_depth_sub, info_color_sub, info_depth_sub);

	sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

	




	
	ros::spin();

	return 0;
}



double  normofTransform (cv :: Mat rvec, cv :: Mat tvec)
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}