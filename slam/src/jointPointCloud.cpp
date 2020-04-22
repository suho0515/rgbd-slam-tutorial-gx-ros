// for multiple subscriber
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// for image processing
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>


#include <iostream>

#include <ros/ros.h>

// for pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

// OpenCV feature detection module
#include <opencv2/features2d/features2d.hpp>

// for header
#include "slamBase.h"


#include <opencv2/core/eigen.hpp>

#include <vector>




// Note: for colorized point cloud, use PointXYZRGBA
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher pointcloud_pub;





int cnt = 0;



// Declare two frames, see include / slamBase.h for FRAME structure
FRAME frame1, frame2;


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
		frame1.rgb = frame2.rgb;
		frame1.depth = frame2.depth;
		frame2.rgb = image_color;
		frame2.depth = image_depth;
        cnt++;
        if(cnt>3) cnt=3;
	}
	
    // Extract features and calculate descriptors
    cout<<"extracting features"<<endl;

    computeKeyPointsAndDesp (frame1);
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

    // Process result
    // Convert the rotation vector into a rotation matrix
    cv :: Mat R;
    cv::Rodrigues( result.rvec, R );
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);

    // Convert the translation vector and rotation matrix into a transformation matrix
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    

    cout<<"translation"<<endl;
    Eigen::AngleAxisd angle(r);
    Eigen::Translation<double,3> trans(result.tvec.at<double>(0,0), result.tvec.at<double>(0,1), result.tvec.at<double>(0,2));
    T = angle;
    T(0,3) = result.tvec.at<double>(0,0); 
    T(1,3) = result.tvec.at<double>(0,1); 
    T(2,3) = result.tvec.at<double>(0,2);
    
    // Convert the point cloud
    cout<<"converting image to clouds"<<endl;
    PointCloud::Ptr cloud1 = image2PointCloud( frame1.rgb, frame1.depth, camera );
    PointCloud::Ptr cloud2 = image2PointCloud( frame2.rgb, frame2.depth, camera );



    // Merge point clouds
    cout<<"combining clouds"<<endl;

    if(cnt==1){
        pcl::transformPointCloud( *cloud1, *output, T.matrix() );
        *output += *cloud2;
    }
    else{
        PointCloud::Ptr output_2 (new PointCloud());
        pcl::transformPointCloud( *output, *output_2, T.matrix() );
        *output_2 += *cloud2;
        *output = *output_2;
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