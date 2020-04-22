

#include "slamBase.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// OpenCV feature detection module
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/nonfree/features2d.hpp> // use this if you want to use SIFT or SURF
//#include <opencv2/calib3d/calib3d.hpp>



#include "slam/PnP_result.h"





// Note: for colorized point cloud, use PointXYZRGBA
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher pointcloud_pub;
ros::Publisher PnP_result_pub;




bool cnt = 0;
cv::Mat rgb1, rgb2, depth1, depth2;

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
		rgb2 = image_color;
		depth2 = image_depth;
		cnt++;
		return;
	}
	else
	{
		rgb1 = rgb2;
		depth1 = depth2;
		rgb2 = image_color;
		depth2 = image_depth;
	}
	
	


    // Declaration feature extractor and descriptor extractor
    cv::Ptr <cv::FeatureDetector> detector;
    cv::Ptr <cv::DescriptorExtractor> descriptor;



    // Build the extractor, both are ORB by default
    
    // If using sift, surf, initialize the nonfree module before
    //cv::initModule_nonfree();
    // _detector = cv::SIFT::create();
    // _descriptor = cv::SIFT::create();
    
	detector = cv::ORB::create();
	descriptor = cv::ORB::create();

    vector<cv::KeyPoint> kp1, kp2; // Key points
    detector->detect (rgb1, kp1);   // Extract key points
    detector->detect( rgb2, kp2 );

    cout<<"Key points of two images: "<<kp1.size()<<", "<<kp2.size()<<endl;

    // Visualize and display key points
    cv::Mat imgShow;
    cv::drawKeypoints( rgb1, kp1, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    cv::imshow( "keypoints", imgShow );
    //cv :: waitKey ( 0 ); // Pause waiting for a key

    // Calculate descriptors
    cv::Mat desp1, desp2;
    descriptor->compute( rgb1, kp1, desp1 );
    descriptor->compute( rgb2, kp2, desp2 );

    // match descriptor
    vector<cv::DMatch > matches_vector; 
    cv::BFMatcher matches;
    matches.match (desp1, desp2, matches_vector);
    cout<<"Find total "<<matches_vector.size()<<" matches."<<endl;

    // Visualization: show matching features
    cv :: Mat imgMatches;
    cv :: drawMatches (rgb1, kp1, rgb2, kp2, matches_vector, imgMatches);
    cv::imshow( "matches", imgMatches );


    // Filter matches and remove too much distance
    // The criterion used here is to remove matches greater than four times the minimum distance
    vector< cv::DMatch > goodMatches;
    double minDis = 9999;
    for ( size_t i=0; i<matches_vector.size(); i++ )
    {
        if ( matches_vector[i].distance < minDis )
            minDis = matches_vector[i].distance;
    }
    cout<<"min dis = "<<minDis<<endl;

    for ( size_t i=0; i<matches_vector.size(); i++ )
    {
        if (matches_vector[i].distance < 10*minDis)
            goodMatches.push_back( matches_vector[i] );
    }

    // Show good matches
    cout<<"good matches="<<goodMatches.size()<<endl;
    cv :: drawMatches (rgb1, kp1, rgb2, kp2, goodMatches, imgMatches);
    cv::imshow( "good matches", imgMatches );



    // Calculate the motion relationship between images
    // Key function: cv :: solvePnPRansac ()
    // Prepare necessary parameters for calling this function
    
    // 3D point of the first frame
    vector<cv::Point3f> pts_obj;
    // Image point of the second frame
    vector< cv::Point2f > pts_img;



    // Camera internal parameters
    CAMERA_INTRINSIC_PARAMETERS C;
    C.cx = info_depth_msg->K[0];
    C.cy = info_depth_msg->K[4];
    C.fx = info_depth_msg->K[2];
    C.fy = info_depth_msg->K[2];
    C.scale = 1000.0 ;
	cout << "fx="<< C.fx << " fy="<<C.fy<<" cx="<<C.cx<<" cy="<<C.cy<<endl;



	for (size_t i=0; i<goodMatches.size(); i++)
	{
		// query is the first and train is the second
		cv::Point2f p = kp1[goodMatches[i].queryIdx].pt;
		// Be careful about getting d! x is right, y is down, so y is the row, x is the column!
		ushort d = depth1.ptr<ushort>( int(p.y) )[ int(p.x) ];
		if (d == 0)
			continue;
		pts_img.push_back( cv::Point2f( kp2[goodMatches[i].trainIdx].pt ) );

		// Convert (u, v, d) to (x, y, z)
		cv::Point3f pt ( p.x, p.y, d );
		cv::Point3f pd = point2dTo3d( pt, C );
		pts_obj.push_back( pd );
	}



    double camera_matrix_data[3][3] = {
        {C.fx, 0, C.cx},
        {0, C.fy, C.cy},
        { 0 , 0 , 1 }
    };

	cout<<"pts_obj.size(): "<<pts_obj.size()<<endl;
	cout<<"pts_img.size(): "<<pts_img.size()<<endl;

	if(pts_obj.size() < 5 || pts_img.size() < 5 ) return;

    // Build the camera matrix
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;
    // solve pnp
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.99, inliers );


    cout<<"inliers: "<<inliers.rows<<endl;
    cout<<"R="<<rvec<<endl;
    cout<<"t="<<tvec<<endl;


    // Draw inliers matches
    vector< cv::DMatch > matchesShow;
    for (size_t i=0; i<inliers.rows; i++)
    {
        matchesShow.push_back( goodMatches[inliers.ptr<int>(i)[0]] );    
    }
    cv::drawMatches (rgb1, kp1, rgb2, kp2, matchesShow, imgMatches);
    cv::imshow( "inlier matches", imgMatches );


	slam::PnP_result PnP_result_msg;
	//PnP_result_msg.R_1 = rvec.ptr<float>(0);
	//float* data = (float*)rvec.data;
	//cout<<"rvec="<<data[0]<<endl;
	double *data; 
	data = (double*)rvec.data;
	PnP_result_msg.R_1 = data[0];
	PnP_result_msg.R_2 = data[1];
	PnP_result_msg.R_3 = data[2];

	data = (double*)tvec.data;
	PnP_result_msg.t_1 = data[0];
	PnP_result_msg.t_2 = data[1];
	PnP_result_msg.t_3 = data[2];

	PnP_result_msg.inliers = inliers.rows;

	PnP_result_msg.cx = C.cx;
	PnP_result_msg.cy = C.cy;
	PnP_result_msg.fx = C.fx;
	PnP_result_msg.fy = C.fy;
	PnP_result_msg.scale = 1000;

	PnP_result_pub.publish(PnP_result_msg);

	//cout<<rvec.at<double>(0,0)<<endl;
	//cout<<data[0]<<endl;
	



    cv::waitKey( 100 );


}

int main(int argc, char** argv) {
	ros::init(argc, argv, "vision_node");

	ros::NodeHandle nh;

	message_filters::Subscriber<Image> image_color_sub(nh,"/camera/color/image_raw", 1);
	message_filters::Subscriber<Image> image_depth_sub(nh,"/camera/depth/image_rect_raw", 1);
	message_filters::Subscriber<CameraInfo> info_color_sub(nh,"/camera/color/camera_info", 1);
	message_filters::Subscriber<CameraInfo> info_depth_sub(nh,"/camera/depth/camera_info", 1);
	pointcloud_pub = nh.advertise<PointCloud> ("mypoints", 1);

	typedef sync_policies::ApproximateTime<Image, Image, CameraInfo, CameraInfo> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_color_sub, image_depth_sub, info_color_sub, info_depth_sub);

	sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

	
	PnP_result_pub = nh.advertise<slam::PnP_result>("/PnP_result_msg", 100);




	
	ros::spin();

	return 0;
}