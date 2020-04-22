/* ************************************************ ************************
	> File Name: src / slamBase.cpp
	> Author: xiang gao
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
    > Implementation of slamBase.h
	> Created Time: Saturday, July 18, 2015 15:31:49
************************************************** ********************* */

#include "slamBase.h"

// for opencv
#include <opencv2/core/eigen.hpp>

PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    PointCloud::Ptr cloud ( new PointCloud );

    for (int m = 0; m < depth.rows; m += 4)
        for (int n=0; n < depth.cols; n += 4)
        {
            // Get the value at (m, n) in the depth map
            ushort d = depth.ptr<ushort>(m)[n];
            // d may have no value, if so, skip this
            if (d == 0)
                continue;
            // d exists, add a point to the point cloud
            PointT p;

            // Calculate the spatial coordinates of this point
            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;
            
            // Get its color from the rgb image
            // rgb is a three-channel BGR format map, so get the colors in the following order
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // Add p to the point cloud
            cloud->points.push_back( p );
        }
    // Set and save the point cloud
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}

cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    cv :: Point3f p; // 3D point
    p.z = double( point.z ) / camera.scale;
    p.x = ( point.x - camera.cx) * p.z / camera.fx;
    p.y = ( point.y - camera.cy) * p.z / camera.fy;
    return p;
}

// computeKeyPointsAndDesp extracts both key points and feature descriptors
void computeKeyPointsAndDesp( FRAME& frame )
{
    cv::Ptr<cv::FeatureDetector> _detector;
    cv::Ptr<cv::DescriptorExtractor> _descriptor;

    _detector = cv::ORB::create();
	_descriptor = cv::ORB::create();

    _detector->detect( frame.rgb, frame.kp );
    _descriptor->compute( frame.rgb, frame.kp, frame.desp );

    return;
}

// estimateMotion calculates the motion between two frames
// Input: Frame 1 and Frame 2
// Output: rvec and tvec
RESULT_OF_PNP estimateMotion( FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    //static ParameterReader pd;
    vector<cv::DMatch> matches_vector;
    cv::BFMatcher matches;
    matches.match(frame1.desp , frame2.desp , matches_vector);
   
    //cout<<"find total "<<matches_vector.size()<<" matches."<<endl;
    vector<cv::DMatch> goodMatches;
    double minDis = 9999;
    double good_match_threshold = 10;
    for ( size_t i=0; i<matches_vector.size(); i++ )
    {
        if ( matches_vector[i].distance < minDis )
            minDis = matches_vector[i].distance;
    }

    //cout<<"min dis = "<<minDis<<endl;
    if ( minDis < 10 ) 
        minDis = 10 ;

    for ( size_t i=0; i<matches_vector.size(); i++ )
    {
        if (matches_vector[i].distance < good_match_threshold*minDis)
            goodMatches.push_back( matches_vector[i] );
    }

    //cout<<"good matches: "<<goodMatches.size()<<endl;

    RESULT_OF_PNP result;
    result.error_flag = false;
    if (goodMatches.size() <= 5) 
    {
        result.rvec = 0;
        result.tvec = 0;
        result.inliers = 0;
        result.error_flag = true;

        return result;
    }

    // 3D point of the first frame
    vector<cv::Point3f> pts_obj;
    // Image point of the second frame
    vector< cv::Point2f > pts_img;

    // Camera internal parameters
    for (size_t i=0; i<goodMatches.size(); i++)
    {
        // query is the first and train is the second
        cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;
        // Be careful about getting d! x is right, y is down, so y is the row, x is the column!
        ushort d = frame1.depth.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
        pts_img.push_back( cv::Point2f( frame2.kp[goodMatches[i].trainIdx].pt ) );

        // Convert (u, v, d) to (x, y, z)
        cv::Point3f pt ( p.x, p.y, d );
        cv::Point3f pd = point2dTo3d( pt, camera );
        pts_obj.push_back( pd );
    }

    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        { 0 , camera. fy , camera. cy },
        { 0 , 0 , 1 }
    };


    if(pts_obj.size() < 5 || pts_img.size() < 5)
    {
        result.rvec = 0;
        result.tvec = 0;
        result.inliers = 0;
        result.error_flag = true;

        return result;
    }
    else
    {

        //cout<<"solving pnp"<<endl;
        // Build the camera matrix
        cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
        cv :: Mat rvec, tvec, inliers;
        // solve pnp
        //cout << "pts_obj.size() : " << pts_obj.size() << endl;
        //cout << "pts_img.size() : " << pts_img.size() << endl;
        cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 8.0, 0.99, inliers );
        //cout << "inliers.rows : " << inliers.rows << endl;

        result.rvec = rvec;
        result.tvec = tvec;
        result.inliers = inliers.rows;
        result.error_flag = false;
        
        return result;
    }
    
}



// own cvMat2
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec )
{
    cv::Mat R;
    cv::Rodrigues(rvec,R);
    Eigen::Matrix3d r;
    cv::cv2eigen(R, r);
  
    // Convert the translation vector and rotation matrix into a transformation matrix
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    Eigen::Translation<double,3> trans(tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2));
    T = angle;
    T(0,3) = tvec.at<double>(0,0); 
    T(1,3) = tvec.at<double>(1,0); 
    T(2,3) = tvec.at<double>(2,0);
    return T;
}



// joinPointCloud 
// input: original point cloud, new frame and its pose
// output: the image after adding the new frame to the original frame
PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera ) 
{
    PointCloud::Ptr newCloud = image2PointCloud( newFrame.rgb, newFrame.depth, camera );

    // Merge point clouds
    PointCloud::Ptr output (new PointCloud());
    pcl::transformPointCloud( *original, *output, T.matrix() );
    *output += *newCloud;

    //return output;

    // Voxel grid filtering downsampling
    
    static pcl::VoxelGrid<PointT> voxel;
    double gridsize = 0.01;
    voxel.setLeafSize( gridsize, gridsize, gridsize );
    voxel.setInputCloud( output );
    PointCloud::Ptr tmp( new PointCloud() );
    voxel.filter( *tmp );
    return tmp;
    
}
