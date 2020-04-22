  
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

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h> 
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>


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


/* ******************************
// Added: initialization of g2o
****************************** */
// Select optimization method
typedef g2o::BlockSolver_6_3 SlamBlockSolver; 
typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver; 



g2o::SparseOptimizer globalOptimizer;   // The last thing used is this stuff






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

        computeKeyPointsAndDesp (frame2);

        // Add the first vertex to globalOptimizer
        g2o::VertexSE3* v = new g2o::VertexSE3();
        v-> setId (cnt);
        v-> setEstimate ( Eigen :: Isometry3d :: Identity ()); // estimated as identity matrix
        v-> setFixed ( true ); // The first vertex is fixed, no optimization
        globalOptimizer. addVertex (v);


        frame1 = frame2;

		cnt++;
		return;
	}
	else
	{
		frame1 = frame2;

		frame2.rgb = image_color;
		frame2.depth = image_depth;
        
	}
	


    // Extract features and calculate descriptors
    cout<<"extracting features"<<endl;

    //if(cnt==1)computeKeyPointsAndDesp (frame1);
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




    // Add the edge of this vertex to the previous frame to g2o
    // Vertex part
    // The vertex only needs to set the id
    g2o::VertexSE3 *v = new g2o::VertexSE3();
    v-> setId (cnt);
    v-> setEstimate ( Eigen :: Isometry3d :: Identity ());
    globalOptimizer. addVertex (v);

    // side part
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    // Connect the two vertex ids on this side
    edge->vertices() [0] = globalOptimizer.vertex( cnt-1 );
    edge->vertices() [1] = globalOptimizer.vertex( cnt );

    // Information matrix
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
    // Information matrix is ​​the inverse of the covariance matrix, indicating our pre-estimation of edge accuracy
    // Because the pose is 6D, the information matrix is ​​a 6 * 6 matrix, assuming that the estimation accuracy of position and angle are both 0.1 and independent of each other
    // Then the covariance is a matrix with a diagonal of 0.01, and the information matrix is ​​a matrix of 100
    information(0,0) = information(1,1) = information(2,2) = 100;
    information(3,3) = information(4,4) = information(5,5) = 100;
    // You can also set the angle larger, indicating that the angle estimate is more accurate
    edge->setInformation( information );
    // The edge estimate is the result of pnp solution
    edge->setMeasurement( T );
    // Add this edge to the graph
    globalOptimizer.addEdge(edge);

    if(((cnt*100)%100)==0)
    {
        cout<<"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
        globalOptimizer.initializeOptimization();
        globalOptimizer. optimize ( 100 ); // You can specify the number of optimization steps
        cout<<"Optimization done."<<endl;
    }



    cv::waitKey( 100 );

    cnt++;
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

	// Initialize the solver
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );

    globalOptimizer.setAlgorithm( solver ); 
    // Do not output debugging information
    globalOptimizer. setVerbose ( false );



	
	ros::spin();

	return 0;
}



double  normofTransform (cv :: Mat rvec, cv :: Mat tvec)
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}