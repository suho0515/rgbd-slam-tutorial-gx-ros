  
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
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

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


// Check two frames, result definition
enum CHECK_RESULT {NOT_MATCHED=0, TOO_FAR_AWAY, TOO_CLOSE, KEYFRAME}; 
// Function declaration
CHECK_RESULT checkKeyframes( FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops=false );
// Detect close loops
void checkNearbyLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti );
// Random detection loopback
void checkRandomLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti );






// Camera internal parameters
CAMERA_INTRINSIC_PARAMETERS camera;


// All key frames are here
vector< FRAME > keyframes; 


double keyframe_threshold = 0.1;
bool check_loop_closure = true;



bool toggle = false;


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

        camera.cx = info_depth_msg->K[0];
        camera.cy = info_depth_msg->K[4];
        camera.fx = info_depth_msg->K[2];
        camera.fy = info_depth_msg->K[2];
        camera.scale = 1000.0 ;
        // cout << "fx="<< camera.fx << " fy="<<camera.fy<<" cx="<<camera.cx<<" cy="<<camera.cy<<endl;

		frame2.rgb = image_color;
		frame2.depth = image_depth;

        computeKeyPointsAndDesp (frame2);

        // Initialize the solver
        SlamLinearSolver* linearSolver = new SlamLinearSolver();
        linearSolver->setBlockOrdering(false);
        SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );

        globalOptimizer.setAlgorithm( solver ); 
        // Do not output debugging information
        globalOptimizer.setVerbose ( false );

        // Add the first vertex to globalOptimizer
        g2o::VertexSE3* v = new g2o::VertexSE3();
        v-> setId (cnt);
        v-> setEstimate ( Eigen :: Isometry3d :: Identity ()); // estimated as identity matrix
        v-> setFixed ( true ); // The first vertex is fixed, no optimization
        globalOptimizer. addVertex (v);

        keyframes.push_back( frame2 );

        

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
    //cout<<"extracting features"<<endl;

    //if(cnt==1)computeKeyPointsAndDesp (frame1);
    computeKeyPointsAndDesp (frame2);

    CHECK_RESULT result = checkKeyframes (keyframes.back(), frame2, globalOptimizer); // Match this frame with the last frame in keyframes

    switch (result) // According to different matching results, adopt different strategies
    {
    case NOT_MATCHED:
        // No match, just skip
        cout<<"Not enough inliers."<<endl;
        break;
    case TOO_FAR_AWAY:
        // Too close, just jump
        cout<<"Too far away, may be an error."<<endl;
        break;
    case TOO_CLOSE:
        // It's too far, maybe something went wrong
        cout<<"Too close, not a keyframe"<<endl;
        break;
    case KEYFRAME:
        cout<<"This is a new keyframe"<<endl;
        // Not far away, just right
        // detect loopback
        if (check_loop_closure)
        {
            checkNearbyLoops (keyframes, frame2, globalOptimizer);
            checkRandomLoops( keyframes, frame2, globalOptimizer );
        }
        keyframes.push_back( frame2 );
        
        cnt++;
        if(toggle == true) toggle = false;

        break;
    default:
        break;
    }

    cout << "cnt : " << cnt << endl;

    if((cnt+10)%10==0 && toggle == false)
    {
        // Optimize
        cout<<"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
        globalOptimizer.initializeOptimization();
        globalOptimizer. optimize ( 100 ); // You can specify the number of optimization steps
        cout<<"Optimization done."<<endl;

        // Splice point cloud map
        cout<<"saving the point cloud map..."<<endl;
        PointCloud::Ptr tmp ( new PointCloud() );

        pcl :: VoxelGrid <PointT> voxel; // Grid filter, adjust the map resolution
        pcl :: PassThrough <PointT> pass; // Z-direction interval filter, because the effective depth interval of rgbd camera is limited, remove too far
        pass.setFilterFieldName("z");
        pass.setFilterLimits ( 0.0 , 4.0 ); // No more than 4m

        double gridsize = 0.01; // The resolution graph can be adjusted in parameters.txt
        voxel.setLeafSize( gridsize, gridsize, gridsize );

        for (size_t i=0; i<keyframes.size(); i++)
        {
            // Remove a frame from g2o
            g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex( keyframes[i].frameID ));
            Eigen :: Isometry3d pose = vertex-> estimate (); // The optimized pose of the frame
            PointCloud::Ptr newCloud = image2PointCloud( keyframes[i].rgb, keyframes[i].depth, camera ); //转成点云
            // The following is filtering
            voxel.setInputCloud( newCloud );
            voxel.filter( *tmp );
            pass.setInputCloud( tmp );
            pass.filter( *newCloud );
            // Add the point cloud to the global map after transformation
            pcl::transformPointCloud( *newCloud, *tmp, pose.matrix() );
            *output += *tmp;
            tmp->clear();
            newCloud->clear();
        }

        voxel.setInputCloud( output );
        voxel.filter( *tmp );

        // produce a point cloud
        output->header = pcl_conversions::toPCL(image_depth_msg->header);
        output->height = 1;
        output->width = output->points.size();
        pointcloud_pub.publish (output);

        toggle = true;
    }



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



CHECK_RESULT checkKeyframes( FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops)
{
    static int min_inliers = 5;
    static double max_norm = 0.2;
    static double keyframe_threshold = 0.1;
    static double max_norm_lp = 2.0;
    static CAMERA_INTRINSIC_PARAMETERS _camera = camera;
    // Compare f1 and f2
    RESULT_OF_PNP result = estimateMotion( f1, f2, _camera );
    if (result. inliers <min_inliers) // inliers is not enough, discard the frame
        return NOT_MATCHED;
    // Calculate whether the range of motion is too large
    double norm = normofTransform(result.rvec, result.tvec);

    cout << "norm : " << norm << endl;

    if ( is_loops == false )
    {
        if ( norm >= max_norm )
            return TOO_FAR_AWAY;   // too far away, may be error
    }
    else
    {
        if ( norm >= max_norm_lp)
            return TOO_FAR_AWAY;
    }

    if ( norm <= keyframe_threshold )
        return TOO_CLOSE;   // too adjacent frame
    // Add the edge of this vertex to the previous frame to g2o
    // Vertex part
    // The vertex only needs to set the id
    if (is_loops == false)
    {
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( f2.frameID );
        v-> setEstimate ( Eigen :: Isometry3d :: Identity ());
        opti. addVertex (v);
    }
    // side part
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    // Connect the two vertex ids on this side
    edge->setVertex( 0, opti.vertex(f1.frameID ));
    edge->setVertex( 1, opti.vertex(f2.frameID ));
    edge->setRobustKernel( new g2o::RobustKernelHuber() );
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
    Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );
    // edge->setMeasurement( T );
    edge->setMeasurement( T.inverse() );
    // Add this edge to the graph
    opti.addEdge(edge);
    return KEYFRAME;
}

void checkNearbyLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti )
{
    static int nearby_loops = 5;
    
    // Just test the last few of currFrame and frames
    if ( frames.size() <= nearby_loops )
    {
        // no enough keyframes, check everyone
        for (size_t i=0; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true );
        }
    }
    else
    {
        // check the nearest ones
        for (size_t i = frames.size()-nearby_loops; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true );
        }
    }
}

void checkRandomLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti )
{
    static int random_loops = 5;
    srand( (unsigned int) time(NULL) );
    // Randomly take some frames for detection
    
    if ( frames.size() <= random_loops )
    {
        // no enough keyframes, check everyone
        for (size_t i=0; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true );
        }
    }
    else
    {
        // randomly check loops
        for (int i=0; i<random_loops; i++)
        {
            int  index = rand ()% frames. size ();
            checkKeyframes( frames[index], currFrame, opti, true );
        }
    }
}