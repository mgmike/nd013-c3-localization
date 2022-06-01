#include <pcl/io/pcd_io.h>
#include <Eigen/Geometry>
#include <Eigen/SVD>
using namespace Eigen;
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>   // TicToc

#include "helper.h"

struct Pair{

	Point p1;
	Point p2;

	Pair(Point setP1, Point setP2)
		: p1(setP1), p2(setP2){}
};

Eigen::Matrix4d ICP(PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose, int iterations);
Eigen::Matrix4d ICPS(PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose, int iterations);
Eigen::Matrix4d NDT(pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt, PointCloudT::Ptr ssource, Pose startingPose, int iterations);
