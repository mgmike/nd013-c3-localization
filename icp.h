#include "scan_matching.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   // TicToc

class ICP : public Scan_Matching
{
public:
	Pose startingPose;
	int iterations;
    Eigen::Matrix4d transformation_matrix;
    Eigen::Matrix4d initTransform;

    ICP(PointCloudT::Ptr target, Pose startingPose, int iterations);
	void set_map(PointCloudT::Ptr target);
	Eigen::Matrix4d get_transform(PointCloudT::Ptr source);
};