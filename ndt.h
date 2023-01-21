#include "scan_matching.h"
#include <pcl/registration/ndt.h>

class NDT : public Scan_Matching
{
public:
	Pose startingPose;
	int iterations;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    NDT(PointCloudT::Ptr target, Pose startingPose, int iterations);
	void set_map(PointCloudT::Ptr target);
	Eigen::Matrix4d get_transform(PointCloudT::Ptr source);
};