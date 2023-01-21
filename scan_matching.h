#include "helper.h"
#include <Eigen/Geometry>

#ifndef SCANMATCHING_H
#define SCANMATCHING_H

class Scan_Matching
{
public:
	PointCloudT::Ptr target;
	
	void set_map(PointCloudT::Ptr target);
	Eigen::Matrix4d get_transform(PointCloudT::Ptr source);
};

#endif