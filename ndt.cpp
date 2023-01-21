#include "ndt.h"

NDT::NDT(PointCloudT::Ptr t, Pose sp, int iter): startingPose(sp), iterations(iter) {
	target = t;
    ndt.setTransformationEpsilon(0.0001);
    ndt.setInputTarget(target);
    ndt.setResolution(1);
    ndt.setStepSize(1);
}

void NDT::set_map(PointCloudT::Ptr t){
    target = t;
} 

Eigen::Matrix4d NDT::get_transform(PointCloudT::Ptr source){

  	Eigen::Matrix4f init_guess = transform3D(startingPose.rotation.yaw, startingPose.rotation.pitch, startingPose.rotation.roll, startingPose.position.x, startingPose.position.y, startingPose.position.z).cast<float>();
  
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ndt(new pcl::PointCloud<pcl::PointXYZ>);
  	
  	ndt.setMaximumIterations(iterations);
  	ndt.setInputSource(source);
  	ndt.align(*cloud_ndt, init_guess);
  	Eigen::Matrix4d transformation_matrix = ndt.getFinalTransformation().cast<double>();
  
  	return transformation_matrix;
}