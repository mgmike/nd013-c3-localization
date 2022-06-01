#include "scan_matching.h"

Eigen::Matrix4d ICP(PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose, int iterations){
 
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

	// 1. Transform the source to the startingPose
	Eigen::Matrix4d initTransform = transform3D(startingPose.rotation.yaw, startingPose.rotation.pitch, startingPose.rotation.roll, startingPose.position.x, startingPose.position.y, startingPose.position.z);
  	PointCloudT::Ptr transformSource (new PointCloudT);
  	pcl::transformPointCloud(*source, *transformSource, initTransform);
  
	//2. Create the PCL icp object
	pcl::console::TicToc time;
  	time.tic ();
  	pcl::IterativeClosestPoint<PointT, PointT> icp;
	//3. Set the icp object's values
  	icp.setMaximumIterations(iterations);
  	icp.setInputSource(transformSource);
  	icp.setInputTarget(target);
  	icp.setMaxCorrespondenceDistance(2);
  
	//4. Call align on the icp object
  	PointCloudT::Ptr tempSource (new PointCloudT);
  	icp.align(*tempSource);
  
  	if(icp.hasConverged()){
		//std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		transformation_matrix = icp.getFinalTransformation().cast<double>();
		transformation_matrix = transformation_matrix * initTransform;
		return transformation_matrix;
    }
  	std::cout << "WARNING: ICP did not converge" << std::endl;

	return transformation_matrix;
}

double Score(vector<int> pairs, PointCloudT::Ptr target, PointCloudT::Ptr source, Eigen::Matrix4d transform){
	double score = 0;
	int index = 0;
	for(int i : pairs){
		Eigen::MatrixXd p(4, 1);
		p(0,0) = (*source)[index].x;
		p(1,0) = (*source)[index].y;
		p(2,0) = 0.0;
		p(3,0) = 1.0;
		Eigen::MatrixXd p2 = transform * p;
		PointT association = (*target)[i];
		score += sqrt( (p2(0,0) - association.x) * (p2(0,0) - association.x) + (p2(1,0) - association.y) * (p2(1,0) - association.y) );
		index++;
	}
	return score;
}

vector<int> NN(PointCloudT::Ptr target, PointCloudT::Ptr source, Eigen::Matrix4d initTransform, double dist){
	
	vector<int> associations;

	// This function returns a vector of target indicies that correspond to each source index inorder.
	// E.G. source index 0 -> target index 32, source index 1 -> target index 5, source index 2 -> target index 17, ... 

	// Create a KDtree with target as input
	pcl::KdTreeFLANN<PointT> kdtree;
  	kdtree.setInputCloud(target);
  
	// Transform source by initTransform
  	PointCloudT::Ptr transformSource (new PointCloudT);
  	pcl::transformPointCloud(*source, *transformSource, initTransform);

	// Loop through each transformed source point and using the KDtree find the transformed source point's nearest target point. Append the nearest point to associaitons 
	int i = 0;
	for (PointT pt : transformSource->points){
		vector<int> pointIdxRadiusSearch;
		vector<float> pointRadiusSquaredDistance;
		if (kdtree.radiusSearch(pt, dist, pointIdxRadiusSearch, pointRadiusSquaredDistance) >= 0){
			associations.push_back(pointIdxRadiusSearch[0]);
        } else {
			associations.push_back(-1);
        }
		i++;
    }
  
	return associations;
}

vector<Pair> PairPoints(vector<int> associations, PointCloudT::Ptr target, PointCloudT::Ptr source){

	vector<Pair> pairs;

	// Loop through each source point and using the corresponding associations append a Pair of (source point, associated target point)
  	int i = 0;
	for (PointT pt : source->points){
		int ti = associations[i];
		if (ti >= 0){
			PointT association = (*target)[ti];
			Pair pair(Point(pt.x, pt.y, pt.z), Point(association.x, association.y, association.z));
			pairs.push_back(pair);
		}
      	i++;
    }

	return pairs;
}

Eigen::Matrix4d ICPS(PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose, int iterations){

  	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

  	// Transform source by startingPose
  	Eigen::Matrix4d initTransform = transform3D(startingPose.rotation.yaw, startingPose.rotation.pitch, startingPose.rotation.roll, 
                                                startingPose.position.x, startingPose.position.y, startingPose.position.z);
  	PointCloudT::Ptr transformSource(new PointCloudT);
  	pcl::transformPointCloud(*source, *transformSource, initTransform);
  
  	vector<int> associations = NN(target, source, initTransform, iterations);
  
  	vector<Pair> pairs = PairPoints(associations, target, transformSource);
  
  	// Create matrices P and Q which are both 3 x 1 and represent mean point of pairs 1 and pairs 2 respectivley.
  	// In other words P is the mean point of source and Q is the mean point target 
  	// P = [ mean p1 x] Q = [ mean p2 x]
  	//	   [ mean p1 y]	    [ mean p2 y]
  	//     [ mean p1 z]     [ mean p2 z]
  	Eigen::MatrixXd P(3, 1);
  	Eigen::MatrixXd Q(3, 1);
  	P << Eigen::MatrixXd::Zero(3, 1);
  	Q << Eigen::MatrixXd::Zero(3, 1);
  
  	for (Pair pair: pairs){
      	P(0, 0) += pair.p1.x;
      	P(1, 0) += pair.p1.y;
      	P(2, 0) += pair.p1.z;
      	Q(0, 0) += pair.p2.x;
      	Q(1, 0) += pair.p2.y;
      	Q(2, 0) += pair.p2.z;
    }
  	P(0, 0) = P(0, 0) / pairs.size();
  	P(1, 0) = P(1, 0) / pairs.size();
  	P(2, 0) = P(2, 0) / pairs.size();
  	Q(0, 0) = Q(0, 0) / pairs.size();
  	Q(1, 0) = Q(1, 0) / pairs.size();
  	Q(2, 0) = Q(2, 0) / pairs.size();

  	// Get pairs of points from PairPoints and create matrices X and Y which are both 3 x n where n is number of pairs.
  	// X is pair 1 x point with pair 2 x point for each column and Y is the same except for y points
  	// X = [p1 x0 , p1 x1 , p1 x2 , .... , p1 xn ] - [Px]   Y = [p2 x0 , p2 x1 , p2 x2 , .... , p2 xn ] - [Qx]
  	//     [p1 y0 , p1 y1 , p1 y2 , .... , p1 yn ]   [Py]       [p2 y0 , p2 y1 , p2 y2 , .... , p2 yn ]   [Qy]
  	//     [p1 z0 , p1 z1 , p1 z2 , .... , p1 zn ]   [Pz]       [p2 z0 , p2 z1 , p2 z2 , .... , p2 zn ]   [Qz]
  
  	Eigen::MatrixXd X(3, pairs.size());
  	Eigen::MatrixXd Y(3, pairs.size());
  	int i = 0;
  	for (Pair pair: pairs){
    	X(0, i) = pair.p1.x - P(0, 0);
    	X(1, i) = pair.p1.y - P(1, 0);
    	X(2, i) = pair.p1.z - P(2, 0);
    	Y(0, i) = pair.p2.x - Q(0, 0);
    	Y(1, i) = pair.p2.y - Q(1, 0);
    	Y(2, i) = pair.p2.z - Q(2, 0);
      	i++;
    }

  	// Create matrix S using equation 3 from the svd_rot.pdf. Note W is simply the identity matrix because weights are all 1
  	Eigen::MatrixXd S = X * Y.transpose();

  	// Create matrix R, the optimal rotation using equation 4 from the svd_rot.pdf and using SVD of S
  	JacobiSVD<MatrixXd> svd(S, ComputeFullV | ComputeFullU);
  	int n = svd.matrixV().cols();
  	Eigen::MatrixXd M = Eigen::MatrixXd::Identity(n, n);
  	M(n - 1, n - 1) = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  	Eigen::MatrixXd R = svd.matrixV() * M * svd.matrixU().transpose();
  
  	// Create mtarix t, the optimal translatation using equation 5 from svd_rot.pdf
  	Eigen::MatrixXd t = Q - R * P;

  	// Set transformation_matrix based on above R, and t matrices
  	// [ R R R t]
  	// [ R R R t]
  	// [ R R R t]
  	// [ 0 0 0 1]
  	transformation_matrix(0, 0) = R(0, 0);
  	transformation_matrix(0, 1) = R(0, 1);
  	transformation_matrix(0, 2) = R(0, 2);
  	transformation_matrix(1, 0) = R(1, 0);
  	transformation_matrix(1, 1) = R(1, 1);
  	transformation_matrix(1, 2) = R(1, 2);
  	transformation_matrix(2, 0) = R(2, 0);
  	transformation_matrix(2, 1) = R(2, 1);
  	transformation_matrix(2, 2) = R(2, 2);
  	transformation_matrix(0, 3) = t(0, 0);
  	transformation_matrix(1, 3) = t(1, 0);
  	transformation_matrix(2, 3) = t(2, 0);

	transformation_matrix = transformation_matrix * initTransform;

  	return transformation_matrix;

}

Eigen::Matrix4d NDT(pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt, PointCloudT::Ptr source, Pose startingPose, int iterations){

  	Eigen::Matrix4f init_guess = transform3D(startingPose.rotation.yaw, startingPose.rotation.pitch, startingPose.rotation.roll, startingPose.position.x, startingPose.position.y, startingPose.position.z).cast<float>();
  
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ndt(new pcl::PointCloud<pcl::PointXYZ>);
  	
  	ndt.setMaximumIterations(iterations);
  	ndt.setInputSource(source);
  	ndt.align(*cloud_ndt, init_guess);
  	Eigen::Matrix4d transformation_matrix = ndt.getFinalTransformation().cast<double>();
  
  	return transformation_matrix;
}

