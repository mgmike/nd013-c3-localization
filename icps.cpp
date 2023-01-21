#include "icps.h"

ICPS::ICPS(PointCloudT::Ptr t, Pose sp, int iter, int dist = 2): startingPose(sp), iterations(iter), dist(dist) {
  target = t;
}

void ICPS::set_map(PointCloudT::Ptr t){
    target = t;
} 

vector<int> NN(pcl::KdTreeFLANN<PointT> kdtree, PointCloudT::Ptr source, double dist){
	
	vector<int> associations;

	// This function returns a vector of target indicies that correspond to each source index inorder.
	// E.G. source index 0 -> target index 32, source index 1 -> target index 5, source index 2 -> target index 17, ... 

	// Loop through each transformed source point and using the KDtree find the transformed source point's nearest target point. Append the nearest point to associaitons 
	int i = 0;
	for (PointT pt : source->points){

		vector<int> pointIdxRadiusSearch;
		vector<float> pointRadiusSquaredDistance;
		if (kdtree.radiusSearch(pt, dist, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
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

Eigen::Matrix4d ICPS::get_transform(PointCloudT::Ptr source){
  
  // Create a KDtree once with target as input
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(target);

  // Transform source by startingPose
  transformation_matrix_old = transform3D(startingPose.rotation.yaw, startingPose.rotation.pitch, startingPose.rotation.roll, 
                                              startingPose.position.x, startingPose.position.y, startingPose.position.z);
  transformation_matrix = transformation_matrix_old;

  PointCloudT::Ptr transformSourceOld(source);

  //std::cout << "TSO: " << transformation_matrix_old << std::endl;

  vector<Pair> pairs;

  for (int j = 0; j < iterations; j++){
    // Update old point cloud and delete 
    // if (j > 0){
    // 	delete transformSourceOld;  
    // }
    // transformSourceOld = transformSource;
    // transformSource = (new PointCloudT);

    PointCloudT::Ptr transformSource(new PointCloudT);
    
    pcl::transformPointCloud(*transformSourceOld, *transformSource, transformation_matrix);

    transformSourceOld = transformSource;

    pairs = PairPoints(NN(kdtree, transformSource, dist), target, transformSource);


    // Create matrices P and Q which are both 3 x 1 and represent mean point of pairs 1 and pairs 2 respectivley.
    // In other words P is the mean point of source and Q is the mean point target 
    // P = [ mean p1 x] Q = [ mean p2 x]
    //     [ mean p1 y]     [ mean p2 y]
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
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(S, Eigen::ComputeFullV | Eigen::ComputeFullU);
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
  
    // Update transformation matricies
  
    //std::cout << "TSO: " << transformation_matrix_old << std::endl << std::endl << " TS: " << transformation_matrix << std::endl;
    transformation_matrix = transformation_matrix * transformation_matrix_old;
    transformation_matrix_old = transformation_matrix;
  }

  return transformation_matrix;

}