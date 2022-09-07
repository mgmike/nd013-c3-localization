# Localization

## My implimentation

 I was able to impliment icp and ndt successfully by using the pcl libraries IterativeClosestPoint and NormalDistributionsTransform. The following gif shows a successful run with icp at speed 3. 

 ![Localization ICP](images/Localization1.gif)

 The next gif shows a successful run with ndt at speeds 4 then 3.

 ![Localization NDT](images/Localization_ndt_4-3.gif)

 I also scanned a larger map to use as input. Due to the way larger size of this new map, localization is a lot slower, and was unable to localize as effiecntly as previously. I plan on reducing the target map to only contain points a certain distance away from the vehicle's last known position to reduce localization costs. 

 I moved these scan matching algorithms into the new files scan_matching.cpp and scan_matching.h. I also converted the 2D non pcl ICP from the exercise to 3D and moved the initialization of the transformed source point cloud, associations and pairs into a for loop in an attempt to implement iterations of the ICP algorithm. The gif below shows that my ICP implimentation from scratch runs without error, however it is not able to localize. I have been pretty thorough in where the point cloud pointers point and how the tranformation matricies are populated, however I was unable to figure out why this is.

 ![Localization on larger map](images/Localization_ndt_3_loop.gif)

 The last change I made to the provided code was the addition of command line input arguments to change hyper-parameters. This was necessary to avoid having to recompile the code every time I wanted to make a change. These variables can change the type of scan matching algorithm, either ICP or NDT and the amount of iterations for each, the lidar blueprint parameters, the target map, the maximum distance ICP Nearest Neighbour function will use to find target points near each source point, and the cp_size and voxel leaf size for NDT. 


## Creating ICP from scratch:

1. Nearest neighbor associations for each source point and its nearest target point
    - Multiple source points can have the same target point as its nearest neighbor
    - Create a KDtree (similar to BST) from target
    - Transform source by initial position transform
    - For every point, use radiusSearch to find the index of the nearest target point
    - Return associations where the index of associations is the index of the source point and the value is the index of the target
    - Associations[5] == 17, source[5] -> target[17]
2. Turn the list of associations into a list of Pair objects (which contain 2 PointT objects) and return the list of pairs
3. ICP https://igl.ethz.ch/projects/ARAP/svd_rot.pdf
    - Calculate P dx1, 2x1 (source) and Q dx1, 2x1 (associated target) average matrices  
    - Populate X dxn, 2xnumber of source points and Y dxn, 2xnumber of associated target points
    - Calculate S = X * Y^T
    - Use Jacobi SVD to find V and U
    - Initialize Identity matrix, but set the last value to the determinate as V * M (or D) * U^T
    - Calculate rotation matrix R dxd, 2x2 = V * M (or D) U^T
    - Calculate movement matrix t dx1 2x1 = Q - R * P
    - Populate transformation matrix with R and t
    - Transform transformation matrix by initTransform
