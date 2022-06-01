# Localization

 To enable rapid tests, I added the ability to change variables from input arguments. These variables can change the lidar blueprint parameters, the type of scan matching algorithm, either ICP or NDT, the amount of iterations for each and the voxel leaf size for NDT and the cp size.  

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
