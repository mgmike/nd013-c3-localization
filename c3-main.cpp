
#include <thread>

//pcl code
//#include "render/render.h"


using namespace std::chrono_literals;
using namespace std::string_literals;

using namespace std;

#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include "helper.h"
#include "ndt.h"
#include "icp.h"
#include <sstream>
#include <chrono> 
#include <ctime> 
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>   // TicToc



bool refresh_view = false;

void drawCar(Pose pose, int num, Color color, double alpha, pcl::visualization::PCLVisualizer::Ptr& viewer){

	BoxQ box;
	box.bboxTransform = Eigen::Vector3f(pose.position.x, pose.position.y, 0);
    box.bboxQuaternion = getQuaternion(pose.rotation.yaw);
    box.cube_length = 4;
    box.cube_width = 2;
    box.cube_height = 2;
	renderBox(viewer, box, num, color, alpha);
}

int main(int argc, char* argv[]){
  	// I added the ability to change values through input params for rapid testing
  	enum Registration{ Off, Ndt, Icp};
  	Registration matching = Off;
	int iters = 10;
  	string upper_fov = "15";
	string lower_fov = "-25";
	string channels = "32";
	string range = "30";
	string rotation_frequency = "60";
	string points_per_second = "500000";
	string map_name = "map.pcd";
	int dist = 2;
  	int cp_size = 5000;
  	double leafSize = 0.5;
	bool need_to_write = true;

	Scan_Matching* scan_matching;
	PointCloudT pclCloud;
	std::chrono::time_point<std::chrono::system_clock> currentTime;
  
  	// Handle input
  	if (argc > 1){
    	if (strcmp(argv[1], "ndt") == 0){
          	matching = Ndt;
        } else if (strcmp(argv[1], "icp") == 0){
          	matching = Icp;
        } else {
        	return 0;
        }
		iters = stoi(argv[2]);
      	upper_fov = argv[3];
        lower_fov = argv[4];
        channels = argv[5];
        range = argv[6];
        rotation_frequency = argv[7];
        points_per_second = argv[8];
		map_name = argv[9];
		dist = stoi(argv[10]);
        cp_size = stoi(argv[11]);
        leafSize = stod(argv[12]);
      
    }
  	std::stringstream ss;
  	for (int i = 1; i < argc; i++){
  		ss << argv[i] << " ";
    }

	bool new_scan = true;
	std::chrono::time_point<std::chrono::system_clock> lastScanTime, startTime;

	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (0, 0, 0);

	Pose pose(Point(0,0,0), Rotate(0,0,0));

	// Load map
	PointCloudT::Ptr mapCloud(new PointCloudT);
  	pcl::io::loadPCDFile(map_name, *mapCloud);
  	cout << "Loaded " << mapCloud->points.size() << " data points from " << map_name << endl;
	renderPointCloud(viewer, mapCloud, "map", Color(0,0,1)); 
  
	if (matching == Ndt){
		scan_matching = new NDT(mapCloud, pose, iters);
	} else if (matching == Icp){
		scan_matching = new ICP(mapCloud, pose, iters);
	} else { return 0;}

	typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr scanCloud (new pcl::PointCloud<PointT>);

	// lidar->Listen([&new_scan, &lastScanTime, &scanCloud, cp_size](auto data){

	// 	if(new_scan){
	// 		auto scan = boost::static_pointer_cast<csd::LidarMeasurement>(data);
	// 		for (auto detection : *scan){
	// 			if((detection.point.x*detection.point.x + detection.point.y*detection.point.y + detection.point.z*detection.point.z) > 8.0){ // Don't include points touching ego
	// 				pclCloud.points.push_back(PointT(detection.point.x, detection.point.y, detection.point.z));
	// 			}
	// 		}
	// 		if(pclCloud.points.size() > cp_size){
	// 			lastScanTime = std::chrono::system_clock::now();
	// 			*scanCloud = pclCloud;
	// 			new_scan = false;
	// 		}
	// 	}
	// });
	
	Pose poseRef(Point(0.0, 0.0, 0.0), Rotate(0.0, 0.0, 0.0));
	double maxError = 0;

	while (!viewer->wasStopped())
  	{
		while(new_scan){
			std::this_thread::sleep_for(0.1s);
		}
		if(refresh_view){
			viewer->setCameraPosition(pose.position.x, pose.position.y, 60, pose.position.x+1, pose.position.y+1, 0, 0, 0, 1);
			refresh_view = false;
		}
		
		viewer->removeShape("box0");
		viewer->removeShape("boxFill0");
		Pose truePose = Pose(Point(1.0, 1.0, 1.0), Rotate(0.0, 0.0, 0.0)) - poseRef;
		drawCar(truePose, 0,  Color(1,0,0), 0.7, viewer);
		double theta = truePose.rotation.yaw;
		double stheta = 1 * pi/4 + theta;
		viewer->removeShape("steer");
		renderRay(viewer, Point(truePose.position.x+2*cos(theta), truePose.position.y+2*sin(theta),truePose.position.z),  Point(truePose.position.x+4*cos(stheta), truePose.position.y+4*sin(stheta),truePose.position.z), "steer", Color(0,1,0));


  		viewer->spinOnce ();
		
		if(!new_scan){
			
			new_scan = true;
			// Filter scan using voxel filter
          
          	pcl::VoxelGrid<PointT> vg;
          	vg.setInputCloud(scanCloud);
          	vg.setLeafSize(leafSize, leafSize, leafSize);
          	vg.filter(*cloudFiltered);

			// Find pose transform by using ICP or NDT matching
			Eigen::Matrix4d transform_sm;

			scan_matching->get_transform(mapCloud);

			pose = getPose(transform_sm);

			// Transform scan so it aligns with ego's actual pose and render that scan
          	PointCloudT::Ptr transformed_scan (new PointCloudT);
          	pcl::transformPointCloud(*cloudFiltered, *transformed_scan, transform_sm);
          

			viewer->removePointCloud("scan");
			renderPointCloud(viewer, transformed_scan, "scan", Color(1,0,0) );

			viewer->removeAllShapes();
			drawCar(pose, 1,  Color(0,1,0), 0.35, viewer);
          
          	double poseError = sqrt( (truePose.position.x - pose.position.x) * (truePose.position.x - pose.position.x) + (truePose.position.y - pose.position.y) * (truePose.position.y - pose.position.y) );
			if(poseError > maxError)
				maxError = poseError;
			double distDriven = sqrt( (truePose.position.x) * (truePose.position.x) + (truePose.position.y) * (truePose.position.y) );
			viewer->removeShape("maxE");
			viewer->addText("Max Error: "+to_string(maxError)+" m", 200, 100, 32, 1.0, 1.0, 1.0, "maxE",0);
			viewer->removeShape("derror");
			viewer->addText("Pose error: "+to_string(poseError)+" m", 200, 150, 32, 1.0, 1.0, 1.0, "derror",0);
			viewer->removeShape("dist");
			viewer->addText("Distance: "+to_string(distDriven)+" m", 200, 200, 32, 1.0, 1.0, 1.0, "dist",0);

			if(maxError > 1.2 || distDriven >= 170.0 ){
				viewer->removeShape("eval");
				if(maxError > 1.2){
					viewer->addText("Try Again", 200, 50, 32, 1.0, 0.0, 0.0, "eval",0);
					if (need_to_write){
						std::cout << ss.str() << maxError << " " << distDriven << " " << "F";
						std::ofstream outfile;
						outfile.open("Stats.txt", std::ios_base::app);
						outfile << ss.str() << maxError << " " << distDriven << " " << "F" << std::endl;
						need_to_write = false;
					}
				}
				else{
					viewer->addText("Passed!", 200, 50, 32, 0.0, 1.0, 0.0, "eval",0);
					if (need_to_write){
						std::cout << ss.str() << maxError << " " << distDriven << " " << "P";
						std::ofstream outfile;
						outfile.open("Stats.txt", std::ios_base::app);
						outfile << ss.str() << maxError << " " << distDriven << " " << "P" << std::endl;
						need_to_write = false;
					}
				}
			}

			pclCloud.points.clear();
		}
  	}
	return 0;
}
