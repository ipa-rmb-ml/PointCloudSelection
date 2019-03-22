#include "ipa_database_generation.h"
#include <pcl/visualization/cloud_viewer.h>

// definition
#define TOPIC_POINT_CLOUD_GEN "/pico_flexx/points"  //ROS topic point cloud
#define TOPIC_POINT_CLOUD_PATCH "/filtered_cluster" //ROS topic selected patch 
#define PATH_TO_DIR "/home/robot/Desktop/rmb-ml/PointCloudDataBase/" // Path to store patches

DoorhandleDatabaseGeneration::DoorhandleDatabaseGeneration(ros::NodeHandle nh, sensor_msgs::PointCloud2::Ptr point_cloud_out_msg) :
nh_(nh), point_cloud_out_msg_(point_cloud_out_msg)
{
	distance_thres_ = 0.01;
	initCameraNode(nh,point_cloud_out_msg);	


}


std::string DoorhandleDatabaseGeneration::getFilePathFromParameter(int dist, int angle_XZ, int angle_YZ)
{
		int num_dist = 3;
		int distances[] = {40-dist,55-dist,70-dist};

		int num_angle_xz = 3;
		int angle_xz[] = {-10-angle_XZ,0-angle_XZ,10-angle_XZ}; //deg

		int num_angle_yz = 2;
		int angle_yz[] = {-20-angle_YZ,0-angle_YZ,20-angle_YZ}; // deg

		// setup vec contain this info: angle_xz, angle_yz, distance to door plane

	//distance loop
 		int index_d = 0 ;
		int n_d = abs(distances[0]);
		for (int i = 1; i < num_dist; i++)
		{
			if (distances[i] < n_d)
			{
				n_d = abs(distances[i]); 
				index_d = i ;
			}
 		}
		int final_distance = distances[index_d]+dist;

		int index_a1 = 0 ;
		int n_a1 = abs(angle_xz[0]);
		for (int i = 1; i < num_angle_xz; i++)
		{
			if (angle_xz[i] < n_a1)
			{
				n_a1 = abs(angle_xz[i]); 
				index_a1 = i ;
			}
 		}
		int final_angleXZ = angle_xz[index_a1]+angle_XZ;

		int index_a2 = 0 ;
		int n_a2 = abs(angle_yz[0]);
		for (int i = 1; i < num_angle_yz; i++)
		{
			if (angle_yz[i] < n_a2)
			{
				n_a2 = abs(angle_yz[i]); 
				index_a2 = i ;
			}
 		}
		int final_angleYZ = angle_yz[index_a2]+angle_YZ;

		std::stringstream str1, str2, str3;

		str1 << final_angleXZ;
		str2 << final_angleYZ;
		str3 << final_distance;

		std::string angle_1_str = str1.str();
		std::string angle_2_str = str2.str();
		std::string dist_str = str3.str();

		std::string name_pcd  = "_distance_" + dist_str + "cm_" + "angleXZ_" + 	angle_1_str + "°_" + "angleYZ_"+ angle_2_str + "°.pcd";

		return name_pcd;
}
	
void DoorhandleDatabaseGeneration::pointcloudCallback_2(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*point_cloud_msg, *point_cloud);

	if (point_cloud->points.size() > 0)
	{

		int dist     = setup_(2);
		int angle_XZ = setup_(0);
		int angle_YZ = setup_(1);
 
		std::string filename_append =getFilePathFromParameter(dist,angle_XZ,angle_YZ);

		std::string myString = "";
		std::string model_type = "";
		std::string save_bool = "";


		// open pcd viewer
		pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
		viewer.showCloud(point_cloud);
		while (!viewer.wasStopped ())
		{
		}


		std::string name_pcd  = "door_handle_type_" + model_type + filename_append;
		std::string path_type = PATH_TO_DIR + model_type + "/" + "test.pcd"; //name_pcd ;
		std::cout<<name_pcd<<std::endl;

		while(true)
		{
			std::cout << "Save to file? [y/n]" << std::endl;
			std::getline(std::cin, myString);

			if  (myString.length() != 0)
			{
				save_bool = myString;


				if (save_bool.compare("y") == 0)
				{
					while(true)
					{
						std::cout << "Enter type of handle and confirm with enter:" << std::endl;
						std::getline(std::cin, myString);

						if  (myString.length() != 0)
						{
							model_type = myString;
							break;
						}
					} 

						boost::filesystem::path p(PATH_TO_DIR + model_type);  // avoid repeated path construction below

						if (!exists(p))    // does path p actually exist?
						{
							ROS_WARN("Creating Directory...");
							boost::filesystem::create_directory(p);	
							pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
							pcl::toROSMsg(*point_cloud, *point_cloud_out_msg_);
							// store png 
						}
							ROS_WARN("Writing point cloud to pcd file...");
							//std::cout<<path_type<<std::endl;
							pcl::io::savePCDFileASCII (path_type,*point_cloud);
							pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
							pcl::toROSMsg(*point_cloud, *point_cloud_out_msg_);
					break;
				}

				else
				{
					ROS_WARN("Cancelling selection...");
					break;
				}
			}
		} 
		// vreate saving path -> based on handle type name 
		//save selected patch 
		// save reduced object with 
	}
}
		

void DoorhandleDatabaseGeneration::pointcloudCallback_1(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

	//create new point cloud in pcl format: pointcloud_in_pcl_format
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_rgb_trans(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ModelCoefficients::Ptr plane_coeff (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	//transform imported pointcloud point_cloud_msg to pointcloud_in_pcl_format
	pcl::fromROSMsg(*point_cloud_msg, *point_cloud);	


	if (point_cloud->points.size() != 0)
	{

		planeInformation planeData = detectPlaneInPointCloud(point_cloud);
		inliers = planeData.plane_point_cloud_indices;
		plane_coeff = planeData.plane_coeff;

		/// interpretation of ros data
		pcl::PointXYZRGB pclPoint;
		for (size_t i = 0; i < point_cloud->points.size (); ++i)
		{
			pclPoint.x = point_cloud->points[i].x;
			pclPoint.y = point_cloud->points[i].y;
			pclPoint.z = point_cloud->points[i].z;
			point_cloud_rgb_trans->points.push_back(pclPoint);
		}

		Eigen::Vector3f cam_axis;
		cam_axis << 0,0,1;

		 double scalar_prod_xz = plane_coeff->values[0];
		 double scalar_prod_yz = plane_coeff->values[1];
 
		 double len_xz = pow(plane_coeff->values[0],2) + pow(plane_coeff->values[2],2);
		 double len_yz = pow(plane_coeff->values[1],2) + pow(plane_coeff->values[2],2); 

		// get geometrical lenght
		double cos_alpha_1 = scalar_prod_xz/sqrt(len_xz);
		double cos_alpha_2 = scalar_prod_yz/sqrt(len_yz);

		int angle_1 = 90-acos(cos_alpha_1)*180.0/M_PI;
		int angle_2 = asin(cos_alpha_2)*180.0/M_PI;

		int dist = -plane_coeff->values[3]*100; // to cm

		if ((abs(dist) > 100) || (abs(angle_1) > 180) || (abs(angle_2) > 180)) 
		{
			std::cout<<"angle_xz: " << std::endl;
			std::cout<<"angle_yz: " << std::endl;
			std::cout<<"distance: " << std::endl;

		}

		else
		{
			std::cout<<"angle_xz: "<<angle_1<< "°" << std::endl;
			std::cout<<"angle_yz: "<<angle_2<< "°" << std::endl;
			std::cout<<"distance: "<< dist << "cm" << std::endl;


		}

		setup_ << angle_1,angle_2,dist;  
	}

	std::vector <double> BB_vec = readBoundingBoxFromTXT();

	//filter point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_filt (new pcl::PointCloud<pcl::PointXYZ>);
	point_cloud_filt= filterPointCloud(point_cloud, BB_vec);

	pcl::toROSMsg(*point_cloud_filt, *point_cloud_out_msg_);
	pub2_.publish(point_cloud_out_msg_);


}


void DoorhandleDatabaseGeneration::initCameraNode(ros::NodeHandle nh, sensor_msgs::PointCloud2::Ptr point_cloud_out_msg)
{
	std::cout << "Initialising DoorhandleDatabaseGeneration Constructor." << std::endl;
	
	pub_ = nh_.advertise<sensor_msgs::PointCloud2>(TOPIC_POINT_CLOUD_GEN,1);
	pub2_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_cluster",1);

	ros::Subscriber point_cloud_sub_1_ = nh_.subscribe<sensor_msgs::PointCloud2>(TOPIC_POINT_CLOUD_GEN, 1, &DoorhandleDatabaseGeneration::pointcloudCallback_1, this);

	ros::Subscriber point_cloud_sub_2_ = nh_.subscribe<sensor_msgs::PointCloud2>(TOPIC_POINT_CLOUD_PATCH, 1, &DoorhandleDatabaseGeneration::pointcloudCallback_2, this);

	ros::Duration(1).sleep();
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
	
		ros::spinOnce();
		loop_rate.sleep();
	}

	if (!ros::ok()){
		std::cout << "Quit publishing" << std::endl;
	}
	std::cout << "DoorhandleDatabaseGeneration Constructor Initialised." << std::endl;
}


std::vector <double>  DoorhandleDatabaseGeneration::readBoundingBoxFromTXT()
{
  // read in from path
  std::string pathTXT = "/home/robot/Desktop/rmb-ml/test.txt";

  std::ifstream indata;
  indata.open(pathTXT.c_str());
  std::string line;

std::vector <double> BB_vec;

	

 while(!indata.eof())
            {
              std::string line = "";
              std::getline(indata,line);

              if (!line.empty())
              {
  		BB_vec.push_back(std::atof(line.c_str()));
              }
						} // end if
indata.close();

return BB_vec;

}


pcl::PointCloud<pcl::PointXYZ>::Ptr DoorhandleDatabaseGeneration::filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, std::vector <double> BB_vec)
{
pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_filt_x (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_filt_y (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_filt_z (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filt (new pcl::PointCloud<pcl::PointXYZ>);

// x y z x y z
// left: x, z, y right: x,z,z

	pcl::PointXYZ pclPoint;
	for (size_t i = 0; i < point_cloud->points.size (); ++i)
	{
		pclPoint.x = point_cloud->points[i].x;
		pclPoint.y = -point_cloud->points[i].y;
		pclPoint.z = point_cloud->points[i].z;
		filt->points.push_back(pclPoint);
	}

	point_cloud = filt;

	double x_UL = BB_vec.at(0);
	double y_UL = BB_vec.at(2);
	double x_LR = BB_vec.at(3);
	double y_LR = BB_vec.at(5);

	double min_x = std::min(x_UL,x_LR);
	double max_x = std::max(x_UL,x_LR);

	double min_y = std::min(y_UL,y_LR);
	double max_y = std::max(y_UL,y_LR);


	// X filter
	 pcl::PassThrough<pcl::PointXYZ> pass_x;
	 pass_x.setInputCloud (point_cloud);
	 pass_x.setFilterFieldName ("x");
	 pass_x.setFilterLimits (min_x, max_x);
	 pass_x.filter (*point_cloud_filt_x);

	pcl::ModelCoefficients::Ptr plane_coeff (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	planeInformation planeData = detectPlaneInPointCloud(point_cloud);
	inliers = planeData.plane_point_cloud_indices;
	plane_coeff = planeData.plane_coeff;

	double dist_2_plane = -plane_coeff->values[3]; // m
	double distance_range = 0.15; 
	double tol = 0.1;

	// Z Filter
	pcl::PassThrough<pcl::PointXYZ> pass_z;
	pass_z.setInputCloud (point_cloud_filt_x);
	pass_z.setFilterFieldName ("z");
	pass_z.setFilterLimits (dist_2_plane-distance_range, dist_2_plane+tol);
	pass_z.filter (*point_cloud_filt_z);


	// Y Filter
	pcl::PassThrough<pcl::PointXYZ> pass_y;
	pass_y.setInputCloud (point_cloud_filt_z);
	pass_y.setFilterFieldName ("y");
	pass_y.setFilterLimits (min_y, max_y);
	pass_y.filter (*point_cloud_filt_y);


return point_cloud_filt_y;
}

planeInformation DoorhandleDatabaseGeneration::detectPlaneInPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud){

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	pcl::ModelCoefficients::Ptr plane_coeff (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;

	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (distance_thres_);
	seg.setInputCloud (input_cloud);
	// get inliers and plane coeff
	seg.segment (*inliers, *plane_coeff);

	planeInformation planeData;
	planeData.plane_point_cloud_indices = inliers;
 	planeData.plane_coeff = plane_coeff;

	return planeData;
};

// =================================================0
int main(int argc, char **argv)
{		

	ros::init(argc, argv, "Param_Save");
	ros::NodeHandle nh;
	sensor_msgs::PointCloud2::Ptr point_cloud_out_msg(new sensor_msgs::PointCloud2);
	DoorhandleDatabaseGeneration DoorhandleDatabaseGeneration(nh, point_cloud_out_msg);

	return 0;
}


