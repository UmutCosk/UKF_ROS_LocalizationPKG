#ifndef UTILS_H_
#define UTILS_H_

#include <vector>
#include "ros/ros.h"
#include <math.h>
#include <ros/package.h>

#include <Eigen/Core>
//#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include "quaternion_average.h"

#include <opencv2/core.hpp>

const double PI = 3.141592654;

std::vector<geometry_msgs::PoseStamped> RemoveOutliers(const geometry_msgs::PoseStamped & CPSA,
	std::vector<geometry_msgs::PoseStamped> & CPOS, double threshold)
{

	std::vector<geometry_msgs::PoseStamped> temp;
 
	ros::Duration Dur = CPOS[0].header.stamp - CPSA.header.stamp;
	double time = Dur.toSec();

	double x,y,z;
	x = CPSA.pose.position.x;
	y = CPSA.pose.position.y;
	z = CPSA.pose.position.z;

	// double xyzCamPoStAvgPrev = sqrt(pow(CPSA.pose.position.x,2)
	//  				+ pow(CPSA.pose.position.y,2)
	//  				+ pow(CPSA.pose.position.z,2));

	for (int i = 0; i < CPOS.size(); ++i)
	{
		double ix, iy, iz;
		ix = CPOS[i].pose.position.x;
		iy = CPOS[i].pose.position.y;
		iz = CPOS[i].pose.position.z;

		double V3Distance = sqrt(pow(ix-x,2) + pow(iy-y,2) + pow(iz-z,2));
		double V3Speed = V3Distance / (time+0.00001);

		if (V3Speed < threshold) //threshold = speed in m/s
			temp.push_back(CPOS[i]);
	}

	return temp;
}

void clearPoseStamped(geometry_msgs::PoseStamped & PS){

	PS.header.seq = 0;
	PS.header.stamp = ros::Time(0);
	PS.header.frame_id = "";
	PS.pose.position.x = 0;
	PS.pose.position.y = 0;
	PS.pose.position.z = 0;
	PS.pose.orientation.x = 0;
	PS.pose.orientation.y = 0;
	PS.pose.orientation.z = 0;
	PS.pose.orientation.w = 0;
}

bool isEmptyPoseStamped(const geometry_msgs::PoseStamped & PS){

	return PS.header.stamp == ros::Time(0) && PS.pose.position.x == 0
		&& PS.pose.position.y == 0 && PS.pose.position.z == 0;
}

void showPoseStamped(geometry_msgs::PoseStamped & PS){

	std::cout << PS.header.stamp << std::endl;
  	std::cout << PS.header.frame_id << std::endl;
  	std::cout << PS.header.seq << std::endl;
  	std::cout << PS.pose.position.x << std::endl;
  	std::cout << PS.pose.position.y << std::endl;
  	std::cout << PS.pose.position.z << std::endl;
  	std::cout << PS.pose.orientation.x << std::endl;
  	std::cout << PS.pose.orientation.y << std::endl;
  	std::cout << PS.pose.orientation.z << std::endl;
  	std::cout << PS.pose.orientation.w << std::endl;
  	std::cout << std::endl;
}


geometry_msgs::PoseStamped PoseAvg(const std::vector<geometry_msgs::PoseStamped> & CamPoStMul){

	geometry_msgs::PoseStamped CamPoStAvg;

	double roll,pitch,yaw;
	double x,y,z;
	double makrer_qua = CamPoStMul.size();
	//std::cout << "length of CamPoStMul: "<< makrer_qua << std::endl;

	tf::Quaternion PostQ;

	for (size_t i = 0; i < CamPoStMul.size(); ++i)
	{
		
		x = x + CamPoStMul[i].pose.position.x / makrer_qua;
		y = y + CamPoStMul[i].pose.position.y / makrer_qua;
		z = z + CamPoStMul[i].pose.position.z / makrer_qua;


		double iroll,ipitch,iyaw;
		tf::Quaternion Q = {CamPoStMul[i].pose.orientation.x,
							CamPoStMul[i].pose.orientation.y,
							CamPoStMul[i].pose.orientation.z,
							CamPoStMul[i].pose.orientation.w};

		tf::Matrix3x3(Q).getRPY(iroll,ipitch,iyaw);

		roll = roll + iroll / makrer_qua;
		pitch = pitch + ipitch / makrer_qua;
		yaw = yaw + iyaw / makrer_qua;

	}

	PostQ.setRPY(fmod(roll,PI), fmod(pitch,PI), fmod(yaw,PI));

	CamPoStAvg.header.seq = 0;
	CamPoStAvg.header.stamp = CamPoStMul[0].header.stamp;
	CamPoStAvg.header.frame_id = "world";

	CamPoStAvg.pose.position.x = x;
	CamPoStAvg.pose.position.y = y;
	CamPoStAvg.pose.position.z = z;
	CamPoStAvg.pose.orientation.x = PostQ.x();
	CamPoStAvg.pose.orientation.y = PostQ.y();
	CamPoStAvg.pose.orientation.z = PostQ.z();
	CamPoStAvg.pose.orientation.w = PostQ.w();

	return CamPoStAvg;
}

geometry_msgs::PoseStamped PoseAvgQr(const std::vector<geometry_msgs::PoseStamped> & CamPoStMul){

	geometry_msgs::PoseStamped CamPoStAvg;
	Eigen::Quaterniond Quat;
	std::vector<Eigen::Quaterniond> QuatList;

	for (int i = 0; i < CamPoStMul.size(); ++i)
	{	
		Eigen::Quaterniond tQuat;
		tQuat.x()= CamPoStMul[i].pose.orientation.x;
		tQuat.y()= CamPoStMul[i].pose.orientation.y;
		tQuat.z()= CamPoStMul[i].pose.orientation.z;
		tQuat.w()= CamPoStMul[i].pose.orientation.w;

		QuatList.push_back(tQuat);
	}

	Quat = Qr::averageQuaternions<double>(QuatList);

	double x,y,z;

	double makrer_qua = CamPoStMul.size();

	for (size_t i = 0; i < makrer_qua; ++i)
	{
		
		x = x + CamPoStMul[i].pose.position.x / makrer_qua;
		y = y + CamPoStMul[i].pose.position.y / makrer_qua;
		z = z + CamPoStMul[i].pose.position.z / makrer_qua;
	}

	CamPoStAvg.header.seq = 0;
	CamPoStAvg.header.stamp = CamPoStMul[0].header.stamp;
	CamPoStAvg.header.frame_id = "world";

	CamPoStAvg.pose.position.x = x;
	CamPoStAvg.pose.position.y = y;
	CamPoStAvg.pose.position.z = z;

	CamPoStAvg.pose.orientation.x = Quat.x();
	CamPoStAvg.pose.orientation.y = Quat.y();
	CamPoStAvg.pose.orientation.z = Quat.z();
	CamPoStAvg.pose.orientation.w = Quat.w();

	return CamPoStAvg;
}

// geometry_msgs::PoseStamped PoseAvgQrslerp(const std::vector<geometry_msgs::PoseStamped> & CamPoStMul){

// 	geometry_msgs::PoseStamped CamPoStAvg;
// 	int Vsize = CamPoStMul.size();

// 	if (Vsize == 1)
// 	{	
// 		CamPoStAvg = CamPoStMul[0];
// 		CamPoStAvg.header.seq = 0;
// 		CamPoStAvg.header.frame_id = "world";

// 		return CamPoStAvg;
// 	}
// 	else if (Vsize % 2 == 1)
// 	{
// 		CamPoStMul.pop_back();
// 	}

// 	for (int i = 0; i < Vsize-1; ++i)
// 	{
// 		tf2::Quaternion Post1 = CamPoStMul[0];
// 		tf2::Quaternion Post2;
// 		tf2::Quaternion PostAvg;
// 	}

// 	tf2::Quaternion Post1;
// 	tf2::Quaternion Post2;
// 	tf2::Quaternion PostAvg;

// 	PostAvg = Post1.slerp(Post2,0.5);

// 	return CamPoStAvg;
// }


//@ read camera calibration yaml file
bool Readcameracalib(cv::Mat &camerax, cv::Mat &distcoeffs)
{	
	std::string calibfile_path = ros::package::getPath("localization_with_artrack_cv");
    std::string line;
    std::vector<double> camera_matrix;
    std::vector<double> distortion;
    std::stringstream ss;
    std::ifstream posefile(calibfile_path+"/config/camera_calibration.yaml");
    int flag = 1;

    if (posefile.is_open())
    {
         while (getline (posefile, line))
         {
             if(flag == 7)
             {
                int n1 = line.find('[');
                int n2 = line.find(']');
                int n3 = n2-n1-1;
                line = line.substr(n1+1,n3);
                ss.clear();
                ss.str("");
                ss.str(line);

                double mxvalue;

                while(ss >> mxvalue)
                {
                    if (ss.peek() == ',' || ss.peek() == ' ')
                        ss.ignore();

                    camera_matrix.push_back(mxvalue);
                }

             }
             else if(flag == 12)
             {
                 int n1 = line.find('[');
                 int n2 = line.find(']');
                 int n3 = n2-n1-1;
                 line = line.substr(n1+1,n3);
                 ss.clear();
                 ss.str("");
                 ss.str(line);

                 double mxvalue;

                 while(ss >> mxvalue)
                 {
                     if (ss.peek() == ',' || ss.peek() == ' ')
                         ss.ignore();

                     distortion.push_back(mxvalue);
                 }
             }

             flag ++;

         }
         posefile.close();


         camerax =  (cv::Mat_<double>(3, 3) << camera_matrix[0], camera_matrix[1],camera_matrix[2], 
         									camera_matrix[3], camera_matrix[4], camera_matrix[5], 
         									camera_matrix[6], camera_matrix[7], camera_matrix[8]);

         distcoeffs = (cv::Mat_<double>(1, 5) << distortion[0], distortion[1], distortion[2], distortion[3], distortion[4]);

         std::string cm;
         std::string dt;

         for (int i = 0; i < camera_matrix.size(); ++i)
         {	
         	std::stringstream ss;
         	ss << std::setprecision(12) << camera_matrix[i];
        	cm = cm + ss.str() + "  ";
         }

         for (int i = 0; i < distortion.size(); ++i)
         {	
         	std::stringstream ss;
         	ss << std::setprecision(12) << distortion[i];
        	dt = dt + ss.str() + "  ";
         }


         ROS_INFO("camera_matrix:[  %s]" ,cm.c_str());
         ROS_INFO("distortion:[  %s]" ,dt.c_str());
    }

    else
    {
        ROS_WARN("Could not load calibration file.");
        return false;
    }

    return true;
}

#endif