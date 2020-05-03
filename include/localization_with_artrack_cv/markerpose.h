#ifndef LOCALIZATION_INDIV_MULTI_H_
#define LOCALIZATION_INDIV_MULTI_H_

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

#include <unordered_map>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>

#include <ros/package.h>

void ReadMarkerformFile(std::vector<std::vector<double> > & MarkersList, const std::string & MarkerListPath);

class Markerpose
{
	private:
		std::vector < std::vector <double> > pose_marker_id_list;
        std::string marker_list_path;

	public:	
		std::unordered_map<int, std::vector<double>> marker_pose;
		Markerpose();
		std::vector<double> FillMarker(const std::vector<double> & vv);
        void MarkerposeShow();
};

Markerpose::Markerpose()
{	
    std::string marker_list_path = ros::package::getPath("localization_with_artrack_cv");

	ReadMarkerformFile(pose_marker_id_list, marker_list_path);

	for (int i = 0; i < pose_marker_id_list.size(); ++i)
	{
		marker_pose.insert(std::pair<int, std::vector<double> >(int(pose_marker_id_list[i][0]),FillMarker(pose_marker_id_list[i])));
	}
}

std::vector<double> Markerpose::FillMarker(const std::vector<double> & vv)
{
	tf2::Quaternion trans;
    trans.setRPY(vv[4], vv[5], vv[6]);
    std::vector<double> qq = {vv[1], vv[2], vv[3], trans.x(), trans.y(), trans.z(), trans.w()};
    return qq;
}

void Markerpose::MarkerposeShow()
{
    for (int row = 0; row < pose_marker_id_list.size(); row++) {
        for (int column = 0; column < pose_marker_id_list[row].size(); column++) {
            std::cout << pose_marker_id_list[row][column] << " " ;
        }
        std::cout << std::endl; 
    }
}

void ReadMarkerformFile(std::vector < std::vector <double> > &MarkersList, const std::string & MarkerListPath){
    std::string line;
    std::stringstream ss;
    std::vector<double> MarkerList;

    double i;
    std::ifstream posefile(MarkerListPath+"/config/markerposelist.txt");

    if (posefile.is_open()) {
        while (getline (posefile, line)) {
            ss.clear();
            ss.str("");
            ss.str(line);
            MarkerList.clear();
            while(ss >> i) {
                MarkerList.push_back(i);

                if (ss.peek() == ',' || ss.peek() == ' ') {
                    ss.ignore();
                }
            }
            MarkersList.push_back(MarkerList);
        }
        posefile.close();
    }

    else std::cout << "Unable to open file";
}

#endif