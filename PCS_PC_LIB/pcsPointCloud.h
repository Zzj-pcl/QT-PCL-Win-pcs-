#ifndef PCSPOINTCLOUD_H
#define PCSPOINTCLOUD_H

#include "pcs_pc_lib_global.h"

#include <QString>
#include <vector>

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class PCS_PC_LIB_EXPORT pcsPointCloud
{
public:
	//pcsPointCloud(QString name = QString());
	//virtual ~pcsPointCloud();
	
	//typedef std::vector<pcsPointCloud*> Container;
	PointCloudT::Ptr cloud;
};

#endif // PCSPOINTCLOUD_H
