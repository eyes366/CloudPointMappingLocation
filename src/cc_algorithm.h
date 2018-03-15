#pragma 

#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int ccVoxelGridFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cp_in,
						pcl::PointCloud<pcl::PointXYZI>::Ptr& cp_out,
						float fDist);

class CcAlgorithm
{
public:
	CcAlgorithm();
	~CcAlgorithm();

private:

};

