#include <pcl\io\pcd_io.h>
//#include <voxel_grid_.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/local_maximum.h>
#include "LocationOprAPI.h"
#include "DataSimulator.h"
//#include "NdtVisual.h"
//#include "cc_algorithm.h"

using namespace std;

//NdtVisual g_visual;

void result_callback(pcl::PointCloud<pcl::PointXYZI>::Ptr& map_cloud,//地图点云
					 pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_cloud, //实时点云
					 POSE_DATA& pose_data_guess, //未校正gps
					 POSE_DATA& pose_data_ndt,//校正后gps
					 int& nRt)
{
// 	static int g_nlocateInd = 0;
// 	char szSave[1024] = {0};
// 	sprintf_s(szSave, 1024, "%06dMap.pcd", g_nlocateInd);
// 	cout << szSave << endl;
// 	pcl::io::savePCDFileBinary(szSave, *map_cloud);
// 	sprintf_s(szSave, 1024, "%06dResult.pcd", g_nlocateInd);
// 	cout << szSave << endl;
// 	pcl::io::savePCDFileBinary(szSave, *lidar_cloud);
// 	g_nlocateInd++;
//	g_visual.DisplayCloudPoints(map_cloud, lidar_cloud);
}

int main(int argc, char ** argv)
{
	LocationAPIParam LocatorParam;
//	LocatorParam.szMapDir = "D:\\MyData\\20180301\\own\\20180301-2\\map\\";
	LocatorParam.szMapDir = "D:\\MyData\\20180308\\own\\2018-03-08-11-10-01-2\\map\\";
	CLocationOprAPI Locator;
	Locator.Init(LocatorParam);
	Locator.SetResultCallBack(result_callback);
	POSE_DATA init_pose;
	init_pose.pos[0] = 30.4021802736;
	init_pose.pos[1] = 114.4033088874;
	init_pose.pos[2] = 25.8490;
	double dRoll = 0.0;
	double dPitch = 0.0;
	double dHeading = 221.6039;
	Eigen::Matrix3d rot = 
	(Eigen::AngleAxisd((-1.0*dHeading)/180.0*3.141592654, Eigen::Vector3d::UnitZ ()) *
		Eigen::AngleAxisd(dRoll/180.0*3.141592654, Eigen::Vector3d::UnitX ()) *
		Eigen::AngleAxisd(dPitch/180.0*3.141592654, Eigen::Vector3d::UnitY ()) *
		Eigen::AngleAxisd(-3.141592653589*0.5, Eigen::Vector3d::UnitZ ())*
		Eigen::AngleAxisd(3.141592653589, Eigen::Vector3d::UnitY ())
		).matrix();
	Eigen::Quaterniond q(rot);
	init_pose.quat[0] = q.w();
	init_pose.quat[1] = q.x();
	init_pose.quat[2] = q.y();
	init_pose.quat[3] = q.z();
	Locator.StartLocate(init_pose);

	CDataSimulator DataInput;
	FunctionImuFeedCallBack imuCallBack = boost::bind(&CLocationOprAPI::FeedImuData,
		&Locator, _1);
	FunctionVelodyneFeedCallBack VelodyneCallBack = boost::bind(&CLocationOprAPI::FeedVelodynePack,
		&Locator, _1);
	DataInput.SetCallBack(VelodyneCallBack, imuCallBack);
	DataInput.StartSimulator("D:\\MyData\\20180301\\own\\20180301-1\\", uint64_t(double(354191.760)*1000000.0));

// 	while (true)
// 	{
// 		g_visual.SpinOnce();
// 	}
	boost::this_thread::sleep(boost::posix_time::hours(24));

	return 1;
}
