#include <pcl\io\pcd_io.h>
#include "LocationOprAPI.h"
#include "DataSimulator.h"

using namespace std;

void result_callback(pcl::PointCloud<pcl::PointXYZI>::Ptr& map_cloud,//地图点云
					 pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_cloud, //实时点云
					 POSE_DATA& pose_data_guess, //未校正gps
					 POSE_DATA& pose_data_ndt,//校正后gps
					 int& nRt)
{
	static int g_nlocateInd = 0;
	char szSave[1024] = {0};
	sprintf_s(szSave, 1024, "%06dMap.pcd", g_nlocateInd);
	cout << szSave << endl;
	pcl::io::savePCDFileBinary(szSave, *map_cloud);
	sprintf_s(szSave, 1024, "%06dResult.pcd", g_nlocateInd);
	cout << szSave << endl;
	pcl::io::savePCDFileBinary(szSave, *lidar_cloud);
	g_nlocateInd++;
}

int main(int argc, char ** argv)
{
	LocationAPIParam LocatorParam;
	LocatorParam.szMapDir = "D:\\MyData\\20180301\\own\\20180301-1\\map\\";
	CLocationOprAPI Locator;
	Locator.Init(LocatorParam);
	Locator.SetResultCallBack(result_callback);
	POSE_DATA init_pose;
	init_pose.pos[0] = 30.44522251;
	init_pose.pos[1] = 114.39685251;
	init_pose.pos[2] = 25.8490;
	init_pose.quat[0] = 0.00102542;
	init_pose.quat[1] = 0.92273361;
	init_pose.quat[2] = 0.38522257;
	init_pose.quat[3] = -0.01285303;
	Locator.StartLocate(init_pose);

	CDataSimulator DataInput;
	FunctionImuFeedCallBack imuCallBack = boost::bind(&CLocationOprAPI::FeedImuData,
		&Locator, _1);
	FunctionVelodyneFeedCallBack VelodyneCallBack = boost::bind(&CLocationOprAPI::FeedVelodynePack,
		&Locator, _1);
	DataInput.SetCallBack(VelodyneCallBack, imuCallBack);
	DataInput.StartSimulator("D:\\MyData\\20180301\\own\\20180301-1\\");

	boost::this_thread::sleep(boost::posix_time::hours(24));

	return 1;
}
