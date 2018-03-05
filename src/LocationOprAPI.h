#pragma once

#ifdef LocationOprAPI_EXPORTS
#define LOCATIONOPRAPI_API __declspec(dllexport)
#else
#define LOCATIONOPRAPI_API __declspec(dllimport)
#endif

#include <string>
#include <boost/function/function1.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "CommonHeader.h"

typedef boost::function<void(pcl::PointCloud<pcl::PointXYZI>::Ptr&,//地图点云
	pcl::PointCloud<pcl::PointXYZI>::Ptr&, //实时点云
	POSE_DATA&, //未校正gps
	POSE_DATA&,//校正后gps
	int&)> FunctionResultCallBack;//状态

struct LOCATIONOPRAPI_API LocationAPIParam
{
	LocationAPIParam()
	{
		dLocalMapRange = 50.0;	//局部地图长度
		dSegmentDist = 5.0;		//局部地图准备间隔距离
	}
	std::string szMapDir;		//地图文件目录
	double dLocalMapRange;
	double dSegmentDist;
};


class LOCATIONOPRAPI_API CLocationOprAPI
{
public:
	CLocationOprAPI();
	~CLocationOprAPI();

	int Init(LocationAPIParam Param);

	void SetResultCallBack(FunctionResultCallBack result);

	int StartLocate(POSE_DATA& init_pose);

	int FeedVelodynePack(PCAP_DATA* pBuf);
	int FeedImuData(POSE_DATA* pImu);

	int Stop();

private:
	void* m_pUser;
};

