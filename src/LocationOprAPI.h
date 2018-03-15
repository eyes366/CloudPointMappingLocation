#pragma once

#ifdef LocationOprAPI_EXPORTS
#define LOCATIONOPRAPI_API __declspec(dllexport)
#else
#define LOCATIONOPRAPI_API __declspec(dllimport)
#endif

#include <string>
#include <Eigen/Core>
#include <boost/function/function1.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "CommonHeader.h"

typedef boost::function<void(
	pcl::PointCloud<pcl::PointXYZI>::Ptr&,	//地图点云
	pcl::PointCloud<pcl::PointXYZI>::Ptr&,	//Lidar实时点云
	POSE_DATA&,								//未校正gps
	POSE_DATA&,								//校正后gps
	int&									//状态
	)> FunctionResultCallBack;

typedef boost::function<void(
  	Eigen::Matrix4d&,						//纯imu推算的平面位置
  	Eigen::Matrix4d&						//校正后经过imu推算的平面位置
	)> FunctionTrackCallBack;

struct LOCATIONOPRAPI_API LocationAPIParam
{
	LocationAPIParam()
	{
		szMapDir = "";
		dLocalMapRange = 50.0;	
		dSegmentDist = 5.0;		
		nLidarType = 1;
	}
	std::string szMapDir;		//地图文件目录
	double dLocalMapRange;		//局部地图长度
	double dSegmentDist;		//局部地图间隔距离
	int nLidarType;				//0:VLP-16 1:HDL-32
};


class LOCATIONOPRAPI_API CLocationOprAPI
{
public:
	CLocationOprAPI();
	~CLocationOprAPI();

	//初始化读取地图
	//return<0：读取地图失败
	int Init(LocationAPIParam Param);

	LocationAPIParam GetParam();

	//设置结果数据回掉
	void SetResultCallBack(FunctionResultCallBack result);
	void SetTrackCallBack(FunctionTrackCallBack track);

	//以init_pose为初始位姿，开始定位
	//return<0：开始失败
	int StartLocate(POSE_DATA& init_pose);

	//查询是否正在定位
	bool IsRunning();

	//输入数据
	int FeedVelodynePack(const PCAP_DATA* pBuf);
	int FeedImuData(const POSE_DATA* pImu);

	//停止定位
	int Stop();

private:
	void* m_pUser;
};

