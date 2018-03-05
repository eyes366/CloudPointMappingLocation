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

typedef boost::function<void(pcl::PointCloud<pcl::PointXYZI>::Ptr&,//��ͼ����
	pcl::PointCloud<pcl::PointXYZI>::Ptr&, //ʵʱ����
	POSE_DATA&, //δУ��gps
	POSE_DATA&,//У����gps
	int&)> FunctionResultCallBack;//״̬

struct LOCATIONOPRAPI_API LocationAPIParam
{
	LocationAPIParam()
	{
		dLocalMapRange = 50.0;	//�ֲ���ͼ����
		dSegmentDist = 5.0;		//�ֲ���ͼ׼���������
	}
	std::string szMapDir;		//��ͼ�ļ�Ŀ¼
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

