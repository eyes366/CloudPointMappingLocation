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
	pcl::PointCloud<pcl::PointXYZI>::Ptr&,	//��ͼ����
	pcl::PointCloud<pcl::PointXYZI>::Ptr&,	//Lidarʵʱ����
	POSE_DATA&,								//δУ��gps
	POSE_DATA&,								//У����gps
	int&									//״̬
	)> FunctionResultCallBack;

typedef boost::function<void(
  	Eigen::Matrix4d&,						//��imu�����ƽ��λ��
  	Eigen::Matrix4d&						//У���󾭹�imu�����ƽ��λ��
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
	std::string szMapDir;		//��ͼ�ļ�Ŀ¼
	double dLocalMapRange;		//�ֲ���ͼ����
	double dSegmentDist;		//�ֲ���ͼ�������
	int nLidarType;				//0:VLP-16 1:HDL-32
};


class LOCATIONOPRAPI_API CLocationOprAPI
{
public:
	CLocationOprAPI();
	~CLocationOprAPI();

	//��ʼ����ȡ��ͼ
	//return<0����ȡ��ͼʧ��
	int Init(LocationAPIParam Param);

	LocationAPIParam GetParam();

	//���ý�����ݻص�
	void SetResultCallBack(FunctionResultCallBack result);
	void SetTrackCallBack(FunctionTrackCallBack track);

	//��init_poseΪ��ʼλ�ˣ���ʼ��λ
	//return<0����ʼʧ��
	int StartLocate(POSE_DATA& init_pose);

	//��ѯ�Ƿ����ڶ�λ
	bool IsRunning();

	//��������
	int FeedVelodynePack(const PCAP_DATA* pBuf);
	int FeedImuData(const POSE_DATA* pImu);

	//ֹͣ��λ
	int Stop();

private:
	void* m_pUser;
};

