#ifndef _LOCATION_OPR_H_
#define _LOCATION_OPR_H_

#include <string>
#include <list>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ndt.h>
//#include "Imu100Hz.h"
#include "common.h"
#include "MapConstruct.h"
#include "VelodyneOpr.h"
#include "CommonHeader.h"
#include "LocationOprAPI.h"

#define LOCATION_IMU_MAX_SIZE 400	//缓存400frames的imu数据

// struct LocationParam
// {
// 	LocationParam()
// 	{
// 		dLocalMapRange = 50.0;	//局部地图长度
// 		dSegmentDist = 5.0;		//局部地图准备间隔距离
// //		dMatchRadius = 50.0;	
// 	}
// 	std::string szMapDir;
// 	double dLocalMapRange;
// 	double dSegmentDist;
// //	double dMatchRadius;
// };

struct MapInfo
{
	int nId;
	double dTime;
	double dLongitude;
	double dLatitude;
	double dAltitude;
	double dX;
	double dY;
	double dZ;
	double dMile;
	double dRefLongitude;
	double dRefLatitude;
	double dRefAltitude;
	std::string szMapPath;
};

class CLocationOpr
{
public:
	CLocationOpr();
	~CLocationOpr();

	int Init(LocationAPIParam Param);

	void SetResultCallBack(FunctionResultCallBack result);

	int StartLocate(POSE_DATA& init_pose);

	int FeedVelodynePack(PCAP_DATA* pBuf);
	int FeedImuData(POSE_DATA* pImu);

	int Stop();

private:
	POSE_DATA Tf2PoseData(Eigen::Matrix4d& tf, uint64_t lidar_time);
	int NearestKnnSearch(pcl::PointXYZ& point_in, pcl::PointXYZ& point_out, int* id_out=0, float* fSquaredDist_out=0);
	void FeedSectorSector_(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& pt, float start, float end);
	void FeedImuData_(GnssData& imuData);
	int FilterPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& pt_in,
		pcl::PointCloud<pcl::PointXYZI>::Ptr& pt_out);
	int LoadMap();
	int LocateWithMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& pt, Eigen::Matrix4d& tf);
	void LocateThread();
	int GetImuDataByTime(uint64_t nTime, GnssData& imuData);
	void split(std::string& s, std::string& delim,std::vector< std::string >* ret);
	LocationAPIParam m_Param;
	std::list<GnssData> m_ImuList;
	boost::mutex m_ImuLock;
	pcl::PointCloud<pcl::PointXYZI> m_LocalMap;
	Eigen::Matrix4d m_LocalMapTf;
	boost::mutex m_LocalMapLock;
	CMapConstruct m_MapOpr;
	CVelodyneOpr m_VelodyneOpr;
	uint8_t m_szVelBuf[1214];
	Eigen::Affine3f m_Calib;
	int m_nMapInd;
	bool m_bLocalMapIsReady;
	bool m_bIsRunLocation;
	FunctionResultCallBack m_result_callback;

	boost::thread m_thread_Locate;
	std::vector<MapInfo> m_MapInfo;
	pcl::KdTreeFLANN<pcl::PointXYZ> m_kdtree;
	pcl::PointCloud<pcl::PointXYZ> m_track;
	std::vector<Eigen::Matrix4d> m_PoseInMap;
	std::vector<Eigen::Matrix4d> m_PoseDr;

	pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> m_ndt;
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_pt4NdtMappingFilter;
	std::string m_szLogPath;
};


#endif
