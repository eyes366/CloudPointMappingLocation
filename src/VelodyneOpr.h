#ifndef _VELODYNE_OPR_H_
#define _VELODYNE_OPR_H_

#include <boost/function/function1.hpp>
#include <boost/signals2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/grabber.h>
#include "Imu100Hz.h"
#include "MapConstruct.h"

typedef boost::function<void(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&)> VelodyneCallBackSweep;
typedef boost::function<void(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&, float, float)> VelodyneCallBackSector;

struct VelodyneDataReadParam
{
	VelodyneDataReadParam()
	{
		nDevType = 0;
		nReadType = 0;
		nDataFetchType = 0;
		bool bUseExternalCallBack = false;
		nPort = 0;
	}
	int nDevType;	//0:VLP-16 1:HDL-32
	int nReadType;	//0:From EtherNet 1:From pcap
	int nDataFetchType;	//0:queue By time 1:Continue wait
	int bUseExternalCallBack;	//使用外部数据回掉
	std::string szPcapPath;
	std::string szIp;
	int nPort;
};

class CVelodyneOpr
{
public:
	CVelodyneOpr();
	~CVelodyneOpr();

	int Init(VelodyneDataReadParam& Param);

	void SetCallBackSweep(VelodyneCallBackSweep& callback_sweep);
	void SetCallBackSector(VelodyneCallBackSector& callback_sector);

	int Start();

	int GetData(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

	int NextFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
	
	void enqueueHDLPacket (const uint8_t *data, std::size_t bytesReceived);

	int Stop();

private:
	void sectorScan(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& pt, float start, float end);
	void sweepScan(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& pt);
	VelodyneDataReadParam m_Param;
	VelodyneCallBackSweep m_callback_sweep;
	VelodyneCallBackSector m_callback_sector;
	boost::signals2::connection m_cloud_connection_sweep;
	boost::signals2::connection m_cloud_connection_sector;
	pcl::Grabber* m_pgrabber;
	pcl::PointCloud<pcl::PointXYZI> m_PointCloudSweep;
	boost::mutex m_cloud_mutex;
	bool m_bIsNewData;
// 	CImu100HzReader m_ImuOpr;
// 	pcl::PointCloud<pcl::PointXYZI> m_PointCloudMap;
// 	CMapConstruct m_MapOpr;
// 	Eigen::Affine3f m_Calib;
// 	int m_nMapInd;
 	int m_nSweepCont;
};


#endif
