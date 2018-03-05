#ifndef _DATA_SIMULATOR_H_
#define _DATA_SIMULATOR_H_

#include <string>
#include "common.h"
#include "ImuCalc.h"
#include "VelodyneOpr.h"
#include "CommonHeader.h"

typedef boost::function<int(PCAP_DATA*)> FunctionVelodyneFeedCallBack;
typedef boost::function<int(POSE_DATA*)> FunctionImuFeedCallBack;

class CDataSimulator
{
public:
	CDataSimulator();
	~CDataSimulator();

	void SetCallBack(FunctionVelodyneFeedCallBack VelodyneCallBack,
		FunctionImuFeedCallBack ImuCallBack);

	int StartSimulator(std::string szLogDir);

private:
	void ImuReplayThread();
// 	void VelodyneSectorCallBack(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& 
// 		point_sector, float start, float end);
	FunctionVelodyneFeedCallBack m_VelodyneCallBack;
	FunctionImuFeedCallBack m_ImuCallBack;
//	CImu100HzReader m_ImuOpr;
	CImuCalc m_ImuOpr;
//	CVelodyneOpr m_VelOpr;
	std::string m_szLogDir;
	unsigned int m_nImuSearchInd;
	boost::mutex m_lock;
	uint64_t m_nVelodyneTime;
};


#endif
