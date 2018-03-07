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

	int StartSimulator(std::string szLogDir, uint64_t nStartTime = uint64_t(0));

private:
	void LidarReplayThread();
	void ImuReplayThread();
	FunctionVelodyneFeedCallBack m_VelodyneCallBack;
	FunctionImuFeedCallBack m_ImuCallBack;
	CImuCalc m_ImuOpr;
	std::string m_szLogDir;
	unsigned int m_nImuSearchInd;
	boost::mutex m_lock;
	uint64_t m_nVelodyneTime;
	uint64_t m_nStartTime;
};


#endif
