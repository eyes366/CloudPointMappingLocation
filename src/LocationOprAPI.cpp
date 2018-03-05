#include "LocationOprAPI.h"
#include "LocationOpr.h"

CLocationOprAPI::CLocationOprAPI()
{
	m_pUser = new CLocationOpr;
}

CLocationOprAPI::~CLocationOprAPI()
{
	if (m_pUser)
	{
		delete ((CLocationOpr*)(m_pUser));
		m_pUser = 0;
	}
}

int CLocationOprAPI::Init(LocationAPIParam Param)
{
	return ((CLocationOpr*)(m_pUser))->Init(Param);
}

void CLocationOprAPI::SetResultCallBack(FunctionResultCallBack result)
{
	return ((CLocationOpr*)(m_pUser))->SetResultCallBack(result);
}

int CLocationOprAPI::StartLocate(POSE_DATA& init_pose)
{
	return ((CLocationOpr*)(m_pUser))->StartLocate(init_pose);
}

int CLocationOprAPI::FeedVelodynePack(PCAP_DATA* pBuf)
{
	return ((CLocationOpr*)(m_pUser))->FeedVelodynePack(pBuf);
}

int CLocationOprAPI::FeedImuData(POSE_DATA* pImu)
{
	return ((CLocationOpr*)(m_pUser))->FeedImuData(pImu);
}