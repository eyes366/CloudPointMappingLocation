#include<boost/thread.hpp>
#include "DataSimulator.h"

using namespace std;

CDataSimulator::CDataSimulator()
{

}

CDataSimulator::~CDataSimulator()
{

}

void CDataSimulator::SetCallBack(FunctionVelodyneFeedCallBack VelodyneCallBack,
								 FunctionImuFeedCallBack ImuCallBack)
{
	m_VelodyneCallBack = VelodyneCallBack;
	m_ImuCallBack = ImuCallBack;
}

int CDataSimulator::StartSimulator(std::string szLogDir)
{
	m_nVelodyneTime = 0;
	m_nImuSearchInd = 0;

	m_szLogDir = szLogDir;
	int nRt = m_ImuOpr.ReadFile(m_szLogDir+"imu.txt");
	if (nRt <= 0)
	{
		return -1;
	}
	
	boost::thread imuThread(boost::bind(&CDataSimulator::ImuReplayThread, this));

	std::ifstream fs((m_szLogDir+string("test.vel32")).c_str(),
		std::ios::binary);
	if (!fs.is_open())
	{
		std::cout << "readPacketsFromLeadorLog::Open file log failed!!!" << std::endl;
		getchar();
	}
	int nDataLen = sizeof(PCAP_DATA);
	PCAP_DATA buf;
	uint8_t szTemp[1214] = {0};
	uint64_t nVelodyneCont = 0;
	boost::system_time start_sys_time;
	uint64_t start_log_time = 0;
	while (!fs.eof())
	{
		fs.read((char*)(&buf), nDataLen);
		uint64_t nTimeNow = uint64_t(buf.gps_second*1000000.0);
		if (nTimeNow < 353475000000)
		{
			continue;
		}
		if (nVelodyneCont == 0)
		{
			start_sys_time = boost::get_system_time();
			start_log_time = nTimeNow;
		}
		else
		{
			uint64_t delta_log_time = nTimeNow - start_log_time;
			boost::system_time sleep_to = start_sys_time + boost::posix_time::microseconds(delta_log_time);
			boost::thread::sleep(sleep_to);
		}
		m_VelodyneCallBack(&buf);
		m_nVelodyneTime = nTimeNow;
		if (nVelodyneCont%2000 == 0)
		{
			printf("velodyne package cont: %d time:%lld\n", nVelodyneCont, nTimeNow);
		}
		nVelodyneCont++;
	}
	std::cout<<std::endl;
	fs.close();


	return 1;
}

// void CDataSimulator::VelodyneSectorCallBack(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& 
// 											point_sector, float start, float end)
// {
// // 	if (point_sector->header.seq % 80 == 0)
// // 	{
// // 		printf("Lidar time: %lld\n", point_sector->header.stamp);
// // 	}
// 	m_nVelodyneTime = point_sector->header.stamp;
// 	m_VelodyneCallBack(point_sector, start, end);
// }

void CDataSimulator::ImuReplayThread()
{
	uint32_t nImuCont = 0;
	while (true)
	{
		for (unsigned int i = m_nImuSearchInd; i < m_ImuOpr.m_Data.size(); i++)
		{
			uint64_t nTime = uint64_t(m_ImuOpr.m_Data[i].dSecInWeek);
			if (nTime  <= m_nVelodyneTime/* + 50000*/)
			{
				POSE_DATA pose_data = m_ImuOpr.m_Data[i].toPOSE_DATA();
				pose_data.gps_second /= 1000000.0;
				m_ImuCallBack(&pose_data);
				if (nImuCont%200 == 0)
				{
					printf("imu package cont: %d time:%.6f\n", nImuCont, pose_data.gps_second*1000000.0);
				}
				nImuCont++;
			}
			else
			{
				boost::this_thread::sleep(boost::posix_time::milliseconds(1));
				m_nImuSearchInd = i;
				break;
			}
		}
	}
}