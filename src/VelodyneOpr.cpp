#include <pcl/io/pcd_io.h>
#include "vlp_grabber_.h"
#include "hdl_grabber_.h"
#include "VelodyneOpr.h"

using namespace pcl;

CVelodyneOpr::CVelodyneOpr()
{
	m_pgrabber = 0;
	m_bIsNewData =  false;

// 	Eigen::Affine3f calib =
// 		(Eigen::Translation3f (Eigen::Vector3f (-0.186986, -1.376150, -1.376150)) *
// 		Eigen::AngleAxisf (-1.0*-1.563736,   Eigen::Vector3f::UnitZ ()) *
// 		Eigen::AngleAxisf (3.163187, Eigen::Vector3f::UnitX ()) *
// 		Eigen::AngleAxisf (-0.790054,  Eigen::Vector3f::UnitY ()));
// 	Eigen::Matrix4f tt;
// 	tt << 0, 0, -1, 0,
// 		0, 1, 0, 0,
// 		1, 0, 0, 0,
// 		0, 0, 0, 1;
// 	m_Calib = calib*tt;
// 
// 	m_MapOpr.m_dMapInterval = -0.1;
// 	m_MapOpr.m_nLatestCapacity = 16000;
}

CVelodyneOpr::~CVelodyneOpr()
{
	if (m_pgrabber)
	{
		delete m_pgrabber;
		m_pgrabber = 0;
	}
}

int CVelodyneOpr::Init(VelodyneDataReadParam& Param)
{
	m_Param = Param;
	if (m_pgrabber)
	{
		delete m_pgrabber;
		m_pgrabber = 0;
	}
	if (m_Param.nDevType == 0)//vlp-16
	{
		if (m_Param.nReadType == 0)//on line
		{
			m_pgrabber = new VLPGrabber(boost::asio::ip::address_v4::from_string(m_Param.szIp), m_Param.nPort);
		}
		else//pcap
		{
			m_pgrabber = new VLPGrabber(m_Param.szPcapPath);
		}
	}
	else
	{
		if (m_Param.nReadType == 0)//on line
		{
			m_pgrabber = new HDLGrabber(boost::asio::ip::address_v4::from_string(m_Param.szIp), m_Param.nPort);
		}
		else//pcap
		{
			m_pgrabber = new HDLGrabber("", m_Param.szPcapPath);
		}
	}

	((HDLGrabber*)m_pgrabber)->m_nFileType = m_Param.nReadType;

	((HDLGrabber*)m_pgrabber)->m_nDataFetchType = m_Param.nDataFetchType;

	((HDLGrabber*)m_pgrabber)->m_bUseExternalCallBack = m_Param.bUseExternalCallBack;

	((HDLGrabber*)m_pgrabber)->m_bIncludeNanPoint = m_Param.bIncludeNanPoint;

 	m_nSweepCont= 0;

	return 1;
}

int CVelodyneOpr::Start()
{
	VelodyneCallBackSweep callback_sweep = boost::bind(&CVelodyneOpr::sweepScan, this, _1);
	m_cloud_connection_sweep = m_pgrabber->registerCallback(callback_sweep);

//	if (!m_callback_sector.empty())
	{
		VelodyneCallBackSector callback_sector = boost::bind(&CVelodyneOpr::sectorScan, this, _1, _2, _3);
		m_cloud_connection_sector = m_pgrabber->registerCallback(callback_sector);
	}

	m_pgrabber->start();

	return 1;
}

int CVelodyneOpr::Stop()
{
	m_pgrabber->stop();
	m_cloud_connection_sweep.disconnect();
	m_cloud_connection_sector.disconnect();

	return 1;
}

void CVelodyneOpr::SetCallBackSweep(VelodyneCallBackSweep& callback_sweep)
{
	m_callback_sweep = callback_sweep;
}

void CVelodyneOpr::SetCallBackSector(VelodyneCallBackSector& callback_sector)
{
	m_callback_sector = callback_sector;
}

void CVelodyneOpr::sectorScan(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& pt, float start, float end)
{
	if (!m_callback_sector.empty())
	{
		m_callback_sector(pt, start, end);
	}
}

void CVelodyneOpr::sweepScan(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& pt)
{
	if (!m_callback_sweep.empty())
	{
		m_callback_sweep(pt);
	}

	m_cloud_mutex.lock();
	m_PointCloudSweep = *pt;
	m_bIsNewData = true;
	m_cloud_mutex.unlock();
	if (m_nSweepCont%100 == 0)
	{
		std::cout << "Sweep: seq:" << pt->header.seq << " stamp:" << pt->header.stamp 
			<< " points:" << pt->size() << std::endl;
	}

	while (m_bIsNewData)
	{
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	}
	
	m_nSweepCont++;
}

int CVelodyneOpr::GetData(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	m_cloud_mutex.lock();
	if (!m_bIsNewData)
	{
		m_cloud_mutex.unlock ();
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
		return 0;
	}
	*cloud = m_PointCloudSweep;
	m_bIsNewData = false;
	m_cloud_mutex.unlock ();
	return 1;
}

int CVelodyneOpr::NextFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	if (m_Param.nDataFetchType == 0)
	{
		std::cout << "m_Param.nDataFetchType == 0" << std::endl;
		return 0;
	}
	((HDLGrabber*)m_pgrabber)->resume();
	int nRepeat = 0;
	while (1)
	{
		int nRt = GetData(cloud);
		if (nRt == 0)
		{
			nRepeat++;
			if (nRepeat >= 100)
			{
				((HDLGrabber*)m_pgrabber)->pause();
				return 0;
			}
			else
			{
				continue;
			}
		}
		else
		{
			((pcl::HDLGrabber*)m_pgrabber)->pause();

			return 1;
		}
	}


	((pcl::HDLGrabber*)m_pgrabber)->pause();

	return 0;
}

void CVelodyneOpr::enqueueHDLPacket (const uint8_t *data, std::size_t bytesReceived)
{
	((pcl::HDLGrabber*)m_pgrabber)->enqueueHDLPacket(data, bytesReceived);
}