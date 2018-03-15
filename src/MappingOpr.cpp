#include <iomanip>
#include <iostream>
#include <string>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "vlp_grabber_.h"
#include "hdl_grabber_.h"
#include "MappingOpr.h"

using namespace pcl;
using namespace std;

CMappingOpr::CMappingOpr()
{
	Eigen::Affine3d calib =
		(Eigen::Translation3d (Eigen::Vector3d (-0.186986, -1.376150, -1.376150)) *
		Eigen::AngleAxisd (-1.0*-1.563736,   Eigen::Vector3d::UnitZ ()) *
		Eigen::AngleAxisd (3.163187, Eigen::Vector3d::UnitX ()) *
		Eigen::AngleAxisd (-0.790054,  Eigen::Vector3d::UnitY ()));
	Eigen::Matrix4d tt;
	tt << 0, 0, -1, 0,
		0, 1, 0, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;
	m_Calib = calib*tt;
// 	Eigen::Affine3d calib =
// 		(Eigen::Translation3d (Eigen::Vector3d (-0.186986, -1.376150, -1.376150)) *
// 		Eigen::AngleAxisd (-3.141592653589*0.5,   Eigen::Vector3d::UnitZ ()) *
// 		Eigen::AngleAxisd (3.141592653589,   Eigen::Vector3d::UnitY ()) *
// 		Eigen::AngleAxisd (-1.0*-1.563736,   Eigen::Vector3d::UnitZ ()) *
// 		Eigen::AngleAxisd (3.163187, Eigen::Vector3d::UnitX ()) *
// 		Eigen::AngleAxisd (-0.790054,  Eigen::Vector3d::UnitY ()));
// 	Eigen::Matrix4d tt;
// 	tt << 0, 0, -1, 0,
// 		0, 1, 0, 0,
// 		1, 0, 0, 0,
// 		0, 0, 0, 1;
// 	m_Calib = calib*tt;

	m_MapOpr.m_dRepeatAvoidDist = 0.00001;// 0.00001m/0.0005s=0.02m/s=0.07km/h
	m_MapOpr.m_dMapSegmentDist = 5.0;
	m_MapOpr.m_dQueueDist = 5.0;
	m_fLeafSize = 0.5;
}

CMappingOpr::~CMappingOpr()
{
}

int CMappingOpr::StartMapping(std::string szDataPath)
{
	if (szDataPath.back() != '\\' && szDataPath.back() != '/')
	{
		m_szDataPath = szDataPath+string("/");
	}
	else
	{
		m_szDataPath = szDataPath;
	}

	string szNavFilePath, szLidarFilePath;
	boost::filesystem::path full_path(m_szDataPath, boost::filesystem::native);
	if (boost::filesystem::exists(full_path))
	{
		boost::filesystem::directory_iterator item_begin(full_path);
		boost::filesystem::directory_iterator item_end;
		for (; item_begin != item_end; item_begin++)
		{
			if (!boost::filesystem::is_directory(*item_begin))
			{
				string file_path_name = item_begin->path().string();
				int nPos = file_path_name.find(".vel32");
				if (nPos > 0 )
				{
					szLidarFilePath = file_path_name;
				}
				nPos = file_path_name.find(".nav");
				if (nPos > 0)
				{
					szNavFilePath = file_path_name;
				}
			}
		}
	}
	else
	{
		return -1;
	}

	cout << "nav: " << szNavFilePath << endl;
	cout << "lidar: " << szLidarFilePath << endl;
	if (szLidarFilePath == "" || szNavFilePath == "")
	{
		cout << "File not found!!!" << endl;
		return -1;
	}

	cout << "Loading nav data..." << endl;
	m_ImuOpr.ReadFile(szNavFilePath);
//	m_ImuOpr.ReadFile(m_szDataPath+"imu.txt");
	cout << "Finish loading nav data..." << endl;
	m_nMapInd = 0;
	m_nSweepCont= 0;

	m_szMapPath = m_szDataPath+"map/";
	boost::filesystem::create_directories(m_szMapPath);

	m_fsFileMapLog.open(m_szMapPath+"MapInfo.txt");

	VelodyneDataReadParam VelParam;
	VelParam.nDevType = 1;
	VelParam.nReadType = 1;
	VelParam.nDataFetchType = 1;
	VelParam.szPcapPath = szLidarFilePath;// m_szDataPath + "test.vel32";
	CVelodyneOpr VelOpr;
	m_VelodyneOpr.Init(VelParam);
	VelodyneCallBackSweep callback_sweep = boost::bind(&CMappingOpr::sweepScan, this, _1);
	m_VelodyneOpr.SetCallBackSweep(callback_sweep);
	VelodyneCallBackSector callback_sector = boost::bind(&CMappingOpr::sectorScan, this, _1, _2, _3);
	m_VelodyneOpr.SetCallBackSector(callback_sector);
	m_VelodyneOpr.Start();

	return 1;
}

void CMappingOpr::sectorScan(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& pt, float start, float end)
{
	static int g_nSectorCont = 0;
	g_nSectorCont++;
	PointCloud<PointXYZI> mapT;//= *pt;
	mapT.header = pt->header;
	for (unsigned int i = 0; i < pt->size(); i++)
	{
		if (pt->points[i].x < 2.0 && pt->points[i].x > -2.0 && 
			pt->points[i].y < 2.0 && pt->points[i].y > -2.0 && 
			pt->points[i].z < 2.0 && pt->points[i].z > -2.0)
		{
			continue;
		}
		mapT.push_back(pt->points[i]);
	}

	pcl::transformPointCloud(mapT, mapT, m_Calib);
	GnssData gpsData;
	int nImuRt = m_ImuOpr.GetDataByTime(mapT.header.stamp, gpsData);
	if (g_nSectorCont%100000 == 0)
	{
		printf("GetDataByTime == %d: time:%.6f\n", nImuRt, gpsData.dSecInWeek);
	}
	if (nImuRt <= 0)
	{
		return;
	}

// 	if (m_GpsBase.dGpsTime == 0.0)
// 	{
// 		m_GpsBase = gpsData;
// 		cout << "base gps: " << m_GpsBase.dLatitude << "," << m_GpsBase.dLongitude << endl;
// 	}
// 	double dDistY = 111319.55*(gpsData.dLatitude - m_GpsBase.dLatitude);
// 	double dDistX = 111319.55*(gpsData.dLongitude - m_GpsBase.dLongitude)*
// 		cos(m_GpsBase.dLatitude/180.0*3.141592654);
// 	double dDistZ = gpsData.dAltitude - m_GpsBase.dAltitude;
// 
// 	Eigen::Isometry3d tf;
// 	tf = Eigen::Quaterniond(gpsData.qw, gpsData.qx, gpsData.qy, gpsData.qz);
// 	tf(0, 3) = dDistX;
// 	tf(1, 3) =dDistY;
// 	tf(2, 3) = dDistZ;
// 
// 	tf = tf*m_Calib.matrix();
// 	pcl::transformPointCloud(mapT, mapT, tf.matrix());
// 	m_mapT += mapT;
// 	if (m_mapT.size() >= 1000000)
// 	{
// 		static int g_nSaveInd = 0;
// 		char szLogPath_[1024] = {0};
// 		sprintf_s(szLogPath_, 1024, "lidar%06d.pcd", g_nSaveInd++);
// 		pcl::io::savePCDFileBinary(szLogPath_, m_mapT);
// 		cout << szLogPath_ << endl;
// 		m_mapT.clear();
// 	}
// 
// 	return;
	static int g_nMapCont = 0;
	g_nMapCont++;
	int nMapRt = m_MapOpr.AddFrameByGps_QueueType(mapT.makeShared(), gpsData);
// 	if (g_nMapCont%1000 == 0)
// 	{
// 		printf("AddFrameByGps_QueueType == %d: time:%.6f", nMapRt, gpsData.dSecInWeek);
// 	}
	if (nMapRt < 0)
	{
		return;
	}
	else
	{
		PointCloud<PointXYZI> map, mapFilter;
		m_MapOpr.GetMap(map);

// 		pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
// 		sor.setInputCloud(map.makeShared());
// 		sor.setMeanK(50);
// 		sor.setStddevMulThresh(1.0);
// 		sor.filter();

		pcl::VoxelGrid<pcl::PointXYZI> vg;
		vg.setInputCloud (map.makeShared());
		vg.setLeafSize (m_fLeafSize, m_fLeafSize, m_fLeafSize);
		vg.filter (mapFilter);
//		mapFilter = map;
		std::cout << "Point cont filtered!" << mapFilter.size() << std::endl;
		double dCurX, dCurY, dCurZ, dCurMile;
		m_MapOpr.GetLastPosition(dCurX, dCurY, dCurZ, dCurMile);
		GnssData BaseGps;
		m_MapOpr.GetBaseGpsLocation(BaseGps);
		char szFileName[1024] = {0};
		sprintf(szFileName, "%06d.pcd", m_nMapInd);
		pcl::io::savePCDFileBinary(m_szMapPath+szFileName, mapFilter);
		std::cout << "++++++++++++++++++++++++++++++" << std::endl;
		std::cout << szFileName << "Point cont:" << mapFilter.size() <<
			" stamp:" << mapT.header.stamp << std::endl;
		std::cout << "++++++++++++++++++++++++++++++" << std::endl;
		char szLog[1024] = {0};
		sprintf_s(szLog, 1024, "%d %.6f %.7f %.7f %.3f %.3f %.3f %.3f %.3f %.7f %.7f %.3f\n",
			m_nMapInd, gpsData.dSecInWeek, gpsData.dLongitude,
			gpsData.dLatitude, gpsData.dAltitude, dCurX, dCurY, dCurZ, 
			dCurMile, BaseGps.dLongitude, BaseGps.dLatitude, BaseGps.dAltitude);
		m_fsFileMapLog.write(szLog, strlen(szLog));
		m_fsFileMapLog.flush();
		m_nMapInd++;
	}
}

void CMappingOpr::sweepScan(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& pt)
{
	return;
	PointCloud<PointXYZI> mapT;//= *pt;
	mapT.header = pt->header;
	for (unsigned int i = 0; i < pt->size(); i++)
	{
		if (pt->points[i].x < 2.0 && pt->points[i].x > -2.0 && 
			pt->points[i].y < 2.0 && pt->points[i].y > -2.0 && 
			pt->points[i].z < 2.0 && pt->points[i].z > -2.0)
		{
			continue;
		}
		mapT.push_back(pt->points[i]);
	}

	pcl::transformPointCloud(mapT, mapT, m_Calib);
	GnssData gpsData;
	if (m_ImuOpr.GetDataByTime(mapT.header.stamp, gpsData) > 0)
	{
		Eigen::Quaterniond q(gpsData.qw, gpsData.qx, gpsData.qy, gpsData.qz);
		Eigen::Isometry3d h(q.toRotationMatrix());
		h.translate(Eigen::Vector3d(0, 0, 0));
		Eigen::Matrix4d tf = h.matrix();
		pcl::transformPointCloud(mapT, mapT, tf);
		static int g_cont = 0;
		char szLog[1024] = {0};
		sprintf_s(szLog, 1024, "aaa%06d.pcd", g_cont);
		g_cont++;
		pcl::io::savePCDFileBinary(szLog, mapT);
//		cout << gpsData.dSecInWeek << "," << gpsData.qw << "," << gpsData.qx << "," << gpsData.qy << "," << gpsData.qz << endl;
//		getchar();

		return;
	}
}