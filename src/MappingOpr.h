#ifndef _MAPPING_OPR_H_
#define _MAPPING_OPR_H_

#include <string>
#include <fstream>
#include "VelodyneOpr.h"
#include "ImuNav.h"
#include "ImuCalc.h"
#include "common.h"
#include "MapConstruct.h"


class CMappingOpr
{
public:
	CMappingOpr();
	~CMappingOpr();

	int StartMapping(std::string szDataPath);

	void sectorScan(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& pt, float start, float end);
	void sweepScan(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& pt);

private:
	CVelodyneOpr m_VelodyneOpr;
	CImuNav m_ImuOpr;
//	CImuCalc m_ImuOpr;
	pcl::PointCloud<pcl::PointXYZI> m_PointCloudMap;
	CMapConstruct m_MapOpr;
	Eigen::Affine3d m_Calib;
	int m_nMapInd;
	int m_nSweepCont;
	std::ofstream m_fsFileMapLog;
	std::string m_szDataPath;
	std::string m_szMapPath;
	float m_fLeafSize;
	GnssData m_GpsBase;
	pcl::PointCloud<pcl::PointXYZI> m_mapT;
};

#endif
