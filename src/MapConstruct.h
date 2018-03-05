#pragma once

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <vector>
#include <string>
#include <limits.h>
#include "common.h"

class CMapConstruct
{
public:

    CMapConstruct()
    {
        m_dRepeatAvoidDist = -1.0;//移动距离小于m_dRepeatAvoidDist，则不要此数据，过滤停车数据
//        m_nLatestCapacity = 20;
		m_dMapSegmentDist = 10.0;//每移动m_dMapSegmentDist距离，则输出一次地图
		m_dQueueDist = 50.0;//每一次地图包含的范围
    }

//     int AddFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr& points,
//                  Eigen::Isometry3d& pose);
	//达到指定距离m_dMapSegmentDist，则返回地图，并清除局部地图，重新叠加局部地图
//     int AddFrameByGps_CleanType(pcl::PointCloud<pcl::PointXYZI>::Ptr& points,
//                  GnssData imu_data);
	//达到指定距离m_dMapSegmentDist，则返回地图，并更新局部地图
	int AddFrameByGps_QueueType(pcl::PointCloud<pcl::PointXYZI>::Ptr& points,
		GnssData imu_data);
    int GetMap(pcl::PointCloud<pcl::PointXYZI>& points);
    int GetLastPosition(double& x, double& y, double& z, double& dMile);
	int GetLastTf(Eigen::Matrix4d& tf);
	int GetBaseGpsLocation(GnssData& BaseGps);

	double m_dRepeatAvoidDist;
//	unsigned int m_nLatestCapacity;
	double m_dMapSegmentDist;
	double m_dQueueDist;

private:
    pcl::PointCloud<pcl::PointXYZI> m_PtList;
//    std::vector<Eigen::Isometry3d> m_poseList;
    std::vector<GnssData> m_poseImuListHistory;
    std::vector<Eigen::Matrix4d> m_poseImuListHistory_;
    std::vector<size_t> m_PtContList;
	std::vector<double> m_TotalMile;
	double m_dDistAcc;

	pcl::PointCloud<pcl::PointXYZI> m_PtListOut;
};
