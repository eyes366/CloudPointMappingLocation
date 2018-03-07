#pragma once
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

class NdtVisual
{
public:
	NdtVisual();
	~NdtVisual();

	void DisplayCloudPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr& cp_map,
		pcl::PointCloud<pcl::PointXYZI>::Ptr& cp_lidar);

	void SpinOnce();

private:
	int m_nCont;
	void DispThread();
	pcl::visualization::PCLVisualizer* m_p;
	int m_vp1;
	int m_nFreshTime;
	boost::thread m_disp_thread;
	boost::asio::io_service m_io;
	boost::asio::deadline_timer m_timer;
	pcl::PointCloud<pcl::PointXYZI> m_map;
	pcl::PointCloud<pcl::PointXYZI> m_lidar;
	boost::mutex m_lock;
};

