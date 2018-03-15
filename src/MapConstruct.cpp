#include <pcl/common/io.h>
#include "MapConstruct.h"

int CMapConstruct::GetLastPosition(double& x, double& y, double& z, double& dMile)
{
	if (m_poseImuListHistory_.size() <= 0 || m_TotalMile.size() <= 0)
	{
		x = 0.0;
		y = 0.0;
		z = 0.0;
		dMile = 0.0;
		return -1;
	}
	x = m_poseImuListHistory_.back()(0,3);
	y = m_poseImuListHistory_.back()(1,3);
	z = m_poseImuListHistory_.back()(2,3);
	dMile = m_TotalMile.back();

	return 1;
}

int CMapConstruct::GetLastTf(Eigen::Matrix4d& tf)
{
	if (m_poseImuListHistory_.size() <= 0)
	{
		tf = Eigen::Matrix4d::Identity();
		return -1;
	}

	tf = m_poseImuListHistory_.back();

	return 1;
}

int CMapConstruct::GetBaseGpsLocation(GnssData& BaseGps)
{
	if (m_poseImuListHistory.size() <= 0)
	{
		return -1;
	}

	BaseGps = m_poseImuListHistory.front();

	return 1;
}

// int CMapConstruct::AddFrameByGps_CleanType(pcl::PointCloud<pcl::PointXYZI>::Ptr& points,
// 										   GnssData imu_data)
// {
// 	if (m_poseImuListHistory.size() > 0)
// 	{
// 		Imu100HzData last_imu = m_poseImuListHistory.back();
// 		double dDistY = 111319.55*(imu_data.dLatitude - last_imu.dLatitude);
// 		double dDistX = 111319.55*(imu_data.dLongitude - last_imu.dLongitude)*
// 			cos(last_imu.dLatitude/180.0*3.141592654);
// 		double dDist = sqrt(pow(dDistX,2)+pow(dDistY,2));
// 		// 		if (m_poseImuList.size()%100 == 0)
// 		// 		{
// 		// 			std::cout << "diff dist: " << dDist << std::endl;
// 		// 		}
// 		if (dDist < m_dRepeatAvoidDist)
// 		{
// 			return -1;
// 		}
// 		m_dDistAcc += dDist;
// 	}
// 	else
// 	{
// 		m_dDistAcc = 0.0;
// 	}
// 	m_poseImuListHistory.push_back(imu_data);
// 
// 	Imu100HzData first_imu = m_poseImuListHistory.front();
// 	double dDistY = 111319.55*(imu_data.dLatitude - first_imu.dLatitude);
// 	double dDistX = 111319.55*(imu_data.dLongitude - first_imu.dLongitude)*
// 		cos(first_imu.dLatitude/180.0*3.141592654);
// // 	Eigen::Quaterniond q(w,x,y,z);
// // 	Eigen::Matrix4d tf_;
// // 	tf_ = (Eigen::Translation3d(dDistX, dDistY, 0)*q).matrix();
// 	Eigen::Matrix4d tf;
// 	tf = (Eigen::Translation3d(dDistX, dDistY, 0)*
// 		Eigen::AngleAxisd((-1.0*imu_data.fYaw)/180.0*3.141592654, Eigen::Vector3d::UnitZ ()) *
// 		Eigen::AngleAxisd(imu_data.fRoll/180.0*3.141592654, Eigen::Vector3d::UnitX ()) *
// 		Eigen::AngleAxisd(imu_data.fPitch/180.0*3.141592654, Eigen::Vector3d::UnitY ())).matrix();
// 	m_poseImuListHistory_.push_back(tf);
// 
// 	pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>);
// 	pcl::transformPointCloud(*points, *temp, tf);
// 	m_PtList += (*temp);
// 	m_PtContList.push_back(temp->size());
// 
// 	if (m_dDistAcc >= m_dMapSegmentDist)
// 	{
// 		//         m_PtList.erase(m_PtList.begin(), m_PtList.begin()+m_PtContList.back());
// 		//         m_PtContList.erase(m_PtContList.begin(),m_PtContList.begin()+1);
// 		//         m_poseList.erase(m_poseList.begin(), m_poseList.begin()+1);
// 		m_PtListOut = m_PtList;
// 		m_PtList.clear();
// 		m_PtContList.clear();
// 		//		m_poseList.clear();
// 		m_dDistAcc = 0.0;
// 		return 1;
// 	}
// 
// 	return -1/*m_PtList.size()*/;
// }

void CMapConstruct::SetBaseGnssPos(double dLat, double dLong, double dAlt)
{
	m_dBaseLatitude = dLat;
	m_dBaseLongitude = dLong;
	m_dBaseAltitude = dAlt;
	m_bIsBaseSet = true;
}

int CMapConstruct::AddFrameByGps_QueueType(pcl::PointCloud<pcl::PointXYZI>::Ptr& points,
										   GnssData imu_data)
{
	if (!m_bIsBaseSet)//如果没有设置过坐标原点，则用第一帧gnss作为原点
	{
		m_dBaseLatitude = imu_data.dLatitude;
		m_dBaseLongitude = imu_data.dLongitude;
		m_dBaseAltitude = imu_data.dAltitude;
		m_bIsBaseSet = true;
	}
	if (m_poseImuListHistory.size() > 0)
	{
		GnssData last_imu = m_poseImuListHistory.back();
		double dDistY = 111319.55*(imu_data.dLatitude - last_imu.dLatitude);
		double dDistX = 111319.55*(imu_data.dLongitude - last_imu.dLongitude)*
			cos(last_imu.dLatitude/180.0*3.141592654);
		double dDist = sqrt(pow(dDistX,2)+pow(dDistY,2));
		if (dDist < m_dRepeatAvoidDist)
		{
			return -1;
		}
		m_dDistAcc += dDist;
		m_TotalMile.push_back(m_TotalMile.back()+dDist);
	}
	else
	{
		m_dDistAcc = 0.0;
		m_TotalMile.push_back(0.0);
	}
	m_poseImuListHistory.push_back(imu_data);

//	GnssData first_imu = m_poseImuListHistory.front();
	double dDistY = 111319.55*(imu_data.dLatitude - m_dBaseLatitude/*first_imu.dLatitude*/);
	double dDistX = 111319.55*(imu_data.dLongitude - m_dBaseLongitude/*first_imu.dLongitude*/)*
		cos(m_dBaseLatitude/*first_imu.dLatitude*//180.0*3.141592654);
	double dDistZ = imu_data.dAltitude - m_dBaseAltitude/*first_imu.dAltitude*/;
	Eigen::Matrix4d tf;
	if (imu_data.nPoseType == 0)//Euler
	{
		tf = (Eigen::Translation3d(dDistX, dDistY, dDistZ)*
			Eigen::AngleAxisd((-1.0*imu_data.dHeading)/180.0*3.141592654, Eigen::Vector3d::UnitZ ()) *
			Eigen::AngleAxisd(imu_data.dRoll/180.0*3.141592654, Eigen::Vector3d::UnitX ()) *
			Eigen::AngleAxisd(imu_data.dPitch/180.0*3.141592654, Eigen::Vector3d::UnitY ())).matrix();
	}
	else//Quaternion
	{
// 		Eigen::Quaterniond q(imu_data.qw, imu_data.qx, imu_data.qy, imu_data.qz);
// 		Eigen::Isometry3d h(q.toRotationMatrix());
// 		h.translate(Eigen::Vector3d(dDistX, dDistY, dDistZ));
// 		tf = h.matrix();
		Eigen::Isometry3d tf_;
		tf_ = Eigen::Quaterniond(imu_data.qw, imu_data.qx, imu_data.qy, imu_data.qz);
		tf_(0, 3) = dDistX;
		tf_(1, 3) = dDistY;
		tf_(2, 3) = dDistZ;
		tf = tf_.matrix();
//		*(Eigen::Translation3d(0, 0, 0)*
//			Eigen::AngleAxisd(-3.141592653589*0.5, Eigen::Vector3d::UnitZ ())*
//			Eigen::AngleAxisd(3.141592653589, Eigen::Vector3d::UnitY ())).matrix();
	}
	m_poseImuListHistory_.push_back(tf);

	pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::transformPointCloud(*points, *temp, tf);
	m_PtList += (*temp);
	m_PtContList.push_back(temp->size());

	if ((m_dDistAcc >= m_dMapSegmentDist && 
		m_TotalMile.back() - m_TotalMile.front() >= m_dQueueDist))
	{
		int nRemoveInd = 0;
		size_t nRemovePointCont = 0;
		double dLastMile = m_TotalMile.back();
		for (unsigned int i = 0; i < m_TotalMile.size(); i++)
		{
			if (dLastMile - m_TotalMile[i] <= m_dQueueDist)
			{
				nRemoveInd = i;
				break;
			}
			else
			{
				nRemovePointCont += m_PtContList[i];
			}
		}

		size_t nTotalCont = 0;
		for (unsigned int i = 0; i < m_PtContList.size(); i++)
		{
			nTotalCont += m_PtContList[i];
		}

// 		  		std::cout << "erase start ..." << std::endl;
// 		 		std::cout << "point cont(m_PtList):" << m_PtList.size() 
// 		 			<< " point cont(m_PtContList)" << nTotalCont
// 		  			<< " remove point cont:" << nRemovePointCont
// 		 			<< " remove ind" << nRemoveInd 
// 		 			<< std::endl;
		m_PtList.erase(m_PtList.begin(), m_PtList.begin()+nRemovePointCont);
		m_PtContList.erase(m_PtContList.begin(),m_PtContList.begin()+nRemoveInd);
		m_TotalMile.erase(m_TotalMile.begin(),m_TotalMile.begin()+nRemoveInd);
//				std::cout << "erase finish ..." << std::endl;

		m_PtListOut = m_PtList;

		m_dDistAcc = 0.0;
		return 1;
	}

	return -1/*m_PtList.size()*/;
}

// int CMapConstruct::AddFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr& points,
// 							Eigen::Isometry3d& pose)
// {
// 	//Whether robot move far enough to add point cloud into map
// 	if (m_PtContList.size() > 0)
// 	{
// 		Eigen::Isometry3d& last_pose = m_poseList.back();
// 		double dDist = sqrt(pow((last_pose(0,3)-pose(0,3)),2) +
// 			pow((last_pose(1,3)-pose(1,3)),2));
// 		if (dDist < m_dMapInterval)
// 		{
// 			return -1;
// 		}
// 	}
// 
// 	pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>);
// 	pcl::transformPointCloud(*points, *temp, pose.matrix());
// 	m_PtList += (*temp);
// 	m_PtContList.push_back(temp->size());
// 	m_poseList.push_back(pose);
// 
// 	if (m_PtContList.size() > m_nLatestCapacity)
// 	{
// 		m_PtList.erase(m_PtList.begin(), m_PtList.begin()+m_PtContList.back());
// 		m_PtContList.erase(m_PtContList.begin(),m_PtContList.begin()+1);
// 		m_poseList.erase(m_poseList.begin(), m_poseList.begin()+1);
// 	}
// 
// 	return m_PtList.size();
// }

int CMapConstruct::GetMap(pcl::PointCloud<pcl::PointXYZI>& points)
{
	//    (*points) = m_PtListOut;
	pcl::copyPointCloud<pcl::PointXYZI, pcl::PointXYZI>(m_PtListOut, points);

	return 1/*points->size()*/;
}
