#include <time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include "vlp_grabber_.h"
#include "hdl_grabber_.h"
#include "LocationOpr.h"

using namespace pcl;
using namespace std;

CLocationOpr::CLocationOpr():
	m_pt4NdtMappingFilter(new PointCloud<PointXYZI>)
{
	m_bIsRunLocation = false;
}

CLocationOpr::~CLocationOpr()
{
}

int CLocationOpr::Init(LocationAPIParam Param)
{
	m_Param = Param;

	m_nMapInd = 0;

	Eigen::Affine3f calib =
		(Eigen::Translation3f (Eigen::Vector3f (-0.186986, -1.376150, -1.376150)) *
		Eigen::AngleAxisf (-1.0*-1.563736,   Eigen::Vector3f::UnitZ ()) *
		Eigen::AngleAxisf (3.163187, Eigen::Vector3f::UnitX ()) *
		Eigen::AngleAxisf (-0.790054,  Eigen::Vector3f::UnitY ()));

	Eigen::Matrix4f tt;
	tt << 0, 0, -1, 0,
		0, 1, 0, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;
	m_Calib = calib*tt;

	Eigen::Isometry3f calib32_16;
	calib32_16.matrix() << 3.5603612272025249e-002, -9.9904256492177179e-001,   2.5423144331705675e-002, 3.4874775702507222e-003,
		6.9741382768883076e-001, 4.3058732817540829e-002,     7.1537395708577567e-001, 2.9969301888692856e-001,
		-7.1578372134437152e-001, -7.7394445973259299e-003,   6.9827914565432059e-001, -2.7290447621211117e-001, 
		0., 0., 0., 1.;
	m_Calib = m_Calib*calib32_16.inverse();

	m_MapOpr.m_dRepeatAvoidDist = 0.0001;// 0.0001m/0.0005s=0.2m/s=0.7km/h
	m_MapOpr.m_dMapSegmentDist = m_Param.dSegmentDist/*5.0*/;
	m_MapOpr.m_dQueueDist = m_Param.dLocalMapRange/*50.0*/;

	LoadMap();
	if (m_MapInfo.size() <= 0)
	{
		return -1;
	}

	// 	m_ndt.setTransformationEpsilon(0.01);
	// 	m_ndt.setStepSize(2.0);//3.0
	// 	m_ndt.setResolution(2.0);//4.0
	// 	m_ndt.setMaximumIterations(50);
	// 	m_ndt.setInputTarget(m_pt4NdtMappingFilter);

	return 1;
}

void CLocationOpr::SetResultCallBack(FunctionResultCallBack result)
{
	m_result_callback = result;
}

int CLocationOpr::StartLocate(POSE_DATA& init_pose)
{
	time_t t = std::time(0);
	tm* local = localtime(&t);
	char szTime[256] ={0};
	strftime(szTime, 256, "Save%Y%m%d%H%M%S", local);
	m_szLogPath = string(szTime)+string("/");
	boost::filesystem::create_directories(m_szLogPath.c_str());

	//初始GnssData
	GnssData inti_pose_gnss;
	inti_pose_gnss.fromPOSE_DATA(init_pose);
	inti_pose_gnss.rectifyQuaternion();

	//开启velodyne
	VelodyneDataReadParam VelParam;
//	VelParam.nDevType = 1;//hdl-32
	VelParam.nDevType = 0;//vlp-16
	VelParam.nReadType = 1;//
	VelParam.nDataFetchType = 0;//在线运行
	VelParam.bUseExternalCallBack = true;//使用外部回掉
	m_VelodyneOpr.Init(VelParam);
	VelodyneCallBackSector fc = boost::bind(&CLocationOpr::FeedSectorSector_, this, _1, _2, _3);
	m_VelodyneOpr.SetCallBackSector(fc);
	m_VelodyneOpr.Start();

	m_bLocalMapIsReady = false;
	m_LocalMap.clear();
	m_bIsRunLocation = true;

	//在地图坐标系中初始位置及姿态
	MapInfo ori_positon = m_MapInfo.front();
	double dDistY = 111319.55*(inti_pose_gnss.dLatitude - ori_positon.dRefLatitude);
	double dDistX = 111319.55*(inti_pose_gnss.dLongitude - ori_positon.dRefLongitude)*
		cos(ori_positon.dRefLatitude/180.0*3.141592654);
	pcl::PointXYZ start_point(dDistX, dDistY, 0);
	pcl::PointXYZ start_match_point;
	int nNearestInd = 0;
	float fNearestDist = 0.f;
	int nSeracheRt = NearestKnnSearch(start_point, start_match_point, &nNearestInd, &fNearestDist);
	if (nSeracheRt <= 0)
	{
		cout << "find nearest map point failed!" << endl;
		return -1;
	}
	cout << "find nearest map point dist: " << fNearestDist << endl;
	Eigen::Isometry3d tf_;
	tf_ = Eigen::Quaterniond(inti_pose_gnss.qw, inti_pose_gnss.qx, 
		inti_pose_gnss.qy, inti_pose_gnss.qz);
	tf_(0, 3) = dDistX;
	tf_(1, 3) = dDistY;
	tf_(2, 3) = start_match_point.z;//使用地图中最近点的高程
	Eigen::Matrix4d init_tf = tf_.matrix();
	cout << "init_tf:" << endl;
	cout << init_tf << endl;
	m_PoseInMap.push_back(init_tf);//设置在地图坐标系的中初始位置

	m_thread_Locate = boost::thread(boost::bind(&CLocationOpr::LocateThread, this));

	return 1;
}

void CLocationOpr::LocateThread()
{
	m_fsLocation.open((m_szLogPath+string("Location.txt")).c_str());
	char szLogOut[1024] = {0};
	sprintf_s(szLogOut, 1024, "Start Locate Thread...");
	WriteLocationLog(szLogOut);
	try
	{
		while (m_bIsRunLocation)
		{
			boost::this_thread::interruption_point();
			m_LocalMapLock.lock();
			if (m_bLocalMapIsReady)
			{
				PointCloud<PointXYZI> temp;
				Eigen::Matrix4d tf;
				tf = m_LocalMapTf;
				temp = m_LocalMap;
				m_bLocalMapIsReady = false;
				m_LocalMapLock.unlock();
				//////////////////////////////////////////////////////////////////////////
				sprintf_s(szLogOut, 1024, "****************************");
				WriteLocationLog(szLogOut);
				sprintf_s(szLogOut, 1024, "point cont: %d", temp.size());
				WriteLocationLog(szLogOut);
				pcl::transformPointCloud(temp, temp, tf.cast<float>().inverse().matrix());//将局部地图转换到车身(imu)坐标系下
				LocateWithMap(temp.makeShared(), tf);
				//////////////////////////////////////////////////////////////////////////
			}
			else
			{
				m_LocalMapLock.unlock();
				boost::this_thread::sleep(boost::posix_time::milliseconds(10));
			}
		}
	}
	catch (exception& e)
	{
		sprintf_s(szLogOut, 1024, "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
		WriteLocationLog(szLogOut);
		sprintf_s(szLogOut, 1024, "%s", e.what());
		WriteLocationLog(szLogOut);
		sprintf_s(szLogOut, 1024, "Interrupt exception was thrown.");
		WriteLocationLog(szLogOut);
		m_fsLocation.close();
	}
}

int CLocationOpr::LocateWithMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& pt, Eigen::Matrix4d& tf)
{
	char szLogOut[1024] = {0};
	PointCloud<PointXYZI>::Ptr pt4NdtMapping(new PointCloud<PointXYZI>);
	pt4NdtMapping->reserve(2000000);
	PointCloud<PointXYZI>::Ptr pt4NdtMappingFilter(new PointCloud<PointXYZI>);

	sprintf_s(szLogOut, 1024, "Lidar gps time:%lld", pt->header.stamp);
	WriteLocationLog(szLogOut);

	Eigen::Matrix4d delta_pose = m_PoseDr.back().inverse().matrix()*tf;//相对于上一次匹配的相对变化量

	stringstream szTemp;
	szTemp << delta_pose;
	sprintf_s(szLogOut, 1024, "delta_pose: %s", szTemp.str().c_str());
	WriteLocationLog(szLogOut);

	Eigen::Matrix4d guess_pose = m_PoseInMap.back()*delta_pose;			//使用相对变化量预测的当前位置
	PointCloud<PointXYZI>::Ptr pt4NdtFilter(new PointCloud<PointXYZI>);
	FilterPointCloud(pt, pt4NdtFilter);
//	(*pt4NdtFilter) = (*pt);
	sprintf_s(szLogOut, 1024, "pt4NdtFilter %d", pt4NdtFilter->size());
	WriteLocationLog(szLogOut);
	PointCloud<PointXYZI>::Ptr ptGuess(new PointCloud<PointXYZI>);
	pcl::transformPointCloud(*pt4NdtFilter, *ptGuess, guess_pose);//预测位置上的点云，不参与实际运算

	pcl::PointXYZ ptPosition(guess_pose(0,3), 
		guess_pose(1,3), guess_pose(2,3));

	pcl::PointXYZ ptNearest;
	int nMapInd = 0;
	float fDist = 0.f;
	int nKnnSearchRt = NearestKnnSearch(ptPosition, ptNearest, &nMapInd, &fDist);
	if (nKnnSearchRt <= 0)
	{
		if (!m_result_callback.empty())
		{
			POSE_DATA pose_data_guess = Tf2PoseData(guess_pose, pt->header.stamp);
			int nRt = -1;
			m_result_callback(pt4NdtMappingFilter, ptGuess, pose_data_guess, pose_data_guess ,nRt);
		}
		return -1;
	}
	
	int nStart = nMapInd-3<0?0:nMapInd-3;
	int nEnd = nMapInd+2>=m_MapInfo.size()?m_MapInfo.size()-1:nMapInd+2;
	sprintf_s(szLogOut, 1024, "search ind: %d,%d,%d,%d", nMapInd, nStart, nEnd, m_MapInfo.size());
	WriteLocationLog(szLogOut);
	for (int i = nStart; i <= nEnd; i++)
	{
		PointCloud<PointXYZI> temp;
		pcl::io::loadPCDFile(m_MapInfo[i].szMapPath, temp);
		(*pt4NdtMapping) += temp;
	}
	sprintf_s(szLogOut, 1024, "pt4NdtMapping %d", pt4NdtMapping->size());
	WriteLocationLog(szLogOut);

//	FilterPointCloud(pt4NdtMapping, pt4NdtMappingFilter);
	(*pt4NdtMappingFilter) = (*pt4NdtMapping);

	sprintf_s(szLogOut, 1024, "pt4NdtMappingFilter: %d", pt4NdtMappingFilter->size());
	WriteLocationLog(szLogOut);

	boost::posix_time::ptime time_start = boost::posix_time::microsec_clock::universal_time();
	pcl::NormalDistributionsTransform<PointXYZI, PointXYZI> ndt;
	ndt.setTransformationEpsilon(0.01);
	ndt.setStepSize(2.0);//3.0
	ndt.setResolution(2.0);//4.0
	ndt.setMaximumIterations(50);
	ndt.setInputSource(pt4NdtFilter);
	ndt.setInputTarget(pt4NdtMappingFilter);
	PointCloud<PointXYZI> output;
	ndt.align(output, guess_pose.cast<float>());
	Eigen::Matrix4d ndt_pose = ndt.getFinalTransformation().cast<double>();
//	Eigen::Matrix4d ndt_pose = guess_pose;
	int nFinalIter = ndt.getFinalNumIteration();

	boost::posix_time::ptime time_end = boost::posix_time::microsec_clock::universal_time();
	int nMilliSec = (time_end - time_start).total_milliseconds();

	sprintf_s(szLogOut, 1024, "Ndt finish, cost time:%d ms nFinalIter:%d", nMilliSec, nFinalIter);
	WriteLocationLog(szLogOut);

	// 	m_ndt.setInputSource(pt4NdtFilter);
	// 	PointCloud<PointXYZI> output;
	// 	m_ndt.align(output, guess_pose.cast<float>());
	// 	Eigen::Matrix4d ndt_pose = m_ndt.getFinalTransformation().cast<double>();

	static int g_nlocateInd = 0;
	char szSave[1024] = {0};
	sprintf_s(szSave, 1024, "%s%06dMap.pcd", m_szLogPath.c_str(), g_nlocateInd);
	if (pt4NdtMappingFilter->size() > 0)
	{
		WriteLocationLog(szSave);
		pcl::io::savePCDFileBinary(szSave, *pt4NdtMappingFilter);
	}

	sprintf_s(szSave, 1024, "%s%06dMatch.pcd", m_szLogPath.c_str(), g_nlocateInd);
	if (ptGuess->size() > 0)
	{
		WriteLocationLog(szSave);
		pcl::io::savePCDFileBinary(szSave, *ptGuess);
	}

  	sprintf_s(szSave, 1024, "%s%06dResult.pcd", m_szLogPath.c_str(), g_nlocateInd);
	if (output.size() > 0)
	{
		WriteLocationLog(szSave);
		pcl::io::savePCDFileBinary(szSave, output);
	}

	szTemp.str("");
	szTemp << guess_pose.inverse()*ndt_pose;
	sprintf_s(szLogOut, 1024, "Corrected pose: %s", szTemp.str().c_str());
	WriteLocationLog(szLogOut);

	m_PoseDr.push_back(tf);
	m_PoseInMap.push_back(ndt_pose);

	g_nlocateInd++;

	if (!m_result_callback.empty())
	{
		POSE_DATA pose_data_guess = Tf2PoseData(guess_pose, pt->header.stamp);
		POSE_DATA pose_data_ndt = Tf2PoseData(ndt_pose, pt->header.stamp);
		int nRt = 1;
		sprintf_s(szLogOut, 1024, "Enter call back");
		WriteLocationLog(szLogOut);
		m_result_callback(pt4NdtMappingFilter, ptGuess, pose_data_guess, pose_data_ndt ,nRt);
		sprintf_s(szLogOut, 1024, "Leave call back");
		WriteLocationLog(szLogOut);
	}

	return 1;
}

int CLocationOpr::FilterPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& pt_in,
								   pcl::PointCloud<pcl::PointXYZI>::Ptr& pt_out)
{
	float fLeafSize = 1.0;
	pcl::PointCloud<pcl::PointXYZI> temp;
	pcl::VoxelGrid<pcl::PointXYZI> vg;
	vg.setInputCloud(pt_in);
	vg.setLeafSize(fLeafSize, fLeafSize, fLeafSize);
	vg.filter(*pt_out);

	// 	pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
	// 	sor.setInputCloud(temp.makeShared());
	// 	sor.setMeanK (50);
	// 	sor.setStddevMulThresh (1.0);
	// 	PointCloud<PointXYZI>::Ptr cloud_filtered(new PointCloud<PointXYZI>);
	// 	sor.filter (*pt_out);

	return 1;
}

int CLocationOpr::WriteLocationLog(char* pszLog)
{
	cout << string(pszLog) << endl;
	if (m_fsLocation.is_open())
	{
		string strTime = boost::posix_time::to_iso_string(boost::get_system_time());	//log
		m_fsLocation << strTime << "*" << pszLog << endl;
	}
	else
	{
		cout << "Write log failed! m_fsLocation.is_open() == false" << endl;
	}
	return 1;
}

POSE_DATA CLocationOpr::Tf2PoseData(Eigen::Matrix4d& tf, uint64_t lidar_time)
{
	POSE_DATA out;
	if (m_MapInfo.size() <= 0)
	{
		return out;
	}
	MapInfo gps_base = m_MapInfo.front();
	Eigen::Matrix3d rot = tf.block(0,0,3,3).matrix();
	Eigen::Quaterniond q(rot);
	out.quat[0] = q.w();
	out.quat[1] = q.x();
	out.quat[2] = q.y();
	out.quat[3] = q.z();
	double dX = tf(0,3);
	double dY = tf(1,3);
	double dZ = tf(2,3);
	double dLat1 = gps_base.dLatitude + dY/111319.55;
	double dLon1 = gps_base.dLongitude + dX/111319.55/cos(gps_base.dLatitude/180.0*3.1415926);
	double dAlt1 = gps_base.dAltitude + dZ;
	out.pos[0] = dLat1;
	out.pos[1] = dLon1;
	out.pos[2] = dAlt1;
	out.gps_second = double(lidar_time)/1000000.0;
	return out;
}

int CLocationOpr::NearestKnnSearch(pcl::PointXYZ& point_in, pcl::PointXYZ& point_out,
								   int* id_out/*=0*/, float* fSquaredDist_out/*=0*/)
{
	int K = 1;
	vector<int> pointIdxNKNSearch(K);
	vector<float> pointNKNSquaredDistance(K);
	if (m_kdtree.nearestKSearch(point_in, K, pointIdxNKNSearch, pointNKNSquaredDistance) <= 0)
	{
		cout << "kdtree find nearestKSearch failed!!!" << endl;
		getchar();
		return -1;
	}
	if (id_out != 0)
	{
		*id_out = pointIdxNKNSearch[0];
	}
	if (fSquaredDist_out != 0)
	{
		*fSquaredDist_out = pointNKNSquaredDistance[0];
	}
	point_out = m_kdtree.getInputCloud()->at(pointIdxNKNSearch[0]);

	return 1;
}

void CLocationOpr::FeedSectorSector_(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& pt, float start, float end)
{
	if (!m_bIsRunLocation)
	{
		return;
	}
	static int g_nSectorCont = 0;
	g_nSectorCont++;
	GnssData gpsData;
	int nImuRtn = GetImuDataByTime(pt->header.stamp, gpsData);
	if (g_nSectorCont%2000 == 0)
	{
		printf("GetImuDataByTime == %d: time:%.6f\n", nImuRtn, gpsData.dSecInWeek);
		printf("%.6f:%.6f:%.6f\n",double(pt->header.stamp), m_ImuList.front().dSecInWeek, m_ImuList.back().dSecInWeek);
	}
	if (nImuRtn == -2)
	{
		//		std::cout << "GetImuDataByTime no data" << std::endl;
		return;
	}
	else if (nImuRtn == -1)
	{
//		std::cout << "GetImuDataByTime too old::;   LOST LIDAR DATA!!!!!" << std::endl;
		return;
	}
	else if (nImuRtn == 0)
	{
		int nSleepMaxCont = 0;
		while (nImuRtn == 0)
		{
			nSleepMaxCont++;
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
			nImuRtn = GetImuDataByTime(pt->header.stamp, gpsData);
			if (nSleepMaxCont >= 10)
			{
				printf("GetImuDataByTime == 0 :: nSleepMaxCont >= 10 LOST LIDAR DATA!!!!!\n");
				return;
			}
// 			printf("GetImuDataByTime too new.. imu buf size:%d lidar time:%lld: imu time:%lld-%lld, sleep cont:%d\n",
// 				m_ImuList.size(), pt->header.stamp, uint64_t(m_ImuList.front().dSecInWeek), uint64_t(m_ImuList.back().dSecInWeek),
// 				nSleepMaxCont);
			
		}
		if (nImuRtn < 0)
		{
			return;
		}
	}

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
	if (mapT.size() <= 0)
	{
		return;
	}

	pcl::transformPointCloud(mapT, mapT, m_Calib);

	int nLocalMapRt = m_MapOpr.AddFrameByGps_QueueType(mapT.makeShared(), gpsData);
	if (m_PoseDr.size() <= 0)//获取局部地图的初始位置
	{
		m_MapOpr.GetLastTf(m_LocalMapTf);
		m_PoseDr.push_back(m_LocalMapTf);
	}
	if (nLocalMapRt < 0)
	{
		return;
	}
	else
	{
		m_LocalMapLock.lock();
		if (m_MapOpr.GetMap(m_LocalMap) <= 0 ||
			m_MapOpr.GetLastTf(m_LocalMapTf) <= 0)
		{
			m_LocalMap.header = pt->header;//传递时间戳
			m_LocalMapLock.unlock();
			return;
		}
		m_bLocalMapIsReady = true;
		m_LocalMapLock.unlock();
		cout << "Local map refresh!..." << endl;

		m_nMapInd++;
	}
}

void CLocationOpr::FeedImuData_(GnssData& imuData)
{
	m_ImuLock.lock();
	m_ImuList.push_front(imuData);
	if (m_ImuList.size() > LOCATION_IMU_MAX_SIZE)
	{
		m_ImuList.pop_back();
	}
	m_ImuLock.unlock();
}

int CLocationOpr::GetImuDataByTime(uint64_t nTime, GnssData& imuData)
{
	m_ImuLock.lock();
	if (m_ImuList.size() <= 0)			//无imu数据
	{
		m_ImuLock.unlock();
		return -2;
	}
	if (nTime <= m_ImuList.back().dSecInWeek)//请求的数据太旧，
	{
		m_ImuLock.unlock();
		return -1;
	}
	if (nTime >= m_ImuList.front().dSecInWeek)//请求的数据太新，需要等待后从新获取
	{
		m_ImuLock.unlock();
		return 0;
	}

	std::list<GnssData>::iterator it0, it1;
	for (it0 = m_ImuList.begin(), it1 = m_ImuList.begin(), it0++;
		it0 != m_ImuList.end() && it1 != m_ImuList.end(); it0++, it1++)
	{
		if (nTime >= it0->dSecInWeek &&
			nTime <= it1->dSecInWeek)
		{
			GnssData data0 = *it0;
			GnssData data1 = *it1;
			double dP1 = (double)(nTime-it0->dSecInWeek)/(double)(it1->dSecInWeek-it0->dSecInWeek);
			double dP0 = (double)(it1->dSecInWeek-nTime)/(double)(it1->dSecInWeek-it0->dSecInWeek);
			imuData = *it0;
			imuData.dSecInWeek = dP0*data0.dSecInWeek + dP1*data1.dSecInWeek;
			imuData.dLongitude = dP0*data0.dLongitude + dP1*data1.dLongitude;
			imuData.dLatitude = dP0*data0.dLatitude + dP1*data1.dLatitude;
			imuData.dAltitude = dP0*data0.dAltitude + dP1*data1.dAltitude;
			imuData.dRoll = dP0*data0.dRoll + dP1*data1.dRoll;
			imuData.dPitch = dP0*data0.dPitch + dP1*data1.dPitch;
			imuData.dHeading = dP0*data0.dHeading + dP1*data1.dHeading;
			Eigen::Quaterniond q0(data0.qw, data0.qx, data0.qy, data0.qz);//, qb, qres
			Eigen::Quaterniond q1(data1.qw, data1.qx, data1.qy, data1.qz);//, qb, qres;
			Eigen::Quaterniond qres = q0.slerp(dP1, q1);
			imuData.qw = qres.w();
			imuData.qx = qres.x();
			imuData.qy = qres.y();
			imuData.qz = qres.z();

			//如果查询到结果，则删除旧的缓存数据
			it0++;
// 			if (it0 != m_ImuList.end())
// 			{
// 				m_ImuList.erase(it0, m_ImuList.end());
// 			}

			m_ImuLock.unlock();
			return 1;
		}
	}

	std::cout << "unexpected logical branche... " << std::endl;

	m_ImuLock.unlock();
	return -1;
}

void CLocationOpr::split(std::string& s, std::string& delim,std::vector< std::string >* ret)
{  
	size_t last = 0;  
	size_t index=s.find_first_of(delim,last);  
	while (index!=std::string::npos)  
	{  
		ret->push_back(s.substr(last,index-last));  
		last=index+1;  
		index=s.find_first_of(delim,last);  
	}  
	if (index-last>0)  
	{  
		ret->push_back(s.substr(last,index-last));  
	}  
}  

int CLocationOpr::LoadMap()
{
	std::ifstream fs(m_Param.szMapDir+"MapInfo.txt");
	char szLine[1024] = {0};
	while (fs.getline(szLine, 1024))
	{
		std::vector<std::string> strings;
		split(std::string(szLine), string(" "), &strings);
		if (strings.size() != 12)
		{
			break;
		}
		MapInfo info;
		info.nId =			atoi(strings[0].c_str());
		info.dTime =		atof(strings[1].c_str());
		info.dLongitude =	atof(strings[2].c_str());
		info.dLatitude =	atof(strings[3].c_str());
		info.dAltitude =	atof(strings[4].c_str());
		info.dX =			atof(strings[5].c_str());
		info.dY =			atof(strings[6].c_str());
		info.dZ =			atof(strings[7].c_str());
		info.dMile =		atof(strings[8].c_str());
		info.dRefLongitude = atof(strings[9].c_str());
		info.dRefLatitude = atof(strings[10].c_str());
		info.dRefAltitude = atof(strings[11].c_str());
		char szMapPath[1024] = {0};
		sprintf_s(szMapPath, 1024, "%s%06d.pcd", m_Param.szMapDir.c_str(), info.nId);
		info.szMapPath = std::string(szMapPath);
		m_MapInfo.push_back(info);
		// 		PointCloud<PointXYZI>::Ptr temp(new PointCloud<PointXYZI>);
		// 		pcl::io::loadPCDFile(szMapPath, *temp);
		// 		(*m_pt4NdtMappingFilter) += (*temp);
	}
	// 	cout << "Map point cont: " << m_pt4NdtMappingFilter->size() << endl;
	// 	pcl::io::savePCDFileBinary("Map.pcd", *m_pt4NdtMappingFilter);

	m_track.clear();
	for (unsigned int i = 0; i < m_MapInfo.size(); i++)
	{
		PointXYZ pt(m_MapInfo[i].dX, m_MapInfo[i].dY, m_MapInfo[i].dZ);
		m_track.push_back(pt);
	}
	m_kdtree.setInputCloud(m_track.makeShared());

	return m_MapInfo.size();
}

int CLocationOpr::FeedVelodynePack(PCAP_DATA* pBuf)
{
	if (!m_bIsRunLocation)
	{
		return -1;
	}
	PCAP_DATA* pData = (PCAP_DATA*)pBuf;
	memcpy(m_szVelBuf, pData->data, 1206);
	memcpy(m_szVelBuf+1206, &(pData->gps_second), 8);
	static int nlidar_feed = 0;
	if (nlidar_feed%2000 == 0)
	{
//		printf("lidar feed data time stamp:%.6f\n", pData->gps_second);
	}
	nlidar_feed++;
	m_VelodyneOpr.enqueueHDLPacket(&(m_szVelBuf[0]), 1214);
	return 1;
}

int CLocationOpr::FeedImuData(POSE_DATA* pImu)
{
	if (!m_bIsRunLocation)
	{
		return -1;
	}
	GnssData data;
	data.fromPOSE_DATA(*pImu);
	data.rectifyQuaternion();
	data.dSecInWeek *= 1000000.0;
	FeedImuData_(data);
	return 1;
}

int CLocationOpr::Stop()
{
	m_bIsRunLocation = false;
	m_thread_Locate.interrupt();
	m_thread_Locate.join();
	m_VelodyneOpr.Stop();

	return 1;
}