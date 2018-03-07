#include "NdtVisual.h"

using namespace pcl;
using namespace pcl::visualization;

NdtVisual::NdtVisual():
	m_nFreshTime(100),
	m_timer(m_io, boost::posix_time::milliseconds(m_nFreshTime))
{
	m_p = 0;
	m_p = new pcl::visualization::PCLVisualizer("Test");
	m_p->createViewPort (0.0, 0, 1.0, 1.0, m_vp1);
	m_p->setCameraPosition(0.0, 0.0, 100, 0.0, 0.0, 0.0, m_vp1);
	m_p->addCoordinateSystem(1.0,"m_vp1" , m_vp1);
	m_p->spinOnce();
	// 	m_timer.async_wait(boost::bind(&NdtVisual::DispThread, this));
	// 	boost::thread(boost::bind(&boost::asio::io_service::run, &m_io));
	m_nCont = 0;
}

NdtVisual::~NdtVisual()
{
	if (m_p)
	{
		delete m_p;
		m_p = 0;
	}
}

void NdtVisual::DisplayCloudPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr& cp_map,
								   pcl::PointCloud<pcl::PointXYZI>::Ptr& cp_lidar)
{
	m_lock.lock();
	pcl::PointXYZ center(0,0,0);
	for (unsigned int i = 0; i < cp_lidar->size(); i++)
	{
		center.x += cp_lidar->at(i).x;
		center.y += cp_lidar->at(i).y;
		center.z += cp_lidar->at(i).z;
	}
	center.x /= float(cp_lidar->size());
	center.y /= float(cp_lidar->size());
	center.z /= float(cp_lidar->size());
	//	m_map = (*cp_map);
	//	m_lidar = (*cp_lidar);
	PointCloudColorHandler<PointXYZI>* handler0 =
		new PointCloudColorHandlerCustom<PointXYZI> (
		cp_map, 150, 150, 150);
	CloudActorMapPtr ppp = m_p->getCloudActorMap();
// 	vtkSmartPointer<vtkRendererCollection> render = m_p->getRendererCollection();
// 	vtkRenderer* renderer = render->GetNextItem ();
// 	for ( CloudActorMap::iterator am_it = ppp->begin(); am_it != ppp->end(); am_it++)
// 	{
// 		renderer->RemoveActor(am_it->second.actor);
// 	}
	ppp->clear();

	std::string szIdMap("map");
//	m_p->removePointCloud(szIdMap, m_vp1);
//	if (m_nCont == 0)
//	m_p->removeShape(szIdMap, m_vp1);
		m_p->addPointCloud(cp_map, *handler0, szIdMap, m_vp1);
//	else
//		m_p->updatePointCloud(cp_map, *handler0, szIdMap);
 	delete handler0;
	PointCloudColorHandler<PointXYZI>* handler1 =
		new PointCloudColorHandlerCustom<PointXYZI> (
		cp_lidar, 0, 255, 0);
	std::string szIdLidar("lidar");
// 	m_p->removePointCloud(szIdLidar, m_vp1);
//	if (m_nCont == 0)
//	m_p->removeShape(szIdLidar, m_vp1);
 		m_p->addPointCloud(cp_lidar, *handler1, szIdLidar, m_vp1);
//	else
//		m_p->updatePointCloud(cp_lidar, *handler1, szIdLidar);
	delete handler1;
	cout << center << endl;
//  	m_p->setCameraPosition(center.x, center.y, center.z+150,
//  							center.x, center.y, center.z, 0, 0, 0, m_vp1);
// 	m_p->setCameraPosition(0.0, 0.0, 100, 0.0, 0.0, 0.0, m_vp1);
// 	m_p->spinOnce(1);
//	m_p->removeAllPointClouds(m_vp1);
	m_nCont++;
	m_lock.unlock();
}

void NdtVisual::SpinOnce()
{
	m_lock.lock();
	m_p->spinOnce(1);
	m_lock.unlock();
}

void NdtVisual::DispThread()
{
// 	m_p->removePointCloud("map",m_vp1);
// 	m_p->removePointCloud("lidar",m_vp1);
// 	PointCloudColorHandler<PointXYZI>* handler0 =
// 		new PointCloudColorHandlerCustom<PointXYZI> (
// 		m_map.makeShared(), 100, 100, 100);
// 	m_p->addPointCloud(m_map.makeShared(), *handler0, "map", m_vp1);
// 	delete handler0;
// 	PointCloudColorHandler<PointXYZI>* handler1 =
// 		new PointCloudColorHandlerCustom<PointXYZI> (
// 		m_lidar.makeShared(), 0, 255, 0);
// 	m_p->addPointCloud(m_lidar.makeShared(), *handler1, "lidar", m_vp1);
// 	delete handler1;
// 	//	m_p->spinOnce(1);
// 	cout << "spinonce" << endl;
// 	m_timer.expires_at(m_timer.expires_at() + boost::posix_time::milliseconds(m_nFreshTime)) ;
//	m_timer.async_wait(boost::bind(&NdtVisual::DispThread, this));
}