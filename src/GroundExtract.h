#ifndef _GROUND_EXTRACT_H_
#define _GROUND_EXTRACT_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>

struct CGroundExtractParam
{
	enum ALGO_TYPE
	{
		REMOVE_BY_HEIGHT,		//�����ư��ձ궨�����任����������ϵ��ֱ��ȡ[dFilterHeightAbave, dFilterHeightBelow]֮��ĵ�����Ϊ�ϰ���
		REMOVE_BY_SLOPE			//�����ư��ձ궨�����任����������ϵ�󣬰���ÿһ�м�����¶����ж��ϰ��ע��Ҫ�����������δ�����˵�
	};
	enum LIDAR_TYPE
	{
		VLP_16,
		HDL_32
	};
	CGroundExtractParam()
	{
		algo = REMOVE_BY_SLOPE;
		lidar = VLP_16;
		dHeading = 0.0;
		dRoll = 0.0;
		dPitch = 0.0;
		dHeight = 1.0;
		dFilterHeightAbave = 0.3;
		dFilterHeightBelow = 3.0;
		dFilterRange = 100.0;
		dSlopeThreshold = 10.0;
	}
	ALGO_TYPE algo;
	LIDAR_TYPE lidar;
	double dHeading;			//����궨�����degree
	double dRoll;				//����궨�����degree
	double dPitch;				//����궨������degree
	double dHeight;				//�״�궨��ظ߶�meter
	double dFilterHeightAbave;	//�ϰ���ĸ߶����ޣ����ڹ��˵���meter
	double dFilterHeightBelow;	//�ϰ���ĸ߶����ޣ����ڹ����컨��meter
	double dFilterRange;		//ֻ����dFilterRange�ڵĵ���meter
	double dSlopeThreshold;		//����¶�ֵdegree���ö�REMOVE_BY_SLOPE��Ч
};

class CGroundExtractor
{
public:

	CGroundExtractor();
	int Init(CGroundExtractParam Param);
	int RemoveGround(pcl::PointCloud<pcl::PointXYZI>::Ptr& pts_in,
		pcl::PointCloud<pcl::PointXYZI>::Ptr& pts_object_out,
		std::vector<int>& index_object_out);

private:
	int RemoveGroundByHeight(pcl::PointCloud<pcl::PointXYZI>::Ptr& pts_in,
		pcl::PointCloud<pcl::PointXYZI>::Ptr& pts_object_out,
		std::vector<int>& index_object_out);
	int RemoveGroundBySlope(pcl::PointCloud<pcl::PointXYZI>::Ptr& pts_in,
		pcl::PointCloud<pcl::PointXYZI>::Ptr& pts_object_out,
		std::vector<int>& index_object_out);
	CGroundExtractParam m_Param;
	Eigen::Affine3f m_pose;
};

#endif
