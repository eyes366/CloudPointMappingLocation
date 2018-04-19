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
		REMOVE_BY_HEIGHT,		//将点云按照标定参数变换到地面坐标系后，直接取[dFilterHeightAbave, dFilterHeightBelow]之间的点云作为障碍物
		REMOVE_BY_SLOPE			//将点云按照标定参数变换到地面坐标系后，按照每一列激光的坡度来判断障碍物，注：要求输入点云是未经过滤的
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
	double dHeading;			//激光标定航向角degree
	double dRoll;				//激光标定横滚角degree
	double dPitch;				//激光标定俯仰角degree
	double dHeight;				//雷达标定离地高度meter
	double dFilterHeightAbave;	//障碍物的高度下限，用于过滤地面meter
	double dFilterHeightBelow;	//障碍物的高度上限，用于过滤天花板meter
	double dFilterRange;		//只考虑dFilterRange内的点云meter
	double dSlopeThreshold;		//最大坡度值degree，用对REMOVE_BY_SLOPE有效
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
