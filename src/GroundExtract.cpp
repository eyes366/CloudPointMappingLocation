#include <pcl/pcl_macros.h>
#include "GroundExtract.h"

using namespace pcl;


CGroundExtractor::CGroundExtractor()
{

}

int CGroundExtractor::Init(CGroundExtractParam Param)
{
	m_Param = Param;

	m_pose =
		Eigen::Translation3f(Eigen::Vector3f(0, 0, m_Param.dHeight)) *
		Eigen::AngleAxisf(m_Param.dHeading / 180.0*M_PI, Eigen::Vector3f::UnitZ()) *
		Eigen::AngleAxisf(m_Param.dRoll / 180.0*M_PI, Eigen::Vector3f::UnitY()) *
		Eigen::AngleAxisf(m_Param.dPitch / 180.0*M_PI, Eigen::Vector3f::UnitX());

	return 1;
}

int CGroundExtractor::RemoveGround(pcl::PointCloud<pcl::PointXYZI>::Ptr& pts_in,
	pcl::PointCloud<pcl::PointXYZI>::Ptr& pts_object_out,
	std::vector<int>& index_object_out)
{
	if (m_Param.algo == CGroundExtractParam::REMOVE_BY_HEIGHT)
	{
		return RemoveGroundByHeight(pts_in, pts_object_out, index_object_out);
	}
	else
	{
		return RemoveGroundBySlope(pts_in, pts_object_out, index_object_out);
	}
}

int CGroundExtractor::RemoveGroundByHeight(pcl::PointCloud<pcl::PointXYZI>::Ptr& pts_in,
	pcl::PointCloud<pcl::PointXYZI>::Ptr& pts_object_out,
	std::vector<int>& index_object_out)
{
	PointCloud<PointXYZI>::Ptr CloudOri_rot(new PointCloud<PointXYZI>);
	pcl::transformPointCloud(*pts_in, *CloudOri_rot, m_pose.inverse());

	pts_object_out->clear();
	pts_object_out->reserve(CloudOri_rot->size());
	index_object_out.clear();
	index_object_out.reserve(CloudOri_rot->size());
	double dDistSqured = pow(m_Param.dFilterRange, 2);
	for (int i = 0; i < CloudOri_rot->size(); i++)
	{
		PointXYZI& pt = CloudOri_rot->at(i);
		if (pcl_isnan(pt.x) || pcl_isnan(pt.y) || pcl_isnan(pt.z))
		{
			continue;
		}
		if (pt.z < m_Param.dFilterHeightAbave || pt.z > m_Param.dFilterHeightBelow)
		{
			continue;
		}
		double dDistS = pow(pt.x, 2) + pow(pt.y, 2);
		if (dDistS >= dDistSqured)
		{
			continue;
		}
		pts_object_out->push_back(pt);
		index_object_out.push_back(i);
	}

	return pts_object_out->size();
}

int CGroundExtractor::RemoveGroundBySlope(pcl::PointCloud<pcl::PointXYZI>::Ptr& pts_in,
	pcl::PointCloud<pcl::PointXYZI>::Ptr& pts_object_out,
	std::vector<int>& index_object_out)
{
	int hdl32_map[] = { 0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30,
		1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31 };
	int vlp16_map[] = { 0, 2, 4, 6, 8, 10, 12, 14,
		1, 3, 5, 7, 9, 11, 13, 15 };

	PointCloud<PointXYZI>::Ptr CloudOri_rot(new PointCloud<PointXYZI>);
	pcl::transformPointCloud(*pts_in, *CloudOri_rot, m_pose.inverse());

	pts_object_out->clear();
	pts_object_out->reserve(CloudOri_rot->size());
	index_object_out.clear();
	index_object_out.reserve(CloudOri_rot->size());

	double dSlopeThresholdTan = tan(m_Param.dSlopeThreshold/180.0*M_PI);
	int nRows = 16;
	int nCols = pts_in->size() / nRows;
	for (int i = 0; i < nCols; i++)
	{
		// 		int nFirstPointInd = i*nRows + vlp16_map[0];
		// 		PointXYZI& first_point = pts_in->at(nFirstPointInd);
		// 		bool bIsNanPoint = (pcl_isnan(first_point.x) || pcl_isnan(first_point.y) || pcl_isnan(first_point.z));
		// 		if (!bIsNanPoint)
		// 		{
		// 			if (first_point.z >= m_Param.dFilterHeightAbave)//如果每一列的第一个点高于dFilterHeightAbave，则认为是障碍物
		// 			{
		// 				pts_object_out->push_back(first_point);
		// 				index_object_out.push_back(nFirstPointInd);
		// 			}
		// 		}

		for (int j = 1; j < nRows; j++)
		{
			int nInd0 = i*nRows + vlp16_map[j - 1];
			int nInd1 = i*nRows + vlp16_map[j];
			PointXYZI pt0 = pts_in->at(nInd0);
			PointXYZI pt1 = pts_in->at(nInd1);
			if (pcl_isnan(pt0.x) || pcl_isnan(pt0.y) || pcl_isnan(pt0.z))
			{
				continue;
			}
			if (pcl_isnan(pt1.x) || pcl_isnan(pt1.y) || pcl_isnan(pt1.z))
			{
				continue;
			}

			float dz = abs(pt1.z - pt0.z);
			float dxy = sqrt(pow((pt0.x - pt1.x), 2) + pow((pt0.y - pt1.y), 2));
			if (dz / dxy >= dSlopeThresholdTan)
			{
				 pts_object_out->push_back(pt1);
				 index_object_out.push_back(nInd1);
			}
		}
	}

	return pts_object_out->size();
}