#include "GroundExtract.h"

using namespace pcl;

CGroundExtractParam::CGroundExtractParam()
{
    dHeading = 0.0;
    dRoll = 0.0;
    dPitch = 0.0;
    dHeight = 0.0;
    dFilterHeightAbave = 0.3;
    dFilterHeightBelow = 3.0;
    dFilterRange = 30.0;
}

CGroundExtractor::CGroundExtractor()
{

}

int CGroundExtractor::Init(CGroundExtractParam Param)
{
    m_Param = Param;

    m_pose =
            Eigen::Translation3f (Eigen::Vector3f (0, 0, m_Param.dHeight)) *
            Eigen::AngleAxisf (-m_Param.dHeading/180.0*CV_PI,   Eigen::Vector3f::UnitZ ()) *
            Eigen::AngleAxisf (-m_Param.dRoll/180.0*CV_PI, Eigen::Vector3f::UnitY ()) *
            Eigen::AngleAxisf (-m_Param.dPitch/180.0*CV_PI,  Eigen::Vector3f::UnitX ());

    return 1;
}

int CGroundExtractor::RemoveGround(pcl::PointCloud<pcl::PointXYZI>::Ptr pts_in,
                                   pcl::PointCloud<pcl::PointXYZI>::Ptr pts_not_ground_out)
{
    PointCloud<PointXYZI>::Ptr CloudOri_rot(new PointCloud<PointXYZI>);
    pcl::transformPointCloud (*pts_in, *CloudOri_rot, m_pose);

    PointCloud<PointXYZI>::Ptr Cloud_object(new PointCloud<PointXYZI>);
    Cloud_object->reserve(CloudOri_rot->size());
    for(int i = 0; i < CloudOri_rot->size(); i++)
    {
        PointXYZI pt = CloudOri_rot->at(i);
        if(pcl_isnan(pt.x) ||
                pcl_isnan(pt.y) ||
                pcl_isnan(pt.z))
        {
            continue;
        }
        if (pt.z <= m_Param.dFilterHeightAbave || pt.z >= m_Param.dFilterHeightBelow)
        {
            continue;
        }
        double dDist = sqrt(pow(pt.x,2) + pow(pt.y,2));
        if (dDist >= m_Param.dFilterRange)
        {
            continue;
        }
        Cloud_object->push_back(pt);
    }

    pts_not_ground_out->clear();
    pcl::transformPointCloud (*Cloud_object, *pts_not_ground_out, m_pose.inverse());

    return pts_not_ground_out->size();
}
