#ifndef _GROUND_EXTRACT_H_
#define _GROUND_EXTRACT_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp>

struct CGroundExtractParam
{
    CGroundExtractParam();
    double dHeading;
    double dRoll;
    double dPitch;
    double dHeight;
    double dFilterHeightAbave;
    double dFilterHeightBelow;
    double dFilterRange;
};

class CGroundExtractor
{
public:
    enum TYPE
    {
        REMOVE_BY_HEIGHT
    };
    CGroundExtractor();
    int Init(CGroundExtractParam Param);
    int RemoveGround(pcl::PointCloud<pcl::PointXYZI>::Ptr pts_in,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr pts_not_ground_out);

private:
    CGroundExtractParam m_Param;
    Eigen::Affine3f m_pose;
};

#endif
