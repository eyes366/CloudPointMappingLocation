#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <opencv2/opencv.hpp>

namespace segmatch
{

///////////////////////////////////////////////////////////////////////
//Segment of an object in a frame
///////////////////////////////////////////////////////////////////////
class Segment
{
public:
    Segment();
    void calculateCentroid();

    int is_matched;
    int segment_id;
    int frame_id;
    pcl::PointCloud<pcl::PointXYZI> point_cloud;
    cv::Mat feature;
    pcl::PointXYZ center;
    uint64_t time_stamp;
};

///////////////////////////////////////////////////////////////////////
//Match information between two segments
///////////////////////////////////////////////////////////////////////
class SegmentMatch
{
public:
    int id1;
    int id2;
    pcl::PointXYZ center1;
    pcl::PointXYZ center2;
    cv::Mat feature1;
    cv::Mat feature2;
    float confidence;
};

///////////////////////////////////////////////////////////////////////
//Frame that contain many segments
///////////////////////////////////////////////////////////////////////
class SegFrame
{
public:
    SegFrame();
    int LoadDataFromPointCloud(int frame_ind_in, Eigen::Matrix4d pose_in,
//             pcl::PointCloud<pcl::PointXYZI>::Ptr pt_ori_in,
             pcl::PointCloud<pcl::PointXYZI>::Ptr pt_processed_in,
             pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg_clolor_out = 0);
    int SaveSegFrame(std::string szSavePath);

public:
    int frame_id;
    int lidar_seq;
    uint64_t lidar_time;
    std::vector<Segment> segments;
    cv::Mat features;
    Eigen::Isometry3d pose;
//    pcl::PointCloud<pcl::PointXYZI> cloud_ori;

private:
    int EuclideanSegmentationProcess(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                     std::vector<pcl::PointIndices>& clusters);
    int ColorSegmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                          std::vector<pcl::PointIndices>& clusters,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud);
    cv::Mat EigenValueBaseExtractBatch(pcl::PointCloud<pcl::PointXYZI>::Ptr points_in,
                                       std::vector<pcl::PointIndices>& clusters_in);
//    cv::Mat EigenValueBaseExtractBatch(int nFrame, pcl::PointCloud<pcl::PointXYZI>::Ptr points_in,
//                                       std::vector<pcl::PointIndices>& clusters_in,
//                                       bool bIsSavePt, std::string* pszSavePath = 0);
    cv::Vec<double,7> EigenValueBaseExtract(pcl::PointCloud<pcl::PointXYZI>::Ptr points_in);
};

///////////////////////////////////////////////////////////////////////
//Match information between two SegFrames
///////////////////////////////////////////////////////////////////////
class SegFrameMatch
{
public:
    SegFrameMatch();
    int query_frame_id;
    int base_map_id;
    int query_lidar_seq;
    int base_map_lidar_seq;
    std::vector<SegmentMatch> segment_matches;
    std::vector<int> match_valid;
    int match_cont;
    Eigen::Matrix4f transformation;
};

///////////////////////////////////////////////////////////////////////
//SegMatcherParam
///////////////////////////////////////////////////////////////////////
struct SegMatcherParam
{
    SegMatcherParam();
    int nMapInterval;
    std::string szTreePath;
    float fFeatureMatchConfidence;
    double fGCSize;
    int nGCThreshold;
    int nNonMaximumSuppressionRange;
    int nMatchType;     //0:Mapping closure detect  1:Localization
};

///////////////////////////////////////////////////////////////////////
//SegMatcher operator for closure detection
///////////////////////////////////////////////////////////////////////
class SegMatcher
{
public:
    SegMatcher();
    int Init(SegMatcherParam Param);

    void AddBaseMap(SegFrame& map_frame);
    int LoadBaseMap(std::string szMapPath);

    void AddQueryFrame(SegFrame& query_frame);

    std::vector<SegFrameMatch> Match();
    std::vector<SegFrameMatch> MatchWithShow();

    int SaveMatches(std::string szSavePath);

public:
    std::vector<SegFrame> query_frames;
    std::vector<SegFrame> base_map;
    std::vector<SegFrameMatch> frame_matches;

private:
    int ShowFrameMatch(SegFrame& query_frame, SegFrame& map_frame,
                       SegFrameMatch& frame_match);
    int MatchFrame2Frame(SegFrame& query_frame, SegFrame& map_frame,
                  SegFrameMatch& frame_match_out);
    int FilterMatches();
    cv::Mat ComputeEigenFeatureDistance(cv::Mat& feature1, cv::Mat& feature2);
    SegMatcherParam m_Param;
    CvRTrees m_rtrees;
};

///////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////
struct SegmentMappingParam
{
    SegmentMappingParam();

};

// class SegmentMappingOpr
// {
// public:
//     int Init(SegmentMappingParam& Param);
//     int StartMapping(std::string szProjPath);
// 
// private:
//     SegmentMappingParam m_Param;
// };

class SegmentMappingOprEx
{
public:
	int Init(SegmentMappingParam& Param);
	int StartMapping(std::string szProjPath);

private:
	SegmentMappingParam m_Param;
};

}
