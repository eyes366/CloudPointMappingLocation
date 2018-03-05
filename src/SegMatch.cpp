#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <boost/timer.hpp>

#include "SegMatch.h"
#include "TrjLogReader.h"
#include "MapConstruct.h"
#include "VLP16Graber.h"
#include "GroundExtract.h"

using namespace pcl;
using namespace pcl::visualization;
using namespace segmatch;
using namespace std;
using namespace cv;

\
///////////////////////////////////////////////////////////////////////
Segment::Segment()
{
    segment_id = 0;
    frame_id = 0;
    is_matched = 0;
}

void Segment::calculateCentroid()
{
    int kNPoints = point_cloud.size();
    pcl::PointXYZ ptCenter(0.0, 0.0, 0.0);
    for (size_t i = 0u; i < kNPoints; ++i)
    {
        ptCenter.x += point_cloud[i].x;
        ptCenter.y += point_cloud[i].y;
        ptCenter.z += point_cloud[i].z;
    }
    ptCenter.x /= double(kNPoints);
    ptCenter.y /= double(kNPoints);
    ptCenter.z /= double(kNPoints);
    center = ptCenter;
}

///////////////////////////////////////////////////////////////////////
SegmentMappingParam::SegmentMappingParam()
{

}

int SegmentMappingOpr::Init(SegmentMappingParam& Param)
{
    m_Param = Param;

    return 1;
}

static bool g_bIsPause = false;

void visualization_button_callback (const pcl::visualization::KeyboardEvent &event,
                                    void* viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
      if ((event.getKeySym () == "s" || event.getKeySym () == "S")&& event.keyDown ())
      {
            g_bIsPause = !g_bIsPause;
      }
}

int SegmentMappingOpr::StartMapping(std::string szProjPath)
{
    string szProPath(szProjPath);

    CTrjLogReader TrjReader;
    TrjReader.ReadLog(szProPath+"trj.txt");
    if (TrjReader.m_poses.size() <= 0)
    {
        cout << "Read trj.txt failed!" << endl;
        return -1;
    }

    CMapConstruct MapOpr;

    int nSourceType = 1;
    string szSourceAddr = szProPath+"0.pcap";
    ifstream fs_calib(szProPath+"calibration.txt");
    if (!fs_calib.is_open())
    {
        cout << "Failed open  " << szProPath+"calibration.txt" << endl;
        return -1;
    }
    CGroundExtractParam GroundExtractParam;
    fs_calib >> GroundExtractParam.dHeading;
    fs_calib >> GroundExtractParam.dRoll;
    fs_calib >> GroundExtractParam.dPitch;
    fs_calib >> GroundExtractParam.dHeight;
    cout << "dHeading:" << GroundExtractParam.dHeading <<
            "dRoll:" << GroundExtractParam.dRoll <<
            "dPitch:" << GroundExtractParam.dPitch <<
            "dHeight:" << GroundExtractParam.dHeight << endl;

    CGroundExtractor GroundExtractor;
    GroundExtractor.Init(GroundExtractParam);

    VLPGrabber* pGrabber = NULL;
    if (nSourceType == 0)
    {
        pGrabber = new VLPGrabber(boost::asio::ip::address_v4::from_string(szSourceAddr),2368);
    }
    else
    {
        pGrabber = new VLPGrabber(szSourceAddr);
    }

    SimpleVLPViewer<PointXYZI> v(*pGrabber);

    v.setMode(1);
    v.start();

    pcl::visualization::PCLVisualizer* p = NULL;
    int vp_1, vp_2;
    p = new pcl::visualization::PCLVisualizer(szProjPath.c_str());
    p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
    p->setCameraPosition(0.0, 0.0, 20, 0.0, 0.0, 0.0, vp_1);
    p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
    p->setCameraPosition(0.0, 0.0, 20, 0.0, 0.0, 0.0, vp_2);
    p->addCoordinateSystem(1.0,"vp_1" , vp_1);
    p->addCoordinateSystem(1.0,"vp_2" , vp_2);

    boost::signals2::connection button_connection = p->registerKeyboardCallback(visualization_button_callback, (void*)p);

    int nCont = 0;
    mkdir((szProPath+"map").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    ofstream fs_stamp((szProPath+"stamp.csv").c_str());
    while(1)
    {
        PointCloud<PointXYZI>::ConstPtr ptCloudPt;
        int nRt = v.nextFrame(ptCloudPt);
        if (nRt == 0)
        {
            cout << "Get pcap data failed!" << endl;
            p->spin();
            continue;
        }
        nCont++;
        boost::timer ter;

        BlockTrj trj_data;
        Eigen::Isometry3d pose_global;
        if (TrjReader.GetPoseByTime(ptCloudPt->header.stamp, pose_global, trj_data) < 0)
        {
            cout << "TrjReader.GetPoseByTime   failed!+++++++++++++++++++++++" << endl;
            continue;
        }

        std::cout << "Point cont:" << ptCloudPt->size() << std::endl;

        PointCloud<PointXYZI>::Ptr cloud_no_ground(new PointCloud<PointXYZI>);
        GroundExtractor.RemoveGround(ptCloudPt->makeShared(), cloud_no_ground);

        int nAddRt = MapOpr.AddFrame(cloud_no_ground, pose_global);
        if (nAddRt > 0)
        {
            fs_stamp << nCont << "," << trj_data.fileDataIdx << "," << trj_data.q[0] << ","
                     << trj_data.q[1] << "," << trj_data.q[2] << "," << trj_data.q[3] << ","
                     << trj_data.pos[0] << "," << trj_data.pos[1] << "," << trj_data.pos[2] << "," << endl;

            PointCloud<PointXYZI>::Ptr CloudMap(new PointCloud<PointXYZI>);
            PointCloud<PointXYZRGB>::Ptr CloudColor(new PointCloud<PointXYZRGB>);
            MapOpr.GetMap(CloudMap);
//            pcl::transformPointCloud (*CloudMap, *CloudMap, pose_global.inverse().matrix());
            string szSavePath = szProPath + "map/";
            SegFrame segframe;
            PointCloud<PointXYZI>::Ptr cloud_ori_rot(new PointCloud<PointXYZI>);
            transformPointCloud(*ptCloudPt, *cloud_ori_rot, pose_global.matrix());
            segframe.LoadDataFromPointCloud(nCont, pose_global, cloud_ori_rot, CloudMap, CloudColor);
            segframe.SaveSegFrame(szSavePath);

//            char szPtCloudSavePath[1024] = {0};
//            sprintf(szPtCloudSavePath, "%spointcloud/ptcoud%06d.pcd", szProPath.c_str(), nCont);
//            PCDWriter wr;
//            wr.writeBinary(szPtCloudSavePath, *CloudMap);

            PointCloudColorHandler<pcl::PointXYZRGB>* handler_2 = new PointCloudColorHandlerRGBField<PointXYZRGB> (CloudColor);
            p->removePointCloud("2",vp_2);
            p->addPointCloud(CloudColor,*handler_2,"2",vp_2);
            delete handler_2;
        }

        PointCloudColorHandler<pcl::PointXYZI>* handler_1 = new PointCloudColorHandlerGenericField<PointXYZI> (cloud_no_ground,"intensity");
        p->removePointCloud("1",vp_1);
        p->addPointCloud(cloud_no_ground,*handler_1, "1", vp_1);
        delete handler_1;

        std::cout<< "Time cost: " <<ter.elapsed()<<std::endl;

        while(g_bIsPause)
        {
            p->spinOnce();
        }

        p->spinOnce();
    }

    button_connection.disconnect();

    return 1;
}

///////////////////////////////////////////////////////////////////////
SegFrame::SegFrame()
{
    frame_id = 0;
    lidar_seq = 0;
    lidar_time = 0;
}

int SegFrame::LoadDataFromPointCloud(int frame_ind_in, Eigen::Isometry3d pose_in,
         pcl::PointCloud<pcl::PointXYZI>::Ptr pt_ori_in,
         pcl::PointCloud<pcl::PointXYZI>::Ptr pt_processed_in,
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg_clolor_out/* = 0*/)
{
    frame_id = frame_ind_in;
    lidar_seq = pt_processed_in->header.seq;
    lidar_time = pt_processed_in->header.stamp;
    pose = pose_in;
    cloud_ori = *pt_ori_in;

    PointCloud<PointXYZI>::Ptr CloudDown(new PointCloud<PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (pt_processed_in);
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.filter (*CloudDown);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor_;
    sor_.setInputCloud (CloudDown);
    sor_.setMeanK (30);
    sor_.setStddevMulThresh (0.5);
    PointCloud<PointXYZI>::Ptr cloud_filtered(new PointCloud<PointXYZI>);
    sor_.filter (*cloud_filtered);

    vector<PointIndices> clusters;
    EuclideanSegmentationProcess(cloud_filtered, clusters);
    if (seg_clolor_out)
    {
        ColorSegmentation(cloud_filtered, clusters, seg_clolor_out);
    }
    features = EigenValueBaseExtractBatch(cloud_filtered, clusters);

    return features.rows;
}

int SegFrame::SaveSegFrame(std::string szSavePath)
{
    for (unsigned int i = 0; i < segments.size(); i++)
    {
        char szLog[1024] = {0};
        sprintf(szLog, "ObjectPoints%06d_%06d.pcd", frame_id, i);
        string szTemp = szSavePath + string(szLog);
        PCDWriter writer;
        writer.writeBinary(szTemp, segments[i].point_cloud);
    }

    char szLog[1024] = {0};
    sprintf(szLog, "ObjectFeature%06d.yml", frame_id);
    string szTemp = szSavePath + string(szLog);
    cv::FileStorage fs;
    fs.open(szTemp.c_str(), FileStorage::WRITE);
    fs << "features" << features;
    fs.release();

    sprintf(szLog, "OriCloudPoints%06d.pcd", frame_id);
    szTemp = szSavePath + string(szLog);
    PCDWriter wt;
    wt.writeBinary(szTemp, cloud_ori);

    return segments.size();
}

int SegFrame::EuclideanSegmentationProcess(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                 std::vector<pcl::PointIndices>& clusters)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    (*cloud_filtered) = (*cloud);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (cloud_filtered);

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (0.2); // 2cm
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (clusters);

    return clusters.size();
}

int SegFrame::ColorSegmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                      std::vector<pcl::PointIndices>& clusters,
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>);

    srand (static_cast<unsigned int> (time (0)));
    std::vector<unsigned char> colors;
    for (size_t i_segment = 0; i_segment < clusters.size (); i_segment++)
    {
        colors.push_back (static_cast<unsigned char> (rand () % 256));
        colors.push_back (static_cast<unsigned char> (rand () % 256));
        colors.push_back (static_cast<unsigned char> (rand () % 256));
    }

    colored_cloud_->width = cloud->width;
    colored_cloud_->height = cloud->height;
    colored_cloud_->is_dense = cloud->is_dense;
    for (size_t i_point = 0; i_point < cloud->size (); i_point++)
    {
        pcl::PointXYZRGB point;
        point.x = *(cloud->points[i_point].data);
        point.y = *(cloud->points[i_point].data + 1);
        point.z = *(cloud->points[i_point].data + 2);
        point.r = 0;
        point.g = 0;
        point.b = 0;
        colored_cloud_->points.push_back (point);
    }

    std::vector< pcl::PointIndices >::iterator i_segment;
    int next_color = 0;
    for (i_segment = clusters.begin (); i_segment != clusters.end (); i_segment++)
    {
        std::vector<int>::iterator i_point;
        for (i_point = i_segment->indices.begin (); i_point != i_segment->indices.end (); i_point++)
        {
            int index;
            index = *i_point;
            pcl::PointXYZRGB pt;
            colored_cloud_->points[index].r = colors[3 * next_color];
            colored_cloud_->points[index].g = colors[3 * next_color + 1];
            colored_cloud_->points[index].b = colors[3 * next_color + 2];
        }
        next_color++;
    }

    (*colored_cloud) = (*colored_cloud_);

    return clusters.size();
}

cv::Mat SegFrame::EigenValueBaseExtractBatch(pcl::PointCloud<pcl::PointXYZI>::Ptr points_in,
                                   std::vector<pcl::PointIndices>& clusters_in)
{
    vector<cv::Vec<double,7> > line_numbers;

    for (unsigned int i = 0; i < clusters_in.size(); i++)
    {
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(points_in);
        pcl::PointIndices::Ptr ppp_( new pcl::PointIndices );
        (*ppp_) = clusters_in[i];
        extract.setIndices(ppp_);
        extract.setNegative(false);
        pcl::PointCloud<pcl::PointXYZI>::Ptr points_target( new pcl::PointCloud<pcl::PointXYZI>);
        extract.filter(*points_target);
        cv::Vec<float,7> feature = EigenValueBaseExtract(points_target);
        line_numbers.push_back(feature);
        Segment segment;
        segment.point_cloud += (*points_target);
        segments.push_back(segment);
    }

    Mat features = Mat(line_numbers.size(), 7, CV_64F, line_numbers.data()).clone();

    return features;
}

//cv::Mat SegFrame::EigenValueBaseExtractBatch(int nFrame, pcl::PointCloud<pcl::PointXYZI>::Ptr points_in,
//                                             std::vector<pcl::PointIndices>& clusters_in,
//                                             bool bIsSavePt, std::string* pszSavePath/* = 0*/)
//{
//    vector<cv::Vec<double,7> > line_numbers;

//    for (unsigned int i = 0; i < clusters_in.size(); i++)
//    {
//        pcl::ExtractIndices<pcl::PointXYZI> extract;
//        extract.setInputCloud(points_in);
//        pcl::PointIndices::Ptr ppp_( new pcl::PointIndices );
//        (*ppp_) = clusters_in[i];
//        extract.setIndices(ppp_);
//        extract.setNegative(false);
//        pcl::PointCloud<pcl::PointXYZI>::Ptr points_target( new pcl::PointCloud<pcl::PointXYZI>);
//        extract.filter(*points_target);
//        cv::Vec<float,7> feature = EigenValueBaseExtract(points_target);
//        line_numbers.push_back(feature);
//        if (bIsSavePt)
//        {
//            char szLog[1024] = {0};
//            sprintf(szLog, "ObjectPoints%06d_%06d.pcd", nFrame, i);
//            if (pszSavePath)
//            {
//                string szTemp = (*pszSavePath) + string(szLog);
//                strcpy(szLog, szTemp.c_str());
//            }
//            PCDWriter writer;
//            writer.writeBinary(string(szLog), *points_target);
//        }
//    }

//    Mat features = Mat(line_numbers.size(), 7, CV_64F, line_numbers.data()).clone();

//    if (bIsSavePt)
//    {
//        char szLog[1024] = {0};
//        sprintf(szLog, "ObjectFeature%06d.yml", nFrame);
//        if (pszSavePath)
//        {
//            string szTemp = (*pszSavePath) + string(szLog);
//            strcpy(szLog, szTemp.c_str());
//        }
//        cv::FileStorage fs;
//        fs.open(szLog, FileStorage::WRITE);
//        fs << "features" << features;
//        fs.release();
//    }


//    return features;
//}


template<typename T>
bool swap_if_gt(T& a, T& b)
{
  if (a > b) {
    std::swap(a, b);
    return true;
  }
  return false;
}

cv::Vec<double,7> SegFrame::EigenValueBaseExtract(pcl::PointCloud<pcl::PointXYZI>::Ptr points_in)
{
    cv::Vec<double,7> feature;
    const size_t kNPoints = points_in->size();
    if (kNPoints <= 0)
    {
        cout << "kNPoints <= 0" << endl;
        getchar();
    }

    //Calculate the center of point cloud
    PointXYZ ptCenter(0.0, 0.0, 0.0);
    for (size_t i = 0u; i < kNPoints; ++i)
    {
        ptCenter.x += points_in->at(i).x;
        ptCenter.y += points_in->at(i).y;
        ptCenter.z += points_in->at(i).z;
    }
    ptCenter.x /= double(kNPoints);
    ptCenter.y /= double(kNPoints);
    ptCenter.z /= double(kNPoints);

    // Find the variances.
    PointCloud<pcl::PointXYZ> variances;
    for (size_t i = 0u; i < kNPoints; ++i)
    {
        variances.push_back(pcl::PointXYZ());
        variances[i].x = points_in->at(i).x - ptCenter.x;
        variances[i].y = points_in->at(i).y - ptCenter.y;
        variances[i].z = points_in->at(i).z - ptCenter.z;
    }

    // Find the covariance matrix. Since it is symmetric, we only bother with the upper diagonal.
    const std::vector<size_t> row_indices_to_access = {0,0,0,1,1,2};
    const std::vector<size_t> col_indices_to_access = {0,1,2,1,2,2};
    Eigen::Matrix3f covariance_matrix;
    for (size_t i = 0u; i < row_indices_to_access.size(); ++i)
    {
        const size_t row = row_indices_to_access[i];
        const size_t col = col_indices_to_access[i];
        double covariance = 0;
        for (size_t k = 0u; k < kNPoints; ++k)
        {
            covariance += variances.points[k].data[row] * variances.points[k].data[col];
        }
        covariance /= kNPoints;
        covariance_matrix(row,col) = covariance;
        covariance_matrix(col,row) = covariance;
    }

    // Compute eigenvalues of covariance matrix.
    constexpr bool compute_eigenvectors = false;
    Eigen::EigenSolver<Eigen::Matrix3f> eigenvalues_solver(covariance_matrix, compute_eigenvectors);
    std::vector<float> eigenvalues(3, 0.0);
    eigenvalues.at(0) = eigenvalues_solver.eigenvalues()[0].real();
    eigenvalues.at(1) = eigenvalues_solver.eigenvalues()[1].real();
    eigenvalues.at(2) = eigenvalues_solver.eigenvalues()[2].real();
    if (eigenvalues_solver.eigenvalues()[0].imag() != 0.0 ||
            eigenvalues_solver.eigenvalues()[1].imag() != 0.0 ||
            eigenvalues_solver.eigenvalues()[2].imag() != 0.0 )
    {
        cout << "eigenvalues_solver.eigenvalues()[0].imag() != 0.0" << endl;
        getchar();
    }

    // Sort eigenvalues from smallest to largest.
    swap_if_gt(eigenvalues.at(0), eigenvalues.at(1));
    swap_if_gt(eigenvalues.at(0), eigenvalues.at(2));
    swap_if_gt(eigenvalues.at(1), eigenvalues.at(2));

    // Normalize eigenvalues.
    double sum_eigenvalues = eigenvalues.at(0) + eigenvalues.at(1) + eigenvalues.at(2);
    double e1 = eigenvalues.at(0) / sum_eigenvalues;
    double e2 = eigenvalues.at(1) / sum_eigenvalues;
    double e3 = eigenvalues.at(2) / sum_eigenvalues;
    if (e1 == e2 || e2 == e3 || e1 == e3)
    {
        cout << "e1 == e2 || e2 == e3 || e1 == e3" << endl;
        getchar();
    }

    // Store inside features.
    const double sum_of_eigenvalues = e1 + e2 + e3;
    constexpr double kOneThird = 1.0/3.0;
    if (e1 == 0.0 || sum_of_eigenvalues == 0.0)
    {
        cout << "e1 == 0.0 || sum_of_eigenvalues == 0.0" << endl;
        getchar();
    }

    const double kNormalizationPercentile = 1.0;

    const double kLinearityMax = 28890.9 * kNormalizationPercentile;
    const double kPlanarityMax = 95919.2 * kNormalizationPercentile;
    const double kScatteringMax = 124811 * kNormalizationPercentile;
    const double kOmnivarianceMax = 0.278636 * kNormalizationPercentile;
    const double kAnisotropyMax = 124810 * kNormalizationPercentile;
    const double kEigenEntropyMax = 0.956129 * kNormalizationPercentile;
    const double kChangeOfCurvatureMax = 0.99702 * kNormalizationPercentile;

    const double kNPointsMax = 13200 * kNormalizationPercentile;

    feature[0] = (e1 - e2) / e1 / kLinearityMax;
    feature[1] = (e2 - e3) / e1 / kPlanarityMax;
    feature[2] = e3 / e1 / kScatteringMax;
    feature[3] = std::pow(e1 * e2 * e3, kOneThird) / kOmnivarianceMax;
    feature[4] = (e1 - e3) / e1 / kAnisotropyMax;
    feature[5] = (e1 * std::log(e1)) + (e2 * std::log(e2)) + (e3 * std::log(e3)) / kEigenEntropyMax;
    feature[6] = e3 / sum_of_eigenvalues / kChangeOfCurvatureMax;

    return feature;
}

///////////////////////////////////////////////////////////////////////
SegFrameMatch::SegFrameMatch()
{
    match_cont = 0;
    query_frame_id = 0;
    base_map_id = 0;
    query_lidar_seq = 0;
    base_map_lidar_seq = 0;
}

///////////////////////////////////////////////////////////////////////
SegMatcherParam::SegMatcherParam()
{
    nMapInterval = 10;
    szTreePath = string("/home/xinzi/catkin_ws/src/segmatch/laser_mapper/demonstration_files/kitti/random_forest_eigen_25trees.xml");
    fFeatureMatchConfidence = 0.7;
    fGCSize = 0.4;
    nGCThreshold = 5/*4*/;
    nNonMaximumSuppressionRange = 25;
    nMatchType = 0;
}

///////////////////////////////////////////////////////////////////////
SegMatcher::SegMatcher()
{

}

int SegMatcher::Init(SegMatcherParam Param)
{
    m_Param = Param;

    m_rtrees.load(m_Param.szTreePath.c_str());
    cout << "m_rtrees.get_tree_count(): " << m_rtrees.get_tree_count() << endl;

    return 1;
}

void SegMatcher::AddBaseMap(SegFrame& map_frame)
{
    base_map.push_back(map_frame);
}

int SegMatcher::LoadBaseMap(std::string szMapPath)
{
    string szProPath = szMapPath;
    vector<Mat> features_list;
    vector<int> index_list;
    for (size_t i = 0u; i < 100000; i++)
    {
        char szLog[1024] = {0};
        sprintf(szLog, "ObjectFeature%06d.yml", i);
        string szLog_ = szProPath + "map/" + string(szLog);
        FileStorage fs(szLog_, FileStorage::READ);
        Mat features;
        fs["features"] >> features;
        if (features.rows > 0)
        {
            features_list.push_back(features);
            index_list.push_back(i);
        }
    }
    cout << "features_list.size() " << features_list.size() << endl;

    for (unsigned int j = 0; j < features_list.size(); j+=1/*m_Param.nMapInterval*/)
    {
        Mat test0 = features_list[j];
        SegFrame frame;
        frame.features = test0.clone();
        frame.lidar_seq = index_list[j];
        frame.frame_id = j;
        frame.lidar_time = 0;
        frame.pose;
        for (unsigned int i = 0; i < test0.rows; i++)
        {
            PointCloud<PointXYZI>::Ptr temp(new PointCloud<PointXYZI>);
            char szPCD[1024] = {0};
            sprintf(szPCD, "ObjectPoints%06d_%06d.pcd", index_list[j], i);
            string szPCD_ = szProPath + "map/" + string(szPCD);
            PCDReader Reader;
            if (Reader.read(szPCD_, *temp) < 0)
            {
                cout << "Read pcd faild! " << szPCD_ << endl;
                getchar();
            }
            segmatch::Segment seg;
            seg.point_cloud = (*temp);
            seg.frame_id = index_list[j];
            seg.segment_id = i;
            seg.feature = features_list[j].row(i);
            seg.calculateCentroid();
            frame.segments.push_back(seg);
        }
        char szPCD_ori[1024] = {0};
        sprintf(szPCD_ori, "OriCloudPoints%06d.pcd", index_list[j]);
        string szPCD_ori_ = szProPath + string("map/") + string(szPCD_ori);
        PCDReader rd;
        rd.read(szPCD_ori_, frame.cloud_ori);
        base_map.push_back(frame);
    }

    return base_map.size();
}

void SegMatcher::AddQueryFrame(SegFrame& query_frame)
{
    query_frames.push_back(query_frame);
}

void str_split(const string& input, const string& delimiters,
         vector<string>& results)
 {
     string::size_type pos;
     size_t size=input.size();
     for(size_t i=0;i<size;++i)
     {
         pos=input.find(delimiters,i); //从第i个位置查找delimiters分割符第一次出现的位置；
         if(pos<size)
         {
             string s=input.substr(i,pos-i);//把从i开始，长度为pos-i的元素拷贝给s;
             results.push_back(s);
             i=pos;
         }
     }

 }

std::vector<SegFrameMatch> SegMatcher::MatchWithShow()
{
    vector<PointXYZ> track;
    ifstream fs_track;
    fs_track.open("/home/xinzi/data/20180201134130/stamp.csv");
    if (!fs_track.is_open())
    {
        cout << "Open track file failed!!!!" << endl;
        getchar();
    }
    string szLine;
    while(getline(fs_track, szLine))
    {
        vector<string> items;
        str_split(szLine, "," , items);
        if (items.size() < 9)
        {
            break;
        }
        PointXYZ pt;
        pt.x = atof(items[6].c_str());
        pt.y = atof(items[7].c_str());
        pt.z = atof(items[8].c_str());
        track.push_back(pt);
    }
    cout << "Track size : " << track.size() << endl;

    pcl::visualization::PCLVisualizer* p = NULL;
    int vp_1, vp_2;
    char szTitle[1024] = {0};
    p = new pcl::visualization::PCLVisualizer("SegMatchTest");
    p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
    p->setCameraPosition(0.0, 0.0, 20, 0.0, 0.0, 0.0, vp_1);
    p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
    p->setCameraPosition(0.0, 0.0, 20, 0.0, 0.0, 0.0, vp_2);
    p->addCoordinateSystem(1.0,"vp_1" , vp_1);
    p->addCoordinateSystem(1.0,"vp_2" , vp_2);

    SegFrame query_frame_disp;
    SegFrame map_frame_disp;
    SegFrameMatch frame_match_disp;
    int range = 50;
    for (int k = 0; k < base_map.size(); k++)
    {
        vector<PointXYZ> track_temp;
        PointCloud<PointXYZI> LocalMap;
        for (int i = k-range; i < k+range; i++)
        {
            if (i < 0 || i >= base_map.size())
            {
                continue;
            }
            SegFrame& frame = base_map[i];
            if (i%5 == 0)
            {
                for (int j = 0; j < frame.segments.size(); j++)
                {
                    LocalMap += frame.segments[j].point_cloud;
                }
            }
            if (i <= k)
            {
                track_temp.push_back(track[i]);
            }
        }


        pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>* handler =
                new visualization::PointCloudColorHandlerCustom<PointXYZI> (
                    LocalMap.makeShared(),255,
                    255,255);
        char szT[1024] = {0};
        sprintf(szT, "%d", k);
        p->removeAllPointClouds(vp_1);
        p->addPointCloud(LocalMap.makeShared(),*handler, szT, vp_1);
        delete handler;

        pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>* handler_ori =
                new visualization::PointCloudColorHandlerCustom<PointXYZI> (
                    base_map[k].cloud_ori.makeShared(),0,155,0);
        sprintf(szT, "Ori%d", k);
        p->addPointCloud(base_map[k].cloud_ori.makeShared(),*handler_ori, szT, vp_1);
        delete handler_ori;

        p->removeAllShapes(vp_1);
        if (k > 0)
        {
            for (int m = 1; m < track_temp.size(); m++)
            {
                sprintf(szT, "Line%d", m);
                p->addLine<PointXYZ, PointXYZ>(track_temp[m-1], track_temp[m],
                        1.0, 0.0, 0.0, szT, vp_1);
            }
            PointXYZ ptView = track_temp.back();
            p->setCameraPosition(ptView.x, ptView.y-90, ptView.z+50,
                                 ptView.x, ptView.y, ptView.z,0.0, 0.0, 0.0, vp_1);

        }

        p->removeAllShapes(vp_1);
        if (k > 0)
        {
            for (int m = 1; m < track_temp.size(); m++)
            {
                sprintf(szT, "Line%d", m);
                p->addLine<PointXYZ, PointXYZ>(track_temp[m-1], track_temp[m],
                        1.0, 0.0, 0.0, szT, vp_1);
            }
        }

        if (k+5 >= 0 && k-5 < base_map.size() )
        {
            SegFrameMatch frame_match;
            if (MatchFrame2Frame(base_map[k-5], base_map[k+5], frame_match) > 0)
            {
                query_frame_disp = base_map[k-5];
                map_frame_disp = base_map[k+5];
                frame_match_disp = frame_match;
            }
        }

        SegFrame query_frame_disp_ = query_frame_disp;
        for (unsigned int i = 0; i < query_frame_disp_.segments.size(); i++)
        {
            for (unsigned int j = 0; j < query_frame_disp_.segments[i].point_cloud.size(); j++)
            {
                query_frame_disp_.segments[i].point_cloud[j].z += 20.0;
            }
        }

        srand (static_cast<unsigned int> (time (0)));
        p->removeAllPointClouds(vp_2);
        for (unsigned int i = 0; i < query_frame_disp_.segments.size(); i++)
        {
            bool bIsValid = false;
            for (unsigned int j = 0; j < frame_match_disp.segment_matches.size(); j++)
            {
                if (frame_match_disp.match_valid[j] == 1 &&
                        frame_match_disp.segment_matches[j].id1 == i)
                {
                    bIsValid = true;
                    break;
                }
            }
            double r = rand ()%255;
            double g = rand ()%255;
            double b = rand ()%255;
            if (!bIsValid)
            {
                continue;
                r = g= b = 50;
            }
            pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>* handler =
                    new visualization::PointCloudColorHandlerCustom<PointXYZI> (
                        query_frame_disp_.segments[i].point_cloud.makeShared(),r,
                        g,b);
            string szT = string("PointCloud1_") + to_string(i);
//                   p->removeAllPointClouds(vp_2);
            p->addPointCloud(query_frame_disp_.segments[i].point_cloud.makeShared(),*handler, szT, vp_2);
            delete handler;
        }

        for (unsigned int i = 0; i < map_frame_disp.segments.size(); i++)
        {
            bool bIsValid = false;
            for (unsigned int j = 0; j < frame_match_disp.segment_matches.size(); j++)
            {
                if (frame_match_disp.match_valid[j] == 1 &&
                        frame_match_disp.segment_matches[j].id2 == i)
                {
                    bIsValid = true;
                    break;
                }
            }
            double r = rand ()%255;
            double g = rand ()%255;
            double b = rand ()%255;
            if (!bIsValid)
            {
//                continue;
                r = g= b = 50;
            }
            pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>* handler =
                    new visualization::PointCloudColorHandlerCustom<PointXYZI> (
                        map_frame_disp.segments[i].point_cloud.makeShared(),r,
                        g,b);
            string szT = string("PointCloud2_") + to_string(i);
//                    p->removePointCloud(szT,vp_2);
            p->addPointCloud(map_frame_disp.segments[i].point_cloud.makeShared(),*handler, szT, vp_2);
            delete handler;
        }

        p->removeAllShapes(vp_2);
        if (k > 0)
        {
            for (int m = 1; m < track_temp.size(); m++)
            {
                sprintf(szT, "Line%d__", m);
                p->addLine<PointXYZ, PointXYZ>(track_temp[m-1], track_temp[m],
                        1.0, 0.0, 0.0, szT, vp_2);
            }
        }
        SegFrameMatch frame_match_disp_ = frame_match_disp;
        for (unsigned int i = 0; i < frame_match_disp_.segment_matches.size(); i++)
        {
            string szT = string("LIne_") + to_string(i);
            frame_match_disp_.segment_matches[i].center1.z += 20.0;
            if (frame_match_disp_.match_valid[i] == 0)
            {
    //            p->addLine<PointXYZ,PointXYZ>(matches[i].center1, matches[i].center2,0.2,
    //                                          0,0, szT);
            }
            else
            {
                p->addLine<PointXYZ,PointXYZ>(frame_match_disp_.segment_matches[i].center1,
                                              frame_match_disp_.segment_matches[i].center2,
                                              0, 1,0, szT, vp_2);
            }
        }


        p->spinOnce();
        if(k >=510)
        {
            p->spin();
        }
    }


    vector<SegFrameMatch> aaa;
    return aaa;
}

std::vector<SegFrameMatch> SegMatcher::Match()
{
    frame_matches.clear();
    if (m_Param.nMatchType == 0)
    {
        for (unsigned int i = 0; i < base_map.size(); i++)
        {
            for (unsigned int j = i; j < base_map.size(); j++)
            {
                if (abs(base_map[i].frame_id - base_map[j].frame_id) <=
                        m_Param.nNonMaximumSuppressionRange)
                {
                    continue;
                }
                SegFrameMatch frame_match;
                if (MatchFrame2Frame(base_map[i], base_map[j],
                                     frame_match) > 0)
                {
                    frame_matches.push_back(frame_match);
                    ShowFrameMatch(base_map[i], base_map[j],
                                   frame_match);
                }
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < query_frames.size(); i++)
        {
            for (unsigned int j = 0; j < base_map.size(); j++)
            {
                SegFrameMatch frame_match;
                if (MatchFrame2Frame(query_frames[i], base_map[j],
                                     frame_match) > 0)
                {
                    frame_matches.push_back(frame_match);
                    ShowFrameMatch(base_map[i], base_map[j],
                                   frame_match);
                }
            }
        }
    }


    FilterMatches();

    return frame_matches;
}

int SegMatcher::SaveMatches(std::string szSavePath)
{
    ofstream fs_save(szSavePath.c_str());
    for (unsigned int i = 0; i < frame_matches.size(); i++)
    {
        fs_save << frame_matches[i].query_frame_id << "," << frame_matches[i].base_map_id
                << "," << frame_matches[i].match_cont;
        for (unsigned int ii = 0; ii < 4; ii++)
        {
            for (unsigned int jj = 0; jj < 4; jj++)
            {
                fs_save << "," << frame_matches[i].transformation(ii,jj);
            }
        }
        fs_save << endl;
    }

    return frame_matches.size();
}

int SegMatcher::ShowFrameMatch(SegFrame& query_frame, SegFrame& map_frame,
                   SegFrameMatch& frame_match)
{
    pcl::visualization::PCLVisualizer* p = NULL;
    int vp_1, vp_2;
    char szTitle[1024] = {0};
    sprintf(szTitle, "%d-%d", frame_match.query_frame_id, frame_match.base_map_id);
    p = new pcl::visualization::PCLVisualizer(string(szTitle));
    p->createViewPort (0.0, 0, 1.0, 1.0, vp_1);
    p->setCameraPosition(0.0, 0.0, 100, 0.0, 0.0, 0.0, vp_1);
    p->addCoordinateSystem(1.0,"vp_1" , vp_1);

    SegFrame query_frame_ = query_frame;

    for (unsigned int i = 0; i < query_frame_.segments.size(); i++)
    {
        for (unsigned int j = 0; j < query_frame_.segments[i].point_cloud.size(); j++)
        {
            query_frame_.segments[i].point_cloud[j].z += 50.0;
        }
    }

//    p->removeAllPointClouds();
    srand (static_cast<unsigned int> (time (0)));
    for (unsigned int i = 0; i < query_frame_.segments.size(); i++)
    {
        pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>* handler =
                new visualization::PointCloudColorHandlerCustom<PointXYZI> (
                    query_frame_.segments[i].point_cloud.makeShared(),rand ()%255,
                    rand ()%255,rand ()%255);
        string szT = string("PointCloud1") + to_string(i);
//        p->removePointCloud(szT,vp_1);
        p->addPointCloud(query_frame_.segments[i].point_cloud.makeShared(),*handler, szT, vp_1);
        delete handler;
    }

    for (unsigned int i = 0; i < map_frame.segments.size(); i++)
    {
        pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>* handler =
                new visualization::PointCloudColorHandlerCustom<PointXYZI> (
                    map_frame.segments[i].point_cloud.makeShared(),rand ()%255,
                    rand ()%255,rand ()%255);
        string szT = string("PointCloud2") + to_string(i);
//        p->removePointCloud("1",vp_1);
        p->addPointCloud(map_frame.segments[i].point_cloud.makeShared(),*handler, szT, vp_1);
        delete handler;
    }

    p->removeAllShapes();

    SegFrameMatch frame_match_ = frame_match;
    for (unsigned int i = 0; i < frame_match_.segment_matches.size(); i++)
    {
        string szT = string("LIne") + to_string(i);
        frame_match_.segment_matches[i].center1.z += 50.0;
        if (frame_match_.match_valid[i] == 0)
        {
//            p->addLine<PointXYZ,PointXYZ>(matches[i].center1, matches[i].center2,0.2,
//                                          0,0, szT);
        }
        else
        {
            p->addLine<PointXYZ,PointXYZ>(frame_match_.segment_matches[i].center1,
                                          frame_match_.segment_matches[i].center2,
                                          0, 1,0, szT);
        }
    }

    p->spin();
}

int SegMatcher::MatchFrame2Frame(SegFrame& query_frame, SegFrame& map_frame,
              SegFrameMatch& frame_match_out)
{
    frame_match_out.base_map_lidar_seq = map_frame.lidar_seq;
    frame_match_out.query_lidar_seq = query_frame.lidar_seq;
    frame_match_out.base_map_id = map_frame.frame_id;
    frame_match_out.query_frame_id = query_frame.frame_id;
    frame_match_out.segment_matches.clear();
    for (unsigned int i = 0; i < query_frame.segments.size(); i++)
    {
        for (unsigned int j = 0; j < map_frame.segments.size(); j++)
        {
            Segment& cluster1 = query_frame.segments[i];
            Segment& cluster2 = map_frame.segments[j];
            Mat feature_diff = ComputeEigenFeatureDistance(
                        cluster1.feature, cluster2.feature);
            Mat feature_diff_;
            feature_diff.convertTo(feature_diff_, CV_32F);
            float confi = m_rtrees.predict_prob(feature_diff_);
            if (confi >= m_Param.fFeatureMatchConfidence)
            {
                segmatch::SegmentMatch match;
                match.id1 = i;
                match.id2 = j;
                match.feature1 = cluster1.feature.clone();
                match.feature2 = cluster2.feature.clone();
                match.center1 = cluster1.center;
                match.center2 = cluster2.center;
                match.confidence = confi;
                frame_match_out.segment_matches.push_back(match);
            }
        }
    }
    frame_match_out.match_valid.resize(frame_match_out.segment_matches.size(), 0);

    //////////////////////////////////////////////////////////////////////////////
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
    PointCloud<PointXYZ>::Ptr first_cloud(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr second_cloud(new PointCloud<PointXYZ>);

    for (size_t i = 0u; i < frame_match_out.segment_matches.size(); ++i)
    {
        // First centroid.
        PointXYZ first_centroid = frame_match_out.segment_matches[i].center1;//matches[i].center1;
        first_cloud->push_back(first_centroid);
        // Second centroid.
        PointXYZ second_centroid = frame_match_out.segment_matches[i].center2;//matches[i].center2;
        second_cloud->push_back(second_centroid);
        float squared_distance = 1.0 - frame_match_out.segment_matches[i].confidence;
        correspondences->push_back(pcl::Correspondence(i, i, squared_distance));
    }

    std::vector<Eigen::Matrix4f,
            Eigen::aligned_allocator<Eigen::Matrix4f> > correspondence_transformations;
    vector<Correspondences> clustered_corrs;
    pcl::GeometricConsistencyGrouping<PointXYZ, PointXYZ> geometric_consistency_grouping;
    geometric_consistency_grouping.setGCSize(m_Param.fGCSize);
    geometric_consistency_grouping.setGCThreshold(m_Param.nGCThreshold);
    geometric_consistency_grouping.setInputCloud(first_cloud);
    geometric_consistency_grouping.setSceneCloud(second_cloud);
    geometric_consistency_grouping.setModelSceneCorrespondences(correspondences);
    geometric_consistency_grouping.recognize(correspondence_transformations, clustered_corrs);

    int largest_cluster_size = 0;
    int largest_cluster_index = -1;
    for (size_t i = 0u; i < clustered_corrs.size(); ++i)
    {
        cout << "Cluster " << i << " has " << clustered_corrs[i].size() << "segments." << endl;
        if (clustered_corrs[i].size() >= largest_cluster_size)
        {
            largest_cluster_size = clustered_corrs[i].size();
            largest_cluster_index = i;
        }
    }

    if (largest_cluster_index < 0)
    {
        cout << "No valid matches found!" << endl;
        return -1;
    }
    else
    {
        frame_match_out.transformation = correspondence_transformations[largest_cluster_index];
        frame_match_out.match_cont = largest_cluster_size;
        cout << "transformation:" << endl << correspondence_transformations[largest_cluster_index] << endl;
    }

    Correspondences& cos = clustered_corrs[largest_cluster_index];
    for (unsigned int i = 0; i < cos.size(); i++)
    {
        if (cos[i].index_query != cos[i].index_match)
        {
            cout << "cos[i].index_query != cos[i].index_match" << endl;
            getchar();
        }
        frame_match_out.match_valid[cos[i].index_query] = 1;
    }

    return 1;
}

int SegMatcher::FilterMatches()
{
    vector<SegFrameMatch> frame_matches_temp;
    frame_matches_temp = frame_matches;
    vector<SegFrameMatch> frame_matches_temp0;
    for (unsigned int i = 0; i < frame_matches_temp.size(); i++)
    {
        SegFrameMatch& Match0 = frame_matches_temp[i];
        bool bIsMax = true;
        for (unsigned int j = 0; j < frame_matches_temp.size(); j++)
        {
            SegFrameMatch& Match1 = frame_matches_temp[j];
            if (i == j)
            {
                continue;
            }
            if (abs(Match1.base_map_id - Match0.base_map_id) > m_Param.nNonMaximumSuppressionRange)
            {
                continue;
            }
            if (abs(Match1.query_frame_id - Match0.query_frame_id) > m_Param.nNonMaximumSuppressionRange)
            {
                continue;
            }
            if (Match1.match_cont > Match0.match_cont)
            {
                bIsMax = false;
                break;
            }
        }
        if (bIsMax)
        {
            frame_matches_temp0.push_back(Match0);
        }
    }

    frame_matches = frame_matches_temp0;

    return frame_matches.size();
}

cv::Mat SegMatcher::ComputeEigenFeatureDistance(cv::Mat& feature1, cv::Mat& feature2)
{
    Mat diff_out = Mat::zeros(1, 35, CV_64F);
    Mat f_diff = cv::abs(feature1 - feature2);
    f_diff.copyTo(diff_out(Range::all(),Range(0,7)));

    Mat f1_abs = cv::abs(feature1);
    Mat f2_abs = cv::abs(feature2);

    Mat f_diff_norm_2 = f_diff/f2_abs;
    Mat f_diff_norm_1 = f_diff/f1_abs;

    f_diff_norm_2.copyTo(diff_out(Range::all(),Range(7,14)));
    f_diff_norm_1.copyTo(diff_out(Range::all(),Range(14,21)));
    f1_abs.copyTo(diff_out(Range::all(),Range(21,28)));
    f1_abs.copyTo(diff_out(Range::all(),Range(28,35)));

    return diff_out;
}

