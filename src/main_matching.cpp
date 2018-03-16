#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/ndt.h>
#include <Eigen/Core>
#include <fstream>
#include <iostream>

using namespace pcl;
using namespace std;

using namespace std;

int main(int argc, char ** argv)
{
	char* szLogPath = argv[2];
	char szPath[1024] = { 0 };
	int ind = atoi(argv[1]);
	PointCloud<PointXYZI>::Ptr map(new PointCloud<PointXYZI>);
	PointCloud<PointXYZI>::Ptr match_without_guess(new PointCloud<PointXYZI>);
	sprintf_s(szPath, 1024, "%s\\%06dMap.pcd", szLogPath, ind);
	pcl::io::loadPCDFile(szPath, *map);
	sprintf_s(szPath, 1024, "%s\\%06dMatchNoGuess.pcd", szLogPath, ind);
	pcl::io::loadPCDFile(szPath, *match_without_guess);
	sprintf_s(szPath, 1024, "%s\\%06dGuessTf.txt", szLogPath, ind);
	std::ifstream fs(szPath);
	Eigen::Matrix4f pose;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			float ddd;
			fs >> ddd;
			pose(i, j) = ddd;
		}
		
	}
	fs.close();
	std::cout << pose << std::endl;
	pcl::NormalDistributionsTransform<PointXYZI, PointXYZI> ndt;
	ndt.setTransformationEpsilon(0.01);
	ndt.setStepSize(2.0);//3.0
	ndt.setResolution(2.0);//4.0
	ndt.setMaximumIterations(25);
	ndt.setInputSource(match_without_guess);
	ndt.setInputTarget(map);
	PointCloud<PointXYZI>::Ptr out(new PointCloud<PointXYZI>);
	ndt.align(*out, pose);
	sprintf_s(szPath, 1024, "%s\\%06dResultWithOutGuess.pcd", szLogPath, ind);
	pcl::io::savePCDFileBinary(szPath, *out);
	std::cout << szPath << std::endl;
	std::cout << ndt.getFinalTransformation() << std::endl;

	PointCloud<PointXYZI>::Ptr map_(new PointCloud<PointXYZI>);
	sprintf_s(szPath, 1024, "%s\\%06dMap.pcd", szLogPath, ind);
	pcl::io::loadPCDFile(szPath, *map_);
	PointCloud<PointXYZI>::Ptr match_with_guess(new PointCloud<PointXYZI>);
	sprintf_s(szPath, 1024, "%s\\%06dMatchWithGuess.pcd", szLogPath, ind);
	pcl::io::loadPCDFile(szPath, *match_with_guess);
	pcl::NormalDistributionsTransform<PointXYZI, PointXYZI> ndt_;
	ndt_.setTransformationEpsilon(0.01);
	ndt_.setStepSize(2.0);//3.0
	ndt_.setResolution(2.0);//4.0
	ndt_.setMaximumIterations(25);
	ndt_.setInputSource(match_with_guess);
	ndt_.setInputTarget(map_);
	PointCloud<PointXYZI>::Ptr out_(new PointCloud<PointXYZI>);
	ndt_.align(*out_);
	sprintf_s(szPath, 1024, "%s\\%06dResultWithGuess.pcd", szLogPath, ind);
	pcl::io::savePCDFileBinary(szPath, *out_);
	std::cout << szPath << std::endl;
	std::cout << pose*ndt_.getFinalTransformation() << std::endl;

// 	Eigen::Matrix4d ndt_pose = ndt.getFinalTransformation().cast<double>();
// 	int nFinalIter = ndt.getFinalNumIteration();
// 	bool bIsConverge = ndt.hasConverged();
// 	cout << ndt_pose << endl;
// 	cout << nFinalIter << endl;
// 	cout << bIsConverge << endl;
// 	cout << pose.cast<double>().inverse()*ndt_pose << endl;

	getchar();

	return 1;

	for (unsigned int i = 0; i < 1000; i++)
	{
		char szPcdPath[1024] = { 0 };
		sprintf_s(szPcdPath, 1024, "D:\\MyData\\20180312\\own\\2018-03-12-15-36-01\\map\\%06d.pcd", i);
		PointCloud<PointXYZI>::Ptr source(new PointCloud<PointXYZI>);
		pcl::io::loadPCDFile(szPcdPath, *source);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
		sor.setInputCloud(source);
		sor.setMeanK(50);
		sor.setStddevMulThresh(1.0);
		PointCloud<PointXYZI>::Ptr filtered(new PointCloud<PointXYZI>);
		sor.filter(*filtered);
		sprintf_s(szPcdPath, 1024, "D:\\MyData\\20180312\\own\\2018-03-12-15-36-01\\map\\%06d_.pcd", i);
		pcl::io::savePCDFileBinary(szPcdPath, *filtered);
		std::cout << szPcdPath << std::endl;
	}


	return 1;
}
