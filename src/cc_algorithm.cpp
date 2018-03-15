#include "cc_algorithm.h"
#include "ccPointCloud.h"
#include "CloudSamplingTools.h"

CcAlgorithm::CcAlgorithm()
{

}

CcAlgorithm::~CcAlgorithm()
{

}

int ccVoxelGridFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cp_in,
	pcl::PointCloud<pcl::PointXYZI>::Ptr& cp_out,
	float fDist)
{
	ccPointCloud *cc = new ccPointCloud;
	unsigned int size = cp_in->size();
	cc->reserve(size);
	for (size_t i = 0; i < size; i++)
	{
		const pcl::PointXYZI &temp = cp_in->at(i);
		cc->addPoint(CCVector3(temp.x, temp.y, temp.z));
	}

	CCLib::ReferenceCloud *sub_sample_src = CCLib::CloudSamplingTools::resampleCloudSpatially(cc, fDist, CCLib::CloudSamplingTools::SFModulationParams());
	cp_out->clear();
	cp_out->reserve(cp_in->size());
	for (unsigned int i = 0; i < sub_sample_src->size(); i++)
	{
		const CCVector3* pt = sub_sample_src->getPoint(i);
		pcl::PointXYZI pt_;
		pt_.x = pt->x;
		pt_.y = pt->y;
		pt_.z = pt->z;
		cp_out->push_back(pt_);
	}

	delete cc;

	return 1;
}