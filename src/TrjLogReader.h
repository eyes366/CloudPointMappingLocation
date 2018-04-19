#pragma once
#include <vector>
#include <string>
#include <stdio.h>
#include "TrjFile.h"
#include <pcl/common/transforms.h>
#include "common.h"

class CTrjLogReader
{
public:
    CTrjLogReader();
    int ReadFile(std::string szLogPath);
	int GetDataByTime(uint64_t nTime, GnssData& pose);

public:
    int m_nSearchInd;
    std::vector<GnssData> m_poses;
};

