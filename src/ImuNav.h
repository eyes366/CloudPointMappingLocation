#pragma once

#include <vector>
#include <string>
#include <cstdint>
#include "common.h"

class CImuNav
{
public:
	CImuNav();
	~CImuNav();

	int ReadFile(std::string szFilePathName);

	int GetDataByTime(uint64_t nTime, GnssData& data);

	void split(std::string& s, std::string& delim, std::vector< std::string >* ret);

private:
	std::vector<GnssData> m_Data;
	int m_nSearchInd;
};


