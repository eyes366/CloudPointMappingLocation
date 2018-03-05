#pragma once

#include <vector>
#include <string>
#include <cstdint>
#include "common.h"

class CImuCalc
{
public:
	CImuCalc();
	~CImuCalc();

	int ReadFile(std::string szFilePathName);

	int GetDataByTime(uint64_t nTime, GnssData& data);

	std::vector<GnssData> m_Data;

private:
	void split(std::string& s, std::string& delim,std::vector< std::string >* ret);
	int m_nSearchInd;
};


