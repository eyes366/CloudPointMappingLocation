#include <iostream>
#include <fstream>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include "ImuCalc.h"

using namespace std;

CImuCalc::CImuCalc()
{
	m_nSearchInd = 0;
}

CImuCalc::~CImuCalc()
{

}

int CImuCalc::ReadFile(std::string szFilePathName)
{
	cout << "Reading:" << szFilePathName << endl;
	std::ifstream fs(szFilePathName.c_str());
	if (!fs.is_open())
	{
		return -1;
	}
	char szLine[1024] = {0};
	while (fs.getline(szLine, 1024))
	{
		std::vector<std::string> strings;
		split(std::string(szLine), string(" "), &strings);
		if (strings.size() != 9)
		{
			break;
		}
		GnssData data;
		data.dGpsWeek =		atoi(strings[0].c_str());
		data.dSecInWeek =	atof(strings[1].c_str());
		data.dSecInWeek = data.dSecInWeek*1000000.0;
		data.qw =			atof(strings[2].c_str());
		data.qx =			atof(strings[3].c_str());
		data.qy =			atof(strings[4].c_str());
		data.qz =			atof(strings[5].c_str());
		data.dLatitude =	atof(strings[6].c_str());
		data.dLongitude =	atof(strings[7].c_str());
		data.dAltitude =	atof(strings[8].c_str());
//		data.rectifyQuaternion();
		data.nPoseType = 1;
		m_Data.push_back(data);
// 		if (m_Data.size() >= 10000)
// 		{
// 			break;
// 		}
	}
	if (m_Data.size() <= 0)
	{
		return -1;
	}

	vector<GnssData> dataT = m_Data;
	int nRelHour = 0;
	for (unsigned int i = 0; i+1 < m_Data.size(); i++)
	{
		m_Data[i].dRelTime = 
			m_Data[i].dRelTime + double(nRelHour)*3600000000.0;
		if (dataT[i+1].dRelTime < 
			dataT[i].dRelTime)
		{
			nRelHour++;
		}
	}

	printf("Start time: %.2f\n", m_Data.front().dSecInWeek);
	printf("End time: %.2f\n", m_Data.back().dSecInWeek);

	fs.close();

	return 1;
}

int CImuCalc::GetDataByTime(uint64_t nTime, GnssData& data)
{
	if (m_Data.size() <= 0)
	{
		return -1;
	}
	if (nTime > m_Data.back().dSecInWeek ||
		nTime < m_Data.front().dSecInWeek)
	{
		return -1;
	}

	for (unsigned int i = m_nSearchInd; i+1 < m_Data.size(); i++)
	{
		if (nTime >= m_Data[i].dSecInWeek &&
			nTime < m_Data[i+1].dSecInWeek)
		{
			m_nSearchInd = i;
			GnssData data0 = m_Data[i];
			GnssData data1 = m_Data[i+1];
			double dP1 = (double)(nTime-m_Data[i].dSecInWeek)/(double)(m_Data[i+1].dSecInWeek-m_Data[i].dSecInWeek);
			double dP0 = (double)(m_Data[i+1].dSecInWeek-nTime)/(double)(m_Data[i+1].dSecInWeek-m_Data[i].dSecInWeek);
			data = m_Data[i];
			data.dGpsWeek= dP0*data0.dGpsWeek + dP1*data1.dGpsWeek;
			data.dSecInWeek = dP0*data0.dSecInWeek + dP1*data1.dSecInWeek;
			data.dLongitude = dP0*data0.dLongitude + dP1*data1.dLongitude;
			data.dLatitude = dP0*data0.dLatitude + dP1*data1.dLatitude;
			data.dAltitude = dP0*data0.dAltitude + dP1*data1.dAltitude;
			data.dRoll = dP0*data0.dRoll + dP1*data1.dRoll;
			data.dPitch = dP0*data0.dPitch + dP1*data1.dPitch;
			data.dHeading = dP0*data0.dHeading + dP1*data1.dHeading;
			Eigen::Quaterniond q0(data0.qw, data0.qx, data0.qy, data0.qz);//²åÖµ
			Eigen::Quaterniond q1(data1.qw, data1.qx, data1.qy, data1.qz);//
			Eigen::Quaterniond qres = q0.slerp(dP1, q1);
			data.qw = qres.w();
			data.qx = qres.x();
			data.qy = qres.y();
			data.qz = qres.z();

			return 1;
		}
	}

	return -1;
}

void CImuCalc::split(std::string& s, std::string& delim,std::vector< std::string >* ret)
{  
	size_t last = 0;  
	size_t index=s.find_first_of(delim,last);  
	while (index!=std::string::npos)  
	{  
		ret->push_back(s.substr(last,index-last));  
		last=index+1;  
		index=s.find_first_of(delim,last);  
	}  
	if (index-last>0)  
	{  
		ret->push_back(s.substr(last,index-last));  
	}  
} 