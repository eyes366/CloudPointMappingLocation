#include <iostream>
#include <fstream>
#include "ImuNav.h"

using namespace std;

CImuNav::CImuNav()
{
	m_nSearchInd = 0;
}

CImuNav::~CImuNav()
{

}

int CImuNav::ReadFile(std::string szFilePathName)
{
	std::ifstream fs(szFilePathName.c_str());
	if (!fs.is_open())
	{
		return -1;
	}
	double temp;
	while(1)
	{
		GnssData data;
		double dGpsWeek;
		fs >> dGpsWeek;
		data.dGpsWeek = dGpsWeek;
		double dGpsSec;
		fs >> dGpsSec;
		data.dSecInWeek = dGpsSec*1000000.0;
//		data.dGpsTime = dGpsWeek*7.0*24.0*3600.0 + dGpsSec;
		double dHour = floor(dGpsSec/3600.0);
		data.dRelTime = (dGpsSec-dHour*3600.0)*1000000.0;
		fs >> data.dLatitude;
		fs >> data.dLongitude;
		fs >> data.dAltitude;
		fs >> temp;
		fs >> temp;
		fs >> temp;
		fs >> data.dPitch;
		fs >> data.dRoll;
		fs >> data.dHeading;
		for (unsigned int i = 0; i < 9; i++)
		{
			fs >> temp;
		}
		if (fs.eof())
		{
			break;
		}
		data.nPoseType = 0;
		m_Data.push_back(data);
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

	return m_Data.size();
}

int CImuNav::GetDataByTime(uint64_t nTime, GnssData& data)
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
//			data.dGpsTime= dP0*data0.dGpsTime + dP1*data1.dGpsTime;
			data.dSecInWeek = dP0*data0.dSecInWeek + dP1*data1.dSecInWeek;
			data.dLongitude = dP0*data0.dLongitude + dP1*data1.dLongitude;
			data.dLatitude = dP0*data0.dLatitude + dP1*data1.dLatitude;
			data.dAltitude = dP0*data0.dAltitude + dP1*data1.dAltitude;
			data.dRoll = dP0*data0.dRoll + dP1*data1.dRoll;
			data.dPitch = dP0*data0.dPitch + dP1*data1.dPitch;
			if (abs(data0.dHeading-data1.dHeading) > 180.0)//处理跨零航向角插值情况
			{
				if (data0.dHeading > data1.dHeading)
					data1.dHeading+=360.0;
				else
					data0.dHeading+=360.0;
			}
			data.dHeading = dP0*data0.dHeading + dP1*data1.dHeading;
			
			return 1;
		}
	}

	return -1;
}

void CImuNav::split(std::string& s, std::string& delim,std::vector< std::string >* ret)
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