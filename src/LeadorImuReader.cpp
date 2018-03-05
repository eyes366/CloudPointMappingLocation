#include <boost/algorithm/string.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <math.h>
#include "LeadorImuReader.h"

using namespace std;

CLeadorImuDataReader::CLeadorImuDataReader()
{
    m_nSearchInd = 0;
}

int CLeadorImuDataReader::ReadLog(std::string szLogPath)
{
    std::ifstream fs(szLogPath.c_str());
    if (!fs.is_open())
    {
        return -1;
    }
    double temp;
    while(1)
    {
        CLeadorIMuData data;
        double dGpsWeek;
        fs >> dGpsWeek;
        double dGpsSec;
        fs >> dGpsSec;
        data.dGpsTime = dGpsWeek*7*24*3600 + dGpsSec;
        double dDay = floor(dGpsSec/(24*3600));
        data.dGpsSecOfDay = (dGpsSec-dDay*24*3600)*1000.0;
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
        m_DataList.push_back(data);
    }
    std::cout << m_DataList.front().dGpsSecOfDay << std::endl;
    std::cout << m_DataList.back().dGpsSecOfDay << std::endl;

    fs.close();

    return 1;
}

int CLeadorImuDataReader::GetDataByTime(double dTime, CLeadorIMuData& data)
{
    if (dTime > m_DataList.back().dGpsSecOfDay ||
        dTime < m_DataList.front().dGpsSecOfDay)
    {
//        m_nSearchInd = 0;
        return -1;
    }
    for (unsigned int i = m_nSearchInd; i+1 < m_DataList.size(); i++)
    {
        if (dTime >= m_DataList[i].dGpsSecOfDay &&
            dTime < m_DataList[i+1].dGpsSecOfDay)
        {
            m_nSearchInd = i;
            data = m_DataList[i];
            return 1;
        }
    }

    return -1;
}
