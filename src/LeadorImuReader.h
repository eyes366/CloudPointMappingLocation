#ifndef LEADOR_IMU_READER_H
#define LEADOR_IMU_READER_H

#include <string>
#include <vector>

class CLeadorIMuData
{
public:
    CLeadorIMuData()
    {
        memset(this, 0, sizeof(CLeadorIMuData));
    }
    double dGpsTime;
    double dGpsSecOfDay;
    double dLatitude;
    double dLongitude;
    double dAltitude;
    double dHeading;
    double dRoll;
    double dPitch;
};

class CLeadorImuDataReader
{
public:
    CLeadorImuDataReader();
    int ReadLog(std::string szLogPath);
    int GetDataByTime(double dTime, CLeadorIMuData& data);

private:
    std::vector<CLeadorIMuData> m_DataList;
    unsigned int m_nSearchInd;
};

#endif
