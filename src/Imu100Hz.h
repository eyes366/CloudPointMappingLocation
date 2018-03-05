#ifndef _IMU_100HZ_H_
#define _IMU_100HZ_H_

#include <list>
#include <vector>
#include <string>
#include <stdint.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include "serialport.h"

#define IMU_100HZ_BUF_SIZE 1000
#define SERIAL_PORT_LIST_SIZE 100

#pragma pack(1)
struct Imu100HzData
{
	short szHead;
	double dTime;		//second
	double dLongitude;	//degree
	double dLatitude;	//degree
	float fAltitude;	//meter
	float fRollTheta;	//degree/sec
	float fPitchTheta;	//degree/sec
	float fHeadingTheta;//degree/sec
	float fRoll;		//degree
	float fPitch;		//degree
	float fYaw;			//degree
	float fTrackHeading;//degree
	float fYawRMS;		//degree
	float fAccEast;		//meter/sec^2
	float fAccNorth;	//meter/sec^2
	float fAccUp;		//meter/sec^2
	float fVelEast;		//m/s
	float fVelNorth;	//m/s
	float fVelUp;		//m/s
	unsigned char nState;
	unsigned char nCheckSum;
};
#pragma pack()


class CImu100HzReader
{
public:
	enum ParseState
	{
		FIND_HEAD_START,
		FIND_HEAD_END,
		FIND_DATA,
		CHECH,
		PARSE
	};
	CImu100HzReader();
	~CImu100HzReader();

	int64_t ReadFile(std::string szFilePathName);

	int GetDataByTime(uint64_t nTime, Imu100HzData& data);

	int OpenSerialPort(std::string szPortName);

	int GetDataByTimeFromBuffer(uint64_t nTime, Imu100HzData& data);

	std::vector<Imu100HzData> m_Data;

private:
	void FeedDataFlow(const char* pData, int nLen);
	void FeedDataFlowList(const char* pData, int nLen);
	std::vector<uint64_t> m_TimeRel;
	std::list<Imu100HzData> m_DataList;
	int m_nTimeRel;
	char m_ParseBuf[IMU_100HZ_BUF_SIZE];
	int m_nCurCapablity;
	int m_nSearchInd;
	SerialPort m_serialPort;
	boost::mutex m_lock;
};


#endif
