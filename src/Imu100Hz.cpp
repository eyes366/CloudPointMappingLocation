#include <iostream>
#include <fstream>
#include "Imu100Hz.h"

using namespace std;

CImu100HzReader::CImu100HzReader()
{
	memset(m_ParseBuf, 0, IMU_100HZ_BUF_SIZE);
	m_nCurCapablity = 0;
	m_nTimeRel= 0;
	m_nSearchInd= 0;
}

CImu100HzReader::~CImu100HzReader()
{

}

int64_t CImu100HzReader::ReadFile(std::string szFilePathName)
{
	ifstream in(szFilePathName.c_str(), ios::in|ios::binary);
	if (!in.is_open())
	{
		return -1;
	}
	char szBuf[88];
	while (in.read(szBuf, 88))
	{
		FeedDataFlow(szBuf, 88);
	}

	cout << "Start time:" << m_Data.front().dTime << endl;
	cout << "End time:" << m_Data.back().dTime << endl;

	return m_Data.size();
}

void CImu100HzReader::FeedDataFlow(const char* pData, int nLen)
{
	if (m_nCurCapablity + nLen > IMU_100HZ_BUF_SIZE)
	{
		cout << "m_ParseBuf over flow!!!!!!!!!" << endl;
		m_nCurCapablity = 0;
		getchar();
	}

	memcpy(&(m_ParseBuf[m_nCurCapablity]), pData, nLen);
	m_nCurCapablity += nLen;

	char szHeader[2] = {0x55, 0xAA};
	int nHeaderLen = 2;
	int nDataLen = 88;
	int nFindHeaderInd = 0;
	int nHeaderStart = -1;
	ParseState state = FIND_HEAD_START;
	for (unsigned int i = 0; i < m_nCurCapablity; i++)
	{
		if (state == FIND_HEAD_START)
		{
			if (m_ParseBuf[i] == szHeader[nFindHeaderInd])
			{
				nFindHeaderInd++;
				nHeaderStart = i;
				state = FIND_HEAD_END;
			}
		}
		else if (state == FIND_HEAD_END)
		{
			if (nFindHeaderInd >= nHeaderLen)
			{
				state = FIND_DATA;
				continue;
			}
			if (m_ParseBuf[i] == szHeader[nFindHeaderInd])
			{
				nFindHeaderInd++;
			}
			else
			{
				nFindHeaderInd = 0;
				nHeaderStart = -1;
				state = FIND_HEAD_START;
			}
		}
		else if (state == FIND_DATA)
		{
			if (i-nHeaderStart == nDataLen-1)
			{
				nFindHeaderInd = 0;
				state = FIND_HEAD_START;
				Imu100HzData dataT;
				memcpy(&dataT, m_ParseBuf+nHeaderStart, nDataLen);

				uint64_t temp = 0;
				uint64_t nSec1 = uint64_t(dataT.dTime*1000000ULL)%3600000000ULL;
				if (m_Data.size() <= 0)
				{
					m_TimeRel.push_back(nSec1);
					temp = nSec1;
				}
				else
				{
					uint64_t nSec0 = uint64_t(m_Data.back().dTime);
					if (nSec1 < nSec0)
					{
						m_nTimeRel++;
					}
					m_TimeRel.push_back(nSec1+m_nTimeRel*3600000000ULL);
					temp = nSec1+m_nTimeRel*3600000000ULL;
				}

				dataT.dTime = temp;
				m_Data.push_back(dataT);
			}
		}
	}

	if (state == FIND_HEAD_START)
	{
		m_nCurCapablity = 0;
	}
	else
	{
		memcpy(m_ParseBuf, m_ParseBuf+nHeaderStart, m_nCurCapablity-nHeaderStart);
		m_nCurCapablity -= nHeaderStart;
	}

}

void CImu100HzReader::FeedDataFlowList(const char* pData, int nLen)
{
	if (m_nCurCapablity + nLen > IMU_100HZ_BUF_SIZE)
	{
		cout << "m_ParseBuf over flow!!!!!!!!!" << endl;
		m_nCurCapablity = 0;
		getchar();
	}

	memcpy(&(m_ParseBuf[m_nCurCapablity]), pData, nLen);
	m_nCurCapablity += nLen;

	char szHeader[2] = {0x55, 0xAA};
	int nHeaderLen = 2;
	int nDataLen = 88;
	int nFindHeaderInd = 0;
	int nHeaderStart = -1;
	ParseState state = FIND_HEAD_START;
	for (unsigned int i = 0; i < m_nCurCapablity; i++)
	{
		if (state == FIND_HEAD_START)
		{
			if (m_ParseBuf[i] == szHeader[nFindHeaderInd])
			{
				nFindHeaderInd++;
				nHeaderStart = i;
				state = FIND_HEAD_END;
			}
		}
		else if (state == FIND_HEAD_END)
		{
			if (nFindHeaderInd >= nHeaderLen)
			{
				state = FIND_DATA;
				continue;
			}
			if (m_ParseBuf[i] == szHeader[nFindHeaderInd])
			{
				nFindHeaderInd++;
			}
			else
			{
				nFindHeaderInd = 0;
				nHeaderStart = -1;
				state = FIND_HEAD_START;
			}
		}
		else if (state == FIND_DATA)
		{
			if (i-nHeaderStart == nDataLen-1)
			{
				nFindHeaderInd = 0;
				state = FIND_HEAD_START;
				Imu100HzData dataT;
				memcpy(&dataT, m_ParseBuf+nHeaderStart, nDataLen);

				uint64_t temp = 0;
				uint64_t nSec1 = uint64_t(dataT.dTime*1000000ULL)%3600000000ULL;
				if (m_Data.size() <= 0)
				{
					temp = nSec1;
				}
				else
				{
					uint64_t nSec0 = uint64_t(m_Data.back().dTime*1000000ULL)%3600000000ULL;
					if (nSec1 < nSec0)
					{
						m_nTimeRel++;
					}
					temp = nSec1+m_nTimeRel*3600000000ULL;
				}
				dataT.dTime = temp;

				m_lock.lock();
				m_DataList.push_back(dataT);
				if (m_DataList.size() > SERIAL_PORT_LIST_SIZE)
				{
					m_DataList.pop_front();
				}
				m_lock.unlock();
			}
		}
	}

	if (state == FIND_HEAD_START)
	{
		m_nCurCapablity = 0;
	}
	else
	{
		memcpy(m_ParseBuf, m_ParseBuf+nHeaderStart, m_nCurCapablity-nHeaderStart);
		m_nCurCapablity -= nHeaderStart;
	}

}

int CImu100HzReader::GetDataByTime(uint64_t nTime, Imu100HzData& data)
{
	if (nTime > m_Data.back().dTime ||
		nTime < m_Data.front().dTime)
	{
		return -1;
	}

	for (unsigned int i = m_nSearchInd; i+1 < m_Data.size(); i++)
	{
		if (nTime >= m_Data[i].dTime &&
			nTime < m_Data[i+1].dTime)
		{
			m_nSearchInd = i;
			Imu100HzData data0 = m_Data[i];
			Imu100HzData data1 = m_Data[i+1];
			double dP1 = (double)(nTime-m_Data[i].dTime)/(double)(m_Data[i+1].dTime-m_Data[i].dTime);
			double dP0 = (double)(m_Data[i+1].dTime-nTime)/(double)(m_Data[i+1].dTime-m_Data[i].dTime);
			data = m_Data[i];
			data.dTime = dP0*data0.dTime + dP1*data1.dTime;
			data.dLongitude = dP0*data0.dLongitude + dP1*data1.dLongitude;
			data.dLatitude = dP0*data0.dLatitude + dP1*data1.dLatitude;
			data.fAltitude = dP0*data0.fAltitude + dP1*data1.fAltitude;
			data.fRoll = dP0*data0.fRoll + dP1*data1.fRoll;
			data.fPitch = dP0*data0.fPitch + dP1*data1.fPitch;
			data.fYaw = dP0*data0.fYaw + dP1*data1.fYaw;
			return 1;
		}
	}

	return -1;
}

int CImu100HzReader::OpenSerialPort(std::string szPortName)
{
	if (!m_serialPort.init_port(szPortName.c_str()))
	{
		cout << "Open serial port failed!!!  " << szPortName << endl;
		getchar();
		return -1;
	}

	RecieveCallBack cb = boost::bind(&CImu100HzReader::FeedDataFlowList, this, _1, _2);
	m_serialPort.setCallBack(cb);

}

int CImu100HzReader::GetDataByTimeFromBuffer(uint64_t nTime, Imu100HzData& data)
{
	m_lock.lock();
	if (nTime <= m_DataList.front().dTime)
	{
		m_lock.unlock();
		return -1;	//请求的数据太旧
	}
	if (nTime >= m_DataList.back().dTime)
	{
		m_lock.unlock();
		return 0;	//请求的数据太新
	}

	std::list<Imu100HzData>::reverse_iterator rit0, rit1;
	for (rit0 = m_DataList.rbegin(), rit1 = m_DataList.rbegin(), rit0++;
		rit0 != m_DataList.rend() && rit1 != m_DataList.rend(); rit0++, rit1++)
	{
		if (nTime >= rit0->dTime &&
			nTime < rit1->dTime)
		{
			Imu100HzData data0 = *rit0;
			Imu100HzData data1 = *rit1;
			double dP1 = (double)(nTime-rit0->dTime)/(double)(rit1->dTime-rit0->dTime);
			double dP0 = (double)(rit1->dTime-nTime)/(double)(rit1->dTime-rit0->dTime);
			data = *rit0;
			data.dTime = dP0*data0.dTime + dP1*data1.dTime;
			data.dLongitude = dP0*data0.dLongitude + dP1*data1.dLongitude;
			data.dLatitude = dP0*data0.dLatitude + dP1*data1.dLatitude;
			data.fAltitude = dP0*data0.fAltitude + dP1*data1.fAltitude;
			data.fRoll = dP0*data0.fRoll + dP1*data1.fRoll;
			data.fPitch = dP0*data0.fPitch + dP1*data1.fPitch;
			data.fYaw = dP0*data0.fYaw + dP1*data1.fYaw;
			m_lock.unlock();
			return 1;
		}
	}

	m_lock.unlock();
	return -1;
}