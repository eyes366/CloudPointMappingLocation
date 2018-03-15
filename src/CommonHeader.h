#pragma once

struct GPS_DATA
{
	long long  pc_micro_second;

	short    gps_week;

	double     gps_second;

	char     status;

	char     satellite_size;

	double     longti;

	double     lati;

	float      height;
};

struct IMU_A05_DATA
{
	long long  pc_micro_second;

	short    gps_week;

	double     gps_second;

	double     acc[3];

	double     gyro[3];

	double     odo;
};

struct PCAP_DATA
{
	long long  pc_micro_second;

	short    gps_week;

	double     gps_second;

	char    data[1206];
};

struct POSE_DATA
{
	short    gps_week;

	double     gps_second;

	double quat[4];
	
	double pos[3];

	double vn[3];
};

#ifdef QT_VERSION
#include <QMetaType>
Q_DECLARE_METATYPE(GPS_DATA)
Q_DECLARE_METATYPE(IMU_A05_DATA)
Q_DECLARE_METATYPE(PCAP_DATA)
#endif
