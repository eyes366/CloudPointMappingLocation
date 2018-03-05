#pragma once

struct GPS_DATA
{
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
	short    gps_week;

	double     gps_second;

	double     acc[3];

	double     gyro[3];

	double     odo;
};

struct PCAP_DATA
{
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
