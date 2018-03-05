#pragma once

#include <cstdint>
#include <string>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include "CommonHeader.h"

struct GnssData
{
	GnssData()
	{
		memset(this, 0, sizeof(GnssData));
	}
	void rectifyQuaternion()
	{
		Eigen::Quaterniond q(qw, qx, qy, qz);
 		Eigen::Matrix3d rot =  (Eigen::AngleAxisd(-3.141592653589*0.5, Eigen::Vector3d::UnitZ ())*
 			Eigen::AngleAxisd(3.141592653589, Eigen::Vector3d::UnitY ())).matrix();
 		Eigen::Quaterniond q_(q.toRotationMatrix()*rot);
		qw = q_.w();
		qx = q_.x();
		qy = q_.y();
		qz = q_.z();
	}
	void fromPOSE_DATA(POSE_DATA& pose_data)
	{
		this->nPoseType = 1;
		this->dRelTime = 0.0;
		this->dGpsWeek =	pose_data.gps_week;
		this->dSecInWeek =	pose_data.gps_second;
		this->dLatitude =	pose_data.pos[0];
		this->dLongitude =	pose_data.pos[1];
		this->dAltitude =	pose_data.pos[2];
		this->qw =			pose_data.quat[0];
		this->qx =			pose_data.quat[1];
		this->qy =			pose_data.quat[2];
		this->qz =			pose_data.quat[3];
		this->dVe =			pose_data.vn[0];
		this->dVn =			pose_data.vn[1];
		this->dVu =			pose_data.vn[2];
	}
	POSE_DATA toPOSE_DATA()
	{
		POSE_DATA pose_data;
		pose_data.gps_week		= this->dGpsWeek;
		pose_data.gps_second	= this->dSecInWeek;
		pose_data.pos[0]		= this->dLatitude;
		pose_data.pos[1]		= this->dLongitude;
		pose_data.pos[2]		= this->dAltitude;
		pose_data.quat[0]		= this->qw;
		pose_data.quat[1]		= this->qx;
		pose_data.quat[2]		= this->qy;
		pose_data.quat[3]		= this->qz;
		pose_data.vn[0]			= this->dVe;
		pose_data.vn[1]			= this->dVn;
		pose_data.vn[2]			= this->dVu;

		return pose_data;
	}
	double dGpsWeek;
	double dSecInWeek;
	double dRelTime;//相对第一帧的时间，微秒
	double dLatitude;
	double dLongitude;
	double dAltitude;
	int nPoseType;//0:欧拉角 1:四元数
	double dHeading;
	double dRoll;
	double dPitch;
	double qw;
	double qx;
	double qy;
	double qz;
	double dVe;
	double dVn;
	double dVu;
};

