#include "TrjLogReader.h"

CTrjLogReader::CTrjLogReader()
{
    m_nSearchInd = 0;
}

int CTrjLogReader::ReadFile(std::string szLogPath)
{
	std::vector<BlockTrj> poses;
    TrjFileParse trj;
    trj.open(szLogPath.c_str());
    HeaderTrj header;
    trj.readHeader(header);
    while(1)
    {
        BlockTrj poseT;
        if (trj.readBlock(poseT) < 0)
        {
            break;
        }
		poseT.time = uint64_t(poseT.time - floor(poseT.time / (24.0*3600.0*1000000.0))*24.0*3600.0*1000000.0);
        poses.push_back(poseT);
		GnssData gps;
		gps.dSecInWeek = poseT.time;// (poseT.time - floor(poseT.time / (24.0*3600.0*1000000.0))*24.0*3600.0*1000000.0);
		gps.nPositionType = 1;
		gps.dX = poseT.pos[0];
		gps.dY = poseT.pos[1];
		gps.dZ = poseT.pos[2];
		gps.nPoseType = 1;
		gps.qw = poseT.q[0];
		gps.qx = poseT.q[1];
		gps.qy = poseT.q[2];
		gps.qz = poseT.q[3];
		m_poses.push_back(gps);
    }
    std::cout << "Poses cont: " << poses.size() << std::endl;
    std::cout << "Start pose" << poses.front().pos[0] << "," <<
                 poses.front().pos[1] << "," <<
                 poses.front().pos[2] << "," << std::endl;
    std::cout << "End pose" << poses.back().pos[0] << "," <<
                 poses.back().pos[1] << "," <<
                 poses.back().pos[2] << "," << std::endl;
    std::cout << "Start time" << poses.front().time << std::endl;
    std::cout << "End time" << poses.back().time << std::endl;

    return poses.size();
}


int CTrjLogReader::GetDataByTime(uint64_t nTime, GnssData& data)
{
	if (m_poses.size() <= 0)
	{
		return -1;
	}
	if (nTime > m_poses.back().dSecInWeek ||
		nTime < m_poses.front().dSecInWeek)
	{
		return -1;
	}

	for (unsigned int i = m_nSearchInd; i + 1 < m_poses.size(); i++)
	{
		if (nTime >= m_poses[i].dSecInWeek &&
			nTime < m_poses[i + 1].dSecInWeek)
		{
			m_nSearchInd = i;
			GnssData data0 = m_poses[i];
			GnssData data1 = m_poses[i + 1];
			double dP1 = (double)(nTime - m_poses[i].dSecInWeek) / (double)(m_poses[i + 1].dSecInWeek - m_poses[i].dSecInWeek);
			double dP0 = (double)(m_poses[i + 1].dSecInWeek - nTime) / (double)(m_poses[i + 1].dSecInWeek - m_poses[i].dSecInWeek);
			data = m_poses[i];
			data.dGpsWeek = dP0*data0.dGpsWeek + dP1*data1.dGpsWeek;
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
