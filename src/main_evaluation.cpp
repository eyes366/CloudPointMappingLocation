#include <string>
#include <iostream>
#include <fstream>
#include "ImuNav.h"

using namespace std;

int main(int argc, char ** argv)
{
	string szNavFilePath(argv[1]);
	string szLocationResult(argv[2]);
	szLocationResult += "\\";
	cout << "szNavFilePath: " << szNavFilePath << endl;
	cout << "szLocationResult: " << szLocationResult << endl;

	CImuNav nav;
	if (nav.ReadFile(szNavFilePath) <= 0)
	{
		cout << "Read nav failed!!!!" << endl;
		getchar();
	}

	ifstream fs((szLocationResult + "Track.txt").c_str());
	if (!fs.is_open())
	{
		cout << "Read result failed!!!!" << endl;
		getchar();
	}

	ofstream fs_result((szLocationResult + "result.txt").c_str());
	char szLine[1024] = { 0 };
	while (fs.getline(szLine, 1024))
	{
		vector<string> segs;
		nav.split(string(szLine), string(" "), &segs);
		if (segs.size() != 8)
		{
			break;
		}
		int nInd = atoi(segs[0].c_str());
		double dResultTime = atof(segs[1].c_str());
		double dResultLat = atof(segs[5].c_str());
		double dResultLong = atof(segs[6].c_str());
		double dResultAlt = atof(segs[7].c_str());
		GnssData Refdata;
		int nRt = nav.GetDataByTime(uint64_t(dResultTime*1000000.0), Refdata);
		if (nRt == -1)
		{
			continue;
		}
		if (nRt == 0 || nRt == -2)
		{
			break;
		}
		double dDistY = 111319.55*(dResultLat - Refdata.dLatitude);
		double dDistX = 111319.55*(dResultLong - Refdata.dLongitude)*
			cos(Refdata.dLatitude/180.0*3.141592654);
		double dDistZ = dResultAlt - Refdata.dAltitude;
		double dErrorH = sqrt(pow(dDistX, 2) + pow(dDistY, 2));
		double dErrorV = dDistZ;
		double dErrorTotal = sqrt(pow(dErrorH, 2) + pow(dErrorV, 2));
		char szResultLine[1024] = { 0 };
		sprintf_s(szResultLine, 1024, "%d %.6f %.7f %.7f %.3f %.7f %.7f %.3f %.3f %.3f %.3f",
			nInd,
			dResultTime,
			dResultLat,
			dResultLong,
			dResultAlt,
			Refdata.dLatitude,
			Refdata.dLongitude,
			Refdata.dAltitude,
			dErrorH,
			dErrorV,
			dErrorTotal);
		fs_result << szResultLine << endl;
		fs_result.flush();
		cout << "dErrorH: " << dErrorH << endl;
	}

	cout << "end" << endl;
	getchar();
	return 1;
}
