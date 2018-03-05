struct  GPRMCInfo
{
	double latitude;
	double longitude;
	double speedInKnots;
	double trueCourse;
	double magneticVariation;
	char timeStamp[12];
	bool isActive;
};

int ParseGPSInfo(unsigned char *context, GPRMCInfo *info);