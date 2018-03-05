#include "GPRMCInfo.h"
#include <string>
#include <string.h>
#include <math.h>

int ParseGPSInfo(unsigned char *context, GPRMCInfo *info)
{
	std::string str = "";
	int lastPos = 0;
	int sel = 0;
	int degree = 0;
	double value = 0.0f;
	for (int i = 0;context[i] != '\r';i++)
	{
		if(context[i] == ',')
		{
			lastPos = i;
			switch (sel)
			{
			case 0:
				if(str == "$GPRMC")
					sel++;
				break;
			case 1:
				memcpy(info->timeStamp, str.c_str(), 6);
				sel++;
				break;
			case 2:
				if(str == "A")
				{
					info->isActive = true;
					sel++;
				}
				else if(str == "V")
				{
					info->isActive = false;
					sel++;
				}
				break;
			case 3:
				value = atof(str.c_str());
				degree = (int)floor(value/100);
				info->latitude = (value - degree*100)/60+degree;
				sel++;
				break;
			case 4:
				//if(str == "N" || str == "S")
					sel++;
				break;
			case 5:
				value = atof(str.c_str());
				degree = (int)floor(value/100);
				info->longitude = (value - degree*100)/60+degree;
				sel++;
				break;
			case 6:
				//if(str == "E" || str == "W")
					sel++;
				break;
			case 7:
				info->speedInKnots = atof(str.c_str());
				sel++;
				break;
			case 8:
				info->trueCourse = atof(str.c_str());
				sel++;
				break;
			case 9:
				memcpy(info->timeStamp+6, str.c_str(), 6);
				sel++;
				break;
			case 10:
				info->magneticVariation = atof(str.c_str());
				sel++;
				break;
			default:
				break;
			}
			str = "";
		}
		else
		{
			str += context[i];
		}
	}

	return sel == 11 ? 0:-1;
}