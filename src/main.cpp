#include <string>
#include <boost/thread.hpp>
#include "MappingOpr.h"

int main(int argc, char ** argv)
{
	if (argc < 2)
	{
		printf("Param miss...Input map original directory\n");
		getchar();
	}
	CMappingOpr maper;
	maper.StartMapping(argv[1]);

//	boost::thread::sleep(boost::system_time(boost::posix_time::pos_infin)); 
	boost::this_thread::sleep(boost::posix_time::seconds(10000));

	return 1;
}
