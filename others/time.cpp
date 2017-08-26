#include <sys/time.h>
#include <stdlib.h>
#include <iostream>

using namespace std;

int main(int argc, char* argv[])
{
	if(argc !=2){
		cout << "Usage: time offset \n";
		return 1;
	}
	int offset = atoi(argv[1]);
	struct timeval tv;
	struct timezone tz;
	gettimeofday(&tv, &tz);

	tv.tv_sec += offset;
	settimeofday(&tv, &tz);
	return 0;

}