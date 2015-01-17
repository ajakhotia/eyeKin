#ifndef __UTILITIES_H__
#define __UTILITIES_H__

namespace personalRobotics
{
	long int gcd(long int aIn, long int bIn);
	long int gcdr(long int a, long int b);
	template<class _t> void logMessage(_t inMessage)
	{
		std::cout << inMessage << std::endl;
	}
}
#endif