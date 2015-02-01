#ifndef __UTILITIES_H__
#define __UTILITIES_H__

#include <mutex>

namespace personalRobotics
{
	long int gcd(long int aIn, long int bIn);
	long int gcdr(long int a, long int b);
	template<class _t> void logMessage(_t inMessage)
	{
		std::cout << inMessage << std::endl;
	}
	class MutexBool
	{
	protected:
		std::mutex mMutex;
		bool mFlag;
		int counter;
	public:
		MutexBool();
		MutexBool(const MutexBool &obj);
		~MutexBool();
		void lockMutex();
		void unlockMutex();
		bool get();
		void set();
		void unset();
		MutexBool operator= (const bool &param);
	};
}
#endif