#ifndef __TIMER_H__
#define __TIMER_H__

#include <iostream>
#include <string>
#include <time.h>

namespace personalRobotics
{
	class Timer
	{
	private:
		clock_t mClock;
		clock_t mStart;
		clock_t mEnd;
		std::string mTag;
	public:
		Timer(std::string tag);
		~Timer();
		void tic();
		void toc();
		double getElapsedTime(bool silence);
	};
}
#endif