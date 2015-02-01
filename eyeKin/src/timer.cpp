#include <timer.h>

personalRobotics::Timer::Timer(std::string tag = "NO-NAME")
{
	mTag = tag;
	tic();
}
personalRobotics::Timer::~Timer()
{

}
void personalRobotics::Timer::tic()
{
	mStart = clock();
}
void personalRobotics::Timer::toc()
{
	mEnd = clock();
	std::cout << "Elapsed time for " << mTag << " is: " << (double)((mEnd - mStart) / CLOCKS_PER_SEC) << " secs." << std::endl;
}
double personalRobotics::Timer::getElapsedTime(bool silence = false)
{
	mEnd = clock();
	double duration = (double)((mEnd - mStart) / CLOCKS_PER_SEC);
	if (!silence)
		std::cout << "Elapsed time for " << mTag << " is: " << duration << " secs." << std::endl;
	return duration;
}