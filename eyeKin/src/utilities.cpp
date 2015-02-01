#include "utilities.h"

long int personalRobotics::gcd(long int aIn, long int bIn)
{
	long int a, b;
	a = aIn>bIn ? aIn : bIn;
	b = aIn<bIn ? aIn : bIn;
	return gcdr(a, b);
}
long int personalRobotics::gcdr(long int a, long int b)
{
	//a is always greater than b
	if (b == 0) return a;
	return gcdr(b, a%b);
}

personalRobotics::MutexBool::MutexBool()
{
	mFlag = false;
	counter = 0;
}
personalRobotics::MutexBool::MutexBool(const personalRobotics::MutexBool &obj)
{
	counter = obj.counter;
	mFlag = obj.mFlag;
}
personalRobotics::MutexBool::~MutexBool()
{
	while (counter != 0)
	{
		if (counter > 0)
			unlockMutex();
		else
			lockMutex();
	}
}
void personalRobotics::MutexBool::lockMutex()
{
	mMutex.lock();
	counter++;
}
void personalRobotics::MutexBool::unlockMutex()
{
	mMutex.unlock();
	counter--;
}
bool personalRobotics::MutexBool::get()
{
	bool temp;
	lockMutex();
	temp = mFlag;
	unlockMutex();
	return temp;
}
void personalRobotics::MutexBool::set()
{
	lockMutex();
	mFlag = true;
	unlockMutex();
}
void personalRobotics::MutexBool::unset()
{
	lockMutex();
	mFlag = false;
	unlockMutex();
}
personalRobotics::MutexBool personalRobotics::MutexBool::operator= (const bool &param)
{
	MutexBool temp;
	temp.mFlag = param;
	return temp;
}