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