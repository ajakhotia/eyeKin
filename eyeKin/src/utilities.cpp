#include <iostream>
#include <iomanip>
#include <sstream>
#include <time.h>

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

std::string personalRobotics::full_date_string(void)
{
  time_t t = time(NULL);
  struct tm *now = localtime(&t);

  std::ostringstream name;
  
  name << std::setfill('0')
       << std::setw(4) <<  (now->tm_year + 1900) << "-"
       << std::setw(2) <<  (now->tm_mon + 1) << "-"
       << std::setw(2) <<  (now->tm_mday) << "-"
       << std::setw(2) <<  (now->tm_hour) << "-"
       << std::setw(2) <<  (now->tm_min) << "-"
       << std::setw(2) <<  (now->tm_sec);

  return name.str();
}
