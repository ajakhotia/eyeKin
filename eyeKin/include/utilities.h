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
	template<typename T> class MutexType
	{
	protected:
		std::mutex mMutex;
		T data;
		int counter;
	public:
		MutexType()
		{
			counter = 0;
		}
		MutexType(T t)
		{
			counter = 0;
			data = t;
		}
		MutexType(const MutexType<T> &obj)
		{
			counter = obj.counter;
			data = obj.data;
		}
		~MutexType()
		{
			while (counter != 0)
			{
				if (counter > 0)
					unlockMutex();
				else
					lockMutex();
			}
		}
		void lockMutex()
		{
			mMutex.lock();
			counter++;
		}
		void unlockMutex()
		{
			mMutex.unlock();
			counter--;
		}
		T get()
		{
			this->lockMutex();
			T temp = this->data;
			this->unlockMutex();
			return temp;
		}
		T* getPtr()
		{
			T* temp;
			lockMutex();
			temp = &data;
			unlockMutex();
			return temp;
		}
		void set(T t)
		{
			lockMutex();
			data = t;
			unlockMutex();
		}
		MutexType<T> operator= (const T &param)
		{
			return MutexType<T>(param);
		}
		MutexType<T>& operator++ ()
		{
			this->lockMutex();
			this->data++;
			this->unlockMutex();
			return *this;
		}
		MutexType<T> operator++ (int)
		{
			MutexType<T> temp;
			this->lockMutex();
			temp.data = data;
			this->data++;
			this->unlockMutex();
			return temp;
		}
		MutexType<T>& operator-- ()
		{
			this->lockMutex();
			this->data--;
			this->unlockMutex();
			return *this;
		}
		MutexType<T> operator-- (int)
		{
			MutexType<T> temp;
			this->lockMutex();
			temp.data = data;
			temp.counter = counter;
			this->data--;
			this->unlockMutex();
			return temp;
		}
	};

	typedef MutexType<bool> MutexBool;


        /// Return the current date to the second in a standard format.
        std::string full_date_string(void);
}
#endif