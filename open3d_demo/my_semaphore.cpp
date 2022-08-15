#include "my_semaphore.h"

void Semaphore::signal()
{

	unique_lock<mutex> unique(mt);
	++count;
	if (count <= 0)
		cond.notify_one();

}

void Semaphore::wait()
{
	unique_lock<mutex> unique(mt);
	--count;
	if (count < 0)
		cond.wait(unique);
}

