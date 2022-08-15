#pragma once
#ifndef _SEMAPHORE_H
#define _SEMAPHORE_H
#include <mutex>
#include <condition_variable>
using namespace std;

class Semaphore
{
public:
	Semaphore(long count = 0) : count(count) {}
	//V²Ù×÷£¬»½ÐÑ
	void signal();
	
	//P²Ù×÷£¬×èÈû
	void wait();


private:
	mutex mt;
	condition_variable cond;
	long count;
};
#endif