#ifndef PCSTIMER_H
#define PCSTIMER_H

#include "pcs_pc_lib_global.h"

class PCS_PC_LIB_EXPORT pcsTimer
{
public:
	static void Init();
	static int Seconds();
	static int MilliSeconds();
};

template<class T> struct pcsSingle
{
	pcsSingle() : instance(0) {}
	~pcsSingle() { 
		release();
	}
	void release() { 
		if (instance) 
			delete instance;
		instance = 0;
	}

	T* instance;
};

#endif // PCSTIMER_H
