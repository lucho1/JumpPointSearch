#ifndef __j1TIMER_H__
#define __j1TIMER_H__

#include "p2Defs.h"

class j1Timer
{
public:

	// Constructor
	j1Timer();

	void Start();
	void Stop();
	void StartFrom(uint32 secs);

	uint32 Read() const;
	float ReadSec() const;

private:

	uint32	started_at;
	uint32  stopped_at;
	bool running;


};

#endif //__j1TIMER_H__