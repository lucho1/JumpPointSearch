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

	bool IsRunning() const;

private:

	uint32	started_at = 0;
	uint32  stopped_at = 0;
	bool running = false;
};

#endif //__j1TIMER_H__