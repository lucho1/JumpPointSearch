// ----------------------------------------------------
// j1Timer.cpp
// Fast timer with milisecons precision
// ----------------------------------------------------

#include "j1Timer.h"
#include "SDL\include\SDL_timer.h"

// ---------------------------------------------
j1Timer::j1Timer()
{
}

void j1Timer::StartFrom(uint32 ms)
{
	running = true;
	started_at = SDL_GetTicks() + (ms);
}

// ---------------------------------------------
void j1Timer::Start()
{
	running = true;
	started_at = SDL_GetTicks();
}

void j1Timer::Stop()
{
	running = false;
	stopped_at = SDL_GetTicks();
}

// ---------------------------------------------
uint32 j1Timer::Read() const
{
	if (running)
		return (SDL_GetTicks() - started_at);
	else
		return stopped_at - started_at;
}

// ---------------------------------------------
float j1Timer::ReadSec() const
{
	if (running)
		return (float)(SDL_GetTicks() - started_at) / 1000.0f;
	else
		return (float)(stopped_at - started_at) / 1000.0f;
}

bool j1Timer::IsRunning() const
{
	return running;
}
