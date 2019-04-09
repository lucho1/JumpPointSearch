// ----------------------------------------------------
// j1Timer.cpp
// Fast timer with milisecons precision
// ----------------------------------------------------

#include "j1Timer.h"
#include "SDL\include\SDL_timer.h"

// ---------------------------------------------
j1Timer::j1Timer()
{
	Start();
}

void j1Timer::StartFrom(uint32 ms) {

	running = true;
	started_at = SDL_GetTicks() + (ms);
}

// ---------------------------------------------
void j1Timer::Start()
{
	started_at = SDL_GetTicks();
}

void j1Timer::Stop() {

	running = false;
	stopped_at = SDL_GetTicks();
}

// ---------------------------------------------
uint32 j1Timer::Read() const
{

	if (running == true)
		return (SDL_GetTicks() - started_at);
	else
		return stopped_at - started_at;
}

// ---------------------------------------------
float j1Timer::ReadSec() const
{

	if (running == true)
		return (float)(SDL_GetTicks() - started_at) / 1000.0f;
	else
		return (float)(stopped_at - started_at) / 1000.0f;
}