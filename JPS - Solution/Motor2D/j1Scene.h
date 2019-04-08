#ifndef __j1SCENE_H__
#define __j1SCENE_H__

#include "j1Module.h"
#include "j1PerfTimer.h"

struct SDL_Texture;

class j1Scene : public j1Module
{
public:

	j1Scene();

	// Destructor
	virtual ~j1Scene();

	// Called before render is available
	bool Awake();

	// Called before the first frame
	bool Start();

	// Called before all Updates
	bool PreUpdate();

	// Called each loop iteration
	bool Update(float dt);

	// Called before all Updates
	bool PostUpdate();

	// Called before quitting
	bool CleanUp();

private:

	//Pathfinding Debug Stuff
	SDL_Texture* debug_tex;
	bool activateJPS = false;
	j1PerfTimer PathfindingTimer;
	double Ptime;

	//Performance Test Showing (Debug Purposes)
	char* AlgorithmUsed = "Algorithm Used: A-Star (press F to change)";
	SDL_Rect algorithmUsed_rect = {0, 0, 1, 1};
	SDL_Texture* algorithmUsed_text = nullptr;

	char* ms_char = "Lasted Time (ms): ";
	SDL_Rect ms_charRect = { 0, 0, 1, 1 };
	SDL_Texture*  ms_charText = nullptr;

	char number_ms[10];
	SDL_Texture *number_msTexture = nullptr;
	SDL_Rect number_ms_rect = { 0, 0, 1, 1 };

};

#endif // __j1SCENE_H__