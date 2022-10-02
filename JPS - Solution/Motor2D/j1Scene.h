#ifndef __j1SCENE_H__
#define __j1SCENE_H__

#include "j1Module.h"
#include "j1PerfTimer.h"
#include "j1Timer.h"

struct SDL_Texture;

struct FontTexture
{
	FontTexture(const std::string& text) : fontText(text) {}

	std::string fontText = "";
	SDL_Rect fontRect = { 0, 0, 1, 1 };
	SDL_Texture* fontTexture = nullptr;
};

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
	
	// Fonts
	SDL_Texture* SetupFontText(const std::string& text, SDL_Rect* textRect);

	// Map setup
	bool LoadMap(const char* mapFilepath);
	void SetupMap(const char* mapFilepath);
	std::string GetCurrentMapBlitText();

	// Switch maps
	void SwitchMap();


private:

	//Pathfinding Debug Stuff
	SDL_Texture* debug_tex;
	bool activateJPS = false;
	j1PerfTimer PathfindingTimer;

	//Performance Test Showing
	FontTexture algorithmInfo = "Algorithm Used: A-Star (press F to change)";
	FontTexture timeInfo = "Lasted Time (ms): ";
	FontTexture millisecondsInfo = "0000";

	// Maps
	const char* map1 = "iso_walk.tmx";
	const char* map2 = "iso.tmx";

	// Map Info
	FontTexture cameraInfo = "Press C to reset the Camera position";
	FontTexture cameraMoveInfo = "Move with WASD or Arrows - Shift to speed up";
	FontTexture currentMap = "none";
	FontTexture mapLoadError = "dummy error";
	FontTexture mapLoadFatal = "dummy fatal error";
	FontTexture quitInfo = "Press ESC to close program";

	j1Timer errorMessageTimer;
	bool showMapFatal = false;
};

#endif // __j1SCENE_H__