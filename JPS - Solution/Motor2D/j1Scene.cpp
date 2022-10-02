#include "p2Defs.h"
#include "p2Log.h"
#include "j1App.h"
#include "j1Input.h"
#include "j1Textures.h"
#include "j1Render.h"
#include "j1Window.h"
#include "j1Map.h"
#include "j1PathFinding.h"
#include "Fonts.h"
#include "j1Scene.h"

j1Scene::j1Scene() : j1Module()
{
	name.assign("scene");
}

// Destructor
j1Scene::~j1Scene()
{}

// Called before render is available
bool j1Scene::Awake()
{
	LOG("Loading Scene");
	return true;
}

// Called before the first frame
bool j1Scene::Start()
{
	LoadMap(map1);

	//Debug Texture to show Pathfinding and mouse position
	debug_tex = App->tex->Load("maps/path2.png");

	//Performance Test Elements - Loading
	algorithmInfo.fontTexture = SetupFontText(algorithmInfo.fontText, &algorithmInfo.fontRect);
	timeInfo.fontTexture = SetupFontText(timeInfo.fontText, &timeInfo.fontRect);
	millisecondsInfo.fontTexture = SetupFontText(millisecondsInfo.fontText, &millisecondsInfo.fontRect);

	// Map Info Text
	cameraInfo.fontTexture = SetupFontText(cameraInfo.fontText, &cameraInfo.fontRect);
	cameraMoveInfo.fontTexture = SetupFontText(cameraMoveInfo.fontText, &cameraMoveInfo.fontRect);
	currentMap.fontTexture = SetupFontText(GetCurrentMapBlitText(), &currentMap.fontRect);
	mapLoadError.fontTexture = SetupFontText(mapLoadError.fontText, &mapLoadError.fontRect);
	mapLoadFatal.fontTexture = SetupFontText(mapLoadFatal.fontText, &mapLoadFatal.fontRect);
	quitInfo.fontTexture = SetupFontText(quitInfo.fontText, &quitInfo.fontRect);

	return true;
}

// Called each loop iteration
bool j1Scene::PreUpdate()
{
	// Reset Camera Position
	if (App->input->GetKey(SDL_SCANCODE_C) == KEY_DOWN)
		App->render->ResetCameraPosition();

	// Switch Maps
	if (App->input->GetKey(SDL_SCANCODE_M) == KEY_DOWN)
	{
		App->pathfinding->ClearLastPath();
		SwitchMap();
		App->tex->UnLoad(currentMap.fontTexture);
		currentMap.fontTexture = SetupFontText(GetCurrentMapBlitText(), &currentMap.fontRect);
	}

	// Switch between A* and JPS
	if (App->input->GetKey(SDL_SCANCODE_F) == KEY_DOWN)
	{
		activateJPS = !activateJPS;
		App->tex->UnLoad(algorithmInfo.fontTexture);

		if (activateJPS)
			algorithmInfo.fontText = "Algorithm Used: JPS (press F to change)";
		else
			algorithmInfo.fontText = "Algorithm Used: A-Star (press F to change)";

		algorithmInfo.fontTexture = SetupFontText(algorithmInfo.fontText, &algorithmInfo.fontRect);
	}


	// debug pathfinding ------------------
	static iPoint origin;
	static bool origin_selected = false;

	int x, y;
	App->input->GetMousePosition(x, y);
	iPoint p = App->render->ScreenToWorld(x, y);
	p = App->map->WorldToMap(p.x, p.y);
	
	if(App->input->GetMouseButtonDown(SDL_BUTTON_LEFT) == KEY_DOWN)
	{
		if(origin_selected == true)
		{
			LOG("========PATHFINDING PERFORMANCE TEST RESULTS=========");
			LOG("Using Algorithm: %s", algorithmInfo.fontText.c_str());

			PathfindingTimer.Start();
			App->pathfinding->CreatePath(origin, p, activateJPS);
			float Ptime = static_cast<float>(PathfindingTimer.ReadMs());
			origin_selected = false;

			//Pathfinding Performance Test - measures changes
			millisecondsInfo.fontText = std::to_string(Ptime);

			App->tex->UnLoad(millisecondsInfo.fontTexture);
			millisecondsInfo.fontTexture = SetupFontText(millisecondsInfo.fontText, &millisecondsInfo.fontRect);

			LOG("PATHFINDING LASTED: %f ms", Ptime);
		}
		else
		{
			origin = p;
			origin_selected = true;
		}
	}

	return true;
}

// Called each loop iteration
bool j1Scene::Update(float dt)
{
	float camSpeed = 3.0f;
	if (App->input->GetKey(SDL_SCANCODE_LSHIFT) == KEY_REPEAT || App->input->GetKey(SDL_SCANCODE_RSHIFT) == KEY_REPEAT)
		camSpeed *= 20.0f;

	if(App->input->GetKey(SDL_SCANCODE_UP) == KEY_REPEAT || App->input->GetKey(SDL_SCANCODE_W) == KEY_REPEAT)
		App->render->camera.y += (int)camSpeed;

	if(App->input->GetKey(SDL_SCANCODE_DOWN) == KEY_REPEAT || App->input->GetKey(SDL_SCANCODE_S) == KEY_REPEAT)
		App->render->camera.y -= (int)camSpeed;

	if(App->input->GetKey(SDL_SCANCODE_LEFT) == KEY_REPEAT || App->input->GetKey(SDL_SCANCODE_A) == KEY_REPEAT)
		App->render->camera.x += (int)camSpeed;

	if(App->input->GetKey(SDL_SCANCODE_RIGHT) == KEY_REPEAT || App->input->GetKey(SDL_SCANCODE_D) == KEY_REPEAT)
		App->render->camera.x -= (int)camSpeed;

	App->map->Draw();

	// Debug pathfinding ------------------------------
	int x, y;
	App->input->GetMousePosition(x, y);
	iPoint p = App->render->ScreenToWorld(x, y);
	p = App->map->WorldToMap(p.x, p.y);
	p = App->map->MapToWorld(p.x, p.y);

	App->render->Blit(debug_tex, p.x, p.y);

	const std::vector<iPoint>* path = App->pathfinding->GetLastPath();

	for(uint i = 0; i < path->size(); ++i)
	{
		iPoint pos = App->map->MapToWorld(path->at(i).x, path->at(i).y);
		App->render->Blit(debug_tex, pos.x, pos.y);
	}

	return true;
}

// Called each loop iteration
bool j1Scene::PostUpdate()
{
	bool ret = true;

	//Blitting elements used for performance test show
	App->render->Blit(algorithmInfo.fontTexture, -App->render->camera.x, -App->render->camera.y, &algorithmInfo.fontRect);
	App->render->Blit(timeInfo.fontTexture, -App->render->camera.x, -App->render->camera.y + 20, &timeInfo.fontRect);
	App->render->Blit(millisecondsInfo.fontTexture, -App->render->camera.x + 150, -App->render->camera.y + 20, &millisecondsInfo.fontRect);

	App->render->Blit(currentMap.fontTexture, -App->render->camera.x, -App->render->camera.y + 70, &currentMap.fontRect);

	if (errorMessageTimer.IsRunning() && errorMessageTimer.ReadSec() < 20.0f)
		App->render->Blit(mapLoadError.fontTexture, -App->render->camera.x + 400, -App->render->camera.y + 70, &mapLoadError.fontRect);
	else if (errorMessageTimer.IsRunning())
		errorMessageTimer.Stop();

	if(showMapFatal)
		App->render->Blit(mapLoadFatal.fontTexture, -App->render->camera.x + 400, -App->render->camera.y + 90, &mapLoadFatal.fontRect);

	App->render->Blit(cameraInfo.fontTexture, -App->render->camera.x, -App->render->camera.y + 90, &cameraInfo.fontRect);
	App->render->Blit(cameraMoveInfo.fontTexture, -App->render->camera.x, -App->render->camera.y + 110, &cameraMoveInfo.fontRect);
	App->render->Blit(quitInfo.fontTexture, -App->render->camera.x, -App->render->camera.y + 130, &quitInfo.fontRect);

	if(App->input->GetKey(SDL_SCANCODE_ESCAPE) == KEY_DOWN)
		ret = false;

	return ret;
}

// Called before quitting
bool j1Scene::CleanUp()
{
	LOG("Freeing scene");
	App->tex->UnLoad(debug_tex);

	//Unloading elements used for performance test show
	App->tex->UnLoad(algorithmInfo.fontTexture);
	App->tex->UnLoad(timeInfo.fontTexture);
	App->tex->UnLoad(millisecondsInfo.fontTexture);

	App->tex->UnLoad(cameraInfo.fontTexture);
	App->tex->UnLoad(cameraMoveInfo.fontTexture);
	App->tex->UnLoad(currentMap.fontTexture);
	App->tex->UnLoad(mapLoadError.fontTexture);
	App->tex->UnLoad(mapLoadFatal.fontTexture);
	App->tex->UnLoad(quitInfo.fontTexture);

	return true;
}


bool j1Scene::LoadMap(const char* mapFilepath)
{
	bool ret = false;
	if (ret = App->map->Load(mapFilepath))
		SetupMap(mapFilepath);
	
	return ret;
}

// Fonts
SDL_Texture* j1Scene::SetupFontText(const std::string& text, SDL_Rect* textRect)
{
	SDL_Texture* fontTexture = App->fonts->Print(text.c_str());
	App->fonts->CalcSize(text.c_str(), textRect->w, textRect->h);
	return fontTexture;
}

// Map functions
void j1Scene::SetupMap(const char* mapFilepath)
{
	int w, h;
	uchar* data = NULL;
	if (App->map->CreateWalkabilityMap(w, h, &data))
		App->pathfinding->SetMap(w, h, data);

	RELEASE_ARRAY(data);
	currentMap.fontText = mapFilepath;
	App->render->ResetCameraPosition();
}

std::string j1Scene::GetCurrentMapBlitText()
{
	return "Current Map (Press M to Switch): " + currentMap.fontText;
}

void j1Scene::SwitchMap()
{
	std::string mapToLoad = map1;
	if (currentMap.fontText == map1)
		mapToLoad = map2;

	if (mapToLoad == currentMap.fontText)
		return;

	if (!App->map->SwitchMap(mapToLoad.c_str()))
	{
		LOG("There was a problem switching to map: '%s'", mapToLoad.c_str());
		LOG("Please check the map file or check 'output/logs.txt'");
		LOG("Reloading original map...");

		errorMessageTimer.Start();
		mapLoadError.fontText = "Error Switching Map, check the map file and 'output/logs.txt' file - Reloading original map...";
		mapLoadError.fontTexture = SetupFontText(mapLoadError.fontText, &mapLoadError.fontRect);

		if (!LoadMap(map1))
		{
			LOG("Failed to reload original map ('%s'), please restart and check 'output/logs.txt'", map1);
			mapLoadFatal.fontText = "Failed to reload original map ('" + std::string(map1) + "'), please restart and check 'output/logs.txt' file";
			mapLoadFatal.fontTexture = SetupFontText(mapLoadFatal.fontText, &mapLoadFatal.fontRect);
			showMapFatal = true;
		}
	}
	else
		SetupMap(mapToLoad.c_str());
}
