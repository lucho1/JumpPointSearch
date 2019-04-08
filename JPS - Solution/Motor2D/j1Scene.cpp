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
	bool ret = true;

	return ret;
}

// Called before the first frame
bool j1Scene::Start()
{

	if(App->map->Load("iso_walk.tmx") == true)
	{
		int w, h;
		uchar* data = NULL;
		if(App->map->CreateWalkabilityMap(w, h, &data))
			App->pathfinding->SetMap(w, h, data);

		RELEASE_ARRAY(data);
	}

	//Debug Texture to show Pahtfinding and mouse position
	debug_tex = App->tex->Load("maps/path2.png");

	//Performance Test Elements - Loading (Debug Purposes)
	algorithmUsed_text = App->fonts->Print(AlgorithmUsed);
	App->fonts->CalcSize(AlgorithmUsed, algorithmUsed_rect.w, algorithmUsed_rect.h);

	ms_charText = App->fonts->Print(ms_char);
	App->fonts->CalcSize(ms_char, ms_charRect.w, ms_charRect.h);

	number_msTexture = App->fonts->Print(number_ms);
	App->fonts->CalcSize(number_ms, number_ms_rect.w, number_ms_rect.h);

	return true;
}

// Called each loop iteration
bool j1Scene::PreUpdate()
{

	// Switch between A* and JPS
	if (App->input->GetKey(SDL_SCANCODE_F) == KEY_DOWN) {

		activateJPS = !activateJPS;
		App->tex->UnLoad(algorithmUsed_text);

		if (activateJPS == true)
			AlgorithmUsed = "Algorithm Used: JPS (press F to change)";
		else
			AlgorithmUsed = "Algorithm Used: A-Star (press F to change)";

		algorithmUsed_text = App->fonts->Print(AlgorithmUsed);
		App->fonts->CalcSize(AlgorithmUsed, algorithmUsed_rect.w, algorithmUsed_rect.h);
	}


	// debug pathfing ------------------
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
			LOG("Using Algorithm: %s", AlgorithmUsed);

			PathfindingTimer.Start();
			App->pathfinding->CreatePath(origin, p, activateJPS);
			Ptime = PathfindingTimer.ReadMs();
			origin_selected = false;

			//Pathfinding Performance Test - measures changes
			sprintf_s(number_ms, "%f", Ptime);

			App->tex->UnLoad(number_msTexture);
			number_msTexture = App->fonts->Print(number_ms);
			App->fonts->CalcSize(number_ms, number_ms_rect.w, number_ms_rect.h);

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

	if(App->input->GetKey(SDL_SCANCODE_UP) == KEY_REPEAT)
		App->render->camera.y += 5;

	if(App->input->GetKey(SDL_SCANCODE_DOWN) == KEY_REPEAT)
		App->render->camera.y -= 5;

	if(App->input->GetKey(SDL_SCANCODE_LEFT) == KEY_REPEAT)
		App->render->camera.x += 5;

	if(App->input->GetKey(SDL_SCANCODE_RIGHT) == KEY_REPEAT)
		App->render->camera.x -= 5;

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
	App->render->Blit(algorithmUsed_text, -App->render->camera.x, -App->render->camera.y, &algorithmUsed_rect);
	App->render->Blit(ms_charText, -App->render->camera.x, -App->render->camera.y + 20, &ms_charRect);
	App->render->Blit(number_msTexture, -App->render->camera.x + 150, -App->render->camera.y + 20, &number_ms_rect);


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
	App->tex->UnLoad(algorithmUsed_text);
	App->tex->UnLoad(ms_charText);
	App->tex->UnLoad(number_msTexture);

	return true;
}