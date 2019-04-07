#include "p2Defs.h"
#include "p2Log.h"
#include "j1App.h"
#include "j1Textures.h"
#include "Fonts.h"

#include "SDL\include\SDL.h"
#include "SDL_TTF\include\SDL_ttf.h"
#pragma comment( lib, "SDL_ttf/libx86/SDL2_ttf.lib" )

Fonts::Fonts() : j1Module()
{
	name.append("fonts");
}

// Destructor
Fonts::~Fonts()
{}

// Called before render is available
bool Fonts::Awake(pugi::xml_node& conf)
{
	LOG("Init True Type Font library");
	bool ret = true;

	if (TTF_Init() == -1)
	{
		LOG("SDL_ttf could not initialize! SDL_ttf Error: %s\n", TTF_GetError());
		ret = false;
	}
	else
	{
		text_font* tmpPtr;
		int i = 0;

		for (pugi::xml_node fontList = conf.first_child(); fontList != fontList.last_child(); fontList = fontList.next_sibling()) {	// @Carles, automatically allocate fonts

			tmpPtr = new text_font;
			tmpPtr->id = (font_id)fontList.attribute("id").as_int((int)font_id::DEFAULT);
			tmpPtr->path = fontList.attribute("file").as_string(DEFAULT_FONT_PATH);
			tmpPtr->size = fontList.attribute("size").as_int(DEFAULT_FONT_SIZE);
			tmpPtr->fontPtr = Load(tmpPtr->path.c_str(), tmpPtr->size);
			fontsList[i] = tmpPtr;
			i++;
		}

		defaultFont = fontsList[(int)font_id::DEFAULT]->fontPtr;
	}

	return ret;
}

// Called before quitting
bool Fonts::CleanUp()
{
	LOG("Freeing True Type fonts and library");

	for (int i = 0; i < (int)font_id::MAX_FONTS; i++) {
		TTF_CloseFont(fontsList[i]->fontPtr);
		RELEASE(fontsList[i]);
	}
	TTF_Quit();

	return true;
}

// Load new texture from file path
TTF_Font* const Fonts::Load(const char* path, int size)
{
	TTF_Font* font = TTF_OpenFont(path, size);

	if (font == NULL)
	{
		LOG("Could not load TTF font with path: %s. TTF_OpenFont: %s", path, TTF_GetError());
	}
	else
	{
		LOG("Successfully loaded font %s size %d", path, size);
	}

	return font;
}

// Print text using font
SDL_Texture* Fonts::Print(const char* text, SDL_Color color, _TTF_Font* font)
{
	SDL_Texture* ret = NULL;
	SDL_Surface* surface = TTF_RenderText_Blended((font) ? font : defaultFont, text, color);

	if (surface == NULL)
	{
		LOG("Unable to render text surface! SDL_ttf Error: %s\n", TTF_GetError());
	}
	else
	{
		ret = App->tex->LoadSurface(surface);
		SDL_FreeSurface(surface);
	}

	return ret;
}

// calculate size of a text
bool Fonts::CalcSize(const char* text, int& width, int& height, _TTF_Font* font) const
{
	bool ret = false;

	if (TTF_SizeText((font) ? font : defaultFont, text, &width, &height) != 0)
		LOG("Unable to calc size of text surface! SDL_ttf Error: %s\n", TTF_GetError());
	else
		ret = true;

	return ret;
}