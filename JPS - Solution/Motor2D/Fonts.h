#ifndef FONTS_H
#define FONTS_H

#include "j1Module.h"
#include "SDL\include\SDL_pixels.h"

#define DEFAULT_FONT_PATH "fonts/open_sans/OpenSans-Regular.ttf"
#define DEFAULT_FONT_SIZE 12
#define DEFAULT_FONT_COLOR { 255, 255, 255, 255 }

struct SDL_Texture;
struct _TTF_Font;

enum class font_id {

	DEFAULT,
	RUSSIAN,
	MOLOT,

	MAX_FONTS
};

struct text_font {

	font_id id;
	std::string path;
	int size;
	_TTF_Font* fontPtr;
};

class Fonts : public j1Module
{
public:

	Fonts();

	// Destructor
	virtual ~Fonts();

	// Called before render is available
	bool Awake(pugi::xml_node&);

	// Called before quitting
	bool CleanUp();

	// Load Font
	_TTF_Font* const Load(const char* path, int size);

	// Create a surface from text
	SDL_Texture* Print(const char* text, SDL_Color color = DEFAULT_FONT_COLOR, _TTF_Font* font = NULL);

	bool CalcSize(const char* text, int& width, int& height, _TTF_Font* font = NULL) const;

public:

	text_font* fontsList[(int)font_id::MAX_FONTS];
	_TTF_Font* defaultFont = nullptr;

};


#endif // FONTS_H