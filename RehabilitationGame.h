#pragma once
#include <SDL.h>
#include <armadillo>
#include <SDL_image.h>
#include <myo/myo.hpp>
#include "MyoController.h"

class RehabilitationGame
{
public:
	MyoController collector;
	bool  startGame = false;
	struct GameObject {
		std::string name;
		SDL_Texture* Sprite;
		SDL_Point Position;
		SDL_Point Size;
		SDL_Rect SrcR;
		SDL_Rect DestR;

		int xdir = 0;
		int ydir = 0;
	};

	void init(const char* title, int posx, int posy, int width, int height, MyoController &collector);
	void update(MyoController& collector);
	void render();
	void initGameObject(GameObject& gameObject,int x, int y, int width, int height, const char* spriteLocation, std::string name);
	void UpdateGameObject(GameObject& gameObject);
	void RenderGameObject(GameObject& gameObject);
	void clean();
	SDL_Renderer* renderer;

	
private:
	int buttonw = 100;
	int buttonh = 30;
	int mousecoordinatex;
	int mousecoordinatey;
	int buttonplacemetx = 120;
	int buttonplacementy = 120;
	//bool  startGame = false;
	bool  gameIsInitialized = false;
    SDL_Window* window;
	
	SDL_Texture* background;


};

