#pragma once
#include <SDL.h>
#include <armadillo>
#include <SDL_image.h>
#include <myo/myo.hpp>
#include <SDL_ttf.h>
#include "MyoController.h"

class RehabilitationGame
{
public:
	MyoController collector;
	bool startGame = false;
	int NumberOfPoints = 1;
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
	int quitplacementx = 500;
	int buttonplacementy = 20;
	int buttonw = 100;
	int buttonh = 30;
	SDL_Rect BUTTON_quit;
	
	void init(const char* title, int posx, int posy, int width, int height, MyoController& collector);
	void update(MyoController& collector);
	void render();
	void initGameObject(GameObject& gameObject, int x, int y, int width, int height, const char* spriteLocation, std::string name);
	void UpdateGameObject(GameObject& gameObject);
	void RenderGameObject(GameObject& gameObject);
	void clean();
	SDL_Renderer* renderer;

	struct Button {
		int x;
		int y;
		int w = 10;
		int h = 10;
	};



	void GameSettings();
private:
	//bool  startGame = false;
	bool  gameIsInitialized = false;
	SDL_Window* window;
	SDL_Texture* background;


};