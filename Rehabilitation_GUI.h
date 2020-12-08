#pragma once
#include <SDL.h>
#include <SDL_image.h>
#include <SDL_ttf.h>
#include <myo/myo.hpp>
#include "RehabilitationGame.h"
class Rehabilitation_GUI
{
public:
	void Update();
	//void Settings();
	void MainMenu();
	void Clean();
private:
	//variables to use for the robot
	float armlength = 63;
	int functional_level = 2;
	bool quit = false;
	int mousecoordinatex;
	int mousecoordinatey;
	int startplacementx = 50;
	int quitplacementx = 200;
	int buttonplacementy = 300;
	int buttonw = 120;
	int buttonh = 60;
	bool continue_func = false;

};