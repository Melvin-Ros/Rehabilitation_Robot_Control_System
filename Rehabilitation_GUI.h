#pragma once
#include <SDL.h>
#include <SDL_image.h>
#include <SDL_ttf.h>
#include <myo/myo.hpp>
#include "RehabilitationGame.h"
class Rehabilitation_GUI
{
public:
	void Update(myo::Hub& hub, MyoController& collector);
	void Settings();
	void MainMenu(myo::Hub &hub, MyoController& collector);
	void Clean();
private:
	//variables to use for the robot
	float armlength = 63;
	int functional_level = 2;
	bool quit = false;
	int mousecoordinatex;
	int mousecoordinatey;
	int settingsplacementx = 530;
	int startplacementx = 400;
	int quitplacementx = 660;
	int buttonplacementy = 120;
	int buttonw = 100;
	int buttonh = 30;
	bool continue_func = false;

};

