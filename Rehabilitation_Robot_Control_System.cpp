#define SDL_MAIN_HANDLED
#include "Rehabilitation_GUI.h"

#include <myo/myo.hpp>


int main()
{
	SDL_SetMainReady();
	Rehabilitation_GUI GUI;
	GUI.Update();
	return 0;
}
