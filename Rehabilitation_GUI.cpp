#include "Rehabilitation_GUI.h"

void Rehabilitation_GUI::Update()
{
	while (quit == false) {
		MainMenu();
	}

}


/*
void Rehabilitation_GUI::Settings()
{
	bool settingscont = false;
	SDL_Window* settingswindow = SDL_CreateWindow("GUI", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 957, 538, SDL_WINDOW_SHOWN);
	SDL_Renderer* settingsrenderer = SDL_CreateRenderer(settingswindow, -1, 0);
	SDL_SetRenderDrawColor(settingsrenderer, 255, 0, 0, SDL_ALPHA_OPAQUE);
	SDL_RenderClear(settingsrenderer);
	SDL_RenderPresent(settingsrenderer);
	while (settingscont == false)
	{
		SDL_Event Buttonclick;
		SDL_WaitEvent(&Buttonclick);
		switch (Buttonclick.type)
		{
		case SDL_MOUSEBUTTONDOWN:
			settingscont = true;
			break;
		default:
			break;
		}
	}
	SDL_DestroyWindow(settingswindow);
}
*/
void Rehabilitation_GUI::MainMenu()
{

	// make a window
	SDL_Init(SDL_INIT_EVERYTHING);
	SDL_Window* window = SDL_CreateWindow("GUI", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 957, 538, SDL_WINDOW_SHOWN);
	SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);

	//init image loader functions
	const int FLAGS = IMG_INIT_PNG;

	//upload image file to memory as an sdl surface
	SDL_Surface* background = IMG_Load("armeobackground.png");
	//convert the image into something usefull
	SDL_Texture* armeobackground = SDL_CreateTextureFromSurface(renderer, background);

	SDL_Rect background_rect;
	background_rect.x = 0;
	background_rect.y = 0;
	background_rect.w = 957;
	background_rect.h = 538;

	SDL_RenderClear(renderer);

	//make the texture file into background
	SDL_RenderCopy(renderer, armeobackground, NULL, &background_rect);

	// create button for start
	SDL_Rect BUTTON_start;
	BUTTON_start.w = buttonw;
	BUTTON_start.h = buttonh;
	BUTTON_start.x = startplacementx;
	BUTTON_start.y = buttonplacementy;

	/*// create button for settings
	SDL_Rect BUTTON_settings;
	BUTTON_settings.w = buttonw;
	BUTTON_settings.h = buttonh;
	BUTTON_settings.x = settingsplacementx;
	BUTTON_settings.y = buttonplacementy;
*/
// create button for quitting
	SDL_Rect BUTTON_quit;
	BUTTON_quit.w = buttonw;
	BUTTON_quit.h = buttonh;
	BUTTON_quit.x = quitplacementx;
	BUTTON_quit.y = buttonplacementy;

	SDL_Surface* Start = IMG_Load("Start.png");
	//convert the image into something usefull
	SDL_Texture* StartTexture = SDL_CreateTextureFromSurface(renderer, Start);
	//make the texture file into background
	SDL_RenderCopy(renderer, StartTexture, NULL, &BUTTON_start);

	SDL_Surface* Quit = IMG_Load("Quit.png");
	//convert the image into something usefull
	SDL_Texture* QuitTexture = SDL_CreateTextureFromSurface(renderer, Quit);
	//make the texture file into background
	SDL_RenderCopy(renderer, QuitTexture, NULL, &BUTTON_quit);



	//present the rendered data
	SDL_RenderPresent(renderer);

	

	while (continue_func == false)
	{


		SDL_Event Buttonclick;
		SDL_WaitEvent(&Buttonclick);

		switch (Buttonclick.type)
		{
		case SDL_MOUSEBUTTONDOWN:
			SDL_GetMouseState(&mousecoordinatex, &mousecoordinatey);
			/*if (mousecoordinatex < settingsplacementx + buttonw && mousecoordinatex > settingsplacementx && mousecoordinatey < buttonplacementy + buttonw && mousecoordinatey > buttonplacementy)
			{
				SDL_DestroyWindow(window);
				Settings();
				continue_func = true;
			}*/

			if (mousecoordinatex < startplacementx + buttonw && mousecoordinatex > startplacementx && mousecoordinatey < buttonplacementy + buttonw && mousecoordinatey > buttonplacementy)
			{

				SDL_DestroyWindow(window);
				myo::Hub hub("com.example.hello-myo");
				std::cout << "Attempting to find a Msyo..." << std::endl;
				// Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
				// immediately.
				// waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
				// if that fails, the function will return a null pointer.
				myo::Myo* themyo = hub.waitForMyo(10000);
				// If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
				if (!themyo) {
					throw std::runtime_error("Unable to find a Myo!");
				}
				// We've found a Myo.
				std::cout << "Connected to a Myo armband!" << std::endl << std::endl;
				themyo->unlock(myo::Myo::unlockHold);
				// Next we enable EMG streaming on the found Myo.
				themyo->setStreamEmg(myo::Myo::streamEmgEnabled);

				// Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
				MyoController collector;
				// Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
				// Hub::run() to send events to all registered device listeners.
				hub.addListener(&collector);

				RehabilitationGame game;

				game.init("game", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 600, 600, collector);
				while (game.startGame) {
					hub.run(1);

					game.update(collector);

					game.render();
				}
				game.clean();

				continue_func = true;
			}

			if (mousecoordinatex < quitplacementx + buttonw && mousecoordinatex > quitplacementx && mousecoordinatey < buttonplacementy + buttonw && mousecoordinatey > buttonplacementy)
			{
				quit = true;
				continue_func = true;
			}

			break;
		default:
			break;
		}
	}
	continue_func = false;
}

void Rehabilitation_GUI::Clean()
{

}