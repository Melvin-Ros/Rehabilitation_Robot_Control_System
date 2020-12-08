#include "RehabilitationGame.h"
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <vector>
#include "CrustCrawlerKinematics.h"
#include "CrustCrawlerDynamics.h"
#include "SimpleSerial.h"
#include <string>
#include <sstream>
std::vector<RehabilitationGame::GameObject> asteroids;
RehabilitationGame::GameObject player;
CrustCrawlerKinematics CCK;
CrustCrawlerDynamics Dynamics;
CrustCrawlerKinematics::Angles TargetAngles;
int sampleCounter = 0;
int Scalarx = 3;
int Scalary= 5;
float currentPosX = 0;
float currentPosY = 147;
float currentPosZ = 210;
float theta[4];
float dtheta[4];
float ddtheta[4];
SDL_Event event;
//SimpleSerial serial("COM5", 115);
int counter = 0;
void RehabilitationGame::init(const char* title, int posx, int posy, int width, int height, MyoController& collector)
{
	GameSettings();

	CrustCrawlerKinematics::Pos tempPos;
	tempPos = CCK.ForwardKinematics(0, 40, 100, 0);
	currentPosX = tempPos.x;
	currentPosZ = tempPos.z;
	currentPosY = tempPos.y;
	TargetAngles = CCK.InverseKinematics(currentPosX, currentPosY, currentPosZ);
	while (Dynamics.anglevelocity[3] >= 0.5) {
		
		theta[0] = TargetAngles.theta1 * 3.14 / 180;
		theta[1] = TargetAngles.theta2 * 3.14 / 180;
		theta[2] = TargetAngles.theta3 * 3.14 / 180;
		theta[3] = TargetAngles.theta4 * 3.14 / 180;
		std::cout << "theta 3 before" << theta[2] << " ";
		CrustCrawlerKinematics::Trajectory trajectory = CCK.TrajectoryGeneration(theta, Dynamics.angle, Dynamics.anglevelocity);
		
		theta[0] = trajectory.theta[0];
		theta[1] = trajectory.theta[1] ;
		theta[2] = trajectory.theta[2] ;
		theta[3] = trajectory.theta[3] ;
		std:: cout << "theta 3 after" << theta[2] << " " << std::endl;
		dtheta[0] = trajectory.dtheta[0];
		dtheta[1] = trajectory.dtheta[1];
		dtheta[2] = trajectory.dtheta[2];
		dtheta[3] = trajectory.dtheta[3];
		
		ddtheta[0] = trajectory.ddtheta[0];
		ddtheta[1] = trajectory.ddtheta[1];
		ddtheta[2] = trajectory.ddtheta[2];
		ddtheta[3] = trajectory.ddtheta[3];
		Dynamics.control(theta, dtheta, ddtheta);
	}
	
	/*
	std::string message = "1 " + std::to_string(int(TargetAngles.theta1)) + ":" +
		"2 " + std::to_string(int(TargetAngles.theta2)) + ":" +
		"3 " + std::to_string(int(TargetAngles.theta3)) + ":" +
		"4 " + std::to_string(int(TargetAngles.theta4)) + ":";
	std::cout << message << std::endl;
	serial.writeString(message);
	*/
	SDL_Init(SDL_INIT_EVERYTHING);
	const int FlAGS = IMG_INIT_PNG;
	IMG_Init(IMG_INIT_PNG);
	window = SDL_CreateWindow(title, posx, posy, width, height, SDL_WINDOW_SHOWN);
	renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC);
	SDL_Surface* temp;
	temp = IMG_Load("Sea.png");

	background = SDL_CreateTextureFromSurface(renderer, temp);

	for (int i = 0; i <= NumberOfPoints; i++)
	{
		GameObject asteroid;
		asteroids.push_back(asteroid);
	}
	std::cout << asteroids.size() << std::endl;
	
	startGame = true;
	SDL_Delay(100);
}

void RehabilitationGame::update(MyoController& collector)
{

	if (startGame) {
		BUTTON_quit.w = buttonw;
		BUTTON_quit.h = buttonh;
		BUTTON_quit.x = quitplacementx;
		BUTTON_quit.y = buttonplacementy;
		BUTTON_quit.y = buttonplacementy;
		

		if (!gameIsInitialized) {
			initGameObject(player, 250, 200, 100, 100, "Boat.png", "Player");
		
			//make the texture file into background
			
			int MousePosX, MousePosY;
			while (counter < NumberOfPoints) {
				if (counter == 0) {
					SDL_RenderCopy(renderer, background, NULL, NULL);
					SDL_RenderPresent(renderer);
				}
			
				
				SDL_Event Buttonclick;
				SDL_WaitEvent(&Buttonclick);
				switch (Buttonclick.type)
				{
				case SDL_MOUSEBUTTONDOWN:
					SDL_GetMouseState(&MousePosX, &MousePosY);
					initGameObject(asteroids[counter+1], MousePosX, MousePosY, 100, 100, "Ice.png", "Asteroid");
					SDL_RenderCopy(renderer, background, NULL, NULL);
					RenderGameObject(asteroids[counter + 1]);
					SDL_RenderPresent(renderer);
					counter++;
					break;
				default:
					break;

				}
			}
			initGameObject(asteroids[0], MousePosX, MousePosY, 100, 100, "Ice.png", "Asteroid");
			asteroids[0] = asteroids[1];
			counter = 0;
			gameIsInitialized = true;

		}
		else {
			/*
			int MousePosX, MousePosY;
			SDL_Event Buttonclick;
			SDL_WaitEvent(&Buttonclick);
			switch (Buttonclick.type)
			{
			case SDL_MOUSEBUTTONDOWN:
				SDL_GetMouseState(&MousePosX, &MousePosY);
			
				if (MousePosX < quitplacementx + buttonw && MousePosX > quitplacementx && MousePosY < buttonplacementy + buttonw && MousePosY > buttonplacementy)
				{
					startGame = false;
				}
				//counter++;
				break;
			default:
				break;

			}
			*/

				UpdateGameObject(asteroids[0]);
				//srand(time(NULL) + i);
				if ((player.DestR.x - asteroids[0].DestR.x) * (player.DestR.x - asteroids[0].DestR.x) + (player.DestR.y - asteroids[0].DestR.y) * (player.DestR.y - asteroids[0].DestR.y) < 100 * 100) {
					counter++;
					if (counter <= NumberOfPoints) {
						asteroids[0] = asteroids[counter];
					}
					else {
						asteroids[0] = asteroids[1];
						counter = 1;
					}
					
				}
			
				
			if (collector.Direction == MyoController::Left) {
				//std::cout << "Left" << std::endl;
				player.xdir = -1;
				player.ydir = 0;

				//if (targetX * targetX + targetY * targetY + targetZ * targetZ <= 370 * 370) {
				currentPosX -= player.xdir*Scalarx;
				currentPosZ -= player.ydir * Scalary;
				sampleCounter++;

			}
			else if (collector.Direction == MyoController::Right) {
				//std::cout << "Right" << std::endl;
				player.xdir = 1;
				player.ydir = 0;
				currentPosX -= player.xdir * Scalarx;
				currentPosZ -= player.ydir * Scalary;
				sampleCounter++;
			}
			else if (collector.Direction == MyoController::Down) {
			//	std::cout << "Down" << std::endl;
				player.xdir = 0;
				player.ydir = 1;
				currentPosX -= player.xdir * Scalarx;
				currentPosZ -= player.ydir * Scalary;
				sampleCounter++;

			}
			else if (collector.Direction == MyoController::Up) {
				//std::cout << "Up" << std::endl;
				player.xdir = 0;
				player.ydir = -1;
				currentPosX -= player.xdir * Scalarx;
				currentPosZ -= player.ydir * Scalary;
				sampleCounter++;
			}
			else {
				player.xdir = 0;
				player.ydir = 0;
			}
			TargetAngles = CCK.InverseKinematics(currentPosX, currentPosY, currentPosZ);


			theta[0] = TargetAngles.theta1 * 3.14 / 180;
			theta[1] = TargetAngles.theta2 * 3.14 / 180;
			theta[2] = TargetAngles.theta3 * 3.14 / 180;
			theta[3] = TargetAngles.theta4 * 3.14 / 180;
			//std::cout << "theta 3 before" << theta[2] << " ";
			CrustCrawlerKinematics::Trajectory trajectory = CCK.TrajectoryGeneration(theta, Dynamics.angle, Dynamics.anglevelocity);

			theta[0] = trajectory.theta[0];
			theta[1] = trajectory.theta[1];
			theta[2] = trajectory.theta[2];
			theta[3] = trajectory.theta[3];
			//std::cout << "theta 3 after" << theta[2] << " " << std::endl;
			dtheta[0] = trajectory.dtheta[0];
			dtheta[1] = trajectory.dtheta[1];
			dtheta[2] = trajectory.dtheta[2];
			dtheta[3] = trajectory.dtheta[3];

			ddtheta[0] = trajectory.ddtheta[0];
			ddtheta[1] = trajectory.ddtheta[1];
			ddtheta[2] = trajectory.ddtheta[2];
			ddtheta[3] = trajectory.ddtheta[3];
			Dynamics.control(theta, dtheta, ddtheta);
			CrustCrawlerKinematics::Pos pos;
			pos = CCK.ForwardKinematics(Dynamics.angle[0] * 180 / 3.14, Dynamics.angle[1] * 180 / 3.14, Dynamics.angle[2] * 180 / 3.14, Dynamics.angle[3] * 180 / 3.14);
			std::cout << "x " << pos.x << " z " << pos.z << std::endl;
			UpdateGameObject(player);
		}

	}
	else {
		clean();
	}

}

void RehabilitationGame::render()
{
	SDL_RenderClear(renderer);
	SDL_RenderCopy(renderer, background, NULL, NULL);
	SDL_Surface* Quit = IMG_Load("Quit.png");
	//convert the image into something usefull
	SDL_Texture* QuitTexture = SDL_CreateTextureFromSurface(renderer, Quit);
	SDL_RenderCopy(renderer, QuitTexture, NULL, &BUTTON_quit);
	if (gameIsInitialized) {
		RenderGameObject(player);
			RenderGameObject(asteroids[0]);
		
	}


	SDL_RenderPresent(renderer);
}

void RehabilitationGame::initGameObject(GameObject& gameObject, int x, int y, int width, int height, const char* spriteLocation, std::string name)
{
	gameObject.name = name;
	gameObject.Position.x = x;
	gameObject.Position.y = y;
	gameObject.Size.x = width;
	gameObject.Size.y = height;
	gameObject.DestR.x = gameObject.Position.x;
	gameObject.DestR.y = gameObject.Position.y;
	gameObject.DestR.w = gameObject.Size.x;
	gameObject.DestR.h = gameObject.Size.y;
	SDL_Surface* temp = IMG_Load(spriteLocation);
	gameObject.Sprite = SDL_CreateTextureFromSurface(renderer, temp);
}

void RehabilitationGame::UpdateGameObject(GameObject& gameObject)
{


	gameObject.DestR.x += gameObject.xdir * 5;
	gameObject.DestR.y += gameObject.ydir * 5;
	gameObject.DestR.w = gameObject.Size.x;
	gameObject.DestR.h = gameObject.Size.y;


}

void RehabilitationGame::RenderGameObject(GameObject& gameObject)
{
	SDL_RenderCopy(renderer, gameObject.Sprite, NULL, &gameObject.DestR);
}

void RehabilitationGame::clean()
{
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);

}

void RehabilitationGame::GameSettings()
{
	SDL_Window* settingswindow = SDL_CreateWindow("Game Settings", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 500, 360, SDL_WINDOW_SHOWN);
	SDL_Renderer* settingsrenderer = SDL_CreateRenderer(settingswindow, -1, 0);
	//sdl surface img_load
	// sdl texture
	// render

	//SDL_SetRenderDrawColor(settingsrenderer, 0, 150, 255, SDL_ALPHA_OPAQUE);
	SDL_Surface* background = IMG_Load("GUI.png");
	//convert the image into something usefull
	SDL_Texture* armeobackground = SDL_CreateTextureFromSurface(renderer, background);

	SDL_Rect background_rect;
	background_rect.x = 0;
	background_rect.y = 0;
	background_rect.w = 500;
	background_rect.h = 360;

//	SDL_RenderClear(settingsrenderer);
	
	//SDL_RenderPresent(settingsrenderer);
	SDL_Rect GameSettingsHeight;
	GameSettingsHeight.x = 84;
	GameSettingsHeight.y = 45;
	GameSettingsHeight.w = 181;
	GameSettingsHeight.h = 50;
	SDL_Rect GameSettingsDist;
	GameSettingsDist.x = 84;
	GameSettingsDist.y = 107;
	GameSettingsDist.w = 181;
	GameSettingsDist.h = 50;
	SDL_Rect GameSettingsPoints;
	GameSettingsPoints.x = 84;
	GameSettingsPoints.y = 169;
	GameSettingsPoints.w = 181;
	GameSettingsPoints.h = 50;
	SDL_Rect GameSettingsDone;
	GameSettingsDone.x = 145;
	GameSettingsDone.y = 270;
	GameSettingsDone.w = 208;
	GameSettingsDone.h = 40;

	SDL_Rect Shoulder[3];
	SDL_Rect Point[3];
	SDL_Rect Dis[3];
	SDL_Rect ShoulderText[3];
	SDL_Rect PointText[3];
	SDL_Rect DisText[3];

	
	SDL_Surface* Distance = IMG_Load("Distance.png");
	//convert the image into something usefull
	SDL_Texture* DistanceTexture = SDL_CreateTextureFromSurface(settingsrenderer, Distance);

	SDL_Surface* Shoulders = IMG_Load("Shoulder.png");
	//convert the image into something usefull
	SDL_Texture* ShoulderTexture = SDL_CreateTextureFromSurface(settingsrenderer, Shoulders);

	SDL_Surface* Number = IMG_Load("Number.png");
	//convert the image into something usefull
	SDL_Texture* NumberTexture = SDL_CreateTextureFromSurface(settingsrenderer, Number);

	SDL_Surface* Done = IMG_Load("Start.png");
	//convert the image into something usefull
	SDL_Texture* DoneTexture = SDL_CreateTextureFromSurface(settingsrenderer, Done);

	SDL_RenderCopy(settingsrenderer, armeobackground, NULL, &background_rect);
	SDL_RenderCopy(settingsrenderer, DistanceTexture, NULL, &GameSettingsDist);
	SDL_RenderCopy(settingsrenderer, ShoulderTexture, NULL, &GameSettingsHeight);
	SDL_RenderCopy(settingsrenderer, NumberTexture, NULL, &GameSettingsPoints);
	SDL_RenderCopy(settingsrenderer, DoneTexture, NULL, &GameSettingsDone);
	
	
	TTF_Init();
	TTF_Font* TextFont = TTF_OpenFont("Roboto-Black.ttf", 16);
	SDL_Color black = { 0, 0, 0 };
	
	SDL_SetRenderDrawColor(settingsrenderer, 192, 192, 192, SDL_ALPHA_OPAQUE);

	for (int i = 0; i < 3; i++)
	{
		Shoulder[i].x = 300 + 50 * i;
		Shoulder[i].y = 60;
		Shoulder[i].w = 10;
		Shoulder[i].h = 10;

		Dis[i].x = 300 + 50 * i;
		Dis[i].y = 118;
		Dis[i].w = 10;
		Dis[i].h = 10;

		Point[i].x = 300 + 50 * i;
		Point[i].y = 184;
		Point[i].w = 10;
		Point[i].h = 10;
		
		ShoulderText[i].x = 300 + 50 * i;
		ShoulderText[i].y = 45;
		ShoulderText[i].w = 10;
		ShoulderText[i].h = 10;

		DisText[i].x = 300 + 50 * i;
		DisText[i].y = 105;
		DisText[i].w = 10;
		DisText[i].h = 10;

		PointText[i].x = 300 + 50 * i;
		PointText[i].y = 170;
		PointText[i].w = 10;
		PointText[i].h = 10;
		
		SDL_SetRenderDrawColor(settingsrenderer, 192, 192, 192, SDL_ALPHA_OPAQUE);

		SDL_RenderFillRect(settingsrenderer, &Shoulder[i]);
		SDL_RenderFillRect(settingsrenderer, &Dis[i]);
		SDL_RenderFillRect(settingsrenderer, &Point[i]);
		

	}

	for (int i = 0; i < 3; i++)
	{
		SDL_Surface* Text = TTF_RenderText_Solid(TextFont, "1", black);
		SDL_Texture* Texture = SDL_CreateTextureFromSurface(settingsrenderer, Text);
		switch (i)
		{
		case 0:
			Text = TTF_RenderText_Solid(TextFont, "10", black);
			Texture = SDL_CreateTextureFromSurface(settingsrenderer, Text);
			SDL_RenderCopy(settingsrenderer, Texture, NULL, &ShoulderText[i]);

			Text = TTF_RenderText_Solid(TextFont, "40", black);
			Texture = SDL_CreateTextureFromSurface(settingsrenderer, Text);
			SDL_RenderCopy(settingsrenderer, Texture, NULL, &DisText[i]);

			Text = TTF_RenderText_Solid(TextFont, "2", black);
			Texture = SDL_CreateTextureFromSurface(settingsrenderer, Text);
			SDL_RenderCopy(settingsrenderer, Texture, NULL, &PointText[i]);
			
			break;

		case 1:
			Text = TTF_RenderText_Solid(TextFont, "20", black);
			Texture = SDL_CreateTextureFromSurface(settingsrenderer, Text);
			SDL_RenderCopy(settingsrenderer, Texture, NULL, &ShoulderText[i]);

			Text = TTF_RenderText_Solid(TextFont, "50", black);
			Texture = SDL_CreateTextureFromSurface(settingsrenderer, Text);
			SDL_RenderCopy(settingsrenderer, Texture, NULL, &DisText[i]);

			Text = TTF_RenderText_Solid(TextFont, "3", black);
			Texture = SDL_CreateTextureFromSurface(settingsrenderer, Text);
			SDL_RenderCopy(settingsrenderer, Texture, NULL, &PointText[i]);
			break;

		case 2:
			Text = TTF_RenderText_Solid(TextFont, "30", black);
			Texture = SDL_CreateTextureFromSurface(settingsrenderer, Text);
			SDL_RenderCopy(settingsrenderer, Texture, NULL, &ShoulderText[i]);

			Text = TTF_RenderText_Solid(TextFont, "60", black);
			Texture = SDL_CreateTextureFromSurface(settingsrenderer, Text);
			SDL_RenderCopy(settingsrenderer, Texture, NULL, &DisText[i]);

			Text = TTF_RenderText_Solid(TextFont, "4", black);
			Texture = SDL_CreateTextureFromSurface(settingsrenderer, Text);
			SDL_RenderCopy(settingsrenderer, Texture, NULL, &PointText[i]);
			break;
		default:
			break;
		}

	}
	SDL_RenderPresent(settingsrenderer);
	int MousePosX, MousePosY;
	bool endsettings = false;
	while (endsettings == false)
	{
		SDL_Event Buttonclick;
		SDL_WaitEvent(&Buttonclick);
		switch (Buttonclick.type)
		{
		case SDL_MOUSEBUTTONDOWN:
			SDL_GetMouseState(&MousePosX, &MousePosY);
			if (MousePosX > GameSettingsDone.x && MousePosX < GameSettingsDone.w + GameSettingsDone.x && MousePosY > GameSettingsDone.y && MousePosY < GameSettingsDone.h + GameSettingsDone.y)
			{
				endsettings = true;
			}
			else {
				for (int i = 0; i < 4; i++)
				{
					SDL_SetRenderDrawColor(settingsrenderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
					if (MousePosX > Shoulder[i].x && MousePosX < Shoulder[i].w + Shoulder[i].x && MousePosY > Shoulder[i].y && MousePosY < Shoulder[i].h + Shoulder[i].y)
					{
						SDL_RenderFillRect(settingsrenderer, &Shoulder[i]);
						switch (i)
						{
						case 0:
							CCK.ShoulderHeightFromBase = 0;
							break;
						case 1:
							CCK.ShoulderHeightFromBase = 100;
							break;
						case 2:
							CCK.ShoulderHeightFromBase = 200;
							break;
						default:
							break;
						}
					}
					else if (MousePosX > Dis[i].x && MousePosX < Dis[i].w + Dis[i].x && MousePosY > Dis[i].y && MousePosY < Dis[i].h + Dis[i].y)
					{
						SDL_RenderFillRect(settingsrenderer, &Dis[i]);
						switch (i)
						{

						case 0:
							CCK.ShoulderDistanceFromBase = 400;
							break;
						case 1:
							CCK.ShoulderDistanceFromBase = 500;
							break;
						case 2:
							CCK.ShoulderDistanceFromBase = 600;
							break;
						default:
							break;
						}
					}
					else if (MousePosX > Point[i].x && MousePosX < Point[i].w + Point[i].x && MousePosY > Point[i].y && MousePosY < Point[i].h + Point[i].y)
					{
						SDL_RenderFillRect(settingsrenderer, &Point[i]);

						NumberOfPoints = i+2;
					}
				}
				SDL_RenderPresent(settingsrenderer);

			}
			break;
		default:
			break;
		}
	}
	SDL_DestroyWindow(settingswindow);
}
