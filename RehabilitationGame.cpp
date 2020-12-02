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
float currentPosX = 0;
float currentPosY = 147;
float currentPosZ = 210;
SDL_Event event;
//SimpleSerial serial("COM5", 115);
int counter = 0;
void RehabilitationGame::init(const char* title, int posx, int posy, int width, int height, MyoController& collector)
{
	GameSettings();

	CrustCrawlerKinematics::Pos tempPos;
	tempPos = CCK.ForwardKinematics(0, 0, 90, 0);
	currentPosX = tempPos.x;
	currentPosZ = tempPos.z;
	currentPosY = tempPos.y;
	TargetAngles = CCK.InverseKinematics(currentPosX, currentPosY, currentPosZ);
	
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

	for (int i = 0; i < NumberOfPoints; i++)
	{
		GameObject asteroid;
		asteroids.push_back(asteroid);
	}
	std::cout << asteroids.size() << std::endl;
	startGame = true;
	SDL_Delay(1000);
}

void RehabilitationGame::update(MyoController& collector)
{

	if (startGame) {

		if (!gameIsInitialized) {
			initGameObject(player, 400, 300, 50, 50, "Boat.png", "Player");
			for (int i = 0; i < asteroids.size(); i++)
			{
				srand(time(NULL) + i * i);
				initGameObject(asteroids[i], rand() % 450 + 150, rand() % 450 + 150, 50, 50, "Ice.png", "Asteroid");
				SDL_Delay(100);
				std::cout << "init Asteroid: " << i << std::endl;
			}
			gameIsInitialized = true;

		}
		else {

			for (int i = 0; i < asteroids.size(); i++) {
				UpdateGameObject(asteroids[i]);
				srand(time(NULL) + i);
				if ((player.DestR.x - asteroids[i].DestR.x) * (player.DestR.x - asteroids[i].DestR.x) + (player.DestR.y - asteroids[i].DestR.y) * (player.DestR.y - asteroids[i].DestR.y) < 50 * 50) {
					asteroids[i].DestR.x = rand() % 450 + 150;
					SDL_Delay(100);
					asteroids[i].DestR.y = rand() % 450 + 150, 50, 50;
				}
			}

			if (collector.Direction == MyoController::Left) {
				player.xdir = -1;
				player.ydir = 0;
				float targetX, targetY, targetZ;
				targetX = currentPosX + player.xdir;
				targetY = currentPosY;
				targetZ = currentPosZ + player.ydir;


				//if (targetX * targetX + targetY * targetY + targetZ * targetZ <= 370 * 370) {
				currentPosX -= player.xdir;
				currentPosZ -= player.ydir;
				//}



			}
			else if (collector.Direction == MyoController::Right) {
				player.xdir = 1;
				player.ydir = 0;
				float targetX, targetY, targetZ;
				targetX = currentPosX + player.xdir;
				targetY = currentPosY;
				targetZ = currentPosZ + player.ydir;
				//if (targetX * targetX + targetY * targetY + targetZ * targetZ <= 370 * 370) {
				currentPosX -= player.xdir;
				currentPosZ -= player.ydir;
				//}

			}
			else if (collector.Direction == MyoController::Down) {
				player.xdir = 0;
				player.ydir = 1;
				float targetX, targetY, targetZ;
				targetX = currentPosX + player.xdir;
				targetY = currentPosY;
				targetZ = currentPosZ + player.ydir;
				//if (targetX * targetX + targetY * targetY + targetZ * targetZ <= 370 * 370) {
				currentPosX -= player.xdir;
				currentPosZ -= player.ydir;
				//}

			}
			else if (collector.Direction == MyoController::Up) {
				player.xdir = 0;
				player.ydir = -1;
				float targetX, targetY, targetZ;
				targetX = currentPosX + player.xdir;
				targetY = currentPosY;
				targetZ = currentPosZ + player.ydir;
				//if (targetX * targetX + targetY * targetY + targetZ * targetZ <= 360 * 360) {
				currentPosX -= player.xdir;
				currentPosZ -= player.ydir;

			}
			else {
				player.xdir = 0;
				player.ydir = 0;
			}
			TargetAngles = CCK.InverseKinematics(currentPosX, currentPosY, currentPosZ);
			CrustCrawlerDynamics::Angles theta;
			theta.theta1 = TargetAngles.theta1;
			theta.theta2 = TargetAngles.theta2;
			theta.theta3 = TargetAngles.theta3;
			theta.theta4 = TargetAngles.theta4;
			
			Dynamics.control(theta);
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
	if (gameIsInitialized) {
		RenderGameObject(player);
		for (int i = 0; i < asteroids.size(); i++) {
			RenderGameObject(asteroids[i]);
		}
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


	gameObject.DestR.x += gameObject.xdir * 2;
	gameObject.DestR.y += gameObject.ydir * 2;
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

	SDL_SetRenderDrawColor(settingsrenderer, 0, 150, 255, SDL_ALPHA_OPAQUE);
	SDL_RenderClear(settingsrenderer);
	SDL_RenderPresent(settingsrenderer);
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
		SDL_RenderPresent(settingsrenderer);

	}

	for (int i = 0; i < 3; i++)
	{
		SDL_Surface* Text = TTF_RenderText_Solid(TextFont, "2", black);
		SDL_Texture* Texture = SDL_CreateTextureFromSurface(settingsrenderer, Text);
		switch (i)
		{
		case 0:
			Text = TTF_RenderText_Solid(TextFont, "2", black);
			Texture = SDL_CreateTextureFromSurface(settingsrenderer, Text);
			SDL_RenderCopy(settingsrenderer, Texture, NULL, &ShoulderText[i]);

			Text = TTF_RenderText_Solid(TextFont, "2", black);
			Texture = SDL_CreateTextureFromSurface(settingsrenderer, Text);
			SDL_RenderCopy(settingsrenderer, Texture, NULL, &DisText[i]);

			Text = TTF_RenderText_Solid(TextFont, "2", black);
			Texture = SDL_CreateTextureFromSurface(settingsrenderer, Text);
			SDL_RenderCopy(settingsrenderer, Texture, NULL, &PointText[i]);
			NumberOfPoints = 2;
			break;

		case 1:
			Text = TTF_RenderText_Solid(TextFont, "4", black);
			Texture = SDL_CreateTextureFromSurface(settingsrenderer, Text);
			SDL_RenderCopy(settingsrenderer, Texture, NULL, &ShoulderText[i]);

			Text = TTF_RenderText_Solid(TextFont, "4", black);
			Texture = SDL_CreateTextureFromSurface(settingsrenderer, Text);
			SDL_RenderCopy(settingsrenderer, Texture, NULL, &DisText[i]);

			Text = TTF_RenderText_Solid(TextFont, "4", black);
			Texture = SDL_CreateTextureFromSurface(settingsrenderer, Text);
			SDL_RenderCopy(settingsrenderer, Texture, NULL, &PointText[i]);
			NumberOfPoints = 4;
			break;

		case 2:
			Text = TTF_RenderText_Solid(TextFont, "6", black);
			Texture = SDL_CreateTextureFromSurface(settingsrenderer, Text);
			SDL_RenderCopy(settingsrenderer, Texture, NULL, &ShoulderText[i]);

			Text = TTF_RenderText_Solid(TextFont, "6", black);
			Texture = SDL_CreateTextureFromSurface(settingsrenderer, Text);
			SDL_RenderCopy(settingsrenderer, Texture, NULL, &DisText[i]);

			Text = TTF_RenderText_Solid(TextFont, "6", black);
			Texture = SDL_CreateTextureFromSurface(settingsrenderer, Text);
			SDL_RenderCopy(settingsrenderer, Texture, NULL, &PointText[i]);
			NumberOfPoints = 6;
			break;
		default:
			break;
		}

	}

	SDL_RenderFillRect(settingsrenderer, &GameSettingsDone);


	SDL_RenderFillRect(settingsrenderer, &GameSettingsHeight);
	SDL_RenderFillRect(settingsrenderer, &GameSettingsPoints);
	SDL_RenderFillRect(settingsrenderer, &GameSettingsDist);

	SDL_Surface* HeightText = TTF_RenderText_Solid(TextFont, "  Shoulder Height : ", black);
	SDL_Texture* HeightTexture = SDL_CreateTextureFromSurface(settingsrenderer, HeightText);
	SDL_RenderCopy(settingsrenderer, HeightTexture, NULL, &GameSettingsHeight);

	SDL_Surface* DistText = TTF_RenderText_Solid(TextFont, "  Distance :       ", black);
	SDL_Texture* DistTexture = SDL_CreateTextureFromSurface(settingsrenderer, DistText);
	SDL_RenderCopy(settingsrenderer, DistTexture, NULL, &GameSettingsDist);

	SDL_Surface* PointsText = TTF_RenderText_Solid(TextFont, "  Number of points :", black);
	SDL_Texture* PointsTexture = SDL_CreateTextureFromSurface(settingsrenderer, PointsText);
	SDL_RenderCopy(settingsrenderer, PointsTexture, NULL, &GameSettingsPoints);

	SDL_Surface* DoneText = TTF_RenderText_Solid(TextFont, "  Done  ", black);
	SDL_Texture* DoneTexture = SDL_CreateTextureFromSurface(settingsrenderer, DoneText);
	SDL_RenderCopy(settingsrenderer, DoneTexture, NULL, &GameSettingsDone);
	std::stringstream text;
	text << "" << CCK.ShoulderHeightFromBase;

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
					}
					else if (MousePosX > Dis[i].x && MousePosX < Dis[i].w + Dis[i].x && MousePosY > Dis[i].y && MousePosY < Dis[i].h + Dis[i].y)
					{
						SDL_RenderFillRect(settingsrenderer, &Dis[i]);
					}
					else if (MousePosX > Point[i].x && MousePosX < Point[i].w + Point[i].x && MousePosY > Point[i].y && MousePosY < Point[i].h + Point[i].y)
					{
						SDL_RenderFillRect(settingsrenderer, &Point[i]);
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
