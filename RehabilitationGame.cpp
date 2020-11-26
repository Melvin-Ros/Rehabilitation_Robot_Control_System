#include "RehabilitationGame.h"
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <vector>
#include "CrustCrawlerKinematics.h"
#include "SimpleSerial.h"
#include <string>
std::vector<RehabilitationGame::GameObject> asteroids;
RehabilitationGame::GameObject player;
CrustCrawlerKinematics CCK;
CrustCrawlerKinematics::Angles TargetAngles;
double currentPosX = 0;
double currentPosY = 147;
double currentPosZ = 210;
SDL_Event event;
SimpleSerial serial("COM6",9600);
int counter = 0;
void RehabilitationGame::init(const char* title, int posx, int posy, int width, int height, MyoController &collector)
{
	
	CrustCrawlerKinematics::Pos tempPos;
	tempPos = CCK.ForwardKinematics(0, 45, 80, 180);
	currentPosX = tempPos.x;
	currentPosZ = tempPos.z;
	currentPosY = tempPos.y;
	TargetAngles = CCK.InverseKinematics(currentPosX, currentPosY, currentPosZ);

	TargetAngles.theta1 += 90;
	TargetAngles.theta2 += 180;
	TargetAngles.theta3 += 180;
	TargetAngles.theta4 += 180;

	std::string message = "1 " + std::to_string(int(TargetAngles.theta1)) + ":" +
		"2 " + std::to_string(int(TargetAngles.theta2)) + ":" +
		"3 " + std::to_string(int(TargetAngles.theta3)) + ":" +
		"4 " + std::to_string(int(TargetAngles.theta4)) + ":";
	std::cout << message << std::endl;
	serial.writeString(message);
	
	SDL_Init(SDL_INIT_EVERYTHING);
	const int FlAGS = IMG_INIT_PNG;
	IMG_Init(IMG_INIT_PNG);
	window = SDL_CreateWindow(title, posx, posy, width, height, SDL_WINDOW_SHOWN);
	renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC);
	SDL_Surface *temp;
	temp = IMG_Load("backgroundSpace.png");
	
	background = SDL_CreateTextureFromSurface(renderer, temp);
		
	for (int i = 0; i < 1; i++)
	{
		GameObject asteroid;
		asteroids.push_back(asteroid);
	}
	std::cout << asteroids.size() << std::endl;
	startGame = true;
	SDL_Delay(3000);
}

void RehabilitationGame::update(MyoController& collector)
{
	counter++;
	
	if (startGame) {

		if (!gameIsInitialized) {
			initGameObject(player,400,300,50,50,"SpaceShip.png","Player");
			for (int i = 0; i < asteroids.size(); i++)
			{
				srand(time(NULL)+i*i);
				initGameObject(asteroids[i], rand() % 450 + 150, rand() % 450 + 150, 50, 50, "Asteroid.png", "Asteroid");
				SDL_Delay(100);
				std::cout << "init Asteroid: " << i << std::endl;
			}
			gameIsInitialized = true;
			
		}
		else {
			// In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
		// In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
			

			SDL_Event Buttonclick;

			SDL_WaitEvent(&Buttonclick);
			switch (Buttonclick.type)
			{
			case SDL_MOUSEBUTTONDOWN:
				startGame = false;
				break;
			default:
				break;
			}
			
			for (int i = 0; i < asteroids.size(); i++) {
				UpdateGameObject(asteroids[i]);
				srand(time(NULL) + i);
				if ((player.DestR.x - asteroids[i].DestR.x) * (player.DestR.x - asteroids[i].DestR.x) + (player.DestR.y - asteroids[i].DestR.y) * (player.DestR.y - asteroids[i].DestR.y) < 50 * 50) {
					asteroids[i].DestR.x = rand() % 450 + 150;
					SDL_Delay(300);
					asteroids[i].DestR.y = rand() % 450 + 150, 50, 50;
				}
			}	
			    
			if (collector.Direction == MyoController::Left) {
				player.xdir = -1;
				player.ydir = 0;
				double targetX, targetY, targetZ;
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
				double targetX, targetY, targetZ;
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
				double targetX, targetY, targetZ;
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
				double targetX, targetY, targetZ;
				targetX = currentPosX + player.xdir;
				targetY = currentPosY;
				targetZ = currentPosZ + player.ydir;
				//if (targetX * targetX + targetY * targetY + targetZ * targetZ <= 360 * 360) {
					currentPosX -= player.xdir;
					currentPosZ -= player.ydir;
				//}
				

			}
			else {
				player.xdir = 0;
				player.ydir = 0;
			}
			TargetAngles = CCK.InverseKinematics(currentPosX, currentPosY, currentPosZ);
			std::string message = "";
			if (TargetAngles.theta1 + TargetAngles.theta2 + TargetAngles.theta3 + TargetAngles.theta4 <= 1440) {
				TargetAngles.theta1 += 90;
				TargetAngles.theta2 += 180;
				TargetAngles.theta3 += 180;
				TargetAngles.theta4 += 180;
				message = "1 " + std::to_string(int(TargetAngles.theta1)) + ":" +
					"2 " + std::to_string(int(TargetAngles.theta2)) + ":" +
					"3 " + std::to_string(int(TargetAngles.theta3)) + ":" +
					"4 " + std::to_string(int(TargetAngles.theta4)) + ":";

				if (counter == 10) {
					serial.writeString(message);
					counter = 0;
				}
			}
			else {
				CrustCrawlerKinematics::Pos tempPos;
				tempPos = CCK.ForwardKinematics(0, 45, 80, 180);
				currentPosX = tempPos.x;
				currentPosZ = tempPos.z;
				currentPosY = tempPos.y;
				TargetAngles = CCK.InverseKinematics(currentPosX, currentPosY, currentPosZ);

				TargetAngles.theta1 += 90;
				TargetAngles.theta2 += 180;
				TargetAngles.theta3 += 180;
				TargetAngles.theta4 += 180;

				std::string message = "1 " + std::to_string(int(TargetAngles.theta1)) + ":" +
					"2 " + std::to_string(int(TargetAngles.theta2)) + ":" +
					"3 " + std::to_string(int(TargetAngles.theta3)) + ":" +
					"4 " + std::to_string(int(TargetAngles.theta4)) + ":";
				
				serial.writeString(message);
			}
			
			
			
				
				
				
				
			
			
			
			UpdateGameObject(player);
		}
		
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

void RehabilitationGame::initGameObject(GameObject& gameObject,int x, int y, int width, int height, const char* spriteLocation, std::string name)
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
		

		gameObject.DestR.x += gameObject.xdir*2;
		gameObject.DestR.y += gameObject.ydir*2;
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

