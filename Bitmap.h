#pragma once
#include <vector>
#include"fssimplewindow.h"

struct car {
	int xpos;
	int ypos;
	int heading;
	int velocity;

	car() {

	}
	car(int xpos, int ypos, int heading, int velocity) {
		this->xpos = xpos;
		this->ypos = ypos;
		this->heading = heading;
		this->velocity = velocity;
	}
};

using namespace std;

class CharBitmap
{
public:
	int width, height;
	int scale=2;
	char *pixels;
	char *trans_pixels;

	//creates the bitmap
	void create(int w, int h);

	//sets the value of pixels in the map
	void setPixel(int x, int y, unsigned char p);

	//draws the bitmap on screen
	void draw() const;

	//draws the transformed map on screen
	void drawTransform() const;

	// saves the bitmap in a text file
	void save(const string fName) const;
	void cleanUp();

	// loads the map from text file
	void load(const string fName);

	// draws the car on the screen
	void DrawCar(int, int, int,string);
	bool getPixel(int x, int y);
	void setTransPixel(int x, int y, unsigned char p);

	// draws the generated path on the screen
	void DrawTrajectory(int, int, int ,int);

	//checks if the map free
	bool isFree(int x, int y);

	bool isBlocked(int x, int y);

	//checks if the transformed map is free
	bool isTransFree(int x, int y);
	CharBitmap();
	~CharBitmap();
};

