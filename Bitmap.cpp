#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "fssimplewindow.h"
#include <fstream>
#include <string>
#include<math.h>
#include "Bitmap.h"



void CharBitmap::create(int w, int h)
{
	width = w * scale;
	height = h * scale;
	pixels = new char[w*h + 1];
	trans_pixels = new char[w * h + 1];
	for (int i = 0; i < ((w*h) + 1); i++) {
		pixels[i] = '0';
		trans_pixels[i] = '0';
	}

}

void CharBitmap::setPixel(int x, int y, unsigned char p)
{
	if (x > 0 && y > 0 && x < (width / scale) && y < (height / scale) && pixels != nullptr) {
		pixels[y*(width / scale) + x] = p;
	}
}

void CharBitmap::DrawCar(int x, int y, int theta,string s="empty") {
	double angle = -theta * 3.14 / 180;
	y = (height/scale) - y;
	glColor3ub(255, 255, 255);
	if (s == "filled") {
		glBegin(GL_QUADS);
		glVertex2f((x + (8) * cos(angle) - (4) * sin(angle)) * scale, ((y + (8) * sin(angle) + (4) * cos(angle)) * scale));
		glVertex2f((x + (-8) * cos(angle) - (4) * sin(angle)) * scale, ((y + (-8) * sin(angle) + (4) * cos(angle)) * scale));
		glVertex2f((x + (-8) * cos(angle) - (-4) * sin(angle)) * scale, (y + (-8) * sin(angle) + (-4) * cos(angle)) * scale);
		glVertex2f((x + (8) * cos(angle) - (-4) * sin(angle)) * scale, ((y + (+8) * sin(angle) + (-4) * cos(angle)) * scale));
		glEnd();
	}
	else {
		glColor3ub(255, 255, 0);
		glBegin(GL_QUADS);
		glVertex2f((x + (8) * cos(angle) - (4) * sin(angle)) * scale, ((y + (8) * sin(angle) + (4) * cos(angle)) * scale));
		glVertex2f((x + (-8) * cos(angle) - (4) * sin(angle)) * scale, ((y + (-8) * sin(angle) + (4) * cos(angle)) * scale));
		glVertex2f((x + (-8) * cos(angle) - (-4) * sin(angle)) * scale, (y + (-8) * sin(angle) + (-4) * cos(angle)) * scale);
		glVertex2f((x + (8) * cos(angle) - (-4) * sin(angle)) * scale, ((y + (+8) * sin(angle) + (-4) * cos(angle)) * scale));
		glEnd();
	}
}

bool CharBitmap::getPixel(int x, int y)
{
	if (pixels[y * (width / scale) + x] == '0') {
		return 1;
	}
	else {

		return 0;
	}
}

void CharBitmap::setTransPixel(int x, int y, unsigned char p)
{
	if (x > 0 && y > 0 && x < (width / scale) && y < (height / scale) && pixels != nullptr) {
		trans_pixels[y * (width / scale) + x] = p;
	}
}

void CharBitmap::DrawTrajectory(int sx, int sy, int stheta1, int prim_idx)
{
	const double PI = 3.1415927;
	double cx, cy;
	double stheta = -stheta1 * PI/ 180.0;
	sy= (height / scale) - sy;
	int rad = 44;
	int min, max;
	if (prim_idx == 1) {                      //draws ccw turn motion primitive for all allowed orientations
		glColor3f(0.2, 0.9, 0.2);
		glBegin(GL_LINES); 
		cx = sx + 22 * sin(stheta);
		cy = sy - 22 * cos(stheta);
		
		double mult = stheta / (-PI / 4) - 1;
		min = 0 - 16 * mult;
		max = 16 - 16 * mult;

		for (int i = min; i < max; i++) {
			double angle = (double)i * PI / 64.0;
			double x = (double)(cx*scale) + cos(angle)*(double)rad;
			double y = (double)(cy*scale) + sin(angle)*(double)rad;
			glVertex2d(x, y);
		}
		glEnd();
		
	}
	else if (prim_idx==2) { //draws clockwise turn
		glColor3f(0.2, 0.9, 0.2);
		glBegin(GL_LINES);
		cx = sx + 22 * sin(-stheta);
		cy = sy + 22 * cos(-stheta);
		double mult = stheta / (-PI / 4);
		min = 16 * (6 - mult);
		max = 16 * (7 - mult);

		for (int i = min; i < max; i++) {
			double angle = (double)i * PI / 64.0;
			double x = (double)(cx*scale) + cos(angle)*(double)rad;
			double y = (double)(cy*scale) + sin(angle)*(double)rad;
			glVertex2d(x, y);
		}
		glEnd();
	}
	else if (prim_idx == 3 || prim_idx == 0) { // 1 step back or forward
		
		int x_idx;
		int y_idx;
		double mult = stheta / (-PI / 4);
		if (mult == 0 || mult ==8) {
			x_idx = -1;
			y_idx = 0;
		}
		else if (mult == 1) {
			x_idx = -1;
			y_idx = 1;
		}
		else if (mult == 2) {
			x_idx = 0;
			y_idx = 1;
		}
		else if (mult == 3) {
			x_idx = -1;
			y_idx = 1;
		}
		else if (mult == 4) {
			x_idx = 1;
			y_idx = 0;
		}
		else if (mult == 5) {
			x_idx = 1;
			y_idx = -1;
		}
		else if (mult == 6) {
			x_idx = 0;
			y_idx = -1;
		}
		else if (mult == 7) {
			x_idx = -1;
			y_idx = -1;
		}
		if (prim_idx == 3) {
			glColor3f(1, 0, 0);
			glBegin(GL_LINES);
			glVertex2f((sx)*scale, sy*scale);
			glVertex2f((sx + x_idx)*scale, (sy + y_idx)*scale);
			glEnd();
		}
		else {
			glColor3f(0.2, 0.9, 0.2);
			glBegin(GL_LINES);
			glVertex2f((sx)*scale, sy*scale);
			glVertex2f((sx - x_idx)*scale, (sy - y_idx)*scale);
			glEnd();
		}
	}
	else if (prim_idx == 4) { // a few steps forward
		glColor3f(1, 1, 0.2);
		int x_idx;
		int y_idx;
		double mult = stheta / (-PI / 4);
		if (mult == 0 || mult == 8) {
			x_idx = 10;
			y_idx = 0;
		}
		else if (mult == 1) {
			x_idx = 10;
			y_idx = 10;
		}
		else if (mult == 2) {
			x_idx = 0;
			y_idx = 10;
		}
		else if (mult == 3) {
			x_idx = -10;
			y_idx = 10;
		}
		else if (mult == 4) {
			x_idx = -10;
			y_idx = 0;
		}
		else if (mult == 5) {
			x_idx = -10;
			y_idx = -10;
		}
		else if (mult == 6) {
			x_idx = 0;
			y_idx = -10;
		}
		else if (mult == 7) {
			x_idx = 10;
			y_idx = -10;
		}
		
		glColor3f(0.2, 0.9, 0.2);
		glBegin(GL_LINES);
		glVertex2f((sx)* scale, sy * scale);
		glVertex2f((sx + x_idx) * scale, (sy - y_idx) * scale);
		glEnd();
		
		
	}
}


bool CharBitmap::isFree(int x, int y)
{
	
	y = (height / scale) - y;
	
	if (pixels[y*(width / scale) + x] == '0') {
		return 1;
	}
	else {
		
		return 0;
	}
}

bool CharBitmap::isBlocked(int x, int y)
{
	y = (height / scale) - y;

	if (pixels[y * (width / scale) + x] == '3') {
		return 1;
	}
	else {

		return 0;
	}
	
}

bool CharBitmap::isTransFree(int x, int y)
{
	y = (height / scale) - y;

	if (trans_pixels[y * (width / scale) + x] == '0') {
		return 1;
	}
	else {

		return 0;
	}
}

void CharBitmap::draw() const
{
	int r, g, b;
	int key;
	for (int j = 0; j < (height) / scale; j++)
		for (int i = 0; i < (width) / scale; i++)
		{
			key = pixels[j*(width / scale) + i] - '0';
			switch (key) {
			case 0:
				r = 0;
				g = 0;
				b = 0;
				break;
			case 1:
				r = 0;
				g = 0;
				b = 255;
				break;
			case 2:
				r = 255;
				g = 0;
				b = 0;
				break;
			case 3:
				r = 255;
				g = 0;
				b = 255;
				break;
			case 4:
				r = 0;
				g = 255;
				b = 0;
				break;
			case 5:
				r = 0;
				g = 255;
				b = 255;
				break;
			case 6:
				r = 255;
				g = 255;
				b = 0;
				break;
			case 7:
				r = 255;
				g = 255;
				b = 255;
				break;

			}
			glColor3ub(r, g, b);
			glBegin(GL_QUADS);
			glVertex2i(i * scale, j * scale);
			glVertex2i((i + 1) * scale, j * scale);
			glVertex2i((i + 1) * scale, (j + 1) * scale);
			glVertex2i(i * scale, (j + 1) * scale);
			glEnd();
		
			
		}
}


// if want to visulize the transfomed map
void CharBitmap::drawTransform() const
{
	int r, g, b;
	int key;
	for (int j = 0; j < (height) / scale; j++)
		for (int i = 0; i < (width) / scale; i++)
		{
			key = trans_pixels[j * (width / scale) + i] - '0';
			switch (key) {
			case 0:
				r = 0;
				g = 0;
				b = 0;
				break;
			case 1:
				r = 0;
				g = 0;
				b = 255;
				break;
			case 2:
				r = 255;
				g = 0;
				b = 0;
				break;
			case 3:
				r = 255;
				g = 0;
				b = 255;
				break;
			case 4:
				r = 0;
				g = 255;
				b = 0;
				break;
			case 5:
				r = 0;
				g = 255;
				b = 255;
				break;
			case 6:
				r = 255;
				g = 255;
				b = 0;
				break;
			case 7:
				r = 255;
				g = 255;
				b = 255;
				break;

			}
			glColor3ub(r, g, b);
			glBegin(GL_QUADS);
			glVertex2i(i * scale, j * scale);
			glVertex2i((i + 1) * scale, j * scale);
			glVertex2i((i + 1) * scale, (j + 1) * scale);
			glVertex2i(i * scale, (j + 1) * scale);
			glEnd();


		}
}

void CharBitmap::save(const string fName) const
{
	ofstream outFile;
	outFile.open(fName);
	if (outFile.is_open()) {
		outFile << width / scale << ' ' << height / scale << endl;
		for (int i = 0; i < (height / scale); i++) {
			for (int j = 0; j < (width / scale); j++) {
				
				outFile << pixels[i*(width/scale) + j];
			}
			outFile << endl;
		}
		outFile.close();
	}
}

void CharBitmap::cleanUp()
{
	width = height = 0;
	if (pixels != nullptr)           // If writing this in C instead of C++, it would be NULL instead of nullptr
		delete[] pixels;
	pixels = nullptr;
}

void CharBitmap::load(const string fName) // load a bitmap from a file
{
	ifstream inFile;
	inFile.open(fName);

	if (inFile.is_open()) {
		cleanUp();
		int w, h;
		if (!inFile.eof()) {
			inFile >> w >> h;
			create(w, h);
			for (int i = 0; i < (height/scale); i++) {
				for (int j = 0; j < (width/scale); j++) {
					inFile >> pixels[i*(width/scale) + j];
				}
			}
		}
		inFile.close();

	}
}


CharBitmap::CharBitmap()
{
	
}


CharBitmap::~CharBitmap()
{
	cleanUp();
}
