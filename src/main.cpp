#include<iostream>
#include<stdlib.h>
#include<string>
#include<vector>
#include<time.h>
#include<random>
#include"fssimplewindow.h"
#include"Bitmap.h"
#include"motion_prim.h"
#include"planner.h"
#include <climits>

using namespace std;

// Function that draws the generated path on the screen
void DrawPath(vector<node> plan,CharBitmap* map)
{
	for (int i = 0; i < plan.size()-1; i++) {
		int plan_idx = plan[i+1].theta - plan[i].theta;
		int prim_idx=0;
		if (plan_idx == 0|| plan_idx==360) {
			if (plan[i + 1].pre_cost == 1) {
				prim_idx = 0;
			}
			else if (plan[i + 1].pre_cost == 7) {
				prim_idx = 4;
			}
			else {
				prim_idx = 3;
			}
		}
		else if (plan_idx == 45 || plan_idx == -315) {
			prim_idx = 1;
		}
		else if (plan_idx == -45 || plan_idx == 315) {
			prim_idx = 2;
		}

		//map->DrawCar(plan[i].x, plan[i].y, plan[i].theta,"empty");
		map->DrawTrajectory(plan[i].x, plan[i].y, plan[i].theta, prim_idx);
	}
	return;
}

// Loads random cars in the environment
vector<car> LoadCars(int num_cars) {
	int allowed_headings[4] = { 0,90,180,270 };
	vector<car> rand_cars;
	/*if (num_cars > 5) {
		num_cars = 5;
	}*/
	default_random_engine generator(time(NULL));
	uniform_int_distribution<int> pos_distribution(10, 390);
	uniform_int_distribution<int> vel_distribution(1, 5);
	uniform_int_distribution<int> heading_distribution(0, 3);
	for (int i = 0; i < num_cars; i++) {
		car some_car(pos_distribution(generator),pos_distribution(generator),allowed_headings[heading_distribution(generator)] , vel_distribution(generator));
	
		rand_cars.push_back(some_car);
	}
	return rand_cars;
}

// Draw random cars in the environment
void DrawCars(vector<car> &rand_cars, CharBitmap* map) {
	for (int i = 0;i<rand_cars.size();i++) {
		if(rand_cars[i].xpos>=8 && rand_cars[i].xpos <= 394 && rand_cars[i].ypos >= 8 && rand_cars[i].ypos <= 394)
		if (rand_cars[i].heading == 0) {
			rand_cars[i].xpos = rand_cars[i].xpos + ( rand_cars[i].velocity);
			map->DrawCar(rand_cars[i].xpos, rand_cars[i].ypos, rand_cars[i].heading, "");
		}
		else if (rand_cars[i].heading == 90) {

			rand_cars[i].ypos = rand_cars[i].ypos + (rand_cars[i].velocity);
			map->DrawCar(rand_cars[i].xpos, rand_cars[i].ypos, rand_cars[i].heading, "");
		}
		else if (rand_cars[i].heading == 180) {
			rand_cars[i].xpos = rand_cars[i].xpos - (rand_cars[i].velocity);
			map->DrawCar(rand_cars[i].xpos, rand_cars[i].ypos, rand_cars[i].heading, "");
		}
		else if (rand_cars[i].heading == 270) {
		
			rand_cars[i].ypos = rand_cars[i].ypos- (rand_cars[i].velocity);
			map->DrawCar(rand_cars[i].xpos, rand_cars[i].ypos, rand_cars[i].heading, "");
		}
	}
}

int main()
{
	CharBitmap bitu;                                                         // the cost map
	CharBitmap* map_ptr;
	map_ptr = &bitu;
	lattice_graph* thegraph = new lattice_graph;                              //The lattice graph
	thegraph->set_motion_prims();

	// change start and goal here to start the planner
	node start(300, 200, 0);                                                  //Start pose for the vehicle
	node goal(200, 330, 90);                                                  //Goal pose for the vehicle
	int w, h, key;
	bool terminate=false;
	w = 400;
	h = 400;
	bitu.create(w,h);
	cout << "Start Position" << endl << start << endl;
	cout << "Goal Position" << endl << goal << endl;
	int scale = 2;
	int lb, mb, rb, sx, sy;
	int x, y;
	string filename;

	//Loading the MAP
	cout << "Load file Name? ";
	cin >> filename;
	bitu.load(filename);

	vector<car> other_cars = LoadCars(7);
	block_trajectory(other_cars, map_ptr);
	map_transform(map_ptr);                                                    //Inflate obstacles


	state2d** states = new state2d * [400];                                    //Dynamic array that stores G values from backward A* search
	for (int i = 0; i < 400; ++i) {
		states[i] = new state2d[400];
	}
	heuristic_planner(states, map_ptr, goal);                                  //The planner that calculates heuristics using a backward A* search

	vector<node> plan;
	plan= planner(states,map_ptr, thegraph, start, goal);


	FsOpenWindow(0, 0, w*scale, h*scale, 1);
	int draw_itr = 0;
	int car_itr = 0;

	clock_t beginTime;
	beginTime = clock();
	int time_passed;

	while (!terminate) 
	{
		FsPollDevice();
		key = FsInkey();
		FsGetMouseEvent(lb, mb, rb, sx, sy);
		glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

		// Add real time obstacles to the map
		if (FSKEY_0 <= key && key <= FSKEY_7) {
			x = sx / scale;
			y = sy / scale;
			bitu.setPixel(x, y, '0' + key - FSKEY_0);
		}
		else
		{
			switch (key)
			{
			case FSKEY_ESC:
				terminate = true;

				// Save a map
			case FSKEY_S:
				cout << "Save File Name? ";
				cin >> filename;
				bitu.save(filename);
				break;

				//Load a saved map
			case FSKEY_L:
				cout << "Load file Name? ";
				cin >> filename;
				bitu.load(filename);
				break;
			

			}
		}
		
		bitu.draw();
		
		// If a dynamic obtacle is added the car replans
		if (FSKEY_0 <= key && key <= FSKEY_7) {
			node start1(plan[draw_itr].x, plan[draw_itr].y, plan[draw_itr].theta);
			map_transform(map_ptr);
			heuristic_planner(states, map_ptr, goal);
			plan = planner(states, map_ptr, thegraph, start1, goal);
			draw_itr = 0;
		}

		time_passed = (int(clock() - beginTime)) / CLOCKS_PER_SEC; //keeping track of time
		// car replans every 8 seconds to deal with dynamic obstacles
		if ( time_passed % 8 == 0) {
			node start1(plan[draw_itr].x, plan[draw_itr].y, plan[draw_itr].theta);
			map_transform(map_ptr);
			heuristic_planner(states, map_ptr, goal);
			plan = planner(states, map_ptr, thegraph, start1, goal);
			draw_itr = 0;
		}

		
		DrawCars(other_cars, map_ptr);
		block_trajectory(other_cars, map_ptr);
		Clear_trajectory(other_cars, map_ptr);
		DrawPath(plan, map_ptr);
		bitu.DrawCar(plan[draw_itr].x, plan[draw_itr].y, plan[draw_itr].theta, "filled");
		draw_itr++;


		// When the car has reached its destination
		if (draw_itr >= plan.size()) {
			cout << "goal reached" << endl;
			char go;
			cout << "Move your pointer to next place you want to go and press p" << endl;
			cin >> go;
			FsGetMouseEvent(lb, mb, rb, sx, sy);
			if (go == 'p') {
				draw_itr = plan.size() - 1;
				int goalx, goaly, goaltheta;
				char response;
				node startnew(plan[draw_itr].x, plan[draw_itr].y, plan[draw_itr].theta);
				goalx = sx / scale;
				goaly = 400 - sy / scale;
				cout << " goal X and Y are :" << "X " << goalx << " Y " << goaly << endl;
				cout << "Do you want to park here?" << endl;
				cin >> response;
				if (response == 'y') {
					cout << " What do you want the goal orientation to be? Enter a multiple of 45 " << endl;
					cin >> goaltheta;
					goal.x = goalx;
					goal.y = goaly;
					goal.theta = goaltheta;
					goal.gval = INT_MAX;
					cout << "Planning......." << endl;
					heuristic_planner(states, map_ptr, goal);
					plan = planner(states, map_ptr, thegraph, startnew, goal);
					draw_itr = 0;

				}
				else {
					draw_itr = 0;
				}
			}
		}
		car_itr++;
		FsSwapBuffers();
		FsSleep(700);
	}
	return 0;
}

	

 
 
