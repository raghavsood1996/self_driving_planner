#pragma once
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <map>
#include "Bitmap.h"
using namespace std;

// for 2d heuristics search on a x and y grid space
struct state2d {
	int x;
	int y;
	bool isclosed;
	float gval;
	state2d() {
		this->gval = INT_MAX;
	}
};


// node describing a state in the lattice graph
struct node {
	int x;
	int y;
	int theta;
	float gval;
	float hval;
	int pre_cost;
	node* parent;
	node() {
		this->gval = INT_MAX;
		this->parent = NULL;
	}

	node(int x, int y, int theta) {
		this->x = x;
		this->y = y;
		this->theta = theta;
		this->parent = NULL;
		this->pre_cost = 0;
		this->gval = INT_MAX;
	}

	bool operator==(const node& rhs) const {
		
		if (this->x != rhs.x) {
			return false;
		}
		if (this->y != rhs.y) {
			return false;
		}
		if (this->theta != rhs.theta) {
			return false;
		}
		return true;
		
	}



	friend ostream& operator<<(ostream& os, const node& w)
	{
		os << "***** Node *****" << endl << endl;
		os << "X: ";
		os << w.x;
		os << endl;
		os << "Y: ";
		os << w.y;
		os << endl;
		os << "Heading: ";
		os << w.theta;
		os << endl;
		os << "G value: ";
		os << w.gval;
		os << endl;
		os << "H value: ";
		os << w.hval;
		os << endl;
		os << "Pre cost: ";
		os << w.pre_cost;
		os << endl;
		return os;
	}

	string toString() const
	{
		string temp = "";

		temp += to_string(this->x) + ",";
		temp += to_string(this->y) + ",";
		temp += to_string(this->theta) + ",";
		temp = temp.substr(0, temp.length() - 1);

		return temp;
	}



};

struct motion_prim {
	int dx;
	int dy;
	int dtheta;
	int cost;
};

class lattice_graph {
private:
	vector<node>  the_graph;
	motion_prim fwd;
	motion_prim back;
	motion_prim ccw;
	motion_prim cw;
	motion_prim fws;
	vector<motion_prim> prim_0;   //storing primitives for angle 0 degrees
	vector<motion_prim> prim_45;  //storing primitives for angle 45 degrees
	vector<motion_prim> prim_90;  //storing primitives for angle 90 degrees
	vector<motion_prim> prim_135; //storing primitives for angle 135 degrees
	vector<motion_prim> prim_180; //storing primitives for angle 180 degrees
	vector<motion_prim> prim_225; //storing primitives for angle 225 degrees
	vector<motion_prim> prim_270; //storing primitives for angle 270 degrees
	vector<motion_prim> prim_315; //storing primitives for angle 315 degrees
	vector<motion_prim> prim_360; //storing primitives for angle 360 degrees
	map<int, vector<motion_prim>> prim_map; //mapping angles to motion primitives
	int back_cost=5;			//cost for backward motion primitive
	int cw_cost=17;				//cost for 1/16 counter-clockwise motion primitive
	int ccw_cost=17;			//cost for 1/16 clockwise motion primitive
	int fwd_cost=1;				//cost for 1 step forward
	int fwd_step_cost=7;		//cost for moving more than one step forward
	int fwd_step = 8;           //size for forward step motion primitive
public:

	// sets motion primitives for the graph
	void set_motion_prims() {
		//motion_prims for 0 degrees heading
		prim_0.push_back(fwd);
		prim_0.push_back(back);
		prim_0.push_back(ccw);
		prim_0.push_back(cw);
		prim_0.push_back(fws);
		prim_0[0].dx = 1; //forward one step
		prim_0[0].dy = 0;
		prim_0[0].dtheta = 0;
		prim_0[0].cost = fwd_cost;

		
		prim_0[1].dx = 15; //ccw 1/16 turn
		prim_0[1].dy = 6;
		prim_0[1].dtheta = 1;
		prim_0[1].cost = ccw_cost;

		prim_0[2].dx = 15; //cw 1/16 turn
		prim_0[2].dy = -6;
		prim_0[2].dtheta = -1;
		prim_0[2].cost = ccw_cost;

		prim_0[3].dx = -1; //backward one step
		prim_0[3].dy = 0;
		prim_0[3].dtheta = 0;
		prim_0[3].cost = back_cost;

		prim_0[4].dx = fwd_step; //forward some step
		prim_0[4].dy = 0;
		prim_0[4].dtheta = 0;
		prim_0[4].cost = fwd_step_cost;



		//motion_prims for 45 degrees heading

		prim_45.push_back(fwd);
		prim_45.push_back(back);
		prim_45.push_back(ccw);
		prim_45.push_back(cw);
		prim_45.push_back(fws);

		prim_45[0].dx = 1; //forward one step
		prim_45[0].dy = 1;
		prim_45[0].dtheta = 0;
		prim_45[0].cost = fwd_cost;

		prim_45[1].dx = 6; //ccw 1/16 turn
		prim_45[1].dy = 15;
		prim_45[1].dtheta = 1;
		prim_45[1].cost = ccw_cost;

		prim_45[2].dx = 15; //cw 1/16 turn
		prim_45[2].dy = 6;
		prim_45[2].dtheta = -1;
		prim_45[2].cost = ccw_cost;

		prim_45[3].dx = -1; //backward one step
		prim_45[3].dy = -1;
		prim_45[3].dtheta = 0;
		prim_45[3].cost = back_cost;

		prim_45[4].dx = fwd_step; //forward one step
		prim_45[4].dy = fwd_step;
		prim_45[4].dtheta = 0;
		prim_45[4].cost = fwd_step_cost;




		//motion_prims for 90 degrees heading
		prim_90.push_back(fwd);
		prim_90.push_back(back);
		prim_90.push_back(ccw);
		prim_90.push_back(cw);
		prim_90.push_back(fws);

		prim_90[0].dx = 0; //forward one step
		prim_90[0].dy = 1;
		prim_90[0].dtheta = 0;
		prim_90[0].cost = fwd_cost;

		prim_90[1].dx = -6; //ccw 1/16 turn
		prim_90[1].dy = 15;
		prim_90[1].dtheta = 1;
		prim_90[1].cost = ccw_cost;

		prim_90[2].dx = 6; //cw 1/16 turn
		prim_90[2].dy = 15;
		prim_90[2].dtheta = -1;
		prim_90[2].cost = ccw_cost;

		prim_90[3].dx = 0; //backward one step
		prim_90[3].dy = -1;
		prim_90[3].dtheta = 0;
		prim_90[3].cost = back_cost;

		prim_90[4].dx = 0; //forward one step
		prim_90[4].dy = fwd_step;
		prim_90[4].dtheta = 0;
		prim_90[4].cost = fwd_step_cost;


		//motion_prims for 135 degrees heading
		prim_135.push_back(fwd);
		prim_135.push_back(back);
		prim_135.push_back(ccw);
		prim_135.push_back(cw);
		prim_135.push_back(fws);

		prim_135[0].dx = -1; //forward one step
		prim_135[0].dy = 1;
		prim_135[0].dtheta = 0;
		prim_135[0].cost = fwd_cost;

		prim_135[1].dx = -15; //ccw 1/16 turn
		prim_135[1].dy = 6;  //
		prim_135[1].dtheta = 1;
		prim_135[1].cost = ccw_cost;

		prim_135[2].dx = -6; //cw 1/16 turn
		prim_135[2].dy = 15;
		prim_135[2].dtheta = -1;
		prim_135[2].cost = ccw_cost;

		prim_135[3].dx = 1; //backward one step
		prim_135[3].dy = -1;
		prim_135[3].dtheta = 0;
		prim_135[3].cost = back_cost;

		prim_135[0].dx = -fwd_step; //forward steps
		prim_135[0].dy = fwd_step;
		prim_135[0].dtheta = 0;
		prim_135[0].cost = fwd_step_cost;

		//motion_prims for 180 degrees heading
		prim_180.push_back(fwd);
		prim_180.push_back(back);
		prim_180.push_back(ccw);
		prim_180.push_back(cw);
		prim_180.push_back(fws);

		prim_180[0].dx = -1; //forward one step
		prim_180[0].dy = 0;
		prim_180[0].dtheta = 0;
		prim_180[0].cost = fwd_cost;

		prim_180[1].dx = -15; //ccw 1/16 turn
		prim_180[1].dy = -6;
		prim_180[1].dtheta = 1;
		prim_180[1].cost = ccw_cost;

		prim_180[2].dx = -15; //cw 1/16 turn
		prim_180[2].dy = 6;
		prim_180[2].dtheta = -1;
		prim_180[2].cost = ccw_cost;

		prim_180[3].dx = 1; //backward one step
		prim_180[3].dy = 0;
		prim_180[3].dtheta = 0;
		prim_180[3].cost = back_cost;

		prim_180[4].dx = -fwd_step; //forward step
		prim_180[4].dy = 0;
		prim_180[4].dtheta = 0;
		prim_180[4].cost = fwd_step_cost;


		//motion_prims for 225 degrees heading
		prim_225.push_back(fwd);
		prim_225.push_back(back);
		prim_225.push_back(ccw);
		prim_225.push_back(cw);
		prim_225.push_back(fws);

		prim_225[0].dx = -1; //forward one step
		prim_225[0].dy = -1;
		prim_225[0].dtheta = 0;
		prim_225[0].cost = fwd_cost;

		prim_225[1].dx = -6; //ccw 1/16 turn
		prim_225[1].dy = -15; //
		prim_225[1].dtheta = 1;
		prim_225[1].cost = ccw_cost;

		prim_225[2].dx = -15; //cw 1/16 turn
		prim_225[2].dy = -6;
		prim_225[2].dtheta = -1;
		prim_225[2].cost = ccw_cost;

		prim_225[3].dx = 1; //backward one step
		prim_225[3].dy = 1;
		prim_225[3].dtheta = 0;
		prim_225[3].cost = back_cost;

		prim_225[4].dx = -fwd_step; //forward one step
		prim_225[4].dy = -fwd_step;
		prim_225[4].dtheta = 0;
		prim_225[4].cost = fwd_step_cost;


		//motion_prims for 270 degrees heading
		prim_270.push_back(fwd);
		prim_270.push_back(back);
		prim_270.push_back(ccw);
		prim_270.push_back(cw);
		prim_270.push_back(fws);

		prim_270[0].dx = 0; //forward one step
		prim_270[0].dy = -1;
		prim_270[0].dtheta = 0;
		prim_270[0].cost = fwd_cost;

		prim_270[1].dx = 6; //ccw 1/16 turn
		prim_270[1].dy = -15;
		prim_270[1].dtheta = 1;
		prim_270[1].cost = ccw_cost;

		prim_270[2].dx = -6; //cw 1/16 turn
		prim_270[2].dy = -15;
		prim_270[2].dtheta = -1;
		prim_270[2].cost = ccw_cost;

		prim_270[3].dx = 0; //backward one step
		prim_270[3].dy = 1;
		prim_270[3].dtheta = 0;
		prim_270[3].cost = back_cost;

		prim_270[4].dx = 0; //forward step
		prim_270[4].dy = -fwd_step;
		prim_270[4].dtheta = 0;
		prim_270[4].cost = fwd_step_cost;


		//motion_prims for 315 degrees heading
		prim_315.push_back(fwd);
		prim_315.push_back(back);
		prim_315.push_back(ccw);
		prim_315.push_back(cw);
		prim_315.push_back(fws);

		prim_315[0].dx = 1; //forward one step
		prim_315[0].dy = -1;
		prim_315[0].dtheta = 0;
		prim_315[0].cost = fwd_cost;

		prim_315[1].dx = 15; //ccw 1/16 turn
		prim_315[1].dy = -6;
		prim_315[1].dtheta = 1;
		prim_315[1].cost = ccw_cost;

		prim_315[2].dx = 6; //cw 1/16 turn
		prim_315[2].dy = -15;
		prim_315[2].dtheta = -1;
		prim_315[2].cost = ccw_cost;

		prim_315[3].dx = -1; //backward one step
		prim_315[3].dy = 1;
		prim_315[3].dtheta = 0;
		prim_315[3].cost = back_cost;

		prim_315[4].dx = fwd_step; //forward one step
		prim_315[4].dy = -fwd_step;
		prim_315[4].dtheta = 0;
		prim_315[4].cost = fwd_step_cost;


		prim_map[0] = prim_0;
		prim_map[45] = prim_45;
		prim_map[90] = prim_90;
		prim_map[135] = prim_135;
		prim_map[180] = prim_180;
		prim_map[225] = prim_225;
		prim_map[270] = prim_270;
		prim_map[315] = prim_315;



	}
	
	// generates successors in the lattice graph
	vector<node> getsuccessor(node curr_node,CharBitmap* map ) {
		vector<node> succesors;
		node succesor;
		int currx = curr_node.x;
		int curry = curr_node.y;
		int curr_heading = curr_node.theta;
		if (curr_heading >= 360) {
			curr_heading = curr_heading - 360;
		}
		for (int dir = 0; dir <= 4; dir++) {
			//cout << "direction" << dir << endl;
			if (!collision_check(currx,curry,map,dir,curr_heading)) {
				//cout <<dir<<"  "<<currx<<" x "<<curry<<" y "<< curr_heading<<" theta"<<endl;
				continue;
			}
			else {
				//cout << "adding successor" << endl;
				succesor.x = currx + prim_map[curr_heading][dir].dx;
				succesor.y = curry + prim_map[curr_heading][dir].dy;
				succesor.theta = curr_heading + prim_map[curr_heading][dir].dtheta*45;
				if (succesor.theta < 0) {
					succesor.theta += 360;
				}
				succesor.pre_cost = prim_map[curr_heading][dir].cost;
				succesors.push_back(succesor);
			}
		}
		return succesors;
	}
	
	// collision checking for all motion primitives
	bool collision_check(int currx, int curry, CharBitmap* map, int dir, int heading) {

		//for heading 0 forward
		if (dir == 0 && heading == 0) {
			for (int i = currx + 8; i <= currx + 9; i++) {
				for (int j = curry - 4; j <= curry + 4; j++) {
					if (!map->isFree(i, j)) {
						return 0; //collision if map is not free
					}

				}
			}
			return 1;
		}

		//for heading 0 ccw turn
		else if (dir == 1 && heading == 0) {
			for (int i = currx + 8; i <= currx + 12; i++) {
				for (int j = curry - 4; j <= curry + 4; j++) {
					if (!map->isFree(i, j)) {
						return 0; //collision
					}
				}
			}
			int x = currx + 12;
			int y_start = curry - 4;
			int y_end = curry + 9;
			
			for (x; x - currx <= 18; x++) {

				for (int y = y_start; y <= y_end; y++) {
					if (!map->isFree(x, y)) {
						return 0; //collision
					}

				}
				y_start++;
				y_end++;

			}

			int x_new = currx + 19;
			y_start = curry + 3;
			y_end = curry + 14;

			for (x_new; x_new - currx <= 24; x_new++) {

				for (int y = y_start; y <= y_end; y++) {
					if (!map->isFree(x_new, y)) {
						return 0; //collision
					}

				}
				y_start++;
				y_end--;

			}
			return 1;
		}

		//for heading 0 and cw turn
		else if (dir == 2 && heading == 0) {
			for (int i = currx + 8; i <= currx + 12; i++) {
				for (int j = curry - 4; j <= curry + 4; j++) {
					if (!map->isFree(i, j)) {
						return 0; //collision
					}
				}
			}
			
			int x = currx + 12;
			int y_start = curry + 4;
			int y_end = curry - 9;
			for (x; x - currx <= 18; x++) {

				for (int y = y_start; y >= y_end; y--) {
					if (!map->isFree(x, y)) {
						return 0; //collision
					}

				}
				y_start--;
				y_end--;

			}
			
			int x_new = currx + 19;
			 y_start = curry - 3;
			 y_end = curry - 14;
			
			for (x_new; x_new - currx <= 24; x_new++) {

				for (int y = y_start; y >= y_end; y--) {
					if (!map->isFree(x_new, y)) {
						return 0; //collision
					}
				}
				
				y_start--;
				y_end++;

			}
			return 1;
		}

		//for heading 0 and backward
		else if (dir == 3 && heading == 0)
		{
			for (int i = currx - 8; i >= currx - 9; i--) {
				for (int j = curry - 4; j <= curry + 4; j++) {
					if (!map->isFree(i, j)) {
						return 0; //collision
					}


				}
			}
			return 1;
		}

		//for heading 0 and few steps forward
		else if (dir == 4 && heading == 0) {
				for (int i = currx + 8; i <= currx + 1+fwd_step; i++) {
					for (int j = curry - 4; j <= curry + 4; j++) {
						if (!map->isFree(i, j)) {
							return 0; //collision if map is not free
						}

					}
				}
				return 1;
		}

		//for heading 45 and forward
		else if (dir == 0 && heading == 45) {
			int j = curry + 10;
			for (int i = currx + 4; i <= currx + 10; i++) {
				if (!map->isFree(i, j)) {
					return 0; //collision
				}
				j--;
			}
			return 1;
		}

		//for heading 45 and ccw turn
		else if (dir == 1 && heading == 45) {
			for (int i = currx + 2; i <= currx + 10; i++) {
				for (int j = curry + 4; j <= curry + 23; j++) {
					if (!map->isFree(i, j)) {
						return 0; //collision
					}
				}
			}
			return 1;
		}

		// for heading 45 and cw turn
		else if (dir == 2 && heading == 45) {
			for (int i = currx + 3; i <= currx + 23; i++) {
				for (int j = curry + 2; j <= curry + 10; j++) {
					if (!map->isFree(i, j)) {
						return 0; //collision
					}
				}
			}
			return 1;
		}

		//for heading 45 and backwards
		else if (dir == 3 && heading == 45) {
			int j = curry - 10;
			for (int i = currx - 4; i <= currx + 2; i++) {
				if (!map->isFree(i, j)) {
					return 0; //collision
				}
				j--;
			}
			return 1;
		}

		//for heading 45 and few step forward
		else if (dir == 4 && heading == 45) {
			for (currx; currx <= currx+fwd_step; currx++) {
				int j = curry + 10;
				for (int i = currx + 4; i <= currx + 10; i++) {
					if (!map->isFree(i, j)) {
						return 0; //collision
					}
					j--;
				}
				curry++;
			}
		return 1;
		}

		//for heading 90 and forward
		else if (dir == 0 && heading == 90) {
			for (int i = curry + 8; i <= curry + 9; i++) {
				for (int j = currx - 4; j <= currx + 4; j++) {
					if (!map->isFree(j, i)) {
						return 0; //collision if map is not free
					}
				}
			}
			return 1;

		}

		//for heading 90 and ccw turn
		else if (dir == 1 && heading == 90) {
			for (int i = curry + 8; i <= curry + 12; i++) {
				for (int j = currx - 4; j <= currx + 4; j++) {
					if (!map->isFree(j, i)) {
						return 0; //collision
					}
				}
			}
			int y = curry + 13;
			int x_start = currx + 4;
			int x_end = currx - 9;
			for (y; y - curry <= 18; y++) {

				for (int x = x_start; x > x_end; x--) {
					if (!map->isFree(x, y)) {
						return 0; //collision
					}

				}
				x_start--;
				x_end--;

			}

			int y_new = curry + 19;
			x_start = currx - 3;
			x_end = currx - 15;

			for (y_new; y_new - curry <= 24; y_new++) {

				for (int x = x_start; x > x_end; x--) {
					if (!map->isFree(x, y)) {
						return 0; //collision
					}

				}
				x_start--;
				x_end++;

			}
			return 1;

		}

		//for heading 90 and cw turn
		else if (dir == 2 && heading == 90) {

			for (int i = curry + 8; i <= curry + 12; i++) {
				for (int j = currx - 4; j <= currx + 4; j++) {
					if (!map->isFree(j, i)) {
						return 0; //collision
					}
				}
			}
			int y = curry + 13;
			int x_start = currx - 4;
			int x_end = currx + 9;
			for (y; y - curry <= 18; y++) {

				for (int x = x_start; y < x_end; x++) {
					if (!map->isFree(x, y)) {
						return 0; //collision
					}

				}
				x_start++;
				x_end++;

			}

			int y_new = curry + 19;
			x_start = currx + 3;
			x_end = currx + 15;

			for (y_new; y_new - curry <= 24; y_new++) {

				for (int x = x_start; x < x_end; x++) {
					if (!map->isFree(x, y)) {
						return 0; //collision
					}

				}
				x_start++;
				x_end--;

			}
			return 1;


		}

		//for heading 90 and backwards
		else if (dir == 3 && heading == 90) {
		for (int i = curry - 8; i <= curry - 9; i++) {
			for (int j = currx - 4; j <= currx + 4; j++) {
				if (!map->isFree(j, i)) {
					return 0; //collision if map is not free
				}
			}
		}
		return 1;

		}

		//for heading 90 and few steps forward
		else if (dir == 4 && heading == 90) {
		for (int i = curry + 8; i <= curry + fwd_step+9; i++) {
			for (int j = currx - 4; j <= currx + 4; j++) {
				if (!map->isFree(j, i)) {
					return 0; //collision if map is not free
				}
			}
		}
		return 1;

		}

		//for heading 135 and forward
		else if (dir == 0 && heading == 135) {
		int j = curry + 10;
		for (int i = currx - 4; i >= currx - 10; i--) {
			if (!map->isFree(i, j)) {
				return 0; //collision
			}
			j--;
		}
		return 1;
		}

		//for heading 135 and ccw turn
		else if (dir == 1 && heading == 135) {
		for (int i = currx - 3; i >= currx - 23; i--) {
			for (int j = curry + 2; j <= curry + 10; j++) {
				if (!map->isFree(i, j)) {
					return 0; //collision
				}
			}
		}
		return 1;
		}

		//for heading 135 and cw turn
		else if (dir == 2 && heading == 135) {
		for (int i = currx - 2; i >= currx - 10; i--) {
			for (int j = curry + 4; j <= curry + 23; j++) {
				if (!map->isFree(i, j)) {
					return 0; //collision
				}
			}
		}
		return 1;
		}

		//for heading 135 and backwards
		else if (dir == 3 && heading == 135) {
		int j = curry - 10;
		for (int i = currx + 4; i <= currx + 10; i++) {
			if (!map->isFree(i, j)) {
				return 0; //collision
			}
			j++;
		}
		return 1;
		}

		//for heading 135 and few steps forward
		else if (dir == 4 && heading == 135) {

			for (currx; currx >= currx - fwd_step; currx--) {
				int j = curry + 10;
				for (int i = currx - 4; i >= currx - 10; i--) {
					if (!map->isFree(i, j)) {
						return 0; //collision
					}
					j--;
				}
				curry++;
			}
		return 1;
		}

		//for heading 180 and forward
		else if (dir == 0 && heading == 180) {
		for (int i = currx - 8; i >= currx - 9; i--) {
			for (int j = curry - 4; j <= curry + 4; j++) {
				if (!map->isFree(i, j)) {
					return 0; //collision
				}

			}
		}
		return 1;
		}

		//for heading 180 and ccw turn
		else if (dir == 1 && heading == 180) {
		for (int i = currx - 8; i >= currx - 12; i--) {
			for (int j = curry - 4; j <= curry + 4; j++) {
				if (!map->isFree(i, j)) {
					return 0; //collision
				}
			}
		}
		int x = currx - 13;
		int y_start = curry + 4;
		int y_end = curry - 9;
		for (x; x - currx >= -18; x--) {

			for (int y = y_start; y >= y_end; y--) {
				if (!map->isFree(x, y)) {
					return 0; //collision
				}

			}
			y_start--;
			y_end--;

		}

		int x_new = currx - 19;
		y_start = curry - 3;
		y_end = curry - 15;

		for (x_new; x_new - currx >= -24; x_new--) {

			for (int y = y_start; y >= y_end; y--) {
				if (!map->isFree(x, y)) {
					return 0; //collision
				}
			}
			y_start--;
			y_end++;

		}
		return 1;
		}

		//for heading 180 and cw turn
		else if (dir == 2 && heading == 180)
 {
		for (int i = currx - 8; i >= currx - 12; i--) {
			for (int j = curry - 4; j <= curry + 4; j++) {
				if (!map->isFree(i, j)) {
					return 0; //collision
				}
			}
		}
		int x = currx - 13;
		int y_start = curry - 4;
		int y_end = curry + 9;
		for (x; x - currx >= -18; x--) {

			for (int y = y_start; y < y_end; y++) {
				if (!map->isFree(x, y)) {
					return 0; //collision
				}

			}
			y_start++;
			y_end++;

		}

		int x_new = currx - 19;
		y_start = curry + 3;
		y_end = curry + 15;

		for (x_new; x_new - currx >= -24; x_new--) {

			for (int y = y_start; y <= y_end; y++) {
				if (!map->isFree(x, y)) {
					return 0; //collision
				}

			}
			y_start++;

			y_end--;

		}
		return 1;
		}

		//for heading 180 and backwards
		else if (dir == 3 && heading == 180) {
		for (int i = currx + 8; i <= currx + 9; i++) {
			for (int j = curry - 4; j <= curry + 4; j++) {
				if (!map->isFree(i, j)) {
					return 0; //collision if map is not free
				}

			}
		}
		return 1;

		}

		//for heading 180 and few steps forward
		else if (dir == 4 && heading == 180) {
		for (int i = currx - 8; i >= currx - fwd_step-9; i--) {
			for (int j = curry - 4; j <= curry + 4; j++) {
				if (!map->isFree(i, j)) {
					return 0; //collision
				}

			}
		}
		return 1;
		}

		//for heading 225 and forward
		else if (dir == 0 && heading == 225) {
		int j = curry - 10;
		for (int i = currx - 4; i <= currx + 2; i++) {
			if (!map->isFree(i, j)) {
				return 0; //collision
			}
			j--;
		}
		return 1;
		}

		//for heading 225 and ccw turn
		else if (dir == 1 && heading == 225) {
		for (int i = currx - 2; i >= currx - 10; i--) {
			for (int j = curry - 4; j > curry - 23; j--) {
				if (!map->isFree(i, j)) {
					return 0; //collision
				}
			}
		}
		return 1;
		}

		//for heading 225 and cw turn
		else if (dir == 2 && heading == 225) {
		for (int i = currx - 3; i >= currx - 23; i--) {
			for (int j = curry - 2; j >= curry - 10; j--) {
				if (!map->isFree(i, j)) {
					return 0; //collision
				}
			}
		}
		return 1;
		}

		//for heading 225 and backwards
		else if (dir == 3 && heading == 225) {
		int j = curry + 10;
		for (int i = currx + 4; i <= currx + 10; i++) {
			if (!map->isFree(i, j)) {
				return 0; //collision
			}
			j--;
		}
		return 1;

		}

		//for heading 225 and few steps forward
		else if (dir == 4 && heading == 225) {
			for (currx; currx >= currx - fwd_step; currx--) {
				int j = curry - 10;
				for (int i = currx - 4; i <= currx + 2; i++) {
					if (!map->isFree(i, j)) {
						return 0; //collision
					}
					j--;
				}
				curry--;
			}
		return 1;
		}

		//for heading 270 and forward
		else if (dir == 0 && heading == 270) {
		for (int i = curry - 8; i >= curry - 9; i--) {
			for (int j = currx - 4; j <= currx + 4; j++) {
				if (!map->isFree(j, i)) {
					return 0; //collision if map is not free
				}
			}
		}
		return 1;
		}

		//for heading 270 and ccw turn
		else if (dir == 1 && heading == 270) {
		for (int i = curry - 8; i >= curry - 12; i--) {
			for (int j = currx - 4; j <= currx + 4; j++) {
				if (!map->isFree(j, i)) {
					return 0; //collision
				}
			}
		}
		int y = curry - 13;
		int x_start = currx - 4;
		int x_end = currx + 9;
		for (y; y - curry >= -18; y--) {

			for (int x = x_start; y <= x_end; x++) {
				if (!map->isFree(x, y)) {
					return 0; //collision
				}

			}
			x_start++;
			x_end++;

		}

		int y_new = curry - 19;
		x_start = currx + 3;
		x_end = currx + 15;

		for (y_new; y_new - curry >= -24; y_new--) {

			for (int x = x_start; x <= x_end; x++) {
				if (!map->isFree(x, y)) {
					return 0; //collision
				}

			}
			x_start++;
			x_end--;

		}
		return 1;

		}

		//for heading 270 and cw turn
		else if (dir == 2 && heading == 270) {
		for (int i = curry - 8; i >= curry - 12; i--) {
			for (int j = currx - 4; j <= currx + 4; j++) {
				if (!map->isFree(j, i)) {
					return 0; //collision
				}
			}
		}
		int y = curry - 13;
		int x_start = currx + 4;
		int x_end = currx - 9;
		for (y; y - curry >= -18; y--) {

			for (int x = x_start; x >= x_end; x--) {
				if (!map->isFree(x, y)) {
					return 0; //collision
				}

			}
			x_start--;
			x_end--;

		}

		int y_new = curry - 19;
		x_start = currx - 3;
		x_end = currx - 15;

		for (y_new; y_new - curry >= -24; y_new--) {


			for (int x = x_start; x >= x_end; x--) {
				if (!map->isFree(x, y)) {
					return 0; //collision
				}

			}
			x_start--;
			x_end++;

		}
		return 1;

		}

		//for heading 270 and backwards
		else if (dir == 3 && heading == 270) {
		for (int i = curry + 8; i <= curry + 9; i++) {
			for (int j = currx - 4; j <= currx + 4; j++) {
				if (!map->isFree(j, i)) {
					return 0; //collision if map is not free
				}
			}
		}
		return 1;
		}


		// for heading 270 few steps forward
		else if (dir == 4 && heading == 270) {
		for (int i = curry - 8; i >= curry - fwd_step-9; i--) {
			for (int j = currx - 4; j <= currx + 4; j++) {
				if (!map->isFree(j, i)) {
					return 0; //collision if map is not free
				}
			}
		}
		return 1;
		}


		//for heading 315 and forward
		else if (dir == 0 && heading == 315) {
		int j = curry - 10;
		for (int i = currx + 4; i <= currx + 10; i++) {
			if (!map->isFree(i, j)) {
				return 0; //collision
			}
			j++;
		}
		return 1;
		
		}

		//for heading 315 and ccw turn
		else if(dir ==1 && heading ==315){
		for (int i = currx + 3; i <= currx + 23; i++) {
			for (int j = curry - 2; j >= curry - 10; j--) {
				if (!map->isFree(i, j)) {
					return 0; //collision
				}
			}
		}
		return 1;
		}

		//for heading 315 and cw turn
		else if (dir == 2 && heading == 315) {
		for (int i = currx + 2; i <= currx + 10; i++) {
			for (int j = curry - 4; j >= curry - 23; j--) {
				if (!map->isFree(i, j)) {
					return 0; //collision
				}
			}
		}
		return 1;
		}

		//for heading 315 and backwards
		else if (dir == 3 && heading == 315) {
		int j = curry + 10;
		for (int i = currx - 4; i >= currx - 10; i--) {
			if (!map->isFree(i, j)) {
				return 0; //collision
			}
			j--;
		}
		return 1;
		}

		//for heading 315 and few steps forward
		else if (dir == 4 and heading == 315) {
			for (currx; currx <= currx + fwd_step; currx++) {
				int j = curry - 10;
				for (int i = currx + 4; i <= currx + 10; i++) {
					if (!map->isFree(i, j)) {
						return 0; //collision
					}
					j++;
				}
				curry--;
			}
		return 1;

		}


	}
	

};
