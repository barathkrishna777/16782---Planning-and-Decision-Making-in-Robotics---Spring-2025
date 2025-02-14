/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "planner.h"
#include <math.h>
#include <vector>
#include "utils.h"
#include <queue>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <unordered_map>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8


void initialise(std::vector<state>& states, int goalposeX, int goalposeY, int x_size, int y_size, int collision_thresh, int* map, int finalposeX, int finalposeY) {
    int n = x_size * y_size;
    states.resize(n);
    
    std::filesystem::create_directories("precompute");
    std::string filename = "precompute/" + std::to_string(x_size) + "_" + std::to_string(y_size) + "_" + std::to_string(finalposeX) + "_" + std::to_string(finalposeY) + ".dat";

    if (std::filesystem::exists(filename)) {
        std::cout << "Reading data from binary file: " << filename << std::endl;
        std::ifstream infile(filename, std::ios::binary);
        if (!infile) {
            std::cerr << "Error opening file for reading." << std::endl;
            return;
        }

        int d = 1;
        int d_ = 1.4;

        for (int count = 0; count < n; ++count) {
            states[count].idx = count;
            infile.read(reinterpret_cast<char*>(&states[count].closed), sizeof(states[count].closed));
            states[count].h = d * std::max(abs(getx(count, x_size) - goalposeX), abs(gety(count, x_size) - goalposeY)) + (d_ - d) * std::min(abs(getx(count, x_size) - goalposeX), abs(gety(count, x_size) - goalposeY));
        }
    } 
    else {
        int d = 1, d_ = 1.4;
        std::cout << "Writing data to binary file: " << filename << std::endl;
        std::ofstream outfile(filename, std::ios::binary);
        if (!outfile) {
            std::cerr << "Error opening file for writing." << std::endl;
            return;
        }

        for (int i = 0; i < n; ++i) {
            int x = getx(i, x_size);
            int y = gety(i, x_size);
            
            states[i].idx = i;
            if (int(map[GETMAPINDEX(x, y, x_size, y_size)]) >= collision_thresh) {
                states[i].closed = true;
            }
            states[i].h = d * std::max(abs(x - goalposeX), abs(y - goalposeY)) + (d_ - d) * std::min(abs(x - goalposeX), abs(y - goalposeY));
            outfile.write(reinterpret_cast<const char*>(&states[i].closed), sizeof(states[i].closed));
        }
    }
}

struct CompareStatePtr {
    bool operator()(const state* a, const state* b) const {
        return (a->g + a->w * a->h) > (b->g + b->w * b->h);
    }
};

void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr
    )
{
    auto start_code = std::chrono::high_resolution_clock::now();
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
    int d = 1;
    int d_ = 1.4;

    // catch target at a point based on the current distance to the target
    int targetposeX_ = targetposeX;
    int targetposeY_ = targetposeY;
    int dist = sqrt(pow(robotposeX-targetposeX_, 2) + pow(robotposeY-targetposeY_, 2));
    int final_dist = sqrt(pow(robotposeX-target_traj[target_steps-1], 2) + pow(robotposeY-target_traj[target_steps-1+target_steps], 2));
    int max_dist = sqrt(pow(x_size, 2) + pow(y_size, 2));
    
    if (final_dist >= max_dist/3) {
        targetposeX_ = abs(targetposeX + robotposeX)/2;
        targetposeY_ = abs(targetposeY + robotposeY)/2;
    }
    else {
        targetposeX_ = target_traj[target_steps-1];
        targetposeY_ = target_traj[target_steps-1+target_steps];
    }
    if(target_traj[0] == target_traj[target_steps-1] && target_traj[0+target_steps] == target_traj[target_steps-1+target_steps]) {
        targetposeX_ = target_traj[target_steps-1];
        targetposeY_ = target_traj[target_steps-1+target_steps];
        std::cout << "target not moving, heading towards end pose" << std::endl;
    }

    int goalposeX = targetposeX_;
    int goalposeY = targetposeY_;
    std::cout << "goal: " << goalposeX << " " << goalposeY << std::endl;
    
    std::vector<state> states;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    initialise(states, goalposeX, goalposeY, x_size, y_size, collision_thresh, map, target_traj[target_steps-1], target_traj[target_steps-1+target_steps]);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Initialisation time: " << duration.count() << " ms" << std::endl;
    start = std::chrono::high_resolution_clock::now();

    state first;
    first = states[robotposeX-1 + (robotposeY-1)*x_size];
    first.g = 0;
    first.closed = false;

    int idx_target = goalposeX-1 + (goalposeY-1)*x_size;

    state* goal;
    goal = &states[idx_target];

    std::priority_queue<state*, std::vector<state*>, CompareStatePtr> open;
    open.push(&first);

    while (!open.empty() && goal->closed == false) {
        state* current = open.top();
        open.pop();

        int idx = current->idx;

        for (int dir = 0; dir < NUMOFDIRS; ++dir) {
            int x_new = getx(idx, x_size) + dX[dir];
            int y_new = gety(idx, x_size) + dY[dir];

            if (x_new < 1 || x_new > x_size || y_new < 1 || y_new > y_size)
                continue;

            int idx_new = x_new - 1 + (y_new - 1) * x_size;

            int cost = int(map[GETMAPINDEX(x_new, y_new, x_size, y_size)]);
            if (states[idx_new].closed || cost >= collision_thresh)
                continue;

            int move_cost = (dX[dir] == 0 || dY[dir] == 0) ? d : d_;
            int new_g = current->g + move_cost + cost;

            if (states[idx_new].g > new_g) {
                states[idx_new].g = new_g;
                states[idx_new].parents.insert(current);
                open.push(&states[idx_new]);
            }
            // if (states[idx_new].x == goalposeX && states[idx_new].y == goalposeY)
            //     break;
        }
        states[idx].closed = true;
    }

    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Pathfinding time: " << duration.count() << " ms" << std::endl;

    std::queue<state*> path;
    path.push(goal);
    state* next = goal;

    start = std::chrono::high_resolution_clock::now();

    int x_first = getx(first.idx, x_size);
    int y_first = gety(first.idx, x_size);

    while(getx(path.back()->idx, x_size) != x_first || gety(path.back()->idx, x_size) != y_first) {
        if (path.back()->parents.empty()) {
            std::cout << getx(path.back()->idx, x_size) << " " << gety(path.back()->idx, x_size) << std::endl;
            std::cerr << "Error: Parent pointer is nullptr!" << std::endl;
            break;
        }
        next = path.back();
        path.push(*next->parents.begin());
    }

    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Path reconstruction time: " << duration.count() << " ms" << std::endl;

    robotposeX = getx(next->idx, x_size);
    robotposeY = gety(next->idx, x_size);

    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;

    auto stop_code = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop_code - start_code);
    std::cout << "Total run time: " << duration.count() << " ms" << std::endl;
    start = std::chrono::high_resolution_clock::now();
    
    return;
}

