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
#include <chrono>
#include <filesystem>
#include <utilities.h>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

int w = 1;

struct CompareStatePtr {
    bool operator()(const state* a, const state* b) const {
        return (a->g + w*a->h) > (b->g + w*b->h);
    }
};

int find_closest_valid_goal(int* target_traj, int target_steps, int robotposeX, int robotposeY, int* map, int x_size, int y_size, int collision_thresh) {
    int best_idx = 0;
    int best_dist = std::numeric_limits<int>::max();
    int d = 1;
    int d_ = 1.4;
    float safety_factor = 1;
    
    for (int i = 0; i < target_steps; ++i) {
        int x_target = target_traj[i];
        int y_target = target_traj[i + target_steps];
        
        if (x_target >= 1 && x_target <= x_size && y_target >= 1 && y_target <= y_size) {
            int cost = map[GETMAPINDEX(x_target, y_target, x_size, y_size)];
            
            if (cost < collision_thresh) {
                int dist = d * std::max(abs(robotposeX - x_target), abs(robotposeY - y_target)) 
                         + (d_ - d) * std::min(abs(robotposeX - x_target), abs(robotposeY - y_target));
                
                dist = static_cast<int>(dist * safety_factor);

                if (dist < i) {
                    if (dist < best_dist) {
                        best_idx = i;
                        best_dist = dist;
                    }
                }
            }
        }
    }
    return best_idx;
}

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

    if (curr_time > 0) {
        if (!loadNextPose(robotposeX, robotposeY, curr_time)) {
            std::cerr << "Failed to load next pose! Using robot's current pose." << std::endl;
            
            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
            return;
        }
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        return;
    }

    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
    int d = 1;
    int d_ = 1.4;

    // catch target at a point based on the current distance to the target
    int idx_target = find_closest_valid_goal(target_traj, target_steps, robotposeX, robotposeY, map, x_size, y_size, collision_thresh);

    int goalposeX = target_traj[idx_target];
    int goalposeY = target_traj[idx_target+target_steps];
    std::cout << "goal: " << goalposeX << " " << goalposeY << std::endl;
    
    std::vector<state> states;
    states.resize(x_size*y_size);
    
    auto start = std::chrono::high_resolution_clock::now();
    
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Initialisation time: " << duration.count() << " ms" << std::endl;
    start = std::chrono::high_resolution_clock::now();

    state first;
    first.idx = getidx(robotposeX, robotposeY, x_size);
    first.g = 0;
    first.h = d * std::max(abs(robotposeX - goalposeX), abs(robotposeY - goalposeY)) + (d_ - d) * std::min(abs(robotposeX - goalposeX), abs(robotposeY - goalposeY));
    first.closed = false;
    states[first.idx] = first;

    state goal;
    goal.idx = getidx(goalposeX, goalposeY, x_size);
    goal.g = std::numeric_limits<int>::max();
    goal.h = 0;
    goal.closed = false;
    states[goal.idx] = goal;

    state* goal_state = &states[goal.idx];

    std::priority_queue<state*, std::vector<state*>, CompareStatePtr> open;
    open.push(&first);

    while (!open.empty()) {
        state* current = open.top();
        open.pop();

        int idx = current->idx;

        if (getx(current->idx, x_size) == goalposeX && gety(current->idx, x_size) == goalposeY) {
            goal_state = current;
            break;
        }

        for (int dir = 0; dir < NUMOFDIRS; ++dir) {
            int x_new = getx(idx, x_size) + dX[dir];
            int y_new = gety(idx, x_size) + dY[dir];

            if (x_new < 1 || x_new > x_size || y_new < 1 || y_new > y_size)
                continue;

            int idx_new = getidx(x_new, y_new, x_size);

            int cost = int(map[GETMAPINDEX(x_new, y_new, x_size, y_size)]);
            if (states[idx_new].closed || cost >= collision_thresh)
                continue;

            double move_cost = (dX[dir] == 0 || dY[dir] == 0) ? d : d_;
            int new_g = current->g + move_cost + cost;
            int new_h = d*std::max(abs(x_new-goalposeX), abs(y_new-goalposeY)) + (d_-d)*std::min(abs(x_new-goalposeX), abs(y_new-goalposeY));

            states[idx_new].idx = idx_new;
            states[idx_new].h = new_h;

            if (states[idx_new].g > new_g) {
                states[idx_new].g = new_g;
                states[idx_new].parent = current;
                states[idx_new].closed = false;
                open.push(&states[idx_new]);
            }
        }
        states[idx].closed = true;
    }

    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Pathfinding time: " << duration.count() << " ms" << std::endl;

    std::queue<state*> path;
    path.push(goal_state);
    state* next = goal_state;

    start = std::chrono::high_resolution_clock::now();

    int x_first = getx(first.idx, x_size);
    int y_first = gety(first.idx, x_size);

    while(getx(path.back()->idx, x_size) != x_first || gety(path.back()->idx, x_size) != y_first) {
        next = path.back();
        path.push(next->parent);
    }

    savePathToFile(path, x_size);

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

