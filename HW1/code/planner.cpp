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

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

int w = 3;

struct CompareStatePtr {
    bool operator()(const state* a, const state* b) const {
        return (a->g + w*a->h) > (b->g + w*b->h);
    }
};

// int find_closest_valid_goal(int* target_traj, int target_steps, int robotposeX, int robotposeY, int* map, int x_size, int y_size, int collision_thresh) {
//     int best_idx = 0;
//     int best_dist = std::numeric_limits<int>::max();
//     int d = 1;
//     int d_ = 1.4;
//     float safety_factor = 1.4;
    
//     for (int i = 0; i < target_steps; ++i) {
//         int x_target = target_traj[i];
//         int y_target = target_traj[i + target_steps];
        
//         if (x_target >= 1 && x_target <= x_size && y_target >= 1 && y_target <= y_size) {
//             int cost = map[GETMAPINDEX(x_target, y_target, x_size, y_size)];
            
//             if (cost < collision_thresh) {
//                 int dist = d * std::max(abs(robotposeX - x_target), abs(robotposeY - y_target)) 
//                          + (d_ - d) * std::min(abs(robotposeX - x_target), abs(robotposeY - y_target));
                
//                 dist = static_cast<int>(dist * safety_factor);

//                 if (dist < i) {
//                     if (dist < dist) {
//                         best_idx = i;
//                         best_dist = dist;
//                     }
//                 }
//             }
//         }
//     }
//     return best_idx;
// }

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

    int robotposeX_, robotposeY_;

    if(curr_time == 0) {
        // 8-connected grid
        int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
        int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
        
        int d = 1;
        int d_ = 1.4;
        
        int shortest_path_length = std::numeric_limits<int>::max();

        for(int idx_target = target_steps/2; idx_target < target_steps; ++idx_target) {
            
            int goalposeX = target_traj[idx_target];
            int goalposeY = target_traj[idx_target+target_steps];
            
            std::vector<state> states;
            states.resize(x_size*y_size);

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
                        states[idx_new].idx_parent = idx;
                        states[idx_new].closed = false;
                        open.push(&states[idx_new]);
                    }
                }
                states[idx].closed = true;
            }

            std::queue<state*> reversed_path;
            reversed_path.push(goal_state);
            state* next = goal_state;

            while (reversed_path.back()->idx_parent != first.idx) {
                next = reversed_path.back();
                reversed_path.push(&states[next->idx_parent]);
            }

            // if(reversed_path.size() <= (target_steps-(idx_target)) && reversed_path.size() < shortest_path_length) {
            if(reversed_path.size() <= (idx_target+5) && reversed_path.size() < shortest_path_length) {
                shortest_path_length = reversed_path.size();
                next = reversed_path.back();
                robotposeX_ = getx(next->idx, x_size);
                robotposeY_ = gety(next->idx, x_size);  
                savePathToFile(reversed_path, x_size);
            }
        }
    }
    else {
        if (!loadNextPose(robotposeX, robotposeY, curr_time)) {
            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
            
            return;
        }
    
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
        auto stop_code = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop_code - start_code);
        std::cout << "Total run time: " << duration.count() << " ms" << std::endl;
        
        return;
    }

    robotposeX = robotposeX_;
    robotposeY = robotposeY_;

    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;

    auto stop_code = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop_code - start_code);
    std::cout << "Total run time: " << duration.count() << " ms" << std::endl;
    
    return;
}

