#include <limits>
#include <cmath>
#include <memory>
#include <set>
#include <queue>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stack>

#define PATH_FILE "planned_path.txt"

struct state {
    int idx;
    int g;
    int h;
    bool closed;
    int idx_parent;

    state() : idx(-1), g(std::numeric_limits<int>::max()), h(0), closed(false), idx_parent(-1) {}
};

inline int getx(int idx, int x_size) {
    int x = idx%x_size + 1;
    return x;
}

inline int gety(int idx, int x_size) {
    int y = idx/x_size + 1;
    return y;
}

inline int getidx(int x, int y, int x_size) {
    return x-1 + (y-1)*x_size;
}

void savePathToFile(std::queue<state*> path, int x_size) {
    std::ofstream file(PATH_FILE);
    if (!file) {
        std::cerr << "Error opening file for writing!" << std::endl;
        return;
    }

    std::stack<state*> reverse_stack;
    
    while (!path.empty()) {
        reverse_stack.push(path.front());
        path.pop();
    }

    int count = 0;
    while (!reverse_stack.empty()) {
        state* s = reverse_stack.top();
        reverse_stack.pop();
        int x = getx(s->idx, x_size);
        int y = gety(s->idx, x_size);

        file << std::to_string(count) << "\n" << x << ", " << y << "\n";
        count++;
    }

    file.close();
}

bool loadNextPose(int& next_x, int& next_y, int curr_time) {
    std::ifstream file(PATH_FILE);
    if (!file.is_open()) {
        std::cerr << "Error opening path file for reading!" << std::endl;
        return false;
    }

    std::string line;
    int file_time;
    bool found_timestamp = false;

    while (std::getline(file, line)) {
        if (line.find(',') != std::string::npos) {
            if (found_timestamp) {
                std::stringstream ss(line);
                char comma;
                if (ss >> next_x >> comma >> next_y && comma == ',') {
                    return true;
                }
            }
        } 
        else {
            try {
                file_time = std::stoi(line);
            } 
            catch (const std::exception&) {
                continue;
            }

            if (file_time == curr_time) {
                found_timestamp = true;
            } else {
                found_timestamp = false;
            }
        }
    }
    return false;
}