#include <limits>
#include <cmath>
#include <memory>
#include <set>

struct state {
    int idx = -1;
    int g = std::numeric_limits<int>::max();
    int h = 0;
    bool closed = false;
    double w = 1;
    struct CompareParents {
        bool operator()(const state* a, const state* b) const {
            return a->g < b->g;  // Sort parents in ascending order of g-value
        }
    };
    std::set<state*, CompareParents> parents;
};

inline int getx(int idx, int x_size) {
    int x = idx%x_size + 1;
    return x;
}

inline int gety(int idx, int x_size) {
    int y = idx/x_size + 1;
    return y;
}