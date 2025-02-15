#include <limits>
#include <cmath>
#include <memory>
#include <set>

struct state {
    int idx;
    int g;
    int h;
    bool closed;
    state* parent;

    state() : idx(-1), g(std::numeric_limits<int>::max()), h(0), closed(false), parent(nullptr) {}
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