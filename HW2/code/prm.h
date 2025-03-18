#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>
#include <queue>

#include "utils.h"

class PRM_Planner {
public:
    int connections = 10;
    int x_size, y_size;
    int numofDOFs;
    double *map;
    std::mt19937 generator;

    PRM_Planner(int x_size, int y_size, int numofDOFs, double *map) {
        this->x_size = x_size;
        this->y_size = y_size;
        this->numofDOFs = numofDOFs;
        this->map = map;
        
        std::random_device rd;
        generator = std::mt19937(rd());
    }

    node new_node() {
        std::uniform_real_distribution<double> distribution(0.0, 2*PI);

        node n;
        int max_attempts = 1000;
        int attempts = 0;

        while (attempts < max_attempts) {
            attempts++;
            n.angles.clear();
            for (int i = 0; i < numofDOFs; ++i) {
                n.angles.push_back(distribution(generator));
            }
            
            if (IsValidArmConfiguration(n.angles.data(), numofDOFs, map, x_size, y_size)) {
                return n;
            }
        }
        throw std::runtime_error("Failed to find a valid random node.");

        return n;
    }

    std::vector<int> find_neighbors(std::vector<node>& graph, int id, int k) {
        std::vector<int> nearest_neighbors;
        if (graph.size() <= 1) {
            return nearest_neighbors;
        }
    
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;
    
        for (size_t i = 0; i < graph.size(); ++i) {
            if (graph[i].id != id) {
                pq.emplace(distance(graph[id], graph[i]), graph[i].id);
            }
        }
    
        while (!pq.empty() && nearest_neighbors.size() < k) {
            nearest_neighbors.push_back(pq.top().second);
            pq.pop();
        }
    
        return nearest_neighbors;
    }


    void build_roadmap(std::vector<node>& graph, int max_nodes) {
        int n = 0;
        const int k = 100;
    
        while (n < max_nodes) {
            node sample_node = new_node();
    
            sample_node.id = n;
            graph.push_back(sample_node);
            int id = n;
            n++;

            std::vector<int> nearest_neighbor_ids = find_neighbors(graph, id, k);

            for (int neighbor_id : nearest_neighbor_ids) {
                node& neighbor_node = graph[neighbor_id];

                if (obstacle_free(graph[id], graph[neighbor_id], numofDOFs, x_size, y_size, map) && 
                    neighbor_node.neighbors.size() < connections) {

                    double dist = distance(graph[id], neighbor_node);
                    graph[id].neighbors.emplace_back(neighbor_id, dist);
                    neighbor_node.neighbors.emplace_back(id, dist);
                }
            }
        }
    }


    void query(std::vector<node>& graph, double* armstart_anglesV_rad, double* armgoal_anglesV_rad, int num_nodes) {
        const int k_q = num_nodes;

        node q_init, q_goal;

        q_init.angles.assign(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
        q_goal.angles.assign(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);

        q_init.id = num_nodes;
        q_goal.id = num_nodes + 1;

        q_init.g = 0;

        graph.emplace_back(std::move(q_init));
        graph.emplace_back(std::move(q_goal));

        // Connect q_init to k nearest neighbors
        std::vector<int> init_neighbors = find_neighbors(graph, num_nodes, k_q);
        for (int neighbor_id : init_neighbors) {
            node& neighbor_node = graph[neighbor_id];

            if (obstacle_free(graph[num_nodes], neighbor_node, numofDOFs, x_size, y_size, map)) {

                double dist = distance(graph[num_nodes], neighbor_node);
                graph[num_nodes].neighbors.emplace_back(neighbor_id, dist);
                neighbor_node.neighbors.emplace_back(num_nodes, dist);
            }
        }

        std::cout << "Start has: " << graph[num_nodes].neighbors.size() << " neighbors" << std::endl;

        // Connect q_goal to k nearest neighbors
        std::vector<int> goal_neighbors = find_neighbors(graph, num_nodes + 1, k_q);
        for (int neighbor_id : goal_neighbors) {
            node& neighbor_node = graph[neighbor_id];

            if (obstacle_free(graph[num_nodes + 1], neighbor_node, numofDOFs, x_size, y_size, map)) {

                double dist = distance(graph[num_nodes + 1], neighbor_node);
                graph[num_nodes + 1].neighbors.emplace_back(neighbor_id, dist);
                neighbor_node.neighbors.emplace_back(num_nodes + 1, dist);
            }
        }

        std::cout << "Goal has: " << graph[num_nodes+1].neighbors.size() << " neighbors" << std::endl;
    }

    std::vector<int> dijkstra(std::vector<node> &graph, int q_init_ID, int q_goal_ID) {    
        if (q_init_ID < 0 || q_init_ID >= graph.size() || q_goal_ID < 0 || q_goal_ID >= graph.size()) {
            std::cout << "Invalid start or goal node!" << std::endl;
            return {};
        }
    
        using NodePair = std::pair<double, int>;
        std::priority_queue<NodePair, std::vector<NodePair>, std::greater<NodePair>> open;

        graph[q_init_ID].g = 0;
        open.push({0.0, q_init_ID});
    
        while (!open.empty()) {
            int current_id = open.top().second;
            open.pop();
    
            // If already visited, skip
            if (graph[current_id].closed)
                continue;
            graph[current_id].closed = true;
    
            // If goal is reached, stop search
            if (current_id == q_goal_ID)
                break;
    
            for (auto& [next_id, edge_cost] : graph[current_id].neighbors) {
                if (graph[next_id].closed)
                    continue;
    
                double c_new = graph[current_id].g + edge_cost;
                if (c_new < graph[next_id].g) {
                    graph[next_id].g = c_new;
                    graph[next_id].parent = current_id;
                    open.push({c_new, next_id});
                }
            }
        }
    
        // If no path to goal was found
        if (graph[q_goal_ID].g == std::numeric_limits<double>::infinity()) {
            std::cout << "No path found to the goal node." << std::endl;
            return {};
        }
    
        // Reconstruct path from goal to start
        std::vector<int> path;
        for (int at = q_goal_ID; at >= 0; at = graph[at].parent) {
            path.push_back(at);
        }
        std::reverse(path.begin(), path.end());
    
        std::vector<int> shortcut_path = shortcutting(graph, path);
        
        return shortcut_path;
    }

    std::vector<int> shortcutting(std::vector<node>& graph, std::vector<int>& path) {
        // Perform shortcutting to reduce unnecessary waypoints
        std::vector<int> shortcut_path;
        int current = 0;
        shortcut_path.push_back(path[current]);

        while (current < path.size() - 1) {
            int next = current + 1;
            while (next < path.size() - 1 && obstacle_free(graph[path[current]], graph[path[next + 1]], numofDOFs, x_size, y_size, map)) {
                next++;
            }
            shortcut_path.push_back(path[next]);
            current = next;
        }

        return shortcut_path;
    }
};