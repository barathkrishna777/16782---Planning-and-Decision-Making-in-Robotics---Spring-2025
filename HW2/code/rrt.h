#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>
#include <queue>

#include "utils.h"

class RRT_Planner {
public:
    int x_size, y_size;
    int numofDOFs;
    double *map;
    double eps;
    std::mt19937 generator;
    int goal_id = -1;

    RRT_Planner(int x_size, int y_size, int numofDOFs, double *map, double eps) {
        this->x_size = x_size;
        this->y_size = y_size;
        this->numofDOFs = numofDOFs;
        this->map = map;
        this->eps = eps;
        std::random_device rd;
        generator = std::mt19937(rd());
    }


    node new_node(int numofDOFs, double* armgoal_anglesV_rad, double goal_bias_prob = 0.01) {
        std::uniform_real_distribution<double> distribution(0.0, 2 * PI);
        std::uniform_real_distribution<double> bias_distribution(0.0, 1.0);
    
        node n;
        int max_attempts = 1000;
        int attempts = 0;
    
        while (attempts < max_attempts) {
            attempts++;
            if (bias_distribution(generator) < goal_bias_prob) {
                n.angles.assign(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
            } 
            else {
                n.angles.clear();
                for (int i = 0; i < numofDOFs; ++i) {
                    n.angles.push_back(distribution(generator));
                }
            }
            
            if (IsValidArmConfiguration(n.angles.data(), numofDOFs, map, x_size, y_size)) {
                return n;
            }
        }
        throw std::runtime_error("Failed to find a valid random node.");
    }

    double distance(node& n1, node& n2) {
        double dist = 0;
        for (int i = 0; i < n1.angles.size(); ++i) {
            dist += pow(n1.angles[i] - n2.angles[i], 2);
        }
        return sqrt(dist);
    }

    void build_tree(std::vector<node>& tree, double* armstart_anglesV_rad, double* armgoal_anglesV_rad, const int& max_nodes) {
        int n = 0;
        tree.clear();
    
        node q_init;
        q_init.id = 0;
        q_init.angles.assign(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
        q_init.g = 0;
        tree.push_back(q_init);
    
        int goal_id = -1;
    
        while (n < max_nodes) {
            node q_rand = new_node(numofDOFs, armgoal_anglesV_rad);
            std::cout << "Sampled node " << n << std::endl;
            int status = extend(tree, q_rand, armgoal_anglesV_rad);
    
            if (status == 0)  // Trapped
                continue;
            else {  // Advanced or Reached
                ++n;
            }
        }
    }

    int extend(std::vector<node>& tree, node& q_rand, double* armgoal_anglesV_rad) {
        int nearest_node_id = nearest_neighbor(tree, q_rand);
        node q_extended = interpolate_eps(tree, nearest_node_id, q_rand);
    
        if (q_extended.id == -1) {  
            return 0;
        }
    
        double reached_distance = distance(q_extended, q_rand);

        q_extended.id = tree.size();
        tree.push_back(q_extended);
        tree[nearest_node_id].neighbors.emplace_back(q_extended.id, distance(tree[nearest_node_id], q_extended));
        tree[q_extended.id].neighbors.emplace_back(nearest_node_id, distance(tree[nearest_node_id], q_extended));
    
        // Check if we fully reached the target
        if (reached_distance < 1e-3) {
            return 2; // Reached
        }
    
        return 1; // Advanced
    }

    node interpolate_eps(std::vector<node>& tree, int id, node n) {
        double dist = distance(tree[id], n);
        dist = std::min(dist, eps);
    
        int numofsamples = std::max(1, (int)(dist / (PI / 20)));
    
        std::vector<double> config(numofDOFs);
        std::vector<double> prev_config = tree[id].angles;
    
        for (int i = 0; i < numofsamples; i++) {
            for (int j = 0; j < numofDOFs; j++) {
                config[j] = tree[id].angles[j] + ((double)(i) / (numofsamples - 1)) * (n.angles[j] - tree[id].angles[j]);
            }
    
            if (!IsValidArmConfiguration(config.data(), numofDOFs, map, x_size, y_size)) {
                if (i == 0) {
                    node invalid_node;
                    invalid_node.id = -1;
                    return invalid_node;
                } 
                else {
                    config = prev_config;
                    break;
                }
            }
            prev_config = config;
        }
        
        node interpolated_node;
        interpolated_node.id = tree.size();
        interpolated_node.angles = config;
    
        return interpolated_node;
    }

    int nearest_neighbor(std::vector<node>& tree, node& q_rand) {
        double min_dist = std::numeric_limits<double>::infinity();
        int nearest_node_id = -1;

        for (int i = 0; i < tree.size(); ++i) {
            double dist = distance(tree[i], q_rand);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_node_id = i;
            }
        }
        return nearest_node_id;
    }

    std::vector<int> dijkstra(std::vector<node>& tree, double* armgoal_anglesV_rad) {
        node n;
        n.angles.assign(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
        int q_goal_ID = nearest_neighbor(tree, n);
        
        using NodePair = std::pair<double, int>;
        std::priority_queue<NodePair, std::vector<NodePair>, std::greater<NodePair>> open;
    
        for (auto& node : tree) {
            node.g = std::numeric_limits<double>::infinity();
            node.closed = false;
            node.parent = -1;
        }
    
        tree[0].g = 0;
        open.push({0.0, 0});
    
        while (!open.empty()) {
            int current_id = open.top().second;
            open.pop();
    
            if (tree[current_id].closed)
                continue;
            tree[current_id].closed = true;
    
            if (current_id == q_goal_ID) {
                std::cout << "Goal reached!" << std::endl;
                break;
            }
    
            for (auto& [next_id, edge_cost] : tree[current_id].neighbors) {
                if (tree[next_id].closed)
                    continue;
    
                double tentative_dist = tree[current_id].g + edge_cost;
                if (tentative_dist < tree[next_id].g) {
                    tree[next_id].g = tentative_dist;
                    tree[next_id].parent = current_id;
                    open.push({tentative_dist, next_id});
                }
            }
        }
    
        if (tree[q_goal_ID].g == std::numeric_limits<double>::infinity()) {
            std::cout << "No path found to the goal node." << std::endl;
            return {};
        }
    
        std::vector<int> path;
        for (int at = q_goal_ID; at != -1; at = tree[at].parent) {
            path.push_back(at);
        }
        std::reverse(path.begin(), path.end());

        node n_goal;
        n_goal.angles.assign(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
        double dist = distance(tree[path.back()], n_goal);
        std::cout << "Distance to goal: " << dist << std::endl;
        n_goal.id = tree.size();
        tree.push_back(n_goal);
        path.push_back(n_goal.id);
    
        std::vector<int> shortcut_path = shortcutting(tree, path);
        
        return shortcut_path;
    }

    std::vector<int> shortcutting(std::vector<node>& tree, std::vector<int>& path) {
        // Perform shortcutting to reduce unnecessary waypoints
        std::vector<int> shortcut_path;
        int current = 0;
        shortcut_path.push_back(path[current]);

        while (current < path.size() - 1) {
            int next = current + 1;
            while (next < path.size() - 1 && obstacle_free(tree[path[current]], tree[path[next + 1]])) {
                next++;
            }
            shortcut_path.push_back(path[next]);
            current = next;
        }

        return shortcut_path;
    }
    
    bool obstacle_free(node n1, node n2) {
        double dist = distance(n1, n2);
        int numofsamples = std::max(1, (int)(dist / (PI / 20)));

        std::vector<double> config(numofDOFs);
        for (int i = 0; i < numofsamples; i++) {
            for (int j = 0; j < numofDOFs; j++)
                config[j] = n1.angles[j] + ((double)(i) / (numofsamples - 1)) * (n2.angles[j] - n1.angles[j]);
    
            if (!IsValidArmConfiguration(config.data(), numofDOFs, map, x_size, y_size))
                return false;
        }
        
        return true;
    }

    void visualize_tree(const std::vector<node>& tree, const std::string& filename) {
        std::ofstream ofs(filename);
        if (!ofs.is_open()) {
            std::cerr << "Failed to open file for visualization: " << filename << std::endl;
            return;
        }
        ofs << "digraph RRTTree {" << std::endl;
        // Optionally, label nodes with their ID (and add extra info if desired)
        for (const auto & n : tree) {
            ofs << "  node" << n.id << " [label=\"ID:" << n.id;
            if (n.id == 0)
                ofs << " (Start)\"";
            // If the goal node is the last one added, mark it:
            else if (n.id == tree.back().id)
                ofs << " (Goal)\"";
            else
                ofs << "\"";
            ofs << "];" << std::endl;
    
            // Output edges; here we only output one direction (avoid duplicates)
            for (const auto & neighbor : n.neighbors) {
                if (n.id < neighbor.first) {
                    ofs << "  node" << n.id << " -> node" << neighbor.first 
                        << " [label=\"" << neighbor.second << "\"];" << std::endl;
                }
            }
        }
        ofs << "}" << std::endl;
        ofs.close();
        std::cout << "Visualization written to " << filename << std::endl;
    }
};