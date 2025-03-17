#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>
#include <queue>
#include <cmath>

#include "utils.h"

class RRT_Star_Planner {
public:
    int x_size, y_size;
    int numofDOFs;
    double *map, *armgoal_anglesV_rad;
    double eps;
    std::mt19937 generator;
    int goal_id = -1;

    RRT_Star_Planner(int x_size, int y_size, int numofDOFs, double *map, double eps, double* armgoal_anglesV_rad) {
        this->x_size = x_size;
        this->y_size = y_size;
        this->numofDOFs = numofDOFs;
        this->map = map;
        this->eps = eps;
        this->armgoal_anglesV_rad = armgoal_anglesV_rad;
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
    
        double min_dist_to_goal = std::numeric_limits<double>::max();
        node q_goal;
        q_goal.angles.assign(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
    
        while (n < max_nodes) {
            node q_rand = new_node(numofDOFs, armgoal_anglesV_rad);
            std::cout << "Sampled node " << n << std::endl;
            int status = extend(tree, q_rand, armgoal_anglesV_rad);
    
            if (status == 0)  // Trapped
                continue;
            else {  // Advanced or Reached
                ++n;
            }
            if (distance(tree.back(), q_goal) < min_dist_to_goal) {
                min_dist_to_goal = distance(tree.back(), q_goal);
                goal_id = tree.back().id;
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

        double dist = distance(tree[nearest_node_id], q_extended);

        q_extended.id = tree.size();
        tree.push_back(q_extended);
        tree[nearest_node_id].neighbors.emplace_back(q_extended.id, dist);
        tree[q_extended.id].neighbors.emplace_back(nearest_node_id, dist);
        tree[q_extended.id].parent = nearest_node_id;

        double c_new = tree[nearest_node_id].g + dist;
        if (c_new < tree[q_extended.id].g) {
            tree[q_extended.id].g = c_new;
        }

        rewire(tree);

        // Check if we fully reached the target
        if (reached_distance < 1e-3) {
            return 2; // Reached
        }
    
        return 1; // Advanced
    }

    node interpolate_eps(std::vector<node>& tree, int id, node n) {
        double dist = distance(tree[id], n);
    
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

    void rewire(std::vector<node>& tree) {
        int n = tree.size();
        double delta = std::pow(PI, numofDOFs/2.0) / std::tgamma(1 + numofDOFs/2.0);
        double gamma = 2 * std::pow(1 + 1.0/numofDOFs, 1.0/numofDOFs) * std::pow(1.0/delta, 1.0/numofDOFs);
        
        double r = std::pow((gamma/delta) * std::log(fabs(n))/n, 1.0/numofDOFs);
        r = std::min(r, eps);

        int id = tree.back().id;

        std::vector<std::pair<int, double>> neighbors = find_neighbors(tree, id, r);

        for (auto neighbor : neighbors) {
            if(obstacle_free(tree[id], tree[neighbor.first])) {
                double c_new = tree[neighbor.first].g + neighbor.second;
                if(c_new < tree[id].g) {
                    tree[id].g = c_new;
                    tree[id].parent = neighbor.first;
                }
            }
        }
        for (auto neighbor : neighbors) {
            if(neighbor.first != tree[id].parent && obstacle_free(tree[id], tree[neighbor.first])) {
                double c_new = tree[id].g + neighbor.second;
                if (c_new < tree[neighbor.first].g) {
                    tree[neighbor.first].g = c_new;
                    tree[neighbor.first].parent = id;
                }
            }
        }
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

    std::vector<std::pair<int, double>> find_neighbors(std::vector<node>& tree, int id, double r) {
        std::vector<std::pair<int, double>> nearest_neighbors;

        if (tree.size() <= 1) {
            return nearest_neighbors;
        }
        nearest_neighbors.reserve(tree.size() - 1);
    
        for (size_t i = 0; i < tree.size(); ++i) {
            if (tree[i].id != id) {
                double dist = distance(tree[id], tree[i]);
                if (dist <= r) {
                    nearest_neighbors.emplace_back(tree[i].id, dist);
                }
            }
        }
    
        // Sort distances before adjusting k
        std::sort(nearest_neighbors.begin(), nearest_neighbors.end(), 
                  [](const auto& lhs, const auto& rhs) { return lhs.second < rhs.second; });
    
        return nearest_neighbors;
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

    std::vector<int> reconstruct_path(std::vector<node>& tree) {
        node n;
        n.angles.assign(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
        // int q_goal_ID = nearest_neighbor(tree, n);

        std::cout << "Goal ID: " << goal_id << std::endl;
    
        if (tree[goal_id].g == std::numeric_limits<double>::infinity()) {
            std::cout << "No path found to the goal node." << std::endl;
            return {};
        }
    
        std::vector<int> path;
        for (int at = goal_id; at != -1; at = tree[at].parent) {
            path.push_back(at);
        }
        std::reverse(path.begin(), path.end());

        double dist = distance(tree[path.back()], n);
        std::cout << "Distance to goal: " << dist << std::endl;
        n.id = tree.size();
        tree.push_back(n);
        path.push_back(n.id);
            
        // Perform shortcutting to reduce unnecessary waypoints
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