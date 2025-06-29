// Nicholas Britt Lab1 Search Algorithm

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <queue>
#include <stack>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <sstream>
#include <algorithm>

using namespace std;

const bool debug_mode = false; // Whether or not to run debug

struct edge {
    const string start_state;
    const string end_state;
    const unsigned cost;
    
    edge(string start, string end, unsigned c) : start_state(start), end_state(end), cost(c) {}
};

struct heuristic_data {
    const string state;
    const unsigned value;
    
    heuristic_data(string s, unsigned v) : state(s), value(v) {}
};

class SearchNode {
private:
    const string state;
    const unsigned heuristic;
    vector<edge> edges;
    vector<string> path;
    vector<unsigned> costs;
    
public:
    SearchNode(string s, unsigned h) : state(s), heuristic(h) {}
    
    // Getters
    string getState() const { return state; }
    unsigned getHeuristic() const { return heuristic; }
    unsigned getEvaluationCost() const { return costs.back() + heuristic; }
    edge getEdge(unsigned i) const { return edges.at(i); }
    unsigned getEdgeCount() const { return edges.size(); }
    string getPathMember(unsigned i) const { return path.at(i); }
    unsigned getPathSize() const { return path.size(); }
    unsigned getCostsMember(unsigned i) const { return costs.at(i); }
    unsigned getCostsSize() const { return costs.size(); }
    unsigned getTotalCost() const { return costs.back(); }
    
    // Modifiers
    void addEdge(const edge& e) { edges.push_back(e); }
    void addToPath(const string& s) { path.push_back(s); }
    void clearPath() { path.clear(); }
    void addToCosts(unsigned n) { costs.push_back(n); }
    void clearCosts() { costs.clear(); }
};

class PathFinder {
private:
    string algorithm;
    string start_state;
    string goal_state;
    vector<SearchNode> graph;
    vector<edge> traffic_data;
    vector<heuristic_data> heuristic_info;
    
    // Helper functions
    string removeTrailingWhitespace(string& input) {
        return input.substr(0, input.find_last_not_of(" ") + 1);
    }
    
    int getHeuristicFromVector(const string& state) {
        for (unsigned i = 0; i < heuristic_info.size(); ++i) {
            if (heuristic_info.at(i).state == state) {
                return heuristic_info.at(i).value;
            }
        }
        cerr << "Failed to find heuristic information for state " << state << endl;
        return -1;
    }
    
    int getIndexOfState(const string& state) {
        for (unsigned i = 0; i < graph.size(); ++i) {
            if (graph.at(i).getState() == state) {
                return i;
            }
        }
        cerr << "Failed to find node with state " << state << endl;
        return -1;
    }
    
    void buildGraph() {
        // First create nodes for all states that have outgoing edges
        for (unsigned i = 0; i < traffic_data.size(); ++i) {
            const string node_state = traffic_data.at(i).start_state;
            
            bool node_exists = false;
            int existing_index = -1;
            for (unsigned j = 0; j < graph.size(); ++j) {
                if (graph.at(j).getState() == node_state) {
                    node_exists = true;
                    existing_index = j;
                    break;
                }
            }
            
            if (node_exists) {
                graph.at(existing_index).addEdge(traffic_data.at(i));
            } else {
                int h = getHeuristicFromVector(node_state);
                SearchNode node(node_state, h);
                node.addEdge(traffic_data.at(i));
                graph.push_back(node);
            }
        }
        
        // Add nodes that only exist as destinations (no outgoing edges)
        for (unsigned i = 0; i < traffic_data.size(); ++i) {
            const string node_state = traffic_data.at(i).end_state;
            
            bool node_exists = false;
            for (unsigned j = 0; j < graph.size(); ++j) {
                if (graph.at(j).getState() == node_state) {
                    node_exists = true;
                    break;
                }
            }
            
            if (!node_exists) {
                int h = getHeuristicFromVector(node_state);
                SearchNode node(node_state, h);
                graph.push_back(node);
            }
        }
        
        if (debug_mode) {
            cout << "Graph constructed with " << graph.size() << " nodes" << endl;
        }
    }
    
    void writeOutput(const SearchNode& solution) {
        ofstream out("output.txt");
        for (unsigned i = 0; i < solution.getPathSize(); ++i) {
            out << solution.getPathMember(i) << " " 
                << solution.getCostsMember(i) << endl;
        }
        out.close();
    }
    
    SearchNode BFS(unsigned start_index, unsigned goal_index) {
        deque<unsigned> frontier;
        vector<unsigned> explored;
        frontier.push_back(start_index);
        
        // Initialize start node's path and costs
        graph.at(start_index).clearPath();
        graph.at(start_index).addToPath(graph.at(start_index).getState());
        graph.at(start_index).clearCosts();
        graph.at(start_index).addToCosts(0);
        
        while (!frontier.empty()) {
            unsigned current = frontier.front(); // FIFO
            frontier.pop_front();
            
            if (current == goal_index) {
                return graph.at(current);
            }
            
            explored.push_back(current);
            
            for (unsigned i = 0; i < graph.at(current).getEdgeCount(); ++i) {
                string child_state = graph.at(current).getEdge(i).end_state;
                unsigned child_index = getIndexOfState(child_state);
                
                bool in_explored = find(explored.begin(), explored.end(), child_index) != explored.end();
                bool in_frontier = find(frontier.begin(), frontier.end(), child_index) != frontier.end();
                
                if (!in_explored && !in_frontier) {
                    // Set child's path and costs based on parent
                    graph.at(child_index).clearPath();
                    for (unsigned j = 0; j < graph.at(current).getPathSize(); ++j) {
                        graph.at(child_index).addToPath(graph.at(current).getPathMember(j));
                    }
                    graph.at(child_index).clearCosts();
                    for (unsigned j = 0; j < graph.at(current).getCostsSize(); ++j) {
                        graph.at(child_index).addToCosts(graph.at(current).getCostsMember(j));
                    }
                    
                    // Add transition to child
                    graph.at(child_index).addToPath(child_state);
                    unsigned parent_cost = graph.at(current).getTotalCost();
                    graph.at(child_index).addToCosts(parent_cost + graph.at(current).getEdge(i).cost);
                    
                    frontier.push_back(child_index);
                }
            }
        }
        
        return SearchNode("", 0); // Fail
    }
    
    SearchNode DFS(unsigned start_index, unsigned goal_index) {
        deque<unsigned> frontier;
        vector<unsigned> explored;
        frontier.push_back(start_index);
        
        // Initialize start node's path and costs
        graph.at(start_index).clearPath();
        graph.at(start_index).addToPath(graph.at(start_index).getState());
        graph.at(start_index).clearCosts();
        graph.at(start_index).addToCosts(0);
        
        while (!frontier.empty()) {
            unsigned current = frontier.back(); // LIFO
            frontier.pop_back();
            
            if (current == goal_index) {
                return graph.at(current);
            }
            
            explored.push_back(current);
            
            for (unsigned i = 0; i < graph.at(current).getEdgeCount(); ++i) {
                string child_state = graph.at(current).getEdge(i).end_state;
                unsigned child_index = getIndexOfState(child_state);
                
                bool in_explored = find(explored.begin(), explored.end(), child_index) != explored.end();
                bool in_frontier = find(frontier.begin(), frontier.end(), child_index) != frontier.end();
                
                if (!in_explored && !in_frontier) {
                    // Set child path and costs based on parent
                    graph.at(child_index).clearPath();
                    for (unsigned j = 0; j < graph.at(current).getPathSize(); ++j) {
                        graph.at(child_index).addToPath(graph.at(current).getPathMember(j));
                    }
                    graph.at(child_index).clearCosts();
                    for (unsigned j = 0; j < graph.at(current).getCostsSize(); ++j) {
                        graph.at(child_index).addToCosts(graph.at(current).getCostsMember(j));
                    }
                    
                    // Add transition to child
                    graph.at(child_index).addToPath(child_state);
                    unsigned parent_cost = graph.at(current).getTotalCost();
                    graph.at(child_index).addToCosts(parent_cost + graph.at(current).getEdge(i).cost);
                    
                    frontier.push_back(child_index);
                }
            }
        }
        
        return SearchNode("", 0); // Fail
    }
    
    SearchNode UCS(unsigned start_index, unsigned goal_index) {
        vector<unsigned> frontier; // Stored in descending order, last element is front
        vector<unsigned> explored;
        frontier.push_back(start_index);
        
        // Initialize start node's path and costs
        graph.at(start_index).clearPath();
        graph.at(start_index).addToPath(graph.at(start_index).getState());
        graph.at(start_index).clearCosts();
        graph.at(start_index).addToCosts(0);
        
        while (!frontier.empty()) {
            unsigned current = frontier.back();
            frontier.pop_back();
            
            if (current == goal_index) {
                return graph.at(current);
            }
            
            explored.push_back(current);
            
            vector<unsigned> children;
            for (unsigned i = 0; i < graph.at(current).getEdgeCount(); ++i) {
                string child_state = graph.at(current).getEdge(i).end_state;
                unsigned child_index = getIndexOfState(child_state);
                
                bool in_explored = find(explored.begin(), explored.end(), child_index) != explored.end();
                
                if (!in_explored) {
                    auto child_in_frontier = find(frontier.begin(), frontier.end(), child_index);
                    bool in_frontier = child_in_frontier != frontier.end();
                    
                    bool better_path_exists = false;
                    unsigned parent_cost = graph.at(current).getTotalCost();
                    unsigned new_cost = parent_cost + graph.at(current).getEdge(i).cost;
                    
                    if (in_frontier) {
                        unsigned old_cost = graph.at(child_index).getTotalCost();
                        if (old_cost <= new_cost) {
                            better_path_exists = true;
                        }
                    }
                    
                    if (!better_path_exists) {
                        if (in_frontier) {
                            frontier.erase(child_in_frontier);
                        }
                        
                        // Update child's path and costs
                        graph.at(child_index).clearPath();
                        for (unsigned j = 0; j < graph.at(current).getPathSize(); ++j) {
                            graph.at(child_index).addToPath(graph.at(current).getPathMember(j));
                        }
                        graph.at(child_index).clearCosts();
                        for (unsigned j = 0; j < graph.at(current).getCostsSize(); ++j) {
                            graph.at(child_index).addToCosts(graph.at(current).getCostsMember(j));
                        }
                        
                        graph.at(child_index).addToPath(child_state);
                        graph.at(child_index).addToCosts(new_cost);
                        
                        children.push_back(child_index);
                    }
                }
            }
            
            // Insert children into frontier in sorted order
            for (unsigned child : children) {
                unsigned child_cost = graph.at(child).getTotalCost();
                unsigned insert_pos = frontier.size();
                
                for (unsigned j = 0; j < frontier.size(); ++j) {
                    if (graph.at(frontier.at(j)).getTotalCost() <= child_cost) {
                        insert_pos = j;
                        break;
                    }
                }
                
                frontier.insert(frontier.begin() + insert_pos, child);
            }
        }
        
        return SearchNode("", 0); // Fail
    }
    
    SearchNode AStar(unsigned start_index, unsigned goal_index) {
        vector<unsigned> frontier; // Stored in descending order, last element is front
        vector<unsigned> explored;
        frontier.push_back(start_index);
        
        // Initialize start node's path and costs
        graph.at(start_index).clearPath();
        graph.at(start_index).addToPath(graph.at(start_index).getState());
        graph.at(start_index).clearCosts();
        graph.at(start_index).addToCosts(0);
        
        while (!frontier.empty()) {
            unsigned current = frontier.back();
            frontier.pop_back();
            
            if (current == goal_index) {
                return graph.at(current);
            }
            
            explored.push_back(current);
            
            vector<unsigned> children;
            for (unsigned i = 0; i < graph.at(current).getEdgeCount(); ++i) {
                string child_state = graph.at(current).getEdge(i).end_state;
                unsigned child_index = getIndexOfState(child_state);
                
                bool in_explored = find(explored.begin(), explored.end(), child_index) != explored.end();
                
                if (!in_explored) {
                    auto child_in_frontier = find(frontier.begin(), frontier.end(), child_index);
                    bool in_frontier = child_in_frontier != frontier.end();
                    
                    bool better_path_exists = false;
                    unsigned parent_cost = graph.at(current).getTotalCost();
                    unsigned new_path_cost = parent_cost + graph.at(current).getEdge(i).cost;
                    unsigned new_eval_cost = new_path_cost + graph.at(child_index).getHeuristic();
                    
                    if (in_frontier) {
                        unsigned old_eval_cost = graph.at(child_index).getEvaluationCost();
                        if (old_eval_cost <= new_eval_cost) {
                            better_path_exists = true;
                        }
                    }
                    
                    if (!better_path_exists) {
                        if (in_frontier) {
                            frontier.erase(child_in_frontier);
                        }
                        
                        // Update child's path and costs
                        graph.at(child_index).clearPath();
                        for (unsigned j = 0; j < graph.at(current).getPathSize(); ++j) {
                            graph.at(child_index).addToPath(graph.at(current).getPathMember(j));
                        }
                        graph.at(child_index).clearCosts();
                        for (unsigned j = 0; j < graph.at(current).getCostsSize(); ++j) {
                            graph.at(child_index).addToCosts(graph.at(current).getCostsMember(j));
                        }
                        
                        graph.at(child_index).addToPath(child_state);
                        graph.at(child_index).addToCosts(new_path_cost);
                        
                        children.push_back(child_index);
                    }
                }
            }
            
            // Insert children into frontier in sorted order by evaluation cost
            for (unsigned child : children) {
                unsigned child_eval_cost = graph.at(child).getEvaluationCost();
                unsigned insert_pos = frontier.size();
                
                for (unsigned j = 0; j < frontier.size(); ++j) {
                    if (graph.at(frontier.at(j)).getEvaluationCost() <= child_eval_cost) {
                        insert_pos = j;
                        break;
                    }
                }
                
                frontier.insert(frontier.begin() + insert_pos, child);
            }
        }
        
        return SearchNode("", 0); // Fail
    }
    
    SearchNode getSolution(unsigned start_index, unsigned goal_index) {
        if (algorithm == "BFS") {
            return BFS(start_index, goal_index);
        } else if (algorithm == "DFS") {
            return DFS(start_index, goal_index);
        } else if (algorithm == "UCS") {
            return UCS(start_index, goal_index);
        } else if (algorithm == "A*") {
            return AStar(start_index, goal_index);
        } else {
            cerr << "Invalid algorithm: " << algorithm << ". Defaulting to UCS." << endl;
            return UCS(start_index, goal_index);
        }
    }
    
public:
    void solve() {
        ifstream input("input.txt");
        if (input.fail()) {
            cerr << "Failed to open input.txt" << endl;
            return;
        }
        
        // Read algorithm, start state, and goal state
        getline(input, algorithm);
        algorithm = removeTrailingWhitespace(algorithm);
        getline(input, start_state);
        start_state = removeTrailingWhitespace(start_state);
        getline(input, goal_state);
        goal_state = removeTrailingWhitespace(goal_state);
        
        // Read traffic data
        string line_count_str;
        getline(input, line_count_str);
        unsigned traffic_lines = static_cast<unsigned>(stoi(line_count_str));
        
        for (unsigned i = 0; i < traffic_lines; ++i) {
            string line, from, to;
            unsigned cost;
            getline(input, line);
            line = removeTrailingWhitespace(line);
            
            unsigned first_space = line.find_first_of(" ");
            unsigned last_space = line.find_last_of(" ");
            
            from = line.substr(0, first_space);
            to = line.substr(first_space + 1, last_space - first_space - 1);
            cost = static_cast<unsigned>(stoi(line.substr(last_space + 1)));
            
            traffic_data.push_back(edge(from, to, cost));
        }
        
        // Read heuristic data
        getline(input, line_count_str);
        unsigned heuristic_lines = static_cast<unsigned>(stoi(line_count_str));
        
        for (unsigned i = 0; i < heuristic_lines; ++i) {
            string line, state;
            unsigned value;
            getline(input, line);
            line = removeTrailingWhitespace(line);
            
            unsigned space_pos = line.find_first_of(" ");
            state = line.substr(0, space_pos);
            value = static_cast<unsigned>(stoi(line.substr(space_pos + 1)));
            
            heuristic_info.push_back(heuristic_data(state, value));
        }
        
        input.close();
        
        // Build the graph
        buildGraph();
        
        // Find solution
        unsigned start_index = getIndexOfState(start_state);
        unsigned goal_index = getIndexOfState(goal_state);
        
        SearchNode solution = getSolution(start_index, goal_index);
        
        if (solution.getState() == "") {
            cout << "Failed to find a solution." << endl;
            return;
        }
        
        if (debug_mode) {
            cout << "Found solution using " << algorithm << endl;
            cout << "Path: ";
            for (unsigned i = 0; i < solution.getPathSize(); ++i) {
                cout << solution.getPathMember(i) << " ";
            }
            cout << endl;
        }
        
        writeOutput(solution);
    }
};

int main() {
    PathFinder pathFinder;
    pathFinder.solve();
    return 0;
}