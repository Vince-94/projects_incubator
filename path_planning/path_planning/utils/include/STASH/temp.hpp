#include <vector>
#include <string>
#include <queue>
#include <unordered_map>
#include <optional>
#include <iostream>
#include <cassert>

struct Edge {
    int to;
    double weight; // default 1.0 for unweighted graphs
    Edge(int t, double w = 1.0) : to(t), weight(w) {}
};

class Graph {
public:
    Graph(bool directed = false) : directed_(directed) {}

    // Add a node with optional external name; returns index
    int add_node(const std::optional<std::string>& name = std::nullopt) {
        int idx = (int)adj_.size();
        adj_.emplace_back();
        if (name) name_to_idx_[*name] = idx;
        return idx;
    }

    // Map name -> index; if missing, create node
    int index_of(const std::string& name) {
        auto it = name_to_idx_.find(name);
        if (it != name_to_idx_.end()) return it->second;
        return add_node(name);
    }

    // Add edge by indices
    void add_edge(int u, int v, double weight = 1.0) {
        ensure_index(u);
        ensure_index(v);
        adj_[u].emplace_back(v, weight);
        if (!directed_) adj_[v].emplace_back(u, weight);
    }

    // Add edge by names (auto creates nodes)
    void add_edge(const std::string& a, const std::string& b, double weight = 1.0) {
        int u = index_of(a), v = index_of(b);
        add_edge(u, v, weight);
    }

    int size() const { return (int)adj_.size(); }

    // Single-source BFS distances (-1 unreachable)
    std::vector<int> bfs_distance(int start) const {
        std::vector<int> dist(size(), -1);
        if (start < 0 || start >= size()) return dist;
        std::queue<int> q;
        dist[start] = 0;
        q.push(start);
        while (!q.empty()) {
            int u = q.front(); q.pop();
            for (const auto& e : adj_[u]) {
                if (dist[e.to] == -1) {
                    dist[e.to] = dist[u] + 1;
                    q.push(e.to);
                }
            }
        }
        return dist;
    }

    // BFS distances covering all components
    // Note: distances are relative to the component root (first unvisited node encountered)
    std::vector<int> bfs_all_distances() const {
        std::vector<int> dist(size(), -1);
        std::queue<int> q;
        for (int s = 0; s < size(); ++s) {
            if (dist[s] != -1) continue;
            dist[s] = 0;
            q.push(s);
            while (!q.empty()) {
                int u = q.front(); q.pop();
                for (const auto& e : adj_[u]) {
                    if (dist[e.to] == -1) {
                        dist[e.to] = dist[u] + 1;
                        q.push(e.to);
                    }
                }
            }
        }
        return dist;
    }

    // Simple print adjacency list
    void print_adj(std::ostream& os = std::cout) const {
        for (int i = 0; i < size(); ++i) {
            os << i << ":";
            for (const auto& e : adj_[i]) os << " (" << e.to << "," << e.weight << ")";
            os << "\n";
        }
    }

private:
    std::vector<std::vector<Edge>> adj_;
    std::unordered_map<std::string,int> name_to_idx_;
    bool directed_;

    void ensure_index(int idx) const {
        assert(idx >= 0 && idx < (int)adj_.size());
    }
};







int main() {
    Graph g(false); // undirected
    // named edges auto-create nodes
    g.add_edge("A","B");
    g.add_edge("A","C");
    g.add_edge("B","D");
    g.add_edge("E","F"); // separate component

    g.print_adj();

    // single-source distances from A
    int start = g.index_of("A");
    auto d = g.bfs_distance(start);
    // d for nodes in A-component will be >=0, others -1

    // all-component distances (each component root gets distance 0)
    auto da = g.bfs_all_distances();

    // quick asserts (indices unpredictable due to insertion order, so map names)
    int idxA = g.index_of("A");
    int idxE = g.index_of("E");

    // example checks (not strict—just demonstrate)
    std::cout << "dist(A->D): " << d[g.index_of("D")] << "\n";
    std::cout << "component-root-dist(E): " << da[idxE] << "\n";

    return 0;
}
