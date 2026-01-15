#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>


struct Position {
    double x{};
    double y{};
    double z{};
};


using Cost = double;


using NodeId = std::size_t;


struct Edge {
    Cost cost{};
    NodeId to{};

    // Required for min-heap (std::greater<>)
    bool operator>(const Edge& rhs) const {
        return cost > rhs.cost;
    }
};


using AdjList = std::vector<Edge>;


struct Graph {

    /// @brief Create a Grap of N nodes
    /// @param N Number of nodes
    explicit Graph(std::size_t N = 0) : positions_(N), adj_list_(N) {}

    /// @brief Graph dimension
    /// @return Numbers of nodes
    std::size_t size() const { return adj_list_.size(); }

    void add_edge(NodeId from, NodeId to, Cost cost = 1.0, bool /*bidirectional*/ = false) {
        // Node validation
        this->check_node(from);
        this->check_node(to);

        // Pushing edge
        Edge edge;
        edge.to = to;
        edge.cost = cost;

        adj_list_[from].push_back(edge);
        // if (bidirectional) {
        //     adj_list_[to].push_back({from, cost});
        // }
    }

    const AdjList& neighbors(const NodeId node) const {
        return adj_list_[node];
    }



protected:
    /// @brief Node existance validation
    /// @param node Node to validate
    void check_node(const NodeId node) const {
        if (node >= this->size()) {
            std::string err = "Node: " + std::to_string(node) + " out of range [0.." + std::to_string(size()-1) + "]";
            throw std::out_of_range(err);
        }
    }


private:
    std::vector<Position> positions_;
    std::vector<AdjList> adj_list_;
};



//! Path
struct Path {
    std::vector<NodeId> steps;
    Cost distance;
};




#endif // GRAPH_HPP