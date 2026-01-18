#ifndef GRAPH_HPP
#define GRAPH_HPP


#include <vector>


struct Point {
    int x, y;
    bool operator==(const Point& other) const { return x == other.x && y == other.y; }
};


//! Graph - neighbors list
using Graph = std::vector<std::vector<int>>;

struct GraphPos {
    Graph graph{};
    std::vector<Point> positions{};
};


//! Weighted Graph - adjacency list
struct Edge {
    int to;
    double weight;
    /* optional: id, capacity, label */
};

using NodeEdges = std::vector<Edge>;

using WGraph = std::vector<NodeEdges>;

struct WGraphPos {
    WGraph graph{};
    std::vector<Point> positions{};
};


//! Edge list
struct EdgeEntry { int u, v; double w; };
// std::vector<EdgeEntry> edges;


//! Adjacency matrix
// std::vector<std::vector<int>> mat(N, std::vector<int>(N, 0)); // 0/1 or INF/weight


#endif // GRAPH_HPP