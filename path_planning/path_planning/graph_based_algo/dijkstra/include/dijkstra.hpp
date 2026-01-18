#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include <queue>
#include <vector>
#include <iostream>
#include <algorithm>

#include "graph.hpp"


class Dijkstra {
public:

    Dijkstra(const Graph&& graph);

    ~Dijkstra() = default;

    std::optional<Path> solve(NodeId start, NodeId goal);

private:
    Graph graph_;
};


#endif  // DIJKSTRA_HPP