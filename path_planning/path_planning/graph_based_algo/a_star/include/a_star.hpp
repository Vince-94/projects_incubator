#ifndef AStar_HPP
#define AStar_HPP

#include <queue>
#include <vector>
#include <iostream>
#include <algorithm>

#include "graph.hpp"


class AStar {
public:

    AStar(const Graph&& graph, const std::string heuristic);

    ~AStar() = default;

    std::optional<Path> solve(NodeId start, NodeId goal);

protected:
    Cost heuristic(NodeId node, NodeId goal);

private:
    Graph graph_;
    std::string heuristic_;
};


#endif  // AStar_HPP