#ifndef BFS_HPP
#define BFS_HPP

#include <queue>
#include <vector>
#include <iostream>
#include <algorithm>

#include "graph.hpp"


class BreadthFirstSearch {
public:

    BreadthFirstSearch(const Graph&& graph);

    ~BreadthFirstSearch() = default;

    std::optional<Path> solve(NodeId start, NodeId goal);

private:
    Graph graph_;

    std::queue<NodeId> graph_queue_{};
};


#endif  // BFS_HPP