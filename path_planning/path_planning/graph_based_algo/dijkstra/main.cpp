#include <dijkstra.hpp>
#include <graph.hpp>

#include <iostream>


Graph example_graph() {
    // 0 ─ 1 ─ 2
    // │ \ │   │
    // 3   4 ─ 5
    // │   │   │
    // 6   7 ─ 8

    Graph graph{9};
    graph.add_edge(0, 1, 2.5);
    graph.add_edge(0, 3, 4.0);
    graph.add_edge(0, 4, 1.0);
    graph.add_edge(1, 0, 6.0);
    graph.add_edge(1, 4, 7.0);
    graph.add_edge(1, 2, 3.0);
    graph.add_edge(2, 1, 5.0);
    graph.add_edge(2, 5, 2.0);
    graph.add_edge(3, 0, 1.0);
    graph.add_edge(3, 6, 6.0);
    graph.add_edge(4, 0, 3.0);
    graph.add_edge(4, 1, 2.0);
    graph.add_edge(4, 5, 4.0);
    graph.add_edge(4, 7, 5.0);
    graph.add_edge(5, 6, 2.0);
    graph.add_edge(5, 8, 3.0);
    graph.add_edge(6, 3, 5.0);
    graph.add_edge(7, 4, 6.0);
    graph.add_edge(7, 8, 7.0);
    graph.add_edge(8, 5, 1.0);
    graph.add_edge(8, 7, 3.0);

    return graph;
}



int main() {
    Graph graph = example_graph();

    Dijkstra dijkstra(std::move(graph));

    NodeId start = 0;
    NodeId goal = 8;

    std::optional<Path> path_opt = dijkstra.solve(start, goal);
    if (!path_opt) {
        std::cout << "No feasible path found" << std::endl;
        return 0;
    }

    auto path = *path_opt;
    std::cout << "Path found:" << std::endl;
    std::cout << "* cost = " << path.distance << std::endl;
    std::cout << "* sequence = [";
    for (auto it = path.steps.begin(); it != path.steps.end(); ++it) {
        std::cout << *it;
        if (std::next(it) != path.steps.end())
            std::cout << ", ";
    }

    std::cout << "]" << std::endl;

    return 0;
}
