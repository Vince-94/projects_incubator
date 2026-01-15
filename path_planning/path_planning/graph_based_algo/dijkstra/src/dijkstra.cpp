#include <dijkstra.hpp>

#include <optional>
#include <limits>


Dijkstra::Dijkstra(const Graph&& graph) : graph_(std::move(graph)) {
    std::cout << "Dijkstra Algorithm initialization" << std::endl;
    // Check graph is not empty
    if (graph_.size() == 0) {
        throw std::invalid_argument("Cannot run BFS on an empty graph (0 nodes)");
    }
}


std::optional<Path> Dijkstra::solve(NodeId start, NodeId goal) {
    // Check if start and goal nodes are valid
    if (start >= graph_.size()) {
        std::cerr << "Start node (" << start << ") is out of graph bounds" << std::endl;
        return {};
    }
    if (goal >= graph_.size()) {
        std::cerr << "Goal node (" << goal << ") is out of graph bounds" << std::endl;
        return {};
    }

    // Initialize
    std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>> graph_queue{};
    std::vector<NodeId> parent(graph_.size(), static_cast<NodeId>(-1));
    std::vector<double> distance(graph_.size(), std::numeric_limits<double>::infinity());

    // Initialize distance of start node
    distance[start] = 0.0;

    // Add start node
    Edge start_edge;
    start_edge.to = start;
    start_edge.cost = 0.0;
    graph_queue.push(start_edge);


    // Loop
    while (!graph_queue.empty()) {
        // Get current node from the queue
        const auto current_edge = graph_queue.top();
        const auto current_node = current_edge.to;
        const auto current_cost = current_edge.cost;
        graph_queue.pop();

        // If the cost of the current path is greater than the cost found previously, skip
        if (current_cost > distance[current_node]) {
            continue;
        }

        // Get the connections of the current nodes
        auto connections = graph_.neighbors(current_node);

        for (const auto& edge : connections) {
            const auto next_node = edge.to;
            double new_cost = distance[current_node] + edge.cost;

            // If the new cost is greater than the cost found previously, skip
            if (new_cost > distance[next_node]) {  // TODO bug here
                continue;
            }

            // Update cost
            distance[next_node] = new_cost;

            // Update parent
            parent[next_node] = current_node;

            // Push node to queue
            Edge next_edge;
            next_edge.to = next_node;
            next_edge.cost = new_cost;
            graph_queue.push(next_edge);
        }
    }

    // Goal unreachable
    if (distance[goal] == std::numeric_limits<std::size_t>::infinity()) {
        return {};
    }

    // Reconstruct path
    Path path{};
    path.distance = distance[goal];

    // Count path length
    std::size_t path_length = 1;
    for (NodeId cur = goal; parent[cur] != static_cast<NodeId>(-1); cur = parent[cur]) {
        ++path_length;
    }

    // Reconstruct the steps
    path.steps.reserve(path_length);

    NodeId cur = goal;
    while (cur != static_cast<NodeId>(-1)) {
        path.steps.push_back(cur);
        cur = parent[cur];
    }

    std::reverse(path.steps.begin(), path.steps.end());

    return path;
}
