#include <bfs.hpp>

#include <optional>
#include <limits>


BreadthFirstSearch::BreadthFirstSearch(const Graph&& graph) : graph_(std::move(graph)) {
    std::cout << "BFS Algorithm initialization" << std::endl;

    // Check graph is not empty
    if (graph_.size() == 0) {
        throw std::invalid_argument("Cannot run BFS on an empty graph (0 nodes)");
    }
    std::cout << "Loaded graph with dimension: " << graph_.size() << std::endl;
}


std::optional<Path> BreadthFirstSearch::solve(NodeId start, NodeId goal) {
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
    std::vector<NodeId> parent(graph_.size(), static_cast<NodeId>(-1));
    std::vector<std::size_t> distance(graph_.size(), std::numeric_limits<std::size_t>::infinity());

    // Add start node
    graph_queue_.push(start);

    // Initialize distance of start node
    distance[start] = 0;

    // Loop
    while (!graph_queue_.empty()) {

        // Get current node from the queue
        const auto current_node = graph_queue_.front();
        graph_queue_.pop();

        // Get the connections of the current nodes
        auto connections = graph_.neighbors(current_node);

        for (const auto& edge : connections) {
            NodeId node = edge.to;

            // Check if edge has been visited
            if (distance[node] != std::numeric_limits<std::size_t>::infinity()) continue;

            // Update cost
            distance[node] = distance[current_node] + 1;

            // Update parent
            parent[node] = current_node;

            // Add node to graph queue
            graph_queue_.push(node);

            // Check if the current node is the goal
            if (node == goal) {
                Path path{};

                path.distance = distance[node];

                path.steps.reserve(static_cast<std::size_t>(distance[node]) + 1);
                auto current_parent = parent[node];
                path.steps.push_back(goal);
                while (current_parent > start) {
                    path.steps.push_back(current_parent);
                    current_parent = parent[current_parent];
                }
                path.steps.push_back(start);
                std::reverse(path.steps.begin(), path.steps.end());

                return path;
            }
        }
    }

    // Goal unreachable
    return {};
}

