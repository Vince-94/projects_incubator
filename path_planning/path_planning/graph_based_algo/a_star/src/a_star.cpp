#include <a_star.hpp>

#include <optional>
#include <limits>
#include <cmath>


AStar::AStar(const Graph&& graph, const std::string heuristic = "Euclidean") : graph_(std::move(graph)), heuristic_(heuristic) {
    std::cout << "A* Algorithm initialization" << std::endl;
    // Check graph is not empty
    if (graph_.size() == 0) {
        throw std::invalid_argument("Cannot run BFS on an empty graph (0 nodes)");
    }
}


std::optional<Path> AStar::solve(NodeId start, NodeId goal) {
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
    double start_cost = 0.0 + this->heuristic(start, goal);

    // Add start node
    Edge start_edge;
    start_edge.to = start;
    start_edge.cost = start_cost;
    graph_queue.push(start_edge);


    // Loop
    while (!graph_queue.empty()) {
        // Get current node from the queue
        const auto current_edge = graph_queue.top();
        const auto current_node = current_edge.to;
        auto current_cost = current_edge.cost;
        graph_queue.pop();

        // If the cost of the current path is greater than the cost found previously, skip
        if (current_cost > distance[current_node] + this->heuristic(current_node, goal)) {
            continue;
        }

        // Early exit when the goal is found
        if (current_node == goal) {
            break;
        }

        // Get the connections of the current nodes
        auto connections = graph_.neighbors(current_node);

        for (const auto& edge : connections) {
            const auto next_node = edge.to;
            double new_cost = distance[current_node] + edge.cost;

            // If the new cost is greater than the cost found previously, skip
            if (new_cost > distance[next_node]) {
                continue;
            }

            // Update cost
            distance[next_node] = new_cost;

            // Update parent
            parent[next_node] = current_node;

            // Push node to queue
            Edge next_edge;
            next_edge.to = next_node;
            next_edge.cost = new_cost + this->heuristic(next_node, goal);
            graph_queue.push(next_edge);
        }
    }

    // Goal unreachable
    if (distance[goal] == std::numeric_limits<double>::infinity()) {
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


Cost AStar::heuristic(NodeId node, NodeId goal) {
    const auto& pos_current = graph_.getPosition(node);
    const auto& pos_goal = graph_.getPosition(goal);

    double dx = pos_current.x - pos_goal.x;
    double dy = pos_current.y - pos_goal.y;
    double dz = pos_current.z - pos_goal.z;

    if (heuristic_ == "Manhattan") {  // L1 / Taxicab / City-Block
        return std::abs(dx) + std::abs(dy) + std::abs(dz);

    } else if (heuristic_ == "Euclidean") {  // L2 / Straight-line
        return std::sqrt(dx*dx + dy*dy + dz*dz);

    } else if (heuristic_ == "Chebyshev") {  // L∞ / Chessboard / King-move
        return std::max({std::abs(dx), std::abs(dy), std::abs(dz)});

    } else if (heuristic_ == "Octile") {  // Diagonal + Cardinal hybrid
        // auto [ax, ay] = get_2d_coords(a);  // ignore z or assume 2D
        // auto [bx, by] = get_2d_coords(b);

        // double dx = std::abs(ax - bx);
        // double dy = std::abs(ay - by);

        // constexpr double D = 1.0;        // cardinal cost
        // constexpr double D2 = 1.414213562;  // √2

        // double diag = std::min(dx, dy);
        // double straight = std::abs(dx - dy);

        // return D2 * diag + D * straight;

    } else {
        return 0.0;
    }


}
