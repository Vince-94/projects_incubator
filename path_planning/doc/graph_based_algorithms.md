# Graph-Based Classical Algorithms

- [Graph-Based Classical Algorithms](#graph-based-classical-algorithms)
  - [BFS (Breadth-First Search)](#bfs-breadth-first-search)
  - [Dijkstra Algorithm](#dijkstra-algorithm)
    - [Flow](#flow)
  - [A\*](#a)
    - [Admissible and optimal](#admissible-and-optimal)
    - [Heuristics](#heuristics)
    - [Flow](#flow-1)
  - [D\*](#d)
  - [Theta\*](#theta)


## BFS (Breadth-First Search)

Finds shortest (unweighted) path from a single source using a queue.


## Dijkstra Algorithm

Finds shortest paths from a single source using a priority queue. Complete and optimal for non-negative weights. Time: O((V+E) log V).

### Flow
1. Use min-heap priority queue ordered by accumulated cost `g(n)`.
2. Initialize distances to infinity, start to 0.
3. Dequeue the cheapest node if present, otherwise go to point 9.
4. If the cost is greater than the cost previously found for that node, then skip and return to step 3.
5. Early exit if the node processed is the goal, and go to point 9.
6. During neighbor relaxation, if lesser cost is found, then enqueue the neighbor node.
7. Return to point 3.
8. Reconstruct the path via parents list.


## A*

A* (A-star) is essentially an extension of Dijkstra's algorithm that incorporates a heuristic to guide the search more efficiently toward the goal.

This makes A* faster in practice for path planning in large graphs (e.g., grids, maps, or robotics environments) by prioritizing nodes that seem "promising" based on an estimated remaining cost. If the heuristic is admissible (never overestimates the true cost), A* guarantees optimality like Dijkstra.

Starting from the Dijkstra algorithm, to the accumulated cost from the start `g(n)`, it is added a heuristic (estimated) cost `h(n)`: `f(n) = g(n) + h(n)`

### Admissible and optimal
The heuristic function has to be admissible, in other words the estimated cost is less than the actual cost (not overstimating the cost). If it is admissible for each point of the path found, then it is guaranteed to be optimal.

If `h(n) = 0` then the path degenerates to Dijkstra algorithm.

If `h(n)` is greater than the true cost, then the heuristic is inadmissible.

### Heuristics

| #   | Heuristic Name               | Distance Metric                   | Admissible?    | Consistent? | Typical Use Cases                                                                     | When to Prefer It                                                      | Code Example (C++)                                                                                        |
| --- | ---------------------------- | --------------------------------- | -------------- | ----------- | ------------------------------------------------------------------------------------- | ---------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| 1   | **Manhattan** (L1)           | \|x₁-x₂\| + \|y₁-y₂\| + \|z₁-z₂\| | Yes            | Yes         | 4-connected grids (no diagonals)<br>City-block movement                               | Grid maps without diagonal movement<br>Very fast & perfect consistency | `std::abs(dx) + std::abs(dy) + std::abs(dz)`                                                              |
| 2   | **Euclidean** (L2)           | √(dx² + dy² + dz²)                | Yes            | No          | 8-connected grids<br>Any-angle / continuous space<br>Robotics (drones, omni wheels)   | 3D space, flying robots, smooth movement allowed                       | `std::sqrt(dx*dx + dy*dy + dz*dz)`                                                                        |
| 3   | **Chebyshev** (L∞)           | max(\|dx\|, \|dy\|, \|dz\|)       | Yes            | Yes         | 8-connected grids where diagonal = cardinal cost<br>King movement in chess-like games | Games with uniform diagonal cost<br>Very fast                          | `std::max({std::abs(dx), std::abs(dy), std::abs(dz)})`                                                    |
| 4   | **Octile** (Diagonal hybrid) | D·max(dx,dy) + (D2-D)·min(dx,dy)  | Yes            | Yes         | 8-connected grids with realistic diagonal cost (≈√2)                                  | Games / robotics where diagonal costs √2                               | `double D = 1.0, D2 = 1.414213562;`<br>`return D2 * std::min(dx,dy) + D * (dx + dy - 2*std::min(dx,dy));` |
| 5   | **Zero** (h(n) = 0)          | 0                                 | Yes            | Yes         | No good heuristic available<br>Debugging / baseline comparison                        | When you want pure Dijkstra behavior                                   | `return 0.0;`                                                                                             |
| 6   | **Scaled Euclidean**         | w × Euclidean                     | Yes (if w ≤ 1) | Sometimes   | Weighted grids / terrain with variable cost                                           | Slightly faster than pure Euclidean, still optimal                     | `return w * std::sqrt(dx*dx + dy*dy + dz*dz);` (w ≤ 1)                                                    |
| 7   | **Weighted A* heuristic**    | w × h(n) (w > 1)                  | No             | No          | Need faster search, can accept sub-optimal paths                                      | Real-time games / large maps where speed > optimality                  | `return w * heuristic(n, goal);` (w > 1)                                                                  |

### Flow
1. Use min-heap priority queue ordered by accumulated cost `g(n)`.
2. Initialize distances to infinity, start to 0.
3. Dequeue the cheapest node if present, otherwise go to point 9.
4. Compute the heurstic function `h(n, goal)` and compute the overall cost: `f(n) = g(n) + h(n)`
5. If the cost is greater than the cost previously found for that node, then skip and return to step 3.
6. Early exit if the node processed is the goal, and go to point 9.
7. During neighbor relaxation, if lesser cost is found, then enqueue the neighbor node.
8. Return to point 3.
9. Reconstruct the path via parents list.



## D*

D* (Dynamic A*) and its popular simplified variant D Lite* are incremental heuristic search algorithms designed specifically for dynamic or partially unknown environments, where:
- The map changes over time (new obstacles appear, old ones disappear, costs increase/decrease)
- The robot discovers discrepancies via sensors while following a path
- Replanning needs to happen repeatedly and fast, without restarting from scratch every time


## Theta*

Improves A* by allowing any-angle paths in grids, reducing zigzags for smoother trajectories.



