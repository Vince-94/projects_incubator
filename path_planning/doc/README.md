# Path Planning

- [Path Planning](#path-planning)
  - [Overview](#overview)
  - [Key Principles](#key-principles)
  - [Graph Representations](#graph-representations)
    - [Selection Guidelines](#selection-guidelines)
  - [Algorithms](#algorithms)
    - [Graph-Based Classical Algorithms](#graph-based-classical-algorithms)
    - [Sampling-Based Algorithms](#sampling-based-algorithms)
    - [Optimization-Based Algorithms](#optimization-based-algorithms)
    - [Grid-Based Continuous Refinement](#grid-based-continuous-refinement)
    - [Learning-Based Planning](#learning-based-planning)
  - [Implementation Considerations](#implementation-considerations)
  - [Challenges and Trade-offs](#challenges-and-trade-offs)
  - [Reference](#reference)


## Overview

Path planning is a core challenge in robotics, autonomous vehicles, AI, and game development. It involves computing a collision-free path from a start configuration to a goal configuration in a given environment, while often optimizing for metrics like shortest distance, minimal time, energy efficiency, or smoothness. The environment may include static or dynamic obstacles, and the path must respect the system's kinematic or dynamic constraints (e.g., non-holonomic for wheeled robots).

This documentation provides a structured overview of path planning concepts, starting with foundational principles and graph representations, followed by a categorization of key algorithms. It also covers implementation tips, challenges, and real-world applications. For in-depth code examples, mathematical derivations, or specific algorithm implementations, refer to dedicated files or external resources in the repository.

Key aspects of path planning:
- **Completeness**: Does the algorithm guarantee finding a path if one exists?
- **Optimality**: Does it find the best path according to the optimization criteria?
- **Efficiency**: How does it scale with environment size, obstacle density, or dimensionality?
- **Real-time Capability**: Suitable for dynamic environments or online replanning?


## Key Principles

Effective path planning relies on several foundational principles to ensure robustness, efficiency, and adaptability:
- ***Abstraction and Modeling***: Represent the environment and robot's configuration space (C-space) appropriately. Discretize continuous spaces into graphs or grids for discrete algorithms, or use continuous representations for optimization-based methods.
- ***Collision Detection***: Integrate efficient checks (e.g., bounding volumes, kd-trees) to verify path feasibility.
- ***Heuristics and Cost Functions***: Use admissible heuristics (e.g., Euclidean distance) to guide search in informed algorithms like A*.
- ***Handling Uncertainty***: Account for sensor noise, dynamic obstacles, or partial observability in real-world scenarios.
- ***Multi-Objective Optimization***: Balance trade-offs between path length, safety margins, and computational cost.
- ***Scalability***: Design for high-dimensional spaces (e.g., 6+ DOF for robotic arms) or large environments.

These principles guide the choice of representation and algorithm based on the problem domain.


## Graph Representations

Graphs are a common abstraction for path planning, where nodes represent states (positions, configurations) and edges represent feasible transitions. Choosing the right representation impacts memory usage, traversal speed, and ease of implementation. Below are common options:
1. Adjacency List (Unweighted): Ideal for sparse graphs with integer nodes. Each node stores a list of neighbors. Best for most path planning scenarios due to O(1) average access and low memory.
2. Adjacency List (Weighted or with Metadata): Extends the unweighted version by storing edge weights, directions, capacities, or labels in a struct (e.g., `{int to; double weight;}`). Suitable for algorithms like Dijkstra or A* that consider costs.
3. Edge List: A simple list of all edges (e.g., pairs of nodes with optional weights). Useful for input/output, graph construction, or algorithms like Kruskal's MST. Convert to adjacency lists for efficient traversal.
4. Adjacency Matrix: A 2D array where matrix[i][j] indicates an edge from i to j (with weight or boolean). Provides O(1) edge checks but high memory (O(N²))—use only for dense graphs or small N (< 2000).
5. Compressed Sparse Row (CSR): A compact format for sparse matrices with row pointers, column indices, and values. Offers excellent cache locality and memory efficiency for large-scale graphs in performance-critical applications.
6. Grid Graphs (2D/3D): For grid-based environments, map cells to nodes via linear indexing (e.g., `index = row * cols + col`). Store obstacles in a boolean array. Compute neighbors on-the-fly (4-connectivity for orthogonal moves, 8-connectivity for diagonals). Efficient for maps like occupancy grids in robotics.


### Selection Guidelines
- Sparse graphs (edges ≈ nodes): Adjacency list.
- Dense graphs (small N): Adjacency matrix.
- High-performance needs: CSR.
- Weighted/attributed edges: Use structs in lists.
- Non-integer nodes (e.g., coordinates): Map to integer indices upfront for efficiency.


## Algorithms

Path planning algorithms can be categorized by their approach. Each category includes brief descriptions, strengths, and use cases. For pseudocode or implementations, see linked files.

### Graph-Based Classical Algorithms
These discretize the space into a graph and search for paths.
- BFS (Breadth-First Search)
- Dijkstra Algorithm
- A*
- D*
- Theta*

### Sampling-Based Algorithms
These sample configurations randomly to build roadmaps or trees, excelling in high-dimensional spaces.

- Probabilistic Roadmap (PRM): Samples free space, connects nearby points to form a graph, then searches it. Probabilistic complete; good for static environments.
- Rapidly-Exploring Random Tree (RRT): Grows a tree by sampling and connecting to nearest nodes. Fast exploration; asymptotically optimal variants exist.
- RRT and Variations*: Optimizes RRT for near-optimality by rewiring trees. Includes informed sampling for better convergence.

### Optimization-Based Algorithms
These treat planning as an optimization problem, often producing smooth paths.

- CHOMP (Covariant Hamiltonian Optimization for Motion Planning): Optimizes trajectories using functional gradients; handles obstacles via signed distance fields.
- STOMP (Stochastic Trajectory Optimization for Motion Planning): Uses stochastic perturbations for non-convex optimization; robust to local minima.
- Trajectory Optimization (e.g., MPC, iLQR, CEM): Model Predictive Control (MPC) for real-time; iterative Linear Quadratic Regulator (iLQR) for nonlinear dynamics; Cross-Entropy Method (CEM) for sampling-based optimization.

### Grid-Based Continuous Refinement
These refine paths in continuous spaces, often on grids.

- Potential Fields / Artificial Forces: Assigns attractive forces to goals and repulsive to obstacles. Simple but prone to local minima.
- Fast Marching Methods: Propagates wavefronts like Dijkstra in continuous space; produces smooth level-set paths.

### Learning-Based Planning
Integrates machine learning for improved efficiency or adaptability.

- Learned Heuristics for A*: Trains neural networks to provide better heuristics than traditional ones.
- Learning-to-Plan: End-to-end models (e.g., neural planners) that predict paths from observations.
- Sampling Distribution Learning: Learns to bias sampling in RRT/PRM toward promising regions.


## Implementation Considerations

When implementing path planning:

- Data Structures: Use `std::vector<std::vector<int>>` for adjacency lists in C++; prefer size_t for indices.
- Efficiency: Reserve memory to avoid reallocations; use priority queues (e.g., std::priority_queue) for Dijkstra/A*.
- Testing: Return paths as vectors for unit tests; make methods const where possible.
- Distances: Clarify if computing single-source or all-pairs.
- Libraries: Leverage Boost Graph Library, OMPL (Open Motion Planning Library), or ROS Navigation Stack for production code.
- Edge Cases: Handle disconnected graphs, infinite costs, or floating-point precision.


## Challenges and Trade-offs

- High Dimensionality: Curse of dimensionality affects sampling-based methods.
- Dynamic Environments: Requires replanning (e.g., D* over A*).
- Optimality vs. Speed: Classical algorithms are optimal but slow; sampling-based are fast but probabilistic.
- Real-World Integration: Incorporate sensor data, kinematics, and uncertainty.


## Reference

- Russell, S., & Norvig, P. (2020). Artificial Intelligence: A Modern Approach (4th ed.). Pearson. (Chapters on search and planning)
- LaValle, S. M. (2006). Planning Algorithms. Cambridge University Press. (Comprehensive textbook, available online)
- Karaman, S., & Frazzoli, E. (2011). Sampling-based algorithms for optimal motion planning. International Journal of Robotics Research.
- Open Motion Planning Library (OMPL): https://ompl.kavrakilab.org/
ROS Navigation: https://wiki.ros.org/navigation

