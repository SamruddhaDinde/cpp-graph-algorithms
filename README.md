# C++ Graph Algorithms Library

A C++ template-based graph library implementing various graph algorithms and operations for directed weighted graphs.

## Features

- **Graph Construction**: Create graphs from scratch or load from edge list files
- **Basic Operations**: Add/remove edges, check edge existence, get edge weights
- **Subgraph Detection**: Check if one graph is a subgraph of another
- **Tree Verification**: Determine if a graph forms a tree structure (with isolated vertices allowed)
- **Path Length Calculation**: Compute shortest path distances from a root vertex using BFS
- **Edge Relaxation Validation**: Verify if distance values satisfy the shortest path property (inspired by Bellman-Ford algorithm)

## Implementation Details

The library uses:
- **Adjacency List** representation with unordered maps for efficient edge lookup
- **BFS (Breadth-First Search)** for tree traversal and cycle detection
- **Bellman-Ford-inspired** approach for validating shortest path distances

## Usage Example
```cpp
#include "graph.hpp"

// Create graph with 5 vertices
Graph<int> G(5);

// Add edges (from, to, weight)
G.addEdge(0, 1, 10);
G.addEdge(1, 2, -5);  // Supports negative weights
G.addEdge(0, 2, 8);

// Check if it's a tree
bool isTree = isTreePlusIsolated(G, 0);

// Calculate path lengths from root
std::vector<int> distances = pathLengthsFromRoot(G, 0);

// Verify shortest path distances
bool relaxed = allEdgesRelaxed(distances, G, 0);
```

## Requirements

- C++20 or later (uses `contains()` method)
- Google Test framework for running tests

## Testing

Includes comprehensive Google Test suite covering edge cases for all major functions.
```bash
g++ -std=c++20 main.cpp -lgtest -lgtest_main -pthread -o test
./test
```

## Resources

Implementation references:
- BFS Algorithm implementation
- Bellman-Ford algorithm concepts
