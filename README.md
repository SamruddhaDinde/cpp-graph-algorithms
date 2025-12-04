# cpp-graph-algorithms
A C++ template-based graph library implementing various graph algorithms and operations for directed weighted graphs.
Features

Graph Construction: Create graphs from scratch or load from edge list files
Basic Operations: Add/remove edges, check edge existence, get edge weights
Subgraph Detection: Check if one graph is a subgraph of another
Tree Verification: Determine if a graph forms a tree structure (with isolated vertices allowed)
Path Length Calculation: Compute shortest path distances from a root vertex using BFS
Edge Relaxation Validation: Verify if distance values satisfy the shortest path property (inspired by Bellman-Ford algorithm)

Implementation Details
The library uses:

Adjacency List representation with unordered maps for efficient edge lookup
BFS (Breadth-First Search) for tree traversal and cycle detection
Bellman-Ford-inspired approach for validating shortest path distances

Testing
Includes comprehensive Google Test suite covering edge cases for all major functions.
Requirements

C++20 or later (uses contains() method)
Google Test framework for running tests
