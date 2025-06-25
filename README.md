# Custom Graph Library in C++ (STL + OOP)

A clean and flexible C++ template-based Graph library using STL. Supports both directed and undirected graphs with common algorithms implemented — perfect for learning and testing.

## 📁 Project Structure
```
custom-graph-stl/
├── include/
│ └── graph.hpp # Header-only Graph class
├── tests/
│ ├── test_basic.cpp # BFS, DFS, shortest path
│ ├── test_dijkstra.cpp # Dijkstra's algorithm
│ └── test_toposort.cpp # Topological sort
├── Makefile # Compile & run targets
```

## Features

- `addEdge(u, v, weight)` — supports weighted/unweighted edges
- `printGraph()` — prints adjacency list
- `bfs(start)` — Breadth-First Search
- `dfs(start)` — Depth-First Search (recursive)
- `shortestPath(src, dest)` — unweighted shortest path (BFS-based)
- `dijkstra(src)` — shortest distances using Dijkstra’s algorithm
- `topologicalSort()` — Kahn’s DFS-based Topo Sort (only for directed graphs)

##  Test Instructions

###  Build All
```bash
make
make run_basic
make run_dijkstra
make run_toposort
make clean
