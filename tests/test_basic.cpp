#include <iostream>
#include "../include/Graph.hpp"
using namespace std;

int main() {
    Graph<int> g;

    g.addEdge(1, 2);
    g.addEdge(1, 3);
    g.addEdge(2, 4);
    g.addEdge(3, 4);
    g.addEdge(4, 5);

    cout << "Graph Structure:\n";
    g.printGraph();

    cout << "\nBFS from node 1:\n";
    g.bfs(1);

    cout << "\nDFS from node 1:\n";
    g.dfs(1);

    cout << "\nShortest path from 1 to 5:\n";
    g.shortestPath(1, 5);

    return 0;
}
