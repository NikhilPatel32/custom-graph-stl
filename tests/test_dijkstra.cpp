#include <iostream>
#include "../include/Graph.hpp"
using namespace std;

int main() {
    Graph<int> g(true);

    g.addEdge(0, 1, 4);
    g.addEdge(0, 2, 1);
    g.addEdge(2, 1, 2);
    g.addEdge(1, 3, 1);
    g.addEdge(2, 3, 5);

    g.printGraph();
    g.dijkstra(0);

    return 0;
}
