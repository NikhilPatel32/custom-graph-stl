#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <iostream>
using namespace std;
#include <unordered_map>
#include <algorithm>
#include <vector>
#include <queue>
#include <stack>
#include <limits>
#include <set>

template <typename T>
class Graph {
private:
    unordered_map<T, vector<pair<T, int>>> adj;
    bool isDirected;

public:
    Graph(bool isDirected = false) : isDirected(isDirected) {}

    void addEdge(const T& u, const T& v, int weight = 1) {
        adj[u].push_back({v, weight});
        if (!isDirected) {
            adj[v].push_back({u, weight});
        }
    }

    void printGraph() const {
        for (const auto& node : adj) {
            cout << node.first << ": ";
            for (const auto& neighbor : node.second) {
                cout << "(" << neighbor.first << ", weight=" << neighbor.second << ") ";
            }
            cout << endl;
        }
    }

    void bfs(const T& start) const {
        unordered_map<T, bool> visited;
        queue<T> q;

        q.push(start);
        visited[start] = true;

        while (!q.empty()) {
            T current = q.front();
            q.pop();
            cout << current << " ";

            for (const auto& neighbor : adj.at(current)) {
                if (!visited[neighbor.first]) {
                    visited[neighbor.first] = true;
                    q.push(neighbor.first);
                }
            }
        }
        cout << endl;
    }

    void dfsHelper(const T& node, unordered_map<T, bool>& visited) const {
        visited[node] = true;
        cout << node << " ";

        for (const auto& neighbor : adj.at(node)) {
            if (!visited[neighbor.first]) {
                dfsHelper(neighbor.first, visited);
            }
        }
    }

    void dfs(const T& start) const {
        unordered_map<T, bool> visited;
        dfsHelper(start, visited);
        cout << endl;
    }

    void shortestPath(const T& src, const T& dest) const {
        unordered_map<T, bool> visited;
        unordered_map<T, T> parent;

        queue<T> q;
        q.push(src);
        visited[src] = true;
        parent[src] = src;

        while (!q.empty()) {
            T current = q.front(); 
            q.pop();

            if (current == dest) break;

            for (const auto& neighbor : adj.at(current)) {
                if (!visited[neighbor.first]) {
                    visited[neighbor.first] = true;
                    parent[neighbor.first] = current;
                    q.push(neighbor.first);
                }
            }
        }

        if (!visited[dest]) {
            cout << "No path exists from " << src << " to " << dest << endl;
            return;
        }

        vector<T> path;
        for (T at = dest; at != src; at = parent[at]) {
            path.push_back(at);
        }
        path.push_back(src);
        reverse(path.begin(), path.end());

        cout << "Shortest path: ";
        for (const T& node : path) {
            cout << node << " ";
        }
        cout << endl;
    }

    void dijkstra(const T& src) const {
        unordered_map<T, int> dist;
        for (const auto& pair : adj) {
            dist[pair.first] = INT_MAX;
        }
        dist[src] = 0;

        set<pair<int, T>> pq;
        pq.insert({0, src});

        while (!pq.empty()) {
            auto [currentDist, u] = *pq.begin();
            pq.erase(pq.begin());

            for (auto [v, weight] : adj.at(u)) {
                if (dist[u] + weight < dist[v]) {
                    pq.erase({dist[v], v});
                    dist[v] = dist[u] + weight;
                    pq.insert({dist[v], v});
                }
            }
        }

        cout << "Dijkstra (shortest distances from " << src << "):\n";
        for (auto [node, d] : dist) {
            cout << node << " -> " << d << "\n";
        }
    }

    void topoSortUtil(const T& node, unordered_map<T, bool>& visited, stack<T>& st) const {
        visited[node] = true;

        for (auto [neigh, _] : adj.at(node)) {
            if (!visited[neigh]) {
                topoSortUtil(neigh, visited, st);
            }
        }
        st.push(node);
    }

    void topologicalSort() const {
        if (!isDirected) {
            cout << "Topological sort only works on directed graphs.\n";
            return;
        }

        unordered_map<T, bool> visited;
        stack<T> st;

        for (auto [node, _] : adj) {
            if (!visited[node]) {
                topoSortUtil(node, visited, st);
            }
        }

        cout << "Topological Sort:\n";
        while (!st.empty()) {
            cout << st.top() << " ";
            st.pop();
        }
        cout << endl;
    }
};

#endif
