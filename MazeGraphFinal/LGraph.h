#pragma once

#include <iostream>
#include <limits>
#include <vector>
#include <list>
#include "LEdge.h"
#include "LVertex.h"

const int INF = numeric_limits<int>::max() / 2;

using namespace std;

enum class TYPE {
    DIRECTED, UNDIRECTED
};

template<class T>
class LGraph {
private:
    TYPE type = TYPE::UNDIRECTED;
    int numberOfNodes;
    int numberOfEdges;
    vector<LEdge<T>> edges;
    vector<LVertex<T>> V;  // array of nodes

    // dfs
    vector<bool> visited;
    vector<int> prv;

    // Dijkstra
    vector<int> dist;
    vector<int> previous;

    // in degree
    vector<int> in;

    vector<vector<int>> adj;
    // reverse adjacency vertices
    vector<vector<int>> revAdj;

public:
    LGraph(TYPE type) {
        this->type = type;
        numberOfNodes = 0;
        numberOfEdges = 0;
    };

    TYPE getType() { return type; }

    int getNumberOfNodes() { return numberOfNodes; }
    int getNumberOfEdges() { return numberOfEdges; }

    int indexIs(T);
    T valueIs(int);

    void addNode(const T& nodeName);
    void addEdge(const string&, const T&, const T&, int);
    void updateNodeName(const T&, const T&);
    void updateEdgeWeight(const string&, int);

    void markVisitedAsFalse();
    vector<pair<bool, int>> BFS(const T&);

    void DFS(const T&);
    void visit(int);

    vector<bool> getVisitedDFS() { return visited; }
    vector<int> getPrvDFS() { return prv; }

    void dijkstraPath(const T&);
    void shortestPath(const T&, const T&);

    vector<int> eulerianPathForUG();
    bool isEulerianCircuitForUG();

    void DFSUtil(int u, bool*);
    void revDFSUtil(int u, bool*);
    bool isConnected();
    bool isEulerianCircuitForDG();

    bool isEulerianCircuit();

    void displayAdjList();
    void displayNodesName();
    void displayEdgesName();
    void displayCost();
    void displayAdj();
    void displayRevAdj();
};

template<class T>
int LGraph<T>::indexIs(T name) {
    int size = V.size();
    for (int i = 0; i < size; i++) {
        if (V[i].getVertexName() == name) {
            return i;
        }
    }
    return -1;
}

template<class T>
T LGraph<T>::valueIs(int index) {
    return V[index].getVertexName();
}

template<class T>
void LGraph<T>::addNode(const T& nodeName) {
    numberOfNodes++;
    LVertex<T> vertex(nodeName);
    vector<int> innerAdj;
    vector<int> innerRevAdj;
    adj.push_back(innerAdj);
    if (type == TYPE::DIRECTED) {
        revAdj.push_back(innerRevAdj);
        in.push_back(0);
    }
    V.push_back(vertex);
};

template<class T>
void LGraph<T>::addEdge(const string& name, const T& src, const T& dest, int weight) {
    numberOfEdges++;
    int srcIndex = indexIs(src);
    int destIndex = indexIs(dest);
    if (srcIndex == -1) {
        cout << "Source node does not exist" << endl;
        return;
    }
    if (destIndex == -1) {
        cout << "Destination node does not exist" << endl;
        return;
    }
    if (type == TYPE::DIRECTED) {
        adj[srcIndex].push_back(destIndex);
        revAdj[destIndex].push_back(srcIndex);
        (in[destIndex])++;
        LNode<T>* node = new LNode<T>(dest, weight, NULL);
        if (V[srcIndex].node == NULL) {
            V[srcIndex].node = node;
        }
        else {
            LNode<T>* p = V[srcIndex].node;
            while (p->next != NULL) {
                p = p->next;
            }
            p->next = node;
        }
    }
    else if (type == TYPE::UNDIRECTED) {
        adj[srcIndex].push_back(destIndex);
        LNode<T>* nodeDest = new LNode<T>(dest, weight, NULL, NULL);
        LNode<T>* nodeSrc = new LNode<T>(src, weight, NULL, NULL);
        if (V[srcIndex].node == NULL) {
            V[srcIndex].node = nodeDest;
        }
        else {
            LNode<T>* p = V[srcIndex].node;
            while (p->next != NULL) {
                p = p->next;
            }
            p->next = nodeDest;
        }
        if (V[destIndex].node == NULL) {
            V[destIndex].node = nodeSrc;
        }
        else {
            LNode<T>* p = V[destIndex].node;
            while (p->prev != NULL) {
                p = p->prev;
            }
            p->prev = nodeSrc;
        }
    }
    LEdge<T> edge(name, src, dest, weight);
    edges.push_back(edge);
};

template<class T>
void LGraph<T>::updateNodeName(const T& oldName, const T& newName) {
    int index = indexIs(oldName);
    if (index == -1) {
        cout << "Node '" << oldName << "' does not exist!" << endl;
        return;
    }
    int size = V.size();
    for (int i = 0; i < size; i++) {
        LNode<T>* ptr = V[i].node;
        while (ptr != NULL) {
            if (ptr->getValue() == oldName) {
                ptr->setValue(newName);
            }
            ptr = ptr->next;
        }
        ptr = V[i].node;
        while (ptr->prev != NULL) {
            if (ptr->getValue() == oldName) {
                ptr->setValue(newName);
            }
            ptr = ptr->prev;
        }
    }

    for (int i = 0; i < numberOfEdges; i++) {
        if (edges[i].getSource() == oldName) {
            edges[i].setSource(newName);
        }
        if (edges[i].getDestination() == oldName) {
            edges[i].setDestination(newName);
        }
    }

    V[index].setVertexName(newName);
    cout << "Updated node" << endl;
}

template<class T>
void LGraph<T>::updateEdgeWeight(const string& name, int weight) {
    for (int i = 0; i < numberOfEdges; i++) {
        if (edges[i].getName() == name) {
            int size = V.size();
            for (int i = 0; i < size; i++) {
                LNode<T>* ptr = V[i].node;
                if (V[i].getVertexName() == edges[i].getSource()) {
                    while (ptr != NULL) {
                        if (ptr->getValue() == edges[i].getDestination()) {
                            ptr->setCost(weight);
                        }
                        ptr = ptr->next;
                    }
                }
                if (type == TYPE::UNDIRECTED) {
                    ptr = V[i].node;
                    if (V[i].getVertexName() == edges[i].getDestination()) {
                        while (ptr != NULL) {
                            if (ptr->getValue() == edges[i].getSource()) {
                                ptr->setCost(weight);
                            }
                            ptr = ptr->prev;
                        }
                    }
                }

            }
            edges[i].setWeight(weight);
        }
    }
    cout << "Updated edge" << endl;
}

template<class T>
void LGraph<T>::markVisitedAsFalse() {
    for (int i = 0; i < numberOfNodes; i++) {
        visited[i] = false;
    }
}

template<class T>
vector<pair<bool, int>> LGraph<T>::BFS(const T& start) {
    int s = indexIs(start);
    vector<pair<bool, int>> visited;
    visited.resize(numberOfNodes);
    for (int i = 0; i < numberOfNodes; i++) {
        visited[i].first = false;
    }
    if (s == -1) {
        cout << "Start node does not exist" << endl;
        return visited;
    }
    visited[s].first = true;
    visited[s].second = -1;

    list<int> Queue;
    Queue.push_back(s);
    int u;
    cout << "Breath first search: ";
    while (!Queue.empty()) {
        u = Queue.front();
        cout << valueIs(u) << " ";
        Queue.pop_front();
        for (LNode<T>* vertex = V[u].node; vertex != NULL; vertex = vertex->next) {
            int vertexIndex = indexIs(vertex->getValue());
            if (!visited[vertexIndex].first) {
                visited[vertexIndex].first = true;
                visited[vertexIndex].second = u;
                Queue.push_back(vertexIndex);
            }
        }
        if (type == TYPE::UNDIRECTED) {
            for (LNode<T>* vertex = V[u].node->prev; vertex != NULL; vertex = vertex->prev) {
                int vertexIndex = indexIs(vertex->getValue());
                if (!visited[vertexIndex].first) {
                    visited[vertexIndex].first = true;
                    visited[vertexIndex].second = u;
                    Queue.push_back(vertexIndex);
                }
            }
        }
    }
    cout << endl;
    return visited;
}

template<class T>
void LGraph<T>::DFS(const T& start) {
    int s = indexIs(start);
    if (s == -1) {
        cout << "Start node does not exist" << endl;
        return;
    }
    visited.resize(numberOfNodes);
    markVisitedAsFalse();
    prv.resize(numberOfNodes);
    prv[s] = -1;
    cout << "DFS search: ";
    visit(s);
    cout << endl;
}

template<class T>
void LGraph<T>::visit(int u) {
    visited[u] = true;
    cout << valueIs(u) << " ";
    for (LNode<T>* vertex = V[u].node; vertex != NULL; vertex = vertex->next) {
        int vertexIndex = indexIs(vertex->getValue());
        if (!visited[vertexIndex]) {
            prv[vertexIndex] = u;
            visit(vertexIndex);
        }
    }
    if (type == TYPE::UNDIRECTED) {
        for (LNode<T>* vertex = V[u].node->prev; vertex != NULL; vertex = vertex->prev) {
            int vertexIndex = indexIs(vertex->getValue());
            if (!visited[vertexIndex]) {
                prv[vertexIndex] = u;
                visit(vertexIndex);
            }
        }
    }
}

template<class T>
void LGraph<T>::dijkstraPath(const T& start) {
    int s = indexIs(start);
    if (s == -1) {
        cout << "Start vertex does not exist" << endl;
        return;
    }
    vector<int> Temp;
    vector<int>::iterator it;
    int minEdge, postMinEdge;
    dist.resize(numberOfNodes);
    previous.resize(numberOfNodes);
    for (int i = 0; i < numberOfNodes; i++) {
        dist[i] = INF;
    }
    dist[s] = 0;
    previous[s] = -1;
    for (int i = 0; i < numberOfNodes; i++) {
        Temp.push_back(i);
    }
    while (!Temp.empty()) {
        it = Temp.begin();
        minEdge = Temp[0];
        postMinEdge = 0;
        int size = Temp.size();
        for (int i = 1; i < size; i++) {
            if (dist[Temp[i]] < dist[minEdge]) {
                minEdge = Temp[i];
                postMinEdge = i;
            }
        }
        it += postMinEdge;
        Temp.erase(it);
        for (LNode<T>* vertex = V[minEdge].node; vertex != NULL; vertex = vertex->next) {
            int vertexIndex = indexIs(vertex->getValue());
            int weight = vertex->getCost();

            if (dist[vertexIndex] > dist[minEdge] + weight) {
                dist[vertexIndex] = dist[minEdge] + weight;
                previous[vertexIndex] = minEdge;
            }
        }
        if (type == TYPE::UNDIRECTED) {
            for (LNode<T>* vertex = V[minEdge].node->prev; vertex != NULL; vertex = vertex->prev) {
                int vertexIndex = indexIs(vertex->getValue());
                int weight = vertex->getCost();

                if (dist[vertexIndex] > dist[minEdge] + weight) {
                    dist[vertexIndex] = dist[minEdge] + weight;
                    previous[vertexIndex] = minEdge;
                }
            }
        }
    }
}

template<class T>
void LGraph<T>::shortestPath(const T& v, const T& w) {
    cout << "-----------------Shortest path from " << v << " to " << w << " -----------------\n";
    int srcIndex = indexIs(v);
    int destIndex = indexIs(w);
    if (v == w) {
        cout << v << endl;
    }
    dijkstraPath(v);
    cout << "Path cost: " << dist[destIndex] << endl;
    cout << "Privious node: " << previous[destIndex] << endl;
}

template<class T>
vector<int> LGraph<T>::eulerianPathForUG() {
    int startIndex = 0;
    int x, y;
    vector<int> CE;
    list<int> Stack;
    vector<vector<int>> adj(numberOfNodes);
    int size = V.size();
    for (int i = 0; i < size; i++) {
        LNode<T>* node = V[i].node;
        while (node != NULL) {
            adj[i].push_back(indexIs(node->getValue()));
            node = node->next;
        }
        node = V[i].node->prev;
        while (node != NULL) {
            adj[i].push_back(indexIs(node->getValue()));
            node = node->prev;
        }
    }

    for (int i = 0; i < size; i++) {
        cout << i << " --> ";
        for (auto j = adj[i].begin(); j != adj[i].end(); j++) {
            cout << (*j) << " ";
        }
        cout << endl;
    }
    Stack.push_back(startIndex);
    while (!Stack.empty()) {
        x = Stack.back();
        if (!adj[x].empty()) {
            y = adj[x].back();
            Stack.push_back(y);
            adj[x].pop_back();
            for (auto i = adj[y].begin(); i != adj[y].end(); i++) {
                if ((*i) == x) {
                    adj[y].erase(i);
                    break;
                }
            }
        }
        else {
            Stack.pop_back();
            CE.push_back(x);
        }
    }
    return CE;
}

template<class T>
bool LGraph<T>::isEulerianCircuitForUG() {
    vector<int> path = eulerianPathForUG();
    vector<LEdge<T>> allEdge;
    bool same = false, empty = false;
    auto s = path.begin();
    auto e = path.end() - 1;
    if ((*s) == (*e)) same = true;
    for (int i = 0; i < numberOfEdges; i++) {
        allEdge.push_back(edges[i]);
    }
    for (auto i = path.end() - 1; i != path.begin(); i--) {
        for (auto j = allEdge.begin(); j != allEdge.end(); j++) {
            if ((*j).getSource() == valueIs((*i)) && (*j).getDestination() == valueIs(*(i - 1))) {
                allEdge.erase(j);
                j = allEdge.begin();
                if (allEdge.size() == 0) {
                    empty = true;
                    break;
                }
            }
            else if ((*j).getSource() == valueIs(*(i - 1)) && (*j).getDestination() == valueIs((*i))) {
                allEdge.erase(j);
                j = allEdge.begin();
                if (allEdge.size() == 0) {
                    empty = true;
                    break;
                }
            }
        }
    }
    if (empty && same) {
        return true;
    }
    return false;
}

template<class T>
void LGraph<T>::DFSUtil(int u, bool* visited) {
    visited[u] = true;
    for (auto i = adj[u].begin(); i != adj[u].end(); i++) {
        if (!visited[*i]) {
            DFSUtil(*i, visited);
        }
    }
}

template<class T>
void LGraph<T>::revDFSUtil(int u, bool* visited) {
    visited[u] = true;
    for (auto i = revAdj[u].begin(); i != revAdj[u].end(); i++) {
        if (!visited[*i]) {
            revDFSUtil(*i, visited);
        }
    }
}

template<class T>
bool LGraph<T>::isConnected() {
    bool* visited = (bool*)malloc(numberOfNodes * sizeof(bool));
    for (int i = 0; i < numberOfNodes; i++) {
        visited[i] = false;
    }
    cout << endl;
    int n;
    for (n = 0; n < numberOfNodes; n++) {
        if (adj[n].size() > 0) break;
    }
    DFSUtil(n, visited);

    for (int i = 0; i < numberOfNodes; i++) {
        if (adj[i].size() > 0 && !visited[i])
            return false;
    }

    for (int i = 0; i < numberOfNodes; i++) {
        visited[i] = false;
    }
    revDFSUtil(n, visited);

    for (int i = 0; i < numberOfNodes; i++) {
        if (adj[i].size() > 0 && !visited[i])
            return false;
    }
    return true;
}

template<class T>
bool LGraph<T>::isEulerianCircuitForDG() {
    bool ic = isConnected();
    if (ic == false) {
        return false;
    }
    for (int i = 0; i < numberOfNodes; i++) {
        if (adj[i].size() != in[i]) {
            return false;
        }
    }
    return true;
}

template<class T>
bool LGraph<T>::isEulerianCircuit() {
    if (type == TYPE::UNDIRECTED) {
        return isEulerianCircuitForUG();
    }
    else if (type == TYPE::DIRECTED) {
        return isEulerianCircuitForDG();
    }
    return false;
}

template<class T>
void LGraph<T>::displayAdjList() {
    for (int i = 0; i < V.size(); i++) {
        LNode<T>* ptr = V[i].node;
        cout << V[i].getVertexName() << " --> ";
        if (type == TYPE::DIRECTED) {
            while (ptr != NULL) {
                cout << ptr->getValue() << " ";
                ptr = ptr->next;
            }
            cout << endl;
        }
        else {
            while (ptr != NULL) {
                cout << ptr->getValue() << " ";
                ptr = ptr->next;
            }
            ptr = V[i].node->prev;
            while (ptr != NULL) {
                cout << ptr->getValue() << " ";
                ptr = ptr->prev;
            }
            cout << endl;
        }
    }
}

template<class T>
void LGraph<T>::displayNodesName() {
    cout << "Nodes: ";
    for (int i = 0; i < V.size(); i++) {
        cout << V[i].getVertexName() << " ";
    }
    cout << endl;
}

template<class T>
void LGraph<T>::displayEdgesName() {
    cout << "Edges: ";
    for (int i = 0; i < numberOfEdges; i++) {
        cout << edges[i].getName() << " ";
    }
    cout << endl;
}

template<class T>
void LGraph<T>::displayCost() {
    for (int i = 0; i < V.size(); i++) {
        LNode<T>* ptr = V[i].node;
        cout << V[i].getVertexName() << " --> ";
        while (ptr != NULL) {
            cout << ptr->getCost() << " ";
            ptr = ptr->next;
        }
        if (type == TYPE::UNDIRECTED) {
            ptr = V[i].node->prev;
            while (ptr != NULL) {
                cout << ptr->getCost() << " ";
                ptr = ptr->prev;
            }
        }
        cout << endl;
    }
}

template<class T>
void LGraph<T>::displayAdj() {
    cout << "Display adj node:\n";
    for (int i = 0; i < numberOfNodes; i++) {
        cout << i << " --> ";
        for (auto j = adj[i].begin(); j != adj[i].end(); j++) {
            cout << *j << " ";
        }
        cout << endl;
    }
}

template<class T>
void LGraph<T>::displayRevAdj() {
    cout << "Display revAdj node:\n";
    for (int i = 0; i < numberOfNodes; i++) {
        cout << i << " --> ";
        for (auto j = revAdj[i].begin(); j != revAdj[i].end(); j++) {
            cout << *j << " ";
        }
        cout << endl;
    }
}