
#include <vector>
#include <iostream>
#include <list>
#include <string>
#include "Node.h"
#include "Edge.h"

using namespace std;



template<class T>
class Graph {
protected:
    TYPE type = TYPE::UNDIRECTED;
    int numberOfNodes;
    int numberOfEdges;
    int maxNodes;
    vector<Node<T>*> vertices;
    vector<Edge<T>> edges;
    vector<vector<int>> weightMatrix;
    vector<vector<int>> adjMatrix;
    vector<vector<int>> incidenceMatrix;

    vector<bool> visited;
    vector<int> path;

    // use for Floyd Warshall algorithm
    vector<vector<int>> dist;
    vector<vector<int>> p;

    // in degree
    vector<int> in;

    vector<vector<int>> adj;
    // reverse adjacency vertices
    vector<vector<int>> revAdj;

public:
    Graph(int size) {
        this->type = type;
        numberOfNodes = 0;
        numberOfEdges = 0;
        maxNodes = size;
        in.resize(size, 0);
        adj.resize(size);
        revAdj.resize(size);
        vertices.resize(size);
        visited.resize(size, false);
        weightMatrix.resize(size, vector<int>(size, INF));
        adjMatrix.resize(size, vector<int>(size, 0));
        for (int i = 0; i < size; i++) {
            vertices[i] = NULL;
        }
        for (int i = 0; i < size; i++) {
            weightMatrix[i][i] = 0;
        }
    };
    Graph(TYPE type, int size) {
        this->type = type;
        numberOfNodes = 0;
        numberOfEdges = 0;
        maxNodes = size;
        in.resize(size, 0);
        adj.resize(size);
        revAdj.resize(size);
        vertices.resize(size);
        visited.resize(size, false);
        weightMatrix.resize(size, vector<int>(size, INF));
        adjMatrix.resize(size, vector<int>(size, 0));
        for (int i = 0; i < size; i++) {
            vertices[i] = NULL;
        }
        for (int i = 0; i < size; i++) {
            weightMatrix[i][i] = 0;
        }
    };


    TYPE getType() const { return type; }

    int getIndexOfNode(const string& name) const;
    string getNodeName(int index) const;

    int getNumberOfNodes() const { return numberOfNodes; }
    int getNumberOfEdges() const { return numberOfEdges; }

    void addNode(const string& name, const T& value);
    void updateNode(const string& nodeName, const T& newValue);
    Node<T>* getNode(int index) const { return vertices[index]; }

    void addEdge(const string& name, const string& src, const string& dest);
    void addEdge(const string& name, const string& src, const string& dest, int weight);
    void updateWeight(const string& edgeName, int Weight);

    vector<vector<int>> getAdj() const;
    vector<vector<int>> getAdjMatrix() const;

    vector<pair<bool, int>> BFS(const string& start) const;
    vector<pair<bool, int>> DFS(const string& start) const;
    bool DFS(int start, int goal);

    vector<pair<int, int>> dijsktra(const string& start);
    void floydWarshall();
    void printPath(int, int);
    void findPathFW(const string& src, const string& dest);

    void DFSUtil(int, bool*);
    void revDFSUtil(int, bool*);
    bool isConnected();
    bool isEulerianCircuitForDG();

    vector<int> eulerianPathForUG() const;
    bool isEulerianCircuitForUG();
    bool isEulerianCircuit();

    void displayNode(const string& name);
    void displayEdge(const string& name);
    void displayNodes();
    void displayEdges();
    void displayWeightMatrix();
    void displayAdjMatrix();
    void displayIncidenceMatrix();
    void displayAdj();
    void displayRevAdj();
};

template<class T>
int Graph<T>::getIndexOfNode(const string& name) const {
    for (int i = 0; i < numberOfNodes; i++) {
        if (vertices[i]->getName() == name) return i;
    }
    return -1;
}

template<class T>
string Graph<T>::getNodeName(int index) const {
    return vertices[index]->getName();
}

template<class T>
void Graph<T>::addNode(const string& name, const T& value) {
    if (numberOfNodes == maxNodes) {
        cout << "vertices is full" << endl;
        return;
    }
    vertices[numberOfNodes] = new Node<T>(name, value);
    numberOfNodes++;
}

template<class T>
void Graph<T>::updateNode(const string& nodeName, const T& newValue) {
    int index = getIndexOfNode(nodeName);
    if (index == -1) {
        cout << "Vertex '" << nodeName << "' does not exist!\n";
        return;
    }
    vertices[index]->setValue(newValue);
    cout << "Updated vertex '" << nodeName << "'\n";
}


template<class T>
void Graph<T>::addEdge(const string& name, const string& src, const string& dest) {
    int srcIndex = getIndexOfNode(src);
    int destIndex = getIndexOfNode(dest);
    if (srcIndex == -1) {
        cout << "Source vertex does not exist" << endl;
        return;
    }
    if (destIndex == -1) {
        cout << "Destination vertex does not exist" << endl;
        return;
    }
    adj[srcIndex].push_back(destIndex);
    adjMatrix[srcIndex][destIndex]++;
    if (type == TYPE::DIRECTED) {
        revAdj[destIndex].push_back(srcIndex);
        (in[destIndex])++;
    }
    if (type == TYPE::UNDIRECTED) {
        adj[srcIndex].push_back(destIndex);
        adjMatrix[destIndex][srcIndex]++;
    }
    Edge<T> edge(name, src, dest);
    edges.push_back(edge);
    numberOfEdges++;
}

template<class T>
void Graph<T>::addEdge(const string& name, const string& src, const string& dest, int weight) {
    int srcIndex = getIndexOfNode(src);
    int destIndex = getIndexOfNode(dest);
    if (srcIndex == -1) {
        cout << "Source vertex does not exist" << endl;
        return;
    }
    if (destIndex == -1) {
        cout << "Destination vertex does not exist" << endl;
        return;
    }
    weightMatrix[srcIndex][destIndex] = weight;
    adjMatrix[srcIndex][destIndex]++;
    adj[srcIndex].push_back(destIndex);

    if (type == TYPE::DIRECTED) {
        revAdj[destIndex].push_back(srcIndex);
        (in[destIndex])++;
    }
    if (type == TYPE::UNDIRECTED) {
        weightMatrix[destIndex][srcIndex] = weight;
        adjMatrix[destIndex][srcIndex]++;
        adj[destIndex].push_back(srcIndex);
    }
    Edge<T> edge(name, src, dest, weight);
    edges.push_back(edge);
    numberOfEdges++;
}

template<class T>
void Graph<T>::updateWeight(const string& edgeName, int weight) {
    for (int i = 0; i < numberOfEdges; i++) {
        if (edges[i].getName() == edgeName) {
            edges[i].setWeight(weight);
            string srcVertex = edges[i].getSrcVertex();
            string destVertex = edges[i].getDestVertex();
            int srcIndex = getIndexOfNode(srcVertex);
            int destIndex = getIndexOfNode(destVertex);
            weightMatrix[srcIndex][destIndex] = weight;
            cout << "Updated edges's weight\n";
            break;
        }
    }
}

template<class T>
vector<vector<int>> Graph<T>::getAdj() const {
    return adj;
}

template<class T>
vector<vector<int>> Graph<T>::getAdjMatrix() const {
    return adjMatrix;
}

template<class T>
vector<pair<bool, int>> Graph<T>::BFS(const string& start) const {
    cout << "-------------------------------------------------------------------" << endl;
    int s = getIndexOfNode(start);
    vector<pair<bool, int>> visited;
    visited.resize(numberOfNodes);
    for (int i = 0; i < numberOfNodes; i++) {
        visited[i].first = false;
    }

    if (s == -1) {
        cout << "Start vertex does not exist" << endl;
        return visited;
    }
    visited[s].first = true;
    visited[s].second = -1;
    list<int> Queue;
    Queue.push_back(s);
    cout << "Breath first search: ";
    while (!Queue.empty()) {
        int u = Queue.front();
        cout << getNodeName(u) << " ";
        Queue.pop_front();
        for (auto v = adj[u].begin(); v != adj[u].end(); v++) {
            if (visited[*v].first == false) {
                visited[*v].first = true;
                visited[*v].second = u;
                Queue.push_back(*v);
            }
        }
    }
    cout << endl;
    return visited;
}

template<class T>
vector<pair<bool, int>> Graph<T>::DFS(const string& start) const {
    cout << "-------------------------------------------------------------------" << endl;
    int s = getIndexOfNode(start);
    int u;
    list<int> Stack;
    vector<pair<bool, int>> visited;
    visited.resize(numberOfNodes);
    for (int i = 0; i < numberOfNodes; i++) {
        visited[i].first = false;
    }

    if (s == -1) {
        cout << "Start vertex does not exist" << endl;
        return visited;
    }
    visited[s].first = true;
    visited[s].second = -1;
    Stack.push_back(-1);
    u = s;
    cout << "Depth first search: ";
    cout << getNodeName(u) << " ";
    while (u != -1) {
        auto v = adj[u].begin();
        while (v != adj[u].end()) {
            if (visited[*v].first == false) {
                visited[*v].first = true;
                visited[*v].second = u;
                Stack.push_back(u);
                u = *v;
                v = adj[u].begin();
                cout << getNodeName(u) << " ";
            }
            else v++;
        }
        u = Stack.back();
        Stack.pop_back();
    }
    cout << endl;
    return visited;
}

template<class T>
bool Graph<T>::DFS(int start, int goal) {
    path.push_back(start);
    visited[start] = true;
    if (start == goal) return true;
    for (auto i = adj[start].begin(); i != adj[start].end(); i++) {
        if (visited[*i] == false) {
            if (DFS(*i, goal)) return true;
        }
    }
    path.pop_back();
    return false;
}

template<class T>
vector<pair<int, int>> Graph<T>::dijsktra(const string& start) {
    vector<pair<int, int>> dist;    // store pair of distance and previous vertex
    int s = getIndexOfNode(start);
    if (s == -1) {
        cout << "Start vertex does not exist" << endl;
        return dist;
    }
    int weight = edges[0].getWeight();
    if (weight == NONE) {
        cout << "This graph is unweighted! Can't use dijsktra algorithm.\n";
        return dist;
    }
    int min, minPost;
    vector<int> Tmp;
    vector<int>::iterator it;
    dist.resize(numberOfNodes);
    for (int i = 0; i < numberOfNodes; i++) {
        dist[i].first = INF;   // initialize infinity
    }
    dist[s].first = 0;      // start vertex has weight 0
    dist[s].second = -1;
    for (int i = 0; i < numberOfNodes; i++) {   // all vertices index
        Tmp.push_back(i);
    }
    while (!Tmp.empty()) {
        it = Tmp.begin();
        min = Tmp[0];      // mark 1st element is min
        minPost = 0;
        for (int i = 1; i < Tmp.size(); i++) {  // find element that have min distance
            if (dist[Tmp[i]].first < dist[min].first) {
                min = Tmp[i];
                minPost = i;
            }
        }
        it += minPost;
        Tmp.erase(it);  // remove min element from Tmp
        for (int i = 0; i < numberOfNodes; i++) {       // update new distance if can
            if (dist[i].first > dist[min].first + weightMatrix[min][i]) {
                dist[i].first = dist[min].first + weightMatrix[min][i];
                dist[i].second = min;
            }
        }
    }
    return dist;
}

template<class T>
void Graph<T>::floydWarshall() {
    dist.resize(numberOfNodes, vector<int>(numberOfNodes));
    p.resize(numberOfNodes, vector<int>(numberOfNodes));
    for (int i = 0; i < numberOfNodes; i++) {
        for (int j = 0; j < numberOfNodes; j++) {
            dist[i][j] = weightMatrix[i][j];
            if (i == j || weightMatrix[i][j] == INF) {
                p[i][j] = -1;
            }
            else if (i != j || weightMatrix[i][j] < INF) {
                p[i][j] = i;
            }
        }
    }

    for (int k = 0; k < numberOfNodes; k++) {
        for (int i = 0; i < numberOfNodes; i++) {
            for (int j = 0; j < numberOfNodes; j++) {
                if (dist[i][j] > dist[i][k] + dist[k][j]) {
                    dist[i][j] = dist[i][k] + dist[k][j];
                    p[i][j] = p[k][j];
                }
            }
        }
    }
}

template<class T>
void Graph<T>::printPath(int i, int j) {
    if (i == j) cout << getNodeName(i) << " ";
    else if (p[i][j] == -1) cout << "Path does not exist!\n";
    else {
        printPath(i, p[i][j]);
        cout << getNodeName(j) << " ";
    }
}

template<class T>
void Graph<T>::findPathFW(const string& src, const string& dest) {
    int srcIndex = getIndexOfNode(src);
    int destIndex = getIndexOfNode(dest);
    if (edges[0].getWeight() == NONE) {
        cout << "This graph is unweighted! Can't find shortest path.\n";
        return;
    }
    floydWarshall();
    cout << "Shortest distance form " << src << " to " << dest << ": ";
    cout << dist[srcIndex][destIndex] << endl;
    cout << "Shortest path form " << src << " to " << dest << ": ";
    printPath(srcIndex, destIndex);
}

template<class T>
void Graph<T>::DFSUtil(int u, bool* visited) {
    visited[u] = true;
    for (auto i = adj[u].begin(); i != adj[u].end(); i++) {
        if (!visited[*i]) {
            DFSUtil(*i, visited);
        }
    }
}

template<class T>
void Graph<T>::revDFSUtil(int u, bool* visited) {
    visited[u] = true;
    for (auto i = revAdj[u].begin(); i != revAdj[u].end(); i++) {
        if (!visited[*i]) {
            revDFSUtil(*i, visited);
        }
    }
}

template<class T>
bool Graph<T>::isConnected() {
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
bool Graph<T>::isEulerianCircuitForDG() {
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
vector<int> Graph<T>::eulerianPathForUG() const {
    int startIndex = 0;
    int x, y;
    vector<int> CE;
    vector<vector<int>> adj(numberOfNodes);
    for (int i = 0; i < numberOfNodes; i++) {
        for (int j = 0; j < numberOfNodes; j++) {
            if (adjMatrix[i][j]) {
                adj[i].push_back(j);
            }
        }
    }
    list<int> Stack;
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
bool Graph<T>::isEulerianCircuitForUG() {
    vector<int> path = eulerianPathForUG();
    vector<Edge<T>> allEdge;
    bool same = false, empty = false;
    auto s = path.begin();
    auto e = path.end() - 1;
    if ((*s) == (*e)) same = true;
    for (int i = 0; i < numberOfEdges; i++) {
        allEdge.push_back(edges[i]);
    }
    for (auto i = path.end() - 1; i != path.begin(); i--) {
        for (auto j = allEdge.begin(); j != allEdge.end(); j++) {
            if ((*j).getSrcVertex() == getNodeName((*i)) && (*j).getDestVertex() == getNodeName(*(i - 1))) {
                allEdge.erase(j);
                j = allEdge.begin();
                if (allEdge.size() == 0) {
                    empty = true;
                    break;
                }
            }
            else if ((*j).getSrcVertex() == getNodeName(*(i - 1)) && (*j).getDestVertex() == getNodeName((*i))) {
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
bool Graph<T>::isEulerianCircuit() {
    if (type == TYPE::UNDIRECTED) {
        return isEulerianCircuitForUG();
    }
    else if (type == TYPE::DIRECTED) {
        return isEulerianCircuitForDG();
    }
    return false;
}

template<class T>
void Graph<T>::displayNode(const string& name) {
    int index = getIndexOfNode(name);
    if (index == -1) {
        cout << "Node '" << name << "' does not exist\n!";
        return;
    }
    cout << "Name: " << name << endl;
    cout << "Value: " << vertices[index]->getValue() << endl;
}

template<class T>
void Graph<T>::displayEdge(const string& name) {
    for (int i = 0; i < numberOfEdges; i++) {
        if (edges[i].getName() == name) {
            cout << "Name: " << name << endl;
            cout << "Source vertex: " << edges[i].getSrcVertex() << endl;
            cout << "Destination vertex: " << edges[i].getDestVertex() << endl;
        }
    }
}

template<class T>
void Graph<T>::displayNodes() {
    cout << "-------------------------------------------------------------------" << endl;
    for (int i = 0; i < numberOfNodes; i++) {
        cout << vertices[i]->getName() << " ";
    }
    cout << endl;
}

template<class T>
void Graph<T>::displayEdges() {
    cout << "-------------------------------------------------------------------" << endl;
    for (int i = 0; i < numberOfEdges; i++) {
        cout << edges[i].getName();
        cout << ": " << edges[i].getSrcVertex();
        cout << " -> " << edges[i].getDestVertex();
        cout << endl;
    }
}

template<class T>
void Graph<T>::displayWeightMatrix() {
    cout << "-------------------------------------------------------------------" << endl;
    for (int i = 0; i < numberOfNodes; i++) {
        for (int j = 0; j < numberOfNodes; j++) {
            cout << weightMatrix[i][j] << "\t";
        }
        cout << endl;
    }
}

template<class T>
void Graph<T>::displayAdjMatrix() {
    cout << "-------------------------------------------------------------------" << endl;
    for (int i = 0; i < numberOfNodes; i++) {
        for (int j = 0; j < numberOfNodes; j++) {
            cout << adjMatrix[i][j] << "\t";
        }
        cout << endl;
    }
}

template<class T>
void Graph<T>::displayIncidenceMatrix() {
    incidenceMatrix.resize(numberOfNodes, vector<int>(numberOfEdges, 0));
    if (type == TYPE::UNDIRECTED) {
        for (int i = 0; i < numberOfNodes; i++) {
            for (int j = 0; j < numberOfEdges; j++) {
                if (edges[j].getSrcVertex() == vertices[i]->getName()) {
                    incidenceMatrix[i][j] = 1;
                }
                if (edges[j].getDestVertex() == vertices[i]->getName()) {
                    incidenceMatrix[i][j] = 1;
                }
            }
        }
    }
    else if (type == TYPE::DIRECTED) {
        for (int i = 0; i < numberOfNodes; i++) {
            for (int j = 0; j < numberOfEdges; j++) {
                if (edges[j].getSrcVertex() == vertices[i]->getName()) {
                    incidenceMatrix[i][j] = 1;
                }
                if (edges[j].getDestVertex() == vertices[i]->getName()) {
                    incidenceMatrix[i][j] = -1;
                }
            }
        }
    }

    for (int i = 0; i < numberOfNodes; i++) {
        for (int j = 0; j < numberOfEdges; j++) {
            cout << incidenceMatrix[i][j] << "\t";
        }
        cout << endl;
    }
}

template<class T>
void Graph<T>::displayAdj() {
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
void Graph<T>::displayRevAdj() {
    cout << "Display revAdj node:\n";
    for (int i = 0; i < numberOfNodes; i++) {
        cout << i << " --> ";
        for (auto j = revAdj[i].begin(); j != revAdj[i].end(); j++) {
            cout << *j << " ";
        }
        cout << endl;
    }
}