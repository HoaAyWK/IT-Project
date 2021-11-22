
#include <iostream>
#include "LGraph.h"
#include "Application.h"

using namespace std;

void undirectedLGraph();
void directedLGraph();
void undirectedGraph();
void directedGraph();
void mazeGenerator();

int main()
{
    int choice;
    cout << "Please choose one of the following options to continue:\n\n";
    cout << "Option 1: Undirected graph using adjacency list\n";
    cout << "Option 2: Directed graph using adjacency list\n";
    cout << "Option 3: Undirected graph using adjacency matrix, incidence matrix, weight matrix\n";
    cout << "Option 4: Directed graph using adjacency matrix, incidence matrix, weight matrix\n";
    cout << "Option 5: Maze application\n";
    cout << "Enter you're choice (1-5): ";
    cin >> choice;
    cout << "\n";

    if (choice == 1) undirectedLGraph();
    if (choice == 2) directedLGraph();
    if (choice == 3) undirectedGraph();
    if (choice == 4) directedGraph();
    if (choice == 5) mazeGenerator();

    return 0;
}

void undirectedLGraph() {
    LGraph<int> g1(TYPE::UNDIRECTED);
    g1.addNode(1);
    g1.addNode(2);
    g1.addNode(3);
    g1.addNode(4);
    g1.addNode(5);
    g1.addNode(6);
    g1.addNode(7);
    g1.addNode(8);

    g1.addEdge("a", 1, 6, 2);
    g1.addEdge("b", 1, 5, 4);
    g1.addEdge("c", 2, 5, 10);
    g1.addEdge("d", 2, 6, 8);
    g1.addEdge("e", 3, 6, 5);
    g1.addEdge("l", 3, 5, 2);
    g1.addEdge("f", 4, 6, 3);
    g1.addEdge("g", 4, 5, 7);
    g1.addEdge("i", 4, 7, 12);
    g1.addEdge("j", 4, 8, 9);
    g1.addEdge("k", 7, 8, 6);

    g1.updateNodeName(1, 0);
    g1.updateEdgeWeight("j", 10);

    g1.displayNodesName();
    g1.displayEdgesName();

    cout << "Ajacency list:\n";
    g1.displayAdjList();

    int numberOfNodes = g1.getNumberOfNodes();

    vector<pair<bool, int>> bfs = g1.BFS(0);
    cout << "Visited nodes:\n";

    for (int i = 0; i < numberOfNodes; i++) {
        cout << bfs[i].first << " " << bfs[i].second << endl;
    }
    cout << endl;

    g1.DFS(0);
    vector<bool> visited = g1.getVisitedDFS();
    vector<int> prv = g1.getPrvDFS();
    cout << "Visited nodes:\n";
    for (int i = 0; i < numberOfNodes; i++) {
        cout << visited[i] << " ";
    }
    cout << endl;
    for (int i = 0; i < numberOfNodes; i++) {
        cout << prv[i] << " ";
    }
    cout << endl;

    g1.shortestPath(0, 5);

    bool hasEulerCircuit = g1.isEulerianCircuit();

    if (hasEulerCircuit) cout << "Given graph is Eulerian.\n";
    else cout << "Given graph is not Eulerian.\n";
}

void directedLGraph() {
    LGraph<int> G(TYPE::DIRECTED);
    G.addNode(0);
    G.addNode(1);
    G.addNode(2);
    G.addNode(3);
    G.addNode(4);

    G.addEdge("a", 1, 0, 10);
    G.addEdge("b", 0, 2, 5);
    G.addEdge("c", 2, 1, 2);
    G.addEdge("d", 0, 3, 4);
    G.addEdge("e", 3, 4, 7);
    G.addEdge("f", 4, 0, 9);

    G.updateNodeName(4, 5);
    G.updateEdgeWeight("f", 20);

    int numberOfNodes = G.getNumberOfNodes();

    G.displayNodesName();
    G.displayEdgesName();

    cout << "Ajacency list:\n";
    G.displayAdjList();

    vector<pair<bool, int>> bfs = G.BFS(0);
    for (int i = 0; i < numberOfNodes; i++) {
        cout << bfs[i].first << " " << bfs[i].second << endl;
    }
    cout << endl;

    G.DFS(0);
    vector<bool> visited = G.getVisitedDFS();
    vector<int> prv = G.getPrvDFS();
    cout << "Visited nodes:\n";
    for (int i = 0; i < numberOfNodes; i++) {
        cout << visited[i] << " ";
    }
    cout << endl;
    for (int i = 0; i < numberOfNodes; i++) {
        cout << prv[i] << " ";
    }
    cout << endl;

    G.shortestPath(0, 5);

    if (G.isEulerianCircuit()) {
        cout << "Given graph is Eulerian.\n";
    }
    else {
        cout << "Given graph is not Eulerian.\n";
    }
}

void undirectedGraph() {
    Graph<int> G(TYPE::UNDIRECTED, 8);
    G.addNode("1", 1);
    G.addNode("2", 2);
    G.addNode("3", 3);
    G.addNode("4", 4);
    G.addNode("5", 5);
    G.addNode("6", 6);
    G.addNode("7", 7);
    G.addNode("8", 8);

    G.addEdge("b", "1", "6", 2);
    G.addEdge("a", "1", "5", 4);
    G.addEdge("d", "2", "5", 10);
    G.addEdge("c", "2", "6", 8);
    G.addEdge("f", "3", "6", 5);
    G.addEdge("e", "3", "5", 2);
    G.addEdge("j", "4", "6", 3);
    G.addEdge("i", "4", "5", 7);
    G.addEdge("h", "4", "7", 12);
    G.addEdge("g", "4", "8", 9);
    G.addEdge("v", "7", "8", 6);

    int numberOfNodes = G.getNumberOfNodes();

    G.updateNode("8", 10);
    G.updateWeight("v", 20);

    cout << "Display node:\n";
    G.displayNode("8");
    cout << endl;

    cout << "Display edge:\n";
    G.displayEdge("v");

    cout << "Display all nodes:\n";
    G.displayNodes();
    cout << endl;

    cout << "Display all edges:\n";
    G.displayEdges();

    cout << "Display adjacency matrix:\n";
    G.displayAdjMatrix();

    cout << "Display all incidence matrix:\n";
    G.displayIncidenceMatrix();

    cout << "Display weight matrix:\n";
    G.displayWeightMatrix();

    vector<pair<bool, int>> bfs = G.BFS("1");
    for (int i = 0; i < numberOfNodes; i++) {
        cout << bfs[i].first << " " << bfs[i].second << endl;
    }
    cout << endl;

    vector<pair<bool, int>> dfs = G.DFS("1");
    for (int i = 0; i < numberOfNodes; i++) {
        cout << dfs[i].first << " " << dfs[i].second << endl;
    }
    cout << endl;

    vector<pair<int, int>> dijkstra = G.dijsktra("1");
    cout << "Shortest path form 1 to all another node:\n";
    for (int i = 0; i < numberOfNodes; i++) {
        cout << dijkstra[i].first << " " << dijkstra[i].second << endl;
    }
    cout << endl;

    cout << "Find path using Floyd Warshall algorithm:\n";
    G.findPathFW("1", "7");
    cout << endl;


    if (G.isEulerianCircuit()) {
        cout << "Given graph is Eulerian.\n";
    }
    else {
        cout << "Given graph Eulerian.\n";
    }
}

void directedGraph() {
    Graph<int> G(TYPE::DIRECTED, 5);
    G.addNode("0", 0);
    G.addNode("1", 1);
    G.addNode("2", 2);
    G.addNode("3", 3);
    G.addNode("4", 4);
    G.addEdge("a", "1", "0", 10);
    G.addEdge("b", "0", "2", 5);
    G.addEdge("c", "2", "1", 2);
    G.addEdge("d", "0", "3", 4);
    G.addEdge("e", "3", "4", 7);
    G.addEdge("f", "4", "0", 9);
    G.updateWeight("f", 3);

    int numberOfNodes = G.getNumberOfNodes();

    G.updateNode("4", 5);
    G.updateWeight("f", 20);

    cout << "Display node:\n";
    G.displayNode("4");
    cout << endl;

    cout << "Display edge:\n";
    G.displayEdge("f");

    cout << "Display all nodes:\n";
    G.displayNodes();
    cout << endl;

    cout << "Display all edges:\n";
    G.displayEdges();

    cout << "Display adjacency matrix:\n";
    G.displayAdjMatrix();

    cout << "Display all incidence matrix:\n";
    G.displayIncidenceMatrix();

    cout << "Display weight matrix:\n";
    G.displayWeightMatrix();

    vector<pair<bool, int>> bfs = G.BFS("0");
    for (int i = 0; i < numberOfNodes; i++) {
        cout << bfs[i].first << " " << bfs[i].second << endl;
    }
    cout << endl;

    vector<pair<bool, int>> dfs = G.DFS("0");
    for (int i = 0; i < numberOfNodes; i++) {
        cout << dfs[i].first << " " << dfs[i].second << endl;
    }
    cout << endl;

    vector<pair<int, int>> dijkstra = G.dijsktra("0");
    cout << "Shortest path form 0 to all another node:\n";
    for (int i = 0; i < numberOfNodes; i++) {
        cout << dijkstra[i].first << " " << dijkstra[i].second << endl;
    }
    cout << endl;

    cout << "Find path using Floyd Warshall algorithm:\n";
    G.findPathFW("0", "4");
    cout << endl;


    if (G.isEulerianCircuit()) {
        cout << "Given graph is Eulerian.\n";
    }
    else {
        cout << "Given graph Eulerian.\n";
    }

    G.displayAdj();
}

void mazeGenerator() {
    Application app;
    app.run();
}