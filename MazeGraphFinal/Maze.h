
#include <cstdlib>
#include <algorithm> 
#include <list>
#include <time.h>
#include <SFML/Graphics.hpp>
#include "Graph.h"
#include "Cell.h"

using namespace sf;


struct Point {
    int x;
    int y;
};

typedef struct Point point;

class Maze : public Graph<point> {
private:
    int rows;
    int cols;
    int nrNodes;
    int maxSize;
    int cellWidth;
    int iteratorIndex = 0;

    vector<point> points;
    vector<pair<string, point>> nodes;
    vector<Cell*> cells;
    using Graph<point>::vertices;
    using Graph<point>::adjMatrix;
    using Graph<point>::path;
    using Graph<point>::DFS;
    using Graph<point>::addEdge;
    using Graph<point>::addNode;
public:
    Maze(int size, int rows, int cols, int cellWidth) : Graph<point>(size) {
        this->rows = rows;
        this->cols = cols;
        this->nrNodes = size;
        this->maxSize = rows * cols;
        this->cellWidth = cellWidth;
    }

    int getIndex(int x, int y, int cols) { return y * cols + x; }

    void initPoints();
    void initNodes();
    void generate();
    bool isPath(int);

    void drawMaze();

    void drawPath();

    void display(RenderWindow& window);

    bool isMazeGenerated = false;
    RenderWindow _window;
};

void Maze::initPoints() {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            point p = { i, j };
            points.push_back(p);
            cells.push_back(new Cell(i, j, cellWidth));
        }
    }
}

void Maze::initNodes() {
    initPoints();
    for (int i = 0; i < nrNodes; i++) {
        string name = to_string(i);
        addNode(name, points[i]);
        nodes.push_back(make_pair(name, points[i]));
    }
}

void Maze::generate() {
    cout << "--- Generate maze ---\n";
    initNodes();
    string node1;
    string node2;
    vector<pair<int, int>> offset = { {-1, 0 }, { 0, 1 }, { 1, 0 }, { 0, -1 } };
    point tmp = { 0, 0 };
    vector<vector<bool>> visited(rows, vector<bool>(cols, false));
    visited[0][0] = true;
    point init = { -1, -1 };
    list<point> Stack;
    Stack.push_back(init);
    srand((int)time(0));
    random_shuffle(offset.begin(), offset.end());
    while (tmp.x != -1 && tmp.y != -1) {
        for (int i = 0; i < 4; i++) {
            int x = tmp.x + offset[i].first;
            int y = tmp.y + offset[i].second;
            if (x >= 0 && x < rows) {
                if (y >= 0 && y < cols) {
                    if (!visited[x][y]) {
                        visited[x][y] = true;
                        point p = { x, y };
                        Stack.push_back(tmp);
                        for (int k = 0; k < nrNodes; k++) {
                            if (nodes[k].second.x == tmp.x && nodes[k].second.y == tmp.y) {
                                node1 = nodes[k].first;
                            }
                            if (nodes[k].second.x == x && nodes[k].second.y == y) {
                                node2 = nodes[k].first;
                            }
                        }
                        tmp = p;
                        addEdge(node1, node1, node2);
                        random_shuffle(offset.begin(), offset.end());
                        i = 0;
                    }
                }
            }
        }
        tmp = Stack.back();
        Stack.pop_back();
    }
    isMazeGenerated = true;
}

void Maze::drawMaze() {
    for (int i = 0; i < rows; i++) {
        for (int j = i * cols; j < (i * cols) + cols - 1; j++) {
            if (adjMatrix[j][j + 1]) {

                point pointLeft = vertices[j]->getValue();
                point pointRight = vertices[j + 1]->getValue();

                int indexLeft = getIndex(pointLeft.x, pointLeft.y, cols);
                int indexRight = getIndex(pointRight.x, pointRight.y, cols);

                cells[indexLeft]->setState(State::Neutral);
                cells[indexLeft]->right = false;
                cells[indexRight]->setState(State::Neutral);
                cells[indexRight]->left = false;
            }
        }
        if (i < rows - 1) {
            for (int j = i * cols; j < (i * cols) + cols; j++) {
                if (adjMatrix[j][j + cols]) {

                    point pointAbove = vertices[j]->getValue();
                    point pointBellow = vertices[j + cols]->getValue();

                    int indexAbove = getIndex(pointAbove.x, pointAbove.y, cols);
                    int indexBellow = getIndex(pointBellow.x, pointBellow.y, cols);

                    cells[indexAbove]->setState(State::Neutral);
                    cells[indexAbove]->bottom = false;
                    cells[indexBellow]->setState(State::Neutral);
                    cells[indexBellow]->top = false;
                }
            }
        }
    }
}

bool Maze::isPath(int y) {
    int size = path.size();
    for (int i = 0; i < size; i++) {
        if (y == path[i]) {
            return true;
        }
    }
    return false;
}


void Maze::drawPath() {
    bool hasPath = DFS(0, nrNodes - 1);
    if (hasPath == false) {
        cout << "Can't find path!\n";
        return;
    }
    int size = path.size();

    // Start
    cells[getIndex(0, 0, cols)]->setState(State::Start);

    for (int i = 1; i < size - 1; i++) {
        point point = vertices[path[i]]->getValue();
        int index = getIndex(point.x, point.y, cols);
        cells[index]->setState(State::Path);
    }

    // Target
    cells[getIndex(cols - 1, rows - 1, cols)]->setState(State::Target);
}

void Maze::display(RenderWindow& window)
{
    int size = cells.size();
    for (int i = 0; i < size; i++)
    {
        cells[i]->draw(window);
    }
}