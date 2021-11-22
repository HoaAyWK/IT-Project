#include <SFML/Graphics.hpp>

using namespace sf;

enum class State
{
    Neutral,
    Start,
    Target,
    Path
};

class Cell
{
public:
    int x, y;
    int cellSize;
    bool visited;
    std::vector<Cell*> neighbours;
    Cell* parent = NULL;

    int heapIndex = -1;

public:
    Cell(int x, int y, int cellSize);
    void draw(RenderWindow& window);

    //walls
    bool top = true, right = true, bottom = true, left = true;
    RectangleShape topWall;
    RectangleShape rightWall;
    RectangleShape bottomWall;
    RectangleShape leftWall;

    RectangleShape cellShape;

    void setState(State state);
};

Cell::Cell(int x, int y, int cellSize)
    : x(x)
    , y(y)
    , cellSize(cellSize)
    , visited(false)
{
    int borderWidth = 2;
    topWall.setSize(Vector2f(cellSize, borderWidth));
    topWall.setPosition(Vector2f(x * cellSize, y * cellSize));
    topWall.setFillColor(Color::Black);

    rightWall.setSize(Vector2f(cellSize, borderWidth));
    rightWall.setPosition(Vector2f(x * cellSize + cellSize, y * cellSize));
    rightWall.setFillColor(Color::Black);
    rightWall.rotate(90);

    bottomWall.setSize(Vector2f(cellSize, borderWidth));
    bottomWall.setPosition(Vector2f(x * cellSize, y * cellSize + cellSize));
    bottomWall.setFillColor(Color::Black);

    leftWall.setSize(Vector2f(cellSize, borderWidth));
    leftWall.setPosition(Vector2f(x * cellSize, y * cellSize));
    leftWall.setFillColor(Color::Black);
    leftWall.rotate(90);

    cellShape.setPosition(Vector2f(x * cellSize, y * cellSize));
    cellShape.setSize(Vector2f(cellSize, cellSize));
    cellShape.setFillColor(Color::White);
}

void Cell::draw(RenderWindow& window)
{
    window.draw(cellShape);

    if (top)
        window.draw(topWall);
    if (right)
        window.draw(rightWall);
    if (bottom)
        window.draw(bottomWall);
    if (left)
        window.draw(leftWall);
}

void Cell::setState(State state)
{
    switch (state)
    {
    case State::Neutral:
        cellShape.setFillColor(Color::White);
        break;

    case State::Start:
        cellShape.setFillColor(Color::Green);
        break;

    case State::Target:
        cellShape.setFillColor(Color::Red);
        break;

    case State::Path:
        cellShape.setFillColor(Color::Cyan);
        break;

    default:
        break;
    }
}