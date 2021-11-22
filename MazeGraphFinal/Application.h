
#include <SFML/Graphics.hpp>
#include "Maze.h"

constexpr int WIDTH = 600, HEIGHT = 600;

using namespace sf;

class Application
{
private:
    void pollEvents();
    void update();
    RenderWindow _window;

    Maze* maze;
    int cellWidth;

    bool startPathFinding = false;
    bool isTimerDestroyed = false;

public:
    Application();

    void run();

};

Application::Application()
    : _window({ WIDTH, HEIGHT }, "Maze Generator")
{
    _window.setFramerateLimit(60);

    cellWidth = 20;
    int rows = HEIGHT / cellWidth;
    int cols = WIDTH / cellWidth;
    int size = rows * cols;
    maze = new Maze(size, rows, cols, cellWidth);
}

void Application::run()
{
    //Clear -> update -> draw -> display -> poll events
    while (_window.isOpen())
    {
        _window.clear(Color::White);
        update();
        _window.display();
        pollEvents();
    }
}

void Application::pollEvents()
{
    Event e;
    while (_window.pollEvent(e))
    {
        if (e.type == Event::Closed)
        {
            _window.close();
        }

        if (e.type == Event::KeyPressed)
        {
            if (e.key.code == Keyboard::F)
            {
                startPathFinding = true;
            }
        }
    }
}

void Application::update()
{
    //Update/Draw
    if (!maze->isMazeGenerated) {
        maze->generate();
        maze->drawMaze();
    }

    if (maze->isMazeGenerated && startPathFinding) {
        maze->drawPath();
        startPathFinding = false;
    }
    maze->display(_window);
}