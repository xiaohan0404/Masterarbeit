#pragma once
#ifndef DISTANCE_UTILS_H
#define DISTANCE_UTILS_H

#include"warehouse_map.h"
#include"Robot.h"
#include"Task.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <set>
#include <queue>
#include <utility>

using namespace std;


struct Node {
    int x, y;
    int direction;  // Move direction£º0=UP£¬1=RIGHT£¬2=DOWN£¬3=LEFT;
    double g;        //Cost from Start;
    double h;        //Heuristic to Goal: Manhattan distance;
    double f;        //Total Estimated Cost;
    Node* parent;    //

    Node(int x, int y, int dir = -1)
        : x(x), y(y), direction(dir), g(0), h(0), f(0), parent(nullptr) {}


    //Determine whether a turn occurs£»
    bool isTurning(const Node& next) const {
        if (direction == -1) return false; 

        int dx = next.x - x;
        int dy = next.y - y;
        int nextDir;
        if (dx == 0 && dy == -1) nextDir = 0;        // UP;
        else if (dx == 1 && dy == 0) nextDir = 1;    // RIGHT;
        else if (dx == 0 && dy == 1) nextDir = 2;    // DOWN;
        else if (dx == -1 && dy == 0) nextDir = 3;   // LEFT;
        else return false;

        return direction != nextDir;
    }

};

class PathPlanner {
private:
    const GridVisualizer& map;
    vector<vector<bool>> obstacles;  //Obstacle Grid ÕÏ°­ÎïÍø¸ñ
    int gridWidth;
    int gridHeight;
    const int GRID_STEP = 20;             // Grid step size, assuming it is 20 pixels;
    const double OBSTACLE_PENALTY = 0.5;  // Penalty coefficient around obstacles;
    const int OBSTACLE_INFLUENCE = 2;     // Obstacle impact range; 
    const double TURNING_COST = 0.8;      //Turn Cost;


    void initializeGridDimensions() {
        gridHeight = map.getHeight() / GRID_STEP;
        gridWidth = map.getWidth() / GRID_STEP;
        obstacles.resize(gridHeight, vector<bool>(gridWidth, false));
    }


    void initializeObstacles() {
        initializeGridDimensions();
        // Area A£ºMark the first set of obstacles
        for (int i = 0; i < 4; i++) {
            int startY = (80 + i * 80) / GRID_STEP;
            int endY = (120 + i * 80) / GRID_STEP;
            for (int y = startY; y < endY; y++) {
                for (int x = 100 / GRID_STEP; x < 300 / GRID_STEP; x++) {
                    if (isValidPosition(x, y)) {
                        obstacles[y][x] = true;
                    }
                }
            }
        }
        // Area B£ºMark the second set of obstacles
        for (int i = 0; i < 4; i++) {
            int startY = (80 + i * 80) / GRID_STEP;
            int endY = (120 + i * 80) / GRID_STEP;
            for (int y = startY; y < endY; y++) {
                for (int x = 500 / GRID_STEP; x < 700 / GRID_STEP; x++) {
                    if (isValidPosition(x, y)) {
                        obstacles[y][x] = true;
                    }
                }
            }
        }
    }


    double calculateDistanceToNearestObstacle(int x, int y) {
        double minDistance = OBSTACLE_INFLUENCE + 1.0; 

        for (int dy = -OBSTACLE_INFLUENCE; dy <= OBSTACLE_INFLUENCE; dy++) {
            for (int dx = -OBSTACLE_INFLUENCE; dx <= OBSTACLE_INFLUENCE; dx++) {
                int newX = x + dx;
                int newY = y + dy;

                if (isValidPosition(newX, newY) && obstacles[newY][newX]) {
                    double distance = sqrt(dx * dx + dy * dy);
                    minDistance = min(minDistance, distance);
                }
            }
        }

        return minDistance;
    }


    double calculateMovementCost(const Node& current, const Node& neighbor) {
        double cost = 1.0;
        if (current.isTurning(neighbor)) {
            cost += TURNING_COST;
        }

        double obstacleDistance = calculateDistanceToNearestObstacle(neighbor.x, neighbor.y);

        if (obstacleDistance <= OBSTACLE_INFLUENCE) {
            double penalty = OBSTACLE_PENALTY * (1.0 - obstacleDistance / (OBSTACLE_INFLUENCE + 1.0));
            cost += penalty;
        }

        return cost;
    }

 
    pair<int, int> findNearestGridPoint(int x, int y) {                  // Find the nearest grid point
        int gridX = round(static_cast<double>(x) / GRID_STEP);           // Convert real-world coordinates to grid coordinates
        int gridY = round(static_cast<double>(y) / GRID_STEP);

        gridX = max(0, min(gridX, gridWidth - 1));                       // Make sure the grid points are within the valid range
        gridY = max(0, min(gridY, gridHeight - 1));

        if (!isSafePosition(gridX, gridY)) {                             // If the current point is an obstacle, find the nearest non-obstacle grid point

            const int dx[] = { 0, 1, 0, -1 };                            // Search direction: Up, Right, Down, Left
            const int dy[] = { -1, 0, 1, 0 };

            queue<pair<int, int>> q;
            set<pair<int, int>> visited;
            q.push({ gridX, gridY });
            visited.insert({ gridX, gridY });

            while (!q.empty()) {
                auto current = q.front();
                q.pop();

                for (int i = 0; i < 4; i++) {
                    int newX = current.first + dx[i];
                    int newY = current.second + dy[i];

                    if (isValidPosition(newX, newY) &&
                        visited.find({ newX, newY }) == visited.end()) {
                        if (isSafePosition(newX, newY)) {
                            return { newX, newY };
                        }
                        visited.insert({ newX, newY });
                        q.push({ newX, newY });
                    }
                }
            }
        }
        return { gridX, gridY };
    }


    bool isValidPosition(int x, int y) const {
        return x >= 0 && x < gridWidth && y >= 0 && y < gridHeight;
    }


    bool isSafePosition(int x, int y) const {
        return isValidPosition(x, y) && !obstacles[y][x];
    }


    vector<Node> getNeighbors(const Node& current) {                       // Get neighbor nodes in four directions
        vector<Node> neighbors;

        const int dx[] = { 0, 1, 0, -1 };                                  // Only four directions of movement are allowed: up, down, left, and right;
        const int dy[] = { -1, 0, 1, 0 };

        for (int i = 0; i < 4; i++) {
            int newX = current.x + dx[i];
            int newY = current.y + dy[i];

            if (isSafePosition(newX, newY)) {
                Node neighbor(newX, newY,i);
                neighbor.g = calculateMovementCost(current, neighbor);                             // The Movement Cost is 1
                neighbors.push_back(neighbor);
            }
        }
        return neighbors;
    }

    // Use Manhattan distance as a heuristic function
    double heuristic(int x1, int y1, int x2, int y2) {
        return abs(x2 - x1) + abs(y2 - y1);
    }

public:
    PathPlanner(const GridVisualizer& gridMap) : map(gridMap) {
        initializeObstacles();
    }

    //A* algorithm find path
    vector<pair<int, int>> findPath(int startX, int startY, int goalX, int goalY) {
        auto gridStart = findNearestGridPoint(startX, startY);
        auto gridGoal = findNearestGridPoint(goalX, goalY);

        startX = gridStart.first;
        startY = gridStart.second;
        goalX = gridGoal.first;
        goalY = gridGoal.second;

        if (!isSafePosition(startX, startY)) {
            cout << "Unable To Find A Valid Starting Position!" << endl;
            return {};
        }

        vector<Node*> openList;
        set<pair<int, int>> closedList;

        Node* startNode = new Node(startX, startY, -1);
        startNode->h = heuristic(startX, startY, goalX, goalY);
        startNode->f = startNode->h;

        openList.push_back(startNode);

        while (!openList.empty()) {
            auto it = min_element(openList.begin(), openList.end(),
                [](const Node* a, const Node* b) { return a->f < b->f; });

            Node* current = *it;

            if (current->x == goalX && current->y == goalY) {
                vector<pair<int, int>> path;
                while (current != nullptr) {
                    // Convert grid coordinates back to real coordinates
                    path.emplace_back(
                        current->x * GRID_STEP,
                        current->y * GRID_STEP
                    );
                    current = current->parent;
                }

                for (auto node : openList) {
                    delete node;
                }

                reverse(path.begin(), path.end());
                return path;
            }

            openList.erase(it);
            closedList.insert({ current->x, current->y });

            for (const Node& neighbor : getNeighbors(*current)) {
                if (closedList.find({ neighbor.x, neighbor.y }) != closedList.end()) {
                    continue;
                }

                double newG = current->g + neighbor.g;
                auto nodeIt = find_if(openList.begin(), openList.end(),
                    [&neighbor](const Node* n) { return n->x == neighbor.x && n->y == neighbor.y; });

                if (nodeIt == openList.end()) {
                    Node* newNode = new Node(neighbor.x, neighbor.y, neighbor.direction);
                    newNode->g = newG;
                    newNode->h = heuristic(neighbor.x, neighbor.y, goalX, goalY);
                    newNode->f = newNode->g + newNode->h;
                    newNode->parent = current;
                    openList.push_back(newNode);
                }
                else {
                    Node* existingNode = *nodeIt;
                    if (newG < existingNode->g) {
                        existingNode->g = newG;
                        existingNode->direction = neighbor.direction;
                        existingNode->f = existingNode->g + existingNode->h;
                        existingNode->parent = current;
                    }
                }
            }
        }

        cout << "No Feasible Path found!" << endl;
        return {};
    }


    double calculatePathLength(const vector<pair<int, int>>& path) {
        if (path.size() < 2) return 0.0;

        double length = 0.0;
        for (size_t i = 1; i < path.size(); i++) {
            int dx = path[i].first - path[i - 1].first;
            int dy = path[i].second - path[i - 1].second;
            length += sqrt(dx * dx + dy * dy);
        }
        return length;
    }
};

#endif // DISTANCE_UTILS_H