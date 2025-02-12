#ifndef WAREHOUSE_MAP_H
#define WAREHOUSE_MAP_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <utility>  //  std::pair
using namespace std;

class GridVisualizer {
private:
    int width, height;
    int cell_size;
    std::vector<std::vector<int>> grid;

    void drawGrid(std::ofstream& file) {
        file << "<defs>\n"
            << "  <pattern id=\"grid\" width=\"" << cell_size
            << "\" height=\"" << cell_size << "\" patternUnits=\"userSpaceOnUse\">\n"
            << "    <path d=\"M " << cell_size << " 0 L 0 0 0 "
            << cell_size << "\" fill=\"none\" stroke=\"gray\" stroke-width=\"0.5\"/>\n"
            << "  </pattern>\n"
            << "</defs>\n"
            << "<rect width=\"" << width << "\" height=\"" << height
            << "\" fill=\"url(#grid)\"/>\n";
    }

    void drawShelves(std::ofstream& file) {
        file << "<!-- Area A -->\n";
        for (int i = 0; i < 4; i++) {
            file << "<rect x=\"100\" y=\"" << (80 + i * 80)
                << "\" width=\"200\" height=\"40\" fill=\"black\"/>\n";
        }

        file << "<!-- Area B -->\n";
        for (int i = 0; i < 4; i++) {
            file << "<rect x=\"500\" y=\"" << (80 + i * 80)
                << "\" width=\"200\" height=\"40\" fill=\"black\"/>\n";
        }
    }

    void drawTasks(std::ofstream& file) {
        file << "<!-- High tasks -->\n"
            << "<circle cx=\"120\" cy=\"120\" r=\"8\" fill=\"#FF4081\"/>\n"
            << "<text x=\"130\" y=\"135\" font-size=\"12\">P1</text>\n"
            << "<circle cx=\"240\" cy=\"200\" r=\"8\" fill=\"#FF4081\"/>\n"
            << "<text x=\"250\" y=\"215\" font-size=\"12\">P2</text>\n"
            << "<circle cx=\"540\" cy=\"120\" r=\"8\" fill=\"#FF4081\"/>\n"
            << "<text x=\"550\" y=\"135\" font-size=\"12\">P3</text>\n"
            << "<circle cx=\"640\" cy=\"280\" r=\"8\" fill=\"#FF4081\"/>\n"
            << "<text x=\"650\" y=\"295\" font-size=\"12\">P4</text>\n"
            << "<!-- Low tasks -->\n"
            << "<rect x=\"252\" y=\"72\" width=\"16\" height=\"16\" fill=\"#FFA726\"/>\n"
            << "<text x=\"260\" y=\"70\" font-size=\"12\">T1</text>\n"
            << "<rect x=\"192\" y=\"312\" width=\"16\" height=\"16\" fill=\"#FFA726\"/>\n"
            << "<text x=\"200\" y=\"310\" font-size=\"12\">T2</text>\n"
            << "<rect x=\"572\" y=\"152\" width=\"16\" height=\"16\" fill=\"#FFA726\"/>\n"
            << "<text x=\"580\" y=\"150\" font-size=\"12\">T3</text>\n";
    }

    void drawRobots(std::ofstream& file) {
        file << "<!-- High-equipped robot -->\n"
            << "<g transform=\"translate(150,500)\">\n"
            << "  <circle r=\"15\" fill=\"#4CAF50\"/>\n"
            << "  <line x1=\"0\" y1=\"0\" x2=\"15\" y2=\"0\" stroke=\"white\" stroke-width=\"3\"/>\n"
            << "  <text x=\"25\" y=\"5\" font-size=\"14\" fill=\"#4CAF50\">High-equipped Robot</text>\n"
            << "</g>\n"
            << "<!-- Low-equipped robot -->\n"
            << "<g transform=\"translate(600,500)\">\n"
            << "  <path d=\"M 0,-15 L 13,-7.5 L 13,7.5 L 0,15 L -13,7.5 L -13,-7.5 Z\" fill=\"#2196F3\"/>\n"
            << "  <line x1=\"0\" y1=\"0\" x2=\"13\" y2=\"0\" stroke=\"white\" stroke-width=\"3\"/>\n"
            << "  <text x=\"25\" y=\"5\" font-size=\"14\" fill=\"#2196F3\">Low-equipped Robot</text>\n"
            << "</g>\n";
    }

    void drawChargingStation(std::ofstream& file) {
        file << "<!-- Charging station -->\n"
            << "<rect x=\"350\" y=\"480\" width=\"100\" height=\"60\" fill=\"#FFD700\" opacity=\"0.3\"/>\n"
            << "<text x=\"360\" y=\"515\" font-size=\"16\" fill=\"#B8860B\">Charging</text>\n";
    }

    void drawLegend(std::ofstream& file) {
        file << "<!-- Legend -->\n"
            << "<g transform=\"translate(50, 570)\">\n"
            << "  <circle cx=\"10\" cy=\"0\" r=\"8\" fill=\"#FF4081\"/>\n"
            << "  <text x=\"25\" y=\"5\" font-size=\"12\">High tasks</text>\n"
            << "  <rect x=\"100\" y=\"-8\" width=\"16\" height=\"16\" fill=\"#FFA726\"/>\n"
            << "  <text x=\"125\" y=\"5\" font-size=\"12\">Low tasks</text>\n"
            << "</g>\n";
    }

public:
    GridVisualizer(int w = 800, int h = 600, int cs = 20)
        : width(w), height(h), cell_size(cs) {
        grid.resize(h / cs, std::vector<int>(w / cs, 0));
    }

    /*void generateSVG(const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cout << "Cannot create file: " << filename << std::endl;
            return;
        }

        file << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 "
            << width << " " << height << "\">\n";

        drawGrid(file);
        drawShelves(file);
        drawTasks(file);
        drawRobots(file);
        drawChargingStation(file);
        drawLegend(file);

        file << "</svg>";
        file.close();
    }*/

    bool isValidPosition(int x, int y) const {
        return x >= 0 && x < width / cell_size && y >= 0 && y < height / cell_size;
    }

    int getCellValue(int x, int y) const {
        if (isValidPosition(x, y)) {
            return grid[y][x];
        }
        return -1;
    }

    void setCellValue(int x, int y, int value) {
        if (isValidPosition(x, y)) {
            grid[y][x] = value;
        }
    }

    int getWidth() const { return width; }
    int getHeight() const { return height; }
    int getCellSize() const { return cell_size; }
    const std::vector<std::vector<int>>& getGrid() const { return grid; }

    void drawPath(std::ofstream& file, const vector<pair<int, int>>& path) {
        if (path.empty()) return;

        file << "<!-- Path -->\n"
            << "<path d=\"M";

        // 移动到起点
        file << path[0].first << " " << path[0].second;

        // 连接所有路径点
        for (size_t i = 1; i < path.size(); i++) {
            file << " L " << path[i].first << " " << path[i].second;
        }

        file << "\" stroke=\"#4CAF50\" stroke-width=\"3\" fill=\"none\" "
            << "stroke-dasharray=\"5,5\"/>\n";

        // 在路径起点画一个圆
        file << "<circle cx=\"" << path[0].first << "\" cy=\"" << path[0].second
            << "\" r=\"5\" fill=\"#4CAF50\"/>\n";

        // 在路径终点画一个箭头
        file << "<circle cx=\"" << path.back().first << "\" cy=\"" << path.back().second
            << "\" r=\"5\" fill=\"#FF4081\"/>\n";
    }

    // 修改 generateSVG 方法，添加路径参数
    void generateSVG(const std::string& filename, const vector<pair<int, int>>& path = {}) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cout << "Cannot create file: " << filename << std::endl;
            return;
        }

        file << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 "
            << width << " " << height << "\">\n";

        drawGrid(file);
        drawShelves(file);
        drawTasks(file);
        if (!path.empty()) {
            drawPath(file, path);
        }
        drawRobots(file);
        drawChargingStation(file);
        drawLegend(file);

        file << "</svg>";
        file.close();
    }
};

#endif // WAREHOUSE_MAP_H