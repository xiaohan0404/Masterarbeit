#include"warehouse_map.h"
#include"Task.h"
#include"Robot.h"
#include"distance_utils.h"
#include"Biding.h"
#include"TimeWindowAuctionManager.h"
#include<iostream>
#include<string>
#include<vector>

void testPathPlanning() {
    //Creating a map and route planner  
    //创建地图和路径规划器
    GridVisualizer map(800, 600, 20);
    PathPlanner planner(map);

    //Set the starting point and the end point  
    //设置起点（高配置机器人的位置）和终点（任务P1的位置）
    int startX = 150, startY = 500;  // High configuration robot starting position 高配置机器人起始位置
    int goalX = 120, goalY = 120;    // Location of Task P1 任务P1的位置

    cout << "测试从机器人(" << startX << "," << startY << ") "
        << "到任务点(" << goalX << "," << goalY << ")的路径规划" << endl;

    // Find Path 
    // 寻找路径
    vector<pair<int, int>> path = planner.findPath(startX, startY, goalX, goalY);
    vector<vector<pair<int, int>>> Paths;
    Paths.push_back(path);
    if (!Paths.empty()) {
        cout << "找到路径！路径长度: " << planner.calculatePathLength(path) << endl;
        cout << "路径点: " << endl;
        for (const auto& point : path) {
            cout << "(" << point.first << "," << point.second << ") -> ";
        }
        cout << "终点" << endl;
        //Generate SVG file containing paths 生成包含路径的SVG文件
        map.generateSVG("warehouse_with_path.svg", Paths);
        cout << "已生成路径可视化文件: warehouse_with_path.svg" << endl;
    }
    else {
        cout << "未找到可行路径！" << endl;
        //Generate SVG file without paths 生成没有路径的SVG文件
        map.generateSVG("warehouse_no_path.svg",{});
    }
}

void test_1() {   

    GridVisualizer viz(800, 600, 20);
    PathPlanner planner(viz);                           //Set Map;

    Task P1("P1", { 120, 120 }, 20, 100, 20, true);
    Task P2("P2", { 240, 200 }, 60, 200, 60, true);
    Task P3("P3", { 540, 120 }, 50, 200, 20, true);
    Task P4("P4", { 640, 280 }, 100, 300, 50, true);    //High equipt Tasks;

    Task T1("T1", { 260, 80 }, 20, 200, 40, false);
    Task T2("T2", { 200, 320 }, 60, 150, 40, false);
    Task T3("T3", { 580, 160 }, 100, 300, 40, false);    //Low equipt Tasks;


    P1.setPriorityLevel(4);
    T1.setPriorityLevel(4);
    P2.setPriorityLevel(3);
    P3.setPriorityLevel(3);
    T2.setPriorityLevel(3);
    T3.setPriorityLevel(2);
    P4.setPriorityLevel(1);    //set PriorityLevel;

    P4.addPrerequisite(&P3);
    P4.addPrerequisite(&T3);
    T3.addPrerequisite(&P2);
    T3.addPrerequisite(&T2);
    P3.addPrerequisite(&P1);
    P2.addPrerequisite(&P1);
    T2.addPrerequisite(&T1);   //set Pre Task;

    cout << endl;
    cout << "Number of tasks after P1: " << P1.getSuccessorCount() << endl;               // out put 
    cout << "Number of tasks after T1: " << T1.getSuccessorCount() << endl;               //

    cout << "Number of predecessor tasks for P2: " << P2.getPrerequisiteCount() << endl;  // 
    cout << "Number of tasks after P2: " << P2.getSuccessorCount() << endl;
    cout << "Number of predecessor tasks for P3: " << P3.getPrerequisiteCount() << endl;  // 
    cout << "Number of tasks after P3: " << P3.getSuccessorCount() << endl;
    cout << "Number of predecessor tasks for T2: " << T2.getPrerequisiteCount() << endl;  // 
    cout << "Number of tasks after T2: " << T2.getSuccessorCount() << endl;

    cout << "Number of predecessor tasks for T3: " << T3.getPrerequisiteCount() << endl;  // 
    cout << "Number of tasks after T3: " << T3.getSuccessorCount() << endl;
    cout << "Number of predecessor tasks for P4: " << P4.getPrerequisiteCount() << endl;  //
    cout << "Number of tasks after P4: " << P4.getSuccessorCount() << endl;
    cout << endl;


    //create robot:  (robot_id,initial_position,robot_speed,robot_type)
    Robot robot1("H1", { 150,500 }, 60, true, viz, planner);                   //High equipt Robot;
    Robot robot2("L2", { 600,500 }, 80, false, viz, planner);                  //Low equipt Robot have Higher Speed;
    vector<Robot> Robots;
    Robots.push_back(robot1);
    Robots.push_back(robot2);
    vector<Task*> unassigned_tasks = { &P1, &P2, &P3, &P4, &T1, &T2, &T3 };

    TimeWindowAuctionManager auction_manager(Robots, unassigned_tasks, 40.0);  // 40 time unit window
    auction_manager.runTimeWindowAuction();
}


int main() {
    GridVisualizer viz(800, 600);
    viz.generateSVG("warehouse_map.svg", {});     //create map
    PathPlanner planner(viz);

    test_1();
    //testPathPlanning();

    return 0;
}