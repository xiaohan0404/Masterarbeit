#pragma once
#ifndef Robot_H
#define Robot_H

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <memory>
#include <limits>
#include <algorithm>
#include "distance_utils.h"
#include "Task.h"
#include "warehouse_map.h"
using namespace std;

struct InsertionResult {
    bool feasible;          // 是否可行
    int position;           // 最优插入位置
    double completion_time; // 完成时间
    vector<Task> task_sequence; // 最优任务序列
    vector<pair<int, int>> planned_path;

    InsertionResult() : feasible(false), position(-1), task_sequence({}), planned_path({}),
        completion_time(numeric_limits<double>::infinity())
    {}
};


class Robot {
private:
    string robot_id;                  // 机器人ID
    vector<double> current_position;  // 当前位置
    double speed;                     // 移动速度
    vector<Task> tasks = {};          // 机器人任务列表
    map<string, int> taskIndex;       // 任务索引映射
    bool type;                        // 机器人类型 true =high equipt robot; false = low equipt robot
    const double INF = numeric_limits<double>::infinity();
    double last_completion_time = 0;

    GridVisualizer viz;             // 存储地图引用
    PathPlanner pathPlanner;        // 存储路径规划器引用
    vector<pair<int, int>> movement_path;


    void addToPath(const vector<double>& position) {
        movement_path.push_back({
            static_cast<int>(position[0]),
            static_cast<int>(position[1])
            });
    }

    

    double calculateActualTravelTime(const vector<double>& start, const vector<double>& end) {

        // Use a path planner to find a path avoiding obstacles
        vector<pair<int, int>> path = pathPlanner.findPath(
            static_cast<int>(start[0]),
            static_cast<int>(start[1]),
            static_cast<int>(end[0]),
            static_cast<int>(end[1])
        );

        if (path.empty()) {
            cout << "WARNING: No viable path found!" << endl;
            return -1.0; 
        }

        // Calculate the actual path length
        double pathLength = pathPlanner.calculatePathLength(path);
        return pathLength / speed;  // Returns the actual travel time
    }

public:
    Robot(const string& id, const vector<double>& initial_pos, double move_speed, bool robot_type, const GridVisualizer& visualizer, const PathPlanner& planner) :
        robot_id(id), current_position(initial_pos), speed(move_speed), type(robot_type), viz(visualizer), pathPlanner(planner) {
        addToPath(initial_pos);
    }

    vector<Task>& getTasklist() { return tasks; }
    string& getName() { return robot_id; }
    bool& getRobotType() { return type; }
    const vector<double>& getCurrentPosition() const { return current_position; }


    void generatePathReport() {
        double total_distance = 0.0;
        double max_completion_time = 0.0;

        if (!tasks.empty()) {
            // Calculate total distance
            for (size_t i = 1; i < movement_path.size(); i++) {
                auto& p1 = movement_path[i - 1];
                auto& p2 = movement_path[i];
                int dx = p2.first - p1.first;
                int dy = p2.second - p1.second;
                total_distance += sqrt(dx * dx + dy * dy);
            }

            max_completion_time = tasks.back().time.ft;
        }

        // Generate a map with paths
        string filename = "robot_" + robot_id + "_path.svg";
        viz.generateSVG(filename, movement_path);

        // Output statistics
        cout << "\nRobot " << getName() << " Statistical Reports:" << endl;
        cout << "Total Travel Distance: " << total_distance << endl;
        cout << "Total Finish Time: " << max_completion_time << endl;
        cout << "Path Pap Generated: " << filename << endl;
    }

    
    int  PrintTask(vector<Task>& tasks) {
        for (Task& task : tasks) {
            cout << task.getTaskId() << endl;
        }
        return 0;
    }

    void updateLastCompletionTime() {
        if (!tasks.empty()) {
            last_completion_time = tasks.back().getTimeParameters().ft;
        }
    }


    //Calculate the time constraint of the first task
    TimeParameters calculateInitialTasktime(Task& task) {
        task.time.tt = calculateActualTravelTime(current_position, task.getLocation());

        if (task.time.tt < 0) {
            task.time.tt = -1;
            task.time.isValid = false;
        }
        else {
            task.time.st = max(task.time.tt, task.getEst());
            task.time.ft = task.time.st + task.getDut();
            task.time.isValid = (task.getEst() <= task.time.st &&
                task.time.st <= task.getLst() &&
                task.time.ft <= task.getLft());
        }
        return task.time;
    }

 
    TimeParameters calculateTasktimes(Task& task, const Task& beforeTask) {
        task.time.tt = calculateActualTravelTime(beforeTask.getLocation(), task.getLocation());

        if (task.time.tt < 0) {
            task.time.tt = -1;
            task.time.isValid = false;
            return task.time;
        }

        task.time.st = max(task.getEst(), beforeTask.time.ft + task.time.tt);
        task.time.ft = task.time.st + task.getDut();
        task.time.isValid = (task.time.st >= task.getEst() &&
            task.time.st <= task.getLst() &&
            task.time.ft <= task.getLft());
        return task.time;
    }


    // Finding the time-optimal insertion position
    InsertionResult findOptimalInsertion_Time(Task& newTask, vector<Task>& tasks) {
        InsertionResult result;

        if (tasks.empty()) {
            TimeParameters initialTime = calculateInitialTasktime(newTask);
            if (initialTime.isValid) {
                tasks.push_back(newTask);
                result.feasible = true;
                result.position = 0;
                result.task_sequence = tasks;
                result.completion_time = initialTime.ft;
                return result;
            }
            return result;
        }
        
        double min_completion_time = INF;
        vector<Task> best_sequence;
        int best_position = -1;

        for (int i = 0; i <= tasks.size(); i++) {
            vector<Task> current_sequence = tasks;
            InsertionResult current_result = tryInsertTask(i, newTask, current_sequence);

            if (current_result.feasible && current_result.completion_time < min_completion_time) {
                
                min_completion_time = current_result.completion_time;
                best_sequence = current_result.task_sequence;
                best_position = i;
            }
        }
        if (best_position != -1) {
            result = tryInsertTask(best_position, newTask, tasks);
            return result;
        }

        return result;
    }


    InsertionResult tryInsertTask(int i, Task& newTask, vector<Task>& tasks) {
        InsertionResult result;
        result.position = i;
        result.feasible = false;

        if (i < 0 || i > tasks.size()) return result;

        vector<Task> current_list = tasks;
        Task taskToInsert = newTask;  // Create a copy to avoid modifying the original task

        if (current_list.empty()) {
            taskToInsert.time = calculateInitialTasktime(taskToInsert);
            if (taskToInsert.time.isValid) {
                result.planned_path = pathPlanner.findPath(
                    static_cast<int>(current_position[0]),
                    static_cast<int>(current_position[1]),
                    static_cast<int>(taskToInsert.getLocation()[0]),
                    static_cast<int>(taskToInsert.getLocation()[1])
                );
                if (!result.planned_path.empty()) {
                    current_list = { taskToInsert };
                    result.feasible = true;
                }
            }
        }
        else {
            current_list.insert(current_list.begin() + i, taskToInsert);
            if (i == 0) {
                current_list[0].time = calculateInitialTasktime(current_list[0]);
                result.planned_path = pathPlanner.findPath(
                    static_cast<int>(current_position[0]),
                    static_cast<int>(current_position[1]),
                    static_cast<int>(current_list[0].getLocation()[0]),
                    static_cast<int>(current_list[0].getLocation()[1])
                );
            }
            else {
                current_list[i].time = calculateTasktimes(current_list[i], current_list[i - 1]);
                result.planned_path = pathPlanner.findPath(
                    static_cast<int>(current_list[i - 1].getLocation()[0]),
                    static_cast<int>(current_list[i - 1].getLocation()[1]),
                    static_cast<int>(current_list[i].getLocation()[0]),
                    static_cast<int>(current_list[i].getLocation()[1])
                );
            }

            if (current_list[i].time.isValid) {
                result.feasible = true;
                for (int j = i + 1; j < current_list.size(); j++) {
                    current_list[j].time = calculateTasktimes(current_list[j], current_list[j - 1]);
                    if (!current_list[j].time.isValid) {
                        result.feasible = false;
                        break;
                    }
                }
            }
        }
        if (result.feasible) {
            result.task_sequence = current_list;
            result.completion_time = current_list.back().time.ft;
        }

        return result;
    }


    bool updateTaskSequence(vector<Task>& sequence) {
        for (int j = 1; j < sequence.size(); j++) {
            Task& current_task = sequence[j];
            Task& before_task = sequence[j - 1];
            current_task.time = calculateTasktimes(current_task, before_task);
            if (!current_task.time.isValid) {
                return false;
            }
        }
        return true;
    }


    // Add New Task In List
    bool addTask(Task& task, int i, vector<Task>& tasks) {
        InsertionResult result = tryInsertTask(i, task, tasks);
        
        if (result.feasible) {
            tasks = result.task_sequence; 
            movement_path.insert(movement_path.end(), result.planned_path.begin(), result.planned_path.end());
            
            current_position = task.getLocation();
            return true;
        }
        return false;
    }


    //calculate Bidding price with TePSSI 
    pair<double, InsertionResult> calculateBid_TePSSI(Task& newTask, Robot& robot, double& alpha) {
        vector<Task> currentTasks = robot.getTasklist();  
        double current_finishTime = 0;  

        if (!currentTasks.empty()) {
            current_finishTime = currentTasks.back().getTimeParameters().ft;
        }

        InsertionResult result = robot.findOptimalInsertion_Time(newTask, currentTasks);
        if (!result.feasible) {
            return { numeric_limits<double>::infinity(), result };
        }

        double bid = alpha * result.completion_time + (1 - alpha) * (result.completion_time - current_finishTime);
        return { bid, result };
    }


    //TePSSI Bidding Method With Heterogeneous
    pair<double, InsertionResult> calculateBid_TePSSI_Heterogeneous (Task& newTask, Robot& robot, double& alpha) {
        if (robot.getRobotType() != newTask.getType()) {
            InsertionResult invalid_result;
            return { numeric_limits<double>::infinity(), invalid_result };
        }
        
        vector<Task> currentTasks = robot.getTasklist();
        double current_finishTime = 0;

        if (!currentTasks.empty()) {
            current_finishTime = currentTasks.back().getTimeParameters().ft;
        }

        InsertionResult result = robot.findOptimalInsertion_Time(newTask, currentTasks);
        if (!result.feasible) {
            return { numeric_limits<double>::infinity(), result };
        }

        double bid = alpha * result.completion_time + (1 - alpha) * (result.completion_time - current_finishTime);
        return { bid, result };
    }


};


#endif