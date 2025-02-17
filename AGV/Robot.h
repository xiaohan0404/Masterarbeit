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
    bool feasible;                               // Is it feasible;
    int position;                                // Insertion Position;
    double completion_time;                      // Completion time;
    vector<Task> task_sequence;                  // Task sequence after insertion;
    vector<pair<int, int>> planned_path;         // Paths that we want to insert;

    InsertionResult() : feasible(false), position(-1), task_sequence({}), planned_path({}),
        completion_time(numeric_limits<double>::infinity())
    {}
};


class Robot {
private:
    string robot_id;                  // Robot ID;
    vector<double> current_position;  // Current Position;
    double speed;                     // Move Speed;
    vector<Task> tasks = {};          // Task List of Robot;
    map<string, int> taskIndex;       // Task Index;
    bool type;                        // Type of Robot: true =high equipt robot;  false = low equipt robot
    const double INF = numeric_limits<double>::infinity();
    double last_completion_time = 0;  

    GridVisualizer viz;                            // Map inside;
    PathPlanner pathPlanner;                       // Path planner;
    vector<vector<pair<int, int>>> Paths = {};          // Store each path;
    vector<pair<int, int>> segment_path = {};           // Stores the path segment currently being executed;


    void addPathSegment(const vector<pair<int, int>>& path_segment, vector<vector<pair<int,int>>>& Paths, int i ) {
        if (Paths.empty()) {
            Paths.push_back(path_segment);
        }
        else if (i >= Paths.size()) {
            vector<pair<int, int>> new_segment(path_segment.begin() + 1, path_segment.end());
            Paths.push_back(new_segment);
        }
        else {
            vector<pair<int, int>> new_segment(path_segment.begin() + 1, path_segment.end());
            Paths.insert(Paths.begin() + i, new_segment);
            updatePathSegment(i, Paths);
        }
    }



    void updatePathSegment(int segment_index, vector<vector<pair<int, int>>>& Paths) {
        if (segment_index >= 0 && segment_index < Paths.size() - 1) { 
            if (!Paths[segment_index].empty() && !Paths[segment_index + 1].empty()) { 
                pair<int, int> starPoint = Paths[segment_index].back();
                pair<int, int> endPoint = Paths[segment_index + 1].back();

                vector<pair<int, int>> recalculate_path = pathPlanner.findPath(
                    starPoint.first, starPoint.second, endPoint.first, endPoint.second
                );
                if (!recalculate_path.empty()) {
                    Paths[segment_index + 1] = recalculate_path;
                }
            }
            else {
                cout << "Error: One of the path segments is empty." << std::endl;
            }
        }
        else {
            cout << "Error: segment_index out of bounds." << std::endl;
        }
    }



    bool checkPath(const vector<vector<pair<int, int>>>& Paths) {    //Determine whether the path is continuous;   判断路径是否连续
        if (Paths.size() < 2) {
            return true; 
        }

        for (size_t i = 1; i < Paths.size(); i++) { 
            if (!Paths[i - 1].empty() && !Paths[i].empty()) {
                pair<int, int> starPoint = Paths[i - 1].back();
                pair<int, int> endPoint = Paths[i].front();

                if (starPoint != endPoint) {
                    return false; 
                }
            }
            else {
                return false; 
            }
        }
        return true;
    }


    void rebuildCompletePath(vector<Task>& tasks) {          //Rebuild the full path;
        if (tasks.empty()) return;

        vector<vector<pair<int, int>>> rebuilt_paths;
        vector<double> position = current_position;
        for (int i = 0; i < tasks.size();i++) {
            Task current_task = tasks[i];
            vector<pair<int, int>> current_path = pathPlanner.findPath(position[0], position[1], current_task.getLocation()[0], current_task.getLocation()[1]);
            rebuilt_paths.push_back(current_path);
            position = current_task.getLocation();
        }
        Paths = rebuilt_paths;
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
            std::cout << "WARNING: No viable path found!" << endl;
            return -1.0; 
        }
        // Calculate the actual path length
        double pathLength = pathPlanner.calculatePathLength(path);
        return pathLength / speed;  // Returns the actual travel time
    }

public:
    Robot(const string& id, const vector<double>& initial_pos, double move_speed, bool robot_type, const GridVisualizer& visualizer, const PathPlanner& planner) :
        robot_id(id), current_position(initial_pos), speed(move_speed), type(robot_type), viz(visualizer), pathPlanner(planner) {
    }

    vector<Task>& getTasklist() { return tasks; }
    string& getName() { return robot_id; }
    bool& getRobotType() { return type; }
    const vector<double>& getCurrentPosition() const { return current_position; }
    vector<vector<pair<int, int>>>& getPath() { return Paths; }
    
    const vector<pair<int, int>>& getPathSegment(int index)  {
        if (index >= 0 && index < Paths.size()) {
            return Paths[index];
        }
        static vector<pair<int, int>> empty_path;
        return empty_path;
    }


    void generatePathReport() {
        double total_distance = 0.0;
        double max_completion_time = 0.0;

        if (!tasks.empty()) {
            for (const auto& path_segment : Paths) {
                total_distance += pathPlanner.calculatePathLength(path_segment);
            }
            max_completion_time = tasks.back().time.ft;
        }
        if (!checkPath(Paths)) {
            cout << "Warning: Path continuity check failed!" << endl;
        }
        // Generate a map with paths
        string filename = "robot_" + robot_id + "_path.svg";
        viz.generateSVG(filename, Paths);

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
            vector<pair<int, int>> path = pathPlanner.findPath(current_position[0],current_position[1],newTask.getLocation()[0],newTask.getLocation()[1]);
            if (initialTime.isValid && !path.empty()) {
                result.feasible = true;
                result.position = 0;
                result.task_sequence = tasks;
                result.completion_time = initialTime.ft;
                result.planned_path = path;
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
        Task taskToInsert = newTask;                // Create a copy to avoid modifying the original task

        if (current_list.empty()) {
            taskToInsert.time = calculateInitialTasktime(taskToInsert);
            if (taskToInsert.time.isValid) {
                vector<pair<int, int>> initial_path = pathPlanner.findPath(
                    static_cast<int>(current_position[0]),
                    static_cast<int>(current_position[1]),
                    static_cast<int>(taskToInsert.getLocation()[0]),
                    static_cast<int>(taskToInsert.getLocation()[1])
                );
                if (!initial_path.empty()) {
                    result.planned_path = initial_path;
                    current_list = { taskToInsert };
                    result.feasible = true;
                }
            }
        }
        else {
            // Insert the new task
            current_list.insert(current_list.begin() + i, taskToInsert);

            // Calculate path to the inserted task
            if (i == 0) {
                current_list[0].time = calculateInitialTasktime(current_list[0]);
                vector<pair<int, int>> first_path = pathPlanner.findPath(
                    static_cast<int>(current_position[0]),
                    static_cast<int>(current_position[1]),
                    static_cast<int>(current_list[0].getLocation()[0]),
                    static_cast<int>(current_list[0].getLocation()[1])
                );
                if (!first_path.empty()) {
                    result.planned_path = first_path;
                    result.feasible = current_list[0].time.isValid;
                }
            }
            else {
                current_list[i].time = calculateTasktimes(current_list[i], current_list[i - 1]);
                vector<pair<int, int>> to_inserted_path = pathPlanner.findPath(
                    static_cast<int>(current_list[i - 1].getLocation()[0]),
                    static_cast<int>(current_list[i - 1].getLocation()[1]),
                    static_cast<int>(current_list[i].getLocation()[0]),
                    static_cast<int>(current_list[i].getLocation()[1])
                );
                if (!to_inserted_path.empty()) {
                    result.planned_path = to_inserted_path;
                    result.feasible = current_list[i].time.isValid;
                }
            }
        }
        if (result.feasible && i < current_list.size() - 1) {
            for (int j = i + 1; j < current_list.size(); j++) {
                current_list[j].time = calculateTasktimes(current_list[j], current_list[j - 1]);
                if (!current_list[j].time.isValid) {
                    result.feasible = false;
                    break;
                }
            }
        }

        if (result.feasible) {
            result.task_sequence = current_list;
            result.completion_time = current_list.back().time.ft;
        }
        return result;
    }


    bool updateTaskSequenceTime(vector<Task>& sequence) {      //Recalculate the TimeParameters After an Task Insertion;
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


    void updateLastCompletionTime() {                         //Update last completion time;
        if (!tasks.empty()) {
            last_completion_time = tasks.back().getTimeParameters().ft;
        }
    }


    // Add New Task In List;
    bool addTask( int i, Task& task, vector<Task>& tasks, vector<vector<pair<int,int>>>& Paths ) {
        InsertionResult result = tryInsertTask(i, task, tasks);

        if (result.feasible) {
            tasks = result.task_sequence;
            if (!result.planned_path.empty()) {
                vector<pair<int, int>> new_segment = result.planned_path;
                if (tasks.size() == 1) {
                    Paths.push_back(new_segment);
                    updateLastCompletionTime();
                }
                else {
                    addPathSegment(new_segment, Paths, i);
                    if (checkPath(Paths)) {
                        updateTaskSequenceTime(tasks);
                        updateLastCompletionTime();
                    }
                    else {
                        rebuildCompletePath(tasks);
                        updateLastCompletionTime();
                    }
                }
                return true;
            }
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