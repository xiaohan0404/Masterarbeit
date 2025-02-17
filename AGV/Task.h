#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <memory>
#include <limits>
#include <algorithm>
using namespace std;


struct TimeParameters {
    double tt;  // travel time
    double st;  // start time
    double ft;  // finish time
    bool isValid;  // valid or not

    TimeParameters() : tt(-1), st(-1), ft(-1), isValid(false) {}
};


class Task {
private:
    string task_id;
    vector<double> location_task;
    double Est;              // Earliest Start Time
    double Lft;              // Latest Finish Time
    double Dut;              // Duration Time
    double Lst;              // Latest Start Time
    bool type;               // Task type: true = high level task;    false = low level task;
    bool is_completed;       // Finish or not 
    int priority_level;      // Priority Level 
    vector<Task*> prerequisites = {};    // Prerequisites Task List
    vector<Task*> successors = {};       // Follow_up Task List

    void validateTimes() {
        if (Dut < 0) {
            throw invalid_argument("Duration time (Dut) must be non-negative");
        }
        if (Lft < Est + Dut) {
            throw invalid_argument("Latest finish time (Lft) must allow for the duration");
        }
        Lst = Lft - Dut;
    }

public:
    TimeParameters time;
    
    Task(const string& taskId, const vector<double>& pos_task, double earliestStart, double latestFinish, double duration, bool task_type)
        : task_id(taskId), location_task(pos_task), Est(earliestStart), Lft(latestFinish), Dut(duration), type(task_type), is_completed(false),
        priority_level(0) {
        validateTimes();
    }

    Task(const string& taskId, double x, double y, double earliestStart, double latestFinish, double duration, bool task_type)
        : task_id(taskId), location_task({ x, y }), Est(earliestStart), Lft(latestFinish), Dut(duration), type(task_type) {
        validateTimes();
    }


    // Adding a Predecessor Task
    void addPrerequisite(Task* task) {
        if (task != nullptr) {
            prerequisites.push_back(task);
            task->successors.push_back(this);
            cout << "Added task " << task->getTaskId() << " as prerequisite to " << task_id << endl;
            cout << "Prerequisites count for " << task_id << ": " << prerequisites.size() << endl;
            cout << "Successors count for " << task->getTaskId() << ": " << task->successors.size() << endl;
        }
    }

    void removePrerequisite(Task* task) {
        if (task != nullptr) {
            prerequisites.erase(
                remove(prerequisites.begin(), prerequisites.end(), task),
                prerequisites.end()
            );
            task->successors.erase(
                remove(task->successors.begin(), task->successors.end(), this),
                task->successors.end()
            );
        }
    }

    void printDependencies() const {
        cout << "\nTask " << task_id << " dependencies:" << endl;
        cout << "Prerequisites (" << prerequisites.size() << "): ";
        for (const Task* task : prerequisites) {
            cout << task->getTaskId() << " ";
        }
        cout << "\nSuccessors (" << successors.size() << "): ";
        for (const Task* task : successors) {
            cout << task->getTaskId() << " ";
        }
        cout << endl;
    }

    
    // Getters
    string getTaskId() const { return task_id; }
    const vector<double>& getLocation() const { return location_task; }
    double getEst() const { return Est; }
    double getLft() const { return Lft; }
    double getDut() const { return Dut; }
    double getLst() const { return Lst; }
    const TimeParameters& getTimeParameters() const { return time; }
    bool getType() const { return type; }

    bool isCompleted() const { return is_completed; }
    void setCompleted(bool completed = true) { is_completed = completed; }
    int getPriorityLevel() const { return priority_level; }
    void setPriorityLevel(int level) { priority_level = level; }
    size_t getPrerequisiteCount() const { return prerequisites.size(); }
    size_t getSuccessorCount() const { return successors.size(); }
    const vector<Task*>& getPrerequisites() const { return prerequisites; }
    const vector<Task*>& getSuccessors() const { return successors; }



    // Overload comparison operators (based on precedence level)
    bool operator<(const Task& other) const {
        return priority_level < other.priority_level;
    }

    bool operator>(const Task& other) const {
        return priority_level > other.priority_level;
    }

};