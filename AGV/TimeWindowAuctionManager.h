#ifndef TIMEWINDOWAUCTIONMANAGER_H
#define TIMEWINDOWAUCTIONMANAGER_H

#include"Robot.h"
#include"Task.h"
#include"Biding.h"
#include<vector>
#include<iostream>
#include<algorithm>
using namespace std;

class TimeWindowAuctionManager {
private:
    vector<Robot>& robots;
    vector<Task*>& all_tasks;
    double window_size;      
    double current_time;    
    set<Task*> auctioned_tasks;

    // Check whether the task can participate in the auction
    bool isTaskAuctionable(Task* task) {
        if (task->isCompleted()) {
            return false;
        }

        double window_end = current_time + window_size;
        if (task->getEst() >= window_end || task->getLst() < current_time) {
            return false;
        }

        for (const Task* prereq : task->getPrerequisites()) {
            if (!prereq->isCompleted()) {
                return false;
            }

            if (prereq->time.ft >= current_time + window_size) {
                return false;
            }
        }

        return true;
    }

    // Get auctionable tasks within the current time window
    vector<Task*> getTasksInCurrentWindow() {
        vector<Task*> window_tasks;

        for (Task* task : all_tasks) {
            if (isTaskAuctionable(task)) {
                window_tasks.push_back(task);
            }
        }

        sort(window_tasks.begin(), window_tasks.end(),
            [](Task* a, Task* b) { return a->getPriorityLevel() < b->getPriorityLevel(); });

        return window_tasks;
    }

    // Check if all tasks are completed
    bool areAllTasksProcessed() {
        for (Task* task : all_tasks) {
            if (!task->isCompleted()) {
                if (auctioned_tasks.find(task) == auctioned_tasks.end()) {
                    return false;
                }
            }
        }
        return true;
    }

public:
    TimeWindowAuctionManager(vector<Robot>& robots, vector<Task*>& tasks,
        double window_size = 100.0)
        : robots(robots), all_tasks(tasks),
        window_size(window_size), current_time(0.0)
    {}

    void runTimeWindowAuction() {
        while (!areAllTasksProcessed()) {
            vector<Task*> current_window_tasks = getTasksInCurrentWindow();

            if (current_window_tasks.empty()) {
                current_time += window_size;
                continue;
            }

            cout << "\nCurrent Time Window [" << current_time << ", "
                << current_time + window_size << "]" << endl;
            cout << "Current Number Of Assigned Tasks: " << current_window_tasks.size() << endl;

            for (Task* task : current_window_tasks) {
                cout << "\nAuction Tasks " << task->getTaskId()
                    << " (Priority Level: " << task->getPriorityLevel() << ")" << endl;

                double min_bid = INFINITY;
                Robot* winner = nullptr;
                InsertionResult best_result;

                for (Robot& robot : robots) {
                    double alpha = 0.7;
                    auto current_result = robot.calculateBid_TePSSI_Heterogeneous(*task, robot, alpha);
                    double bid = current_result.first;
                    InsertionResult result = current_result.second;

                    if (bid < min_bid && result.feasible) {
                        min_bid = bid;
                        winner = &robot;
                        best_result = result;
                    }
                }

                if (winner != nullptr) {
                    if (winner->addTask(best_result.position , *task, winner->getTasklist(), winner->getPath())) {
                        for (const Task& planned_task : best_result.task_sequence) {
                            if (planned_task.getTaskId() == task->getTaskId()) {
                                task->time = planned_task.time;
                                break;
                            }
                        }

                        task->setCompleted(true);
                        auctioned_tasks.insert(task);
                        cout << "TASK " << task->getTaskId()
                            << " Assign To Robot " << winner->getName() << endl;
                        cout << "Start Time: " << task->time.st << endl;
                        cout << "Finish Time: " << task->time.ft << endl;
                    }
                }
                else {
                    cout << "Task " << task->getTaskId() << " Unable to Allocate!" << endl;
                }
            }

            cout << "\nCurrent Robot Task List:" << endl;
            for (Robot& robot : robots) {
                cout << robot.getName() << " Task Sequence: ";
                const vector<Task> tasks = robot.getTasklist();
                if (tasks.empty()) {
                    cout << "Пе";
                }
                else {
                    for (size_t i = 0; i < tasks.size(); ++i) {
                        cout << tasks[i].getTaskId();
                        if (i < tasks.size() - 1) {
                            cout << " -> ";
                        }
                    }
                }
                cout << endl;
            }
            cout << "--------------------------------" << endl;

            current_time += window_size;
        }

        cout << "\n=== Task Assignment Completion Statistics ===" << endl;
        for (Robot& robot : robots) {
            robot.generatePathReport();
        }
        cout << "==========================\n" << endl;
    }
};

#endif