#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <memory>
#include <limits>
#include <algorithm>
#include "distance_utils.h"
#include "Task.h"
#include "Robot.h"
using namespace std;



class AuctionManager {
public:
    AuctionManager(vector<Robot>& robots, Task* unassigned_task)
        : robots(robots), unassigned_task(unassigned_task) {}

    // Check if the precedence constraints are satisfied
    bool checkPriorityConstraints(const Task& newTask, const vector<Task>& existingTasks, int insertPosition) {
        for (const Task* prerequisite : newTask.getPrerequisites()) {
            bool prerequisiteFound = false;

            // Check if the predecessor task exists before the insertion position and has been completed
            for (int i = 0; i < insertPosition; i++) {
                if (existingTasks[i].getTaskId() == prerequisite->getTaskId()) {
                    if (!prerequisite->isCompleted()) {
                        return false;  // The predecessor task exists but is not completed
                    }
                    prerequisiteFound = true;
                    break;
                }
            }
            if (!prerequisiteFound) {
                return false;  // Required predecessor task not found
            }
        }

        // Check the tasks after the insertion position to ensure do not violate their predecessor constraints
        for (size_t i = insertPosition; i < existingTasks.size(); i++) {
            const vector<Task*>& laterTaskPrereqs = existingTasks[i].getPrerequisites();
            // Check if the current task is a predecessor of the subsequent task
            for (const Task* prereq : laterTaskPrereqs) {
                if (prereq->getTaskId() == newTask.getTaskId()) {
                    return false;  // Violation of the predecessor constraint of the subsequent task
                }
            }
        }
        return true;
    }


    Robot* runAuction(Task* newTask) {
        double min_bidding = INFINITY;
        Robot* winner = nullptr;
        InsertionResult best_result;

        for (Robot& robot : robots) {
            double alpha=0.7;
            auto current_result = robot.calculateBid_TePSSI(*newTask, robot, alpha);

            double bid = current_result.first;
            InsertionResult result = current_result.second;

            if (bid <  min_bidding && result.feasible) {
                    min_bidding = bid;
                    winner = &robot;
                    best_result = result;
            }
        }
        if (winner != nullptr && min_bidding > -1) {
            if (winner->addTask(best_result.position, *newTask, winner->getTasklist(), winner->getPath())) return winner;
        }
        return nullptr;
    }


private:
    vector<Robot>& robots;
    Task* unassigned_task;
};