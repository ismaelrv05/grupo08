/*
*    Copyright (C) 2023 by YOUR NAME HERE
*
*    This file is part of RoboComp
*
*    RoboComp is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    RoboComp is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "specificworker.h"
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>
#include <cppitertools/enumerate.hpp>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    this->startup_check_flag = startup_check;
}
/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
    std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;
    this->Period = period;
    if(this->startup_check_flag)
    {
        this->startup_check();
    }
    else
    {
        viewer = new AbstractGraphicViewer(this, QRectF(-5000, -5000, 10000, 10000));
        viewer->add_robot(460, 480, 0, 100, QColor("Blue"));
        viewer->show();
        viewer->activateWindow();
        timer.start(Period);
    }
}

void SpecificWorker::compute()
{
    auto ldata = lidar3d_proxy->getLidarData("helios", 0, 360, 1);
    const auto &points = ldata.points;
    if (points.empty()) return;

    /// Filter points above 2000
    RoboCompLidar3D::TPoints filtered_points;
    std::ranges::remove_copy_if(ldata.points, std::back_inserter(filtered_points),
                                [](auto &p) { return p.z > 2000; });

    std::vector<std::pair<float, float>> ranges_list = {{1000, 2500}};
    auto lines = extract_lines(points, ranges_list);
    auto doors = door_detector.detect(lines, &viewer->scene);

    match_door_target(doors, door_target);


    // state machine
    state_machine(doors);

    //draw
    draw_lines(lines, viewer);
    draw_target_door(door_target, viewer);
}

void SpecificWorker::state_machine(const Doors &doors)
{
    // Static variable to keep track of the number of nodes visited in the graph
    static int countNodes=0;

    switch (state)
    {
        // IDLE state: Stop the robot
        case States::IDLE:
        {
            move_robot(0,0,0);
            break;
        }
        // SEARCH_DOOR state: Look for the door with the smallest angle to the robot
        case States::SEARCH_DOOR:
        {
            if(!doors.empty())
            {
                // Find the door with the smallest angle to the robot
                Door closest_door = doors[0];
                for (const auto& door : doors)
                {
                    if (fabs(door.angle_to_robot()) < fabs(closest_door.angle_to_robot()))
                    {
                        closest_door = door;
                    }
                }

                // Set the found door as the target, stop the robot, and transition to GOTO_DOOR state
                door_target = closest_door;
                move_robot(0,0,0);
                state = States::GOTO_DOOR;
                qInfo() << "Door with smallest angle found";
                door_target.print();
            }
            else
                // If no doors are detected, move forward with a small speed
                move_robot(0,0,0.3);
            break;
        }
        case States::GOTO_DOOR:
        {
            qInfo() << "The graph is: ";
            graph.print();
            std::cout << "Actual room: " << countNodes << std::endl;

            // Check if the robot is close enough to the target door
            if(door_target.perp_dist_to_robot() < consts.DOOR_PROXIMITY_THRESHOLD)
            {
                // If close enough, move forward, log information, and transition to ALIGN state
                move_robot(1,0, 0);
                qInfo() << "GOTO_DOOR Target achieved";
                state = States::ALIGN;
            }
            else
            {
                // If not close enough, calculate rotation and advance based on distance and angles
                float rot = -0.5 * door_target.perp_angle_to_robot();
                float adv = consts.MAX_ADV_SPEED * break_adv(door_target.perp_dist_to_robot()) *
                            break_rot(door_target.perp_angle_to_robot()) / 1000.f;
                move_robot(0, adv, rot);
            }
            break;
        }

        // ALIGN state: Align the robot with the target door
        case States::ALIGN:
        {
            // Check if the robot is aligned with the target door within a small angle threshold
            if(fabs(door_target.angle_to_robot()) < 0.01)
            {
                // If aligned, stop the robot and transition to GO_THROUGH state
                move_robot(0,0,0);
                state = States::GO_THROUGH;

                return;
            }
            // Adjust the rotation to align with the target door
            float rot = -0.5 * door_target.angle_to_robot();
            move_robot(0,0,rot);
            break;
        }

        // GO_THROUGH state: Move forward for a certain duration, update the graph, and transition to SEARCH_DOOR state
        case States::GO_THROUGH:
        {
            // Get the current time
            auto now = std::chrono::steady_clock::now();

            // Check if the state just started
            if (!Timer.time_since_epoch().count())
                Timer = now;

            // Check if 10000 ms (10 seconds) have passed
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - Timer) < std::chrono::milliseconds(10000))
            {
                // If less than 10 seconds have passed, continue moving the robot forward
                move_robot(0, 1.0, 0);
            }
            else
            {
                // If 10 seconds have passed, reset the timer and update the graph
                Timer = std::chrono::steady_clock::time_point();  // Reset timer

                // Update the graph by adding nodes and edges based on the current room count
                if(graph.num_nodes()<=3){
                    int addNode = graph.addNode();
                    graph.addEdge(addNode-1, addNode);
                }
                else{
                    countNodes++;
                    if (countNodes == 4){
                        countNodes=0;
                    }

                }
                state = States::SEARCH_DOOR; // Transition to the next state
            }
            break;
        }

    }
}


void SpecificWorker::match_door_target(const Doors &doors, const Door &target)
{
    if(doors.empty())
        return;

    if(auto res = std::ranges::find(doors, target); res != doors.end())
        door_target = *res;
    else
    {
        move_robot(0,0,0);
        state = States::SEARCH_DOOR;
        qInfo() << "GOTO_DOOR Door lost, searching";
    }
}

void SpecificWorker::move_robot(float side, float adv, float rot)
{
    try
    {
        omnirobot_proxy->setSpeedBase(adv, 0, rot);
    }
    catch(const Ice::Exception &e){ std::cout << e << std::endl;}
}

SpecificWorker::Lines SpecificWorker::extract_lines(const RoboCompLidar3D::TPoints &points, const std::vector<std::pair<float, float>> &ranges)
{
    Lines lines(ranges.size());
    for(const auto &p: points)
        for(const auto &[i, r] : ranges | iter::enumerate)
            if(p.z > r.first and p.z < r.second)
                lines[i].emplace_back(p.x, p.y);
    return lines;
}
float SpecificWorker::break_adv(float dist_to_target)
{
    return std::clamp(dist_to_target / consts.DOOR_PROXIMITY_THRESHOLD, 0.f, 1.f );
}
float SpecificWorker::break_rot(float rot)
{
    return rot>=0 ? std::clamp(1-rot, 0.f, 1.f) : std::clamp(rot+1, 0.f, 1.f);
}

int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &points, AbstractGraphicViewer *viewer)
{
    static std::vector<QGraphicsItem*> borrar;
    for(auto &b : borrar) {
        viewer->scene.removeItem(b);
        delete b;
    }
    borrar.clear();

    for(const auto &p : points)
    {
        auto point = viewer->scene.addRect(-50,-50,100, 100,
                                           QPen(QColor("green")), QBrush(QColor("green")));
        point->setPos(p.x, p.y);
        borrar.push_back(point);
    }
}
void SpecificWorker::draw_target_door(const Door &target, AbstractGraphicViewer *viewer, QColor color, QColor color_far)
{
    static std::vector<QGraphicsItem *> borrar;
    for (auto &b: borrar) {
        viewer->scene.removeItem(b);
        delete b;
    }
    borrar.clear();

    auto perp = door_target.point_perpendicular_to_door_at();
    auto middle = viewer->scene.addRect(-100, -100, 200, 200, color, QBrush(color));
    middle->setPos(perp.first.x(), perp.first.y());
    auto middle_far= viewer->scene.addRect(-100, -100, 200, 200, color_far, QBrush(color_far));
    middle_far->setPos(perp.second.x(), perp.second.y());
    borrar.push_back(middle);
    borrar.push_back(middle_far);
}

void SpecificWorker::draw_lines(const Lines &lines, AbstractGraphicViewer *pViewer)
{
    static std::vector<QGraphicsItem*> borrar;
    for(auto &b : borrar) {
        pViewer->scene.removeItem(b);
        delete b;
    }
    borrar.clear();

    for(const auto &line : lines)
        for(const auto &p : line)
        {
            auto point = pViewer->scene.addRect(-50,-50,100, 100,
                                                QPen(QColor("lightblue")), QBrush(QColor("lightblue")));
            point->setPos(p.x(), p.y());
            borrar.push_back(point);
        }
}

// void SpecificWorker::draw_doors(const Doors &doors, QGraphicsScene *scene, QColor color)
//{
//    static std::vector<QGraphicsItem *> borrar;
//    for (auto &b: borrar) {
//        viewer->scene.removeItem(b);
//        delete b;
//    }
//    borrar.clear();

//    for (const auto &d: doors) {
//        auto point = viewer->scene.addRect(-50, -50, 100, 100, QPen(color), QBrush(color));
//        point->setPos(d.left.x, d.left.y);
//        borrar.push_back(point);
//        point = viewer->scene.addRect(-50, -50, 100, 100, QPen(color), QBrush(color));
//        point->setPos(d.right.x, d.right.y);
//        borrar.push_back(point);
//        auto line = viewer->scene.addLine(d.left.x, d.left.y, d.right.x, d.right.y, QPen(color, 50));
//        borrar.push_back(line);
//    }
//}

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData
/*
*
*/