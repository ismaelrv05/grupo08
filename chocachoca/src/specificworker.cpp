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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    this->startup_check_flag = startup_check;
    // Uncomment if there's too many debug messages
    // but it removes the possibility to see the messages
    // shown in the console with qDebug()
//  QLoggingCategory::setFilterRules("*.debug=false\n");
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
//  THE FOLLOWING IS JUST AN EXAMPLE
//  To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//  try
//  {
//     RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//     std::string innermodel_path = par.value;
//     innerModel = std::make_shared(innermodel_path);
//  }
//  catch(const std::exception &e) { qFatal("Error reading config params"); }
    return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;
    this->Period = period;
    state = State::SPIRAL;
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
    try{
        auto ldata = lidar3d_proxy->getLidarData("bpearl", 0, 2 * M_PI, 1);
        const auto &points = ldata.points;
        if (points.empty()) return;

        RoboCompLidar3D::TPoints filtered_points;
        std::ranges::remove_copy_if(ldata.points, std::back_inserter(filtered_points),[](auto &p) { return p.z > 2000; });
        draw_lidar(filtered_points, viewer);

        // State Machine
        switch (state)
        {
            case State::STRAIGHT_LINE:
                std::cout << "STRAIGHT_LINE" << std::endl;
                straight_line(filtered_points);
                break;
            case State::TURN:
                std::cout << "TURN" << std::endl;
                turn(filtered_points);
                break;
            case State::FOLLOW_WALL:
                std::cout << "FOLLOW_WALL" << std::endl;
                follow_wall(filtered_points);
                break;
            case State::SPIRAL:
                std::cout << "SPIRAL" << std::endl;
                spiral(filtered_points);
                break;
        }
    }
    catch(const Ice::Exception &e)
    {
        std::cout << "Error reading from Camera" << e << std::endl;
    }
}

void SpecificWorker::straight_line(const RoboCompLidar3D::TPoints &filtered_points)
{
    int offset = filtered_points.size() / 2 - filtered_points.size() / 5;
    auto min_elem = std::min_element(filtered_points.begin() + offset, filtered_points.end() - offset,
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);});
    float advx = 0.0;
    float advz= 0.0;
    float rot = 0.0;
    std::cout<<"Distance: "<<std::hypot(min_elem->x, min_elem->y)<<std::endl;

    //Si llega a la pared gira
    if(std::hypot(min_elem->x, min_elem->y) < DIST_COL)
    {
        state = State::TURN;
    }
    else {
        advx = 3.0;
        advz = 0.0;
        rot = 0.0;
        omnirobot_proxy->setSpeedBase(advx, advz, rot);
    }
}

void SpecificWorker::turn(const RoboCompLidar3D::TPoints &filtered_points)
{
    int offset = filtered_points.size() / 2 - filtered_points.size() / 5;
    auto min_elem = std::min_element(filtered_points.begin() + offset, filtered_points.end() - offset,
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);});
    std::cout<<"Distance: "<<std::hypot(min_elem->x, min_elem->y)<<std::endl;

    if(std::hypot(min_elem->x, min_elem->y) < DIST_COL)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> distribution(2, 3);
        int choice = distribution(gen);

        switch (choice)
        {
            case 2:
                state = State::STRAIGHT_LINE;
                omnirobot_proxy->setSpeedBase(0, 0, 1);
                break;
            case 3:
                state = State::SPIRAL;
                omnirobot_proxy->setSpeedBase(0, 0, -1);
                break;
            default:
                state = State::TURN;
        }
    }
    else
    {
        state = State::TURN;
    }
}

void SpecificWorker::follow_wall(const RoboCompLidar3D::TPoints &filtered_points)
{
    int offset = filtered_points.size() / 2 - filtered_points.size() / 4;
    auto min_elem = std::min_element(filtered_points.begin() + offset, filtered_points.end() - offset,
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y); });

    float rot = std::atan2(min_elem->y, min_elem->x);
    float rot_ang = 7 * rot; //calculate angular speed
    float advx = 1.5;
    float advz = 0.0;
    int i = 0;

    std::cout<<"Distance: "<<std::hypot(min_elem->x, min_elem->y)<<std::endl;

    if(std::hypot(min_elem->x, min_elem->y) < DIST_PARED)
    {
        omnirobot_proxy->setSpeedBase(advx, advz, rot_ang);
        if (rot_ang>1)
            i++;


        if(i >= 24)
        {
            DIST_PARED = DIST_PARED - 100.0;
            i = 0;
            state = State::STRAIGHT_LINE;
        }
    }
    else
    {
        rot = 0.0;
        omnirobot_proxy->setSpeedBase(advx, advz, rot);
    }
}

void SpecificWorker::spiral(const RoboCompLidar3D::TPoints &filtered_points)
{
    int offset = filtered_points.size() / 2 - filtered_points.size() / 5;
    auto min_elem = std::min_element(filtered_points.begin() + offset, filtered_points.end() - offset,
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);});
    std::cout<<"Distance: "<<std::hypot(min_elem->x, min_elem->y)<<std::endl;
    static float fwSpeed = 0.1;
    static float rotSpeed = 1.2;

    omnirobot_proxy->setSpeedBase(fwSpeed, 0, rotSpeed);

    if (fwSpeed >= 2.0)
    {
        rotSpeed -= 0.005;
    }
    else
    {
        fwSpeed += 0.02;
        rotSpeed += 0.02;
    }

    if (std::hypot(min_elem->x, min_elem->y) < DIST_COL)
    {
        omnirobot_proxy->setSpeedBase(0, 0, 3);
    }
}

int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &points, AbstractGraphicViewer *pViewer)
{
    static std::vector<QGraphicsItem*> borrar;
    for(auto &p: borrar)
    {
        viewer->scene.removeItem(p);
        delete p;
    }
    borrar.clear();
    for(const auto &p: points)
    {
        auto r =  pViewer->scene.addRect(-25, -25, 50, 50, QPen(QColor("green")), QBrush(QColor("green")));
        r->setPos(p.x, p.y);
        borrar.push_back(r);
    }
}
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