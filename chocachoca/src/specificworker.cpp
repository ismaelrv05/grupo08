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
    RoboCompLidar3D::TData ldata;
    try
    {   auto ldata = lidar3d_proxy->getLidarData("bpearl", 0, 360, 4);
        qInfo() << ldata.points.size();
        const auto &points =ldata.points;

        RoboCompLidar3D::TPoints filtered_points;
        std::ranges::copy_if(ldata.points, std::back_inserter(filtered_points), [](auto &p) {return p.z < 2000;});
        draw_lidar(filtered_points, viewer);

        int offset = filtered_points.size()/2-filtered_points.size()/3;
        auto min_elem = std::min_element(points.begin()+offset, points.end()- offset, [](auto a, auto b) {return std::hypot(a.x, a.y)< std::hypot(b.x, b.y);});

    }
    catch (const Ice::Exception & e) { std::cout<<e.what() << std::endl; return;};

    State state = std::get<0>(tuple);
    std::cout << "STATE: " << static_cast<std::underlying_type<State>::type>(state) << std::endl;

    // State Machine
    switch (state)
    {
        default:
            std:: cout << "IDLE" << std::endl;
            tuple = idle(ldata);
            break;
        case State::STRAIGHT_LINE:
            std::cout << "STRAIGHT_LINE" << std::endl;
            tuple = straight_line(ldata);
            break;
        case State::TURN:
            std::cout << "TURN" << std::endl;
            tuple = turn(ldata);
            break;
        case State::FOLLOW_WALL:
            std::cout << "FOLLOW_WALL" << std::endl;
            tuple = follow_wall(ldata);
            break;
        case State::SPIRAL:
            std::cout << "SPIRAL" << std::endl;
           // tuple = spiral(ldata);
            break;
    }
    //start the robot
    auto &[_, adv, rot] = tuple;
    try
    { omnirobot_proxy->setSpeedBase(3, adv, rot); }
    catch (const Ice::Exception & e) { std::cout<<e.what() << std::endl; tuple;}
}


std::tuple<SpecificWorker::State,float,float> SpecificWorker::idle(const RoboCompLidar3D::TData &ldata){
    auto tuple = std::make_tuple(State::STRAIGHT_LINE, MAX_SPEED, 0);
    return tuple;
}


std::tuple<SpecificWorker::State, float,float> SpecificWorker::straight_line(const RoboCompLidar3D::TData &ldata){
    float adv = 0.0;
    float rot = 0.0;
    RoboCompLidar3D::TData avanza;
    avanza.points.assign(ldata.points.begin() + ldata.points.size() / 3, ldata.points.end() - ldata.points.size() / 3);
    std::cout<<"Distancia: "<<avanza.points.front().distance2d<<std::endl;
    State state = State::STRAIGHT_LINE;

    //Si llega a la pared gira
    if((avanza.points.front().distance2d < DIST_COL))
    {
        state = State::TURN;
        adv = 100.0;
        RoboCompLidar3D::TData lateralder;
        lateralder.points.assign(ldata.points.end() - ldata.points.size()/3, ldata.points.end() - 150);
        RoboCompLidar3D::TData lateralizq;
        lateralizq.points.assign(ldata.points.begin() + 150, (ldata.points.begin() + ldata.points.size()/3)-1);
        std::ranges::sort(lateralder.points, {}, &RoboCompLidar3D::TPoint::distance2d);
        std::ranges::sort(lateralizq.points, {}, &RoboCompLidar3D::TPoint::distance2d);
        if (lateralder.points.front().distance2d > lateralizq.points.front().distance2d)
            rot = -1.0;
        else
            rot = 1.0;
        auto tuple= std::make_tuple(State::STRAIGHT_LINE, adv, rot);
        return tuple;
    }
}


std::tuple<SpecificWorker::State, float,float> SpecificWorker::turn(const RoboCompLidar3D::TData &ldata)
{
    RoboCompLidar3D::TData avanza;
    avanza.points.assign(ldata.points.begin() + ldata.points.size()/3, (ldata.points.end() - ldata.points.size()/3)-1);
    std::ranges::sort(avanza.points, {}, &RoboCompLidar3D::TPoint::distance2d);
    std::cout<<"Distance: "<<avanza.points.front().distance2d<<std::endl;
    State state = State::TURN;

    if((avanza.points.front().distance2d > DIST_COL))
    {
        int choice;
        std::srand(time(0));
        choice = rand() % 2;
        switch(choice)
        {
            case 0:
                state = State::STRAIGHT_LINE;
                break;
            case 1:
                state = State::FOLLOW_WALL;
                break;
            default:
                state = State::STRAIGHT_LINE;
                break;
        }
        float adv = MAX_SPEED;
        float rot = 0.0;
        auto tuple = std ::make_tuple(state, adv, rot);
        return tuple;
    }
    else
    {
        auto tuple = std ::make_tuple(state, 0, 0);
        return tuple;
    }
}

std::tuple<SpecificWorker::State, float, float> SpecificWorker::follow_wall(const RoboCompLidar3D::TData &ldata){

    RoboCompLidar3D::TData avanza;
    avanza.points.assign(ldata.points.begin() + ldata.points.size()/3, (ldata.points.end() - ldata.points.size()/3)-1);

    RoboCompLidar3D::TData lateralder;
    lateralder.points.assign(ldata.points.end() - ldata.points.size()/3, ldata.points.end() - 150);
    RoboCompLidar3D::TData lateralizq;
    lateralizq.points.assign(ldata.points.begin() + 150, (ldata.points.begin() + ldata.points.size()/3)-1);

    std::ranges::sort(avanza.points, {}, &RoboCompLidar3D::TPoint::distance2d);
    std::ranges::sort(lateralder.points, {}, &RoboCompLidar3D::TPoint::distance2d);
    std::ranges::sort(lateralizq.points, {}, &RoboCompLidar3D::TPoint::distance2d);

    std::cout<<"Distance: "<<avanza.points.front().distance2d<<std::endl;

    float adv = 0.0;
    float rot = 0.0;
    State state = State::FOLLOW_WALL;


    if((lateralder.points.front().distance2d < DIST_PARED - DELTA))
    {
        state = State::FOLLOW_WALL;
        adv = MAX_SPEED;
        rot = -0.2;
        auto tuple = std::make_tuple(state, adv, rot);
        return tuple;
    }

    if((avanza.points.front().distance2d < DIST_COL))
    {
        state = State::TURN;
        adv = 100.0;

        if (lateralder.points.front().distance2d > lateralizq.points.front().distance2d)
            rot = -1.0;
        else
            rot = 1.0;
        auto tuple = std::make_tuple(state, adv, rot);
        return tuple;
    }

    else if(lateralder.points.front().distance2d >= DIST_PARED + DELTA)
    {
        state = State::FOLLOW_WALL;
        adv = MAX_SPEED;
        rot = 0.2;
        auto tuple = std::make_tuple(state, adv, rot);
        return tuple;
    }

    else
    {
        auto tuple = std ::make_tuple(state, 0, 0);
        return tuple;
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

