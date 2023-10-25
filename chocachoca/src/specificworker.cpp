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
    {   auto ldata = lidar3d_proxy->getLidarData("bpearl", 0, 2 * M_PI, 1);
        qInfo() << ldata.points.size();
        const auto &points =ldata.points;

        RoboCompLidar3D::TPoints filtered_points;
        std::ranges::copy_if(ldata.points, std::back_inserter(filtered_points), [](auto &p) {return p.z < 2000;});
        draw_lidar(filtered_points, viewer);

        State state = std::get<0>(tuple);
        std::cout << "STATE: " << static_cast<std::underlying_type<State>::type>(state) << std::endl;

        // State Machine
        switch (state)
        {
            default:
                std:: cout << "IDLE" << std::endl;
                tuple = idle(filtered_points);
                break;
            case State::STRAIGHT_LINE:
                std::cout << "STRAIGHT_LINE" << std::endl;
                tuple = straight_line(filtered_points);
                break;
            case State::TURN:
                std::cout << "TURN" << std::endl;
                tuple = turn(filtered_points);
                break;
            case State::FOLLOW_WALL:
                std::cout << "FOLLOW_WALL" << std::endl;
                tuple = follow_wall(filtered_points);
                break;
            case State::SPIRAL:
                std::cout << "SPIRAL" << std::endl;
                // tuple = spiral(ldata);
                break;
        }

        // int offset = filtered_points.size()/2-filtered_points.size()/3;
        //auto min_elem = std::min_element(points.begin()+offset, points.end()- offset, [](auto a, auto b) {return std::hypot(a.x, a.y)< std::hypot(b.x, b.y);});
    }
    catch (const Ice::Exception & e) { std::cout<<e.what() << std::endl; return;};

    //start the robot
    auto &[_, adv, rot] = tuple;
    try
    { omnirobot_proxy->setSpeedBase(3, adv, rot); }
    catch (const Ice::Exception & e) { std::cout<<e.what() << std::endl; tuple;}
}


std::tuple<SpecificWorker::State,float,float> SpecificWorker::idle(const RoboCompLidar3D::TPoints &filtered_points){
    auto tuple = std::make_tuple(State::STRAIGHT_LINE, MAX_SPEED, 0);
    return tuple;
}


std::tuple<SpecificWorker::State, float,float> SpecificWorker::straight_line(const RoboCompLidar3D::TPoints &filtered_points){

    int offset = filtered_points.size() / 3;
    auto min_elem = std::min_element(filtered_points.begin() + offset, (filtered_points.end() - offset)-1,
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);});
    float adv = 0.0;
    float rot = 0.0;

    std::cout<<"Distancia: "<<std::hypot(min_elem->x, min_elem->y)<<std::endl;
    State state = State::STRAIGHT_LINE;

    //Si llega a la pared gira
    if((std::hypot(min_elem->x, min_elem->y)< DIST_COL))
    {
        state = State::TURN;
        adv = 100.0;

        int offsetDer = filtered_points.size()/3;
        auto min_elemDer = std::min_element(filtered_points.end() - offsetDer, filtered_points.end() - 150,
                                         [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);});

        int offsetIzq = filtered_points.size()/3;
        auto min_elemIzq = std::min_element(filtered_points.begin() +150, (filtered_points.begin()+ offsetIzq) - 1,
                                            [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);});
        if(std::hypot(min_elemDer->x, min_elem->y)>std::hypot(min_elemIzq->x, min_elem->y))
            rot = -1.0;
        else
            rot = 1.0;

        return std::make_tuple(state, adv, rot);
    }
    state=State::STRAIGHT_LINE;
    adv=MAX_SPEED;
    rot=0.0;

    return std::make_tuple(state,adv,rot);
}


std::tuple<SpecificWorker::State, float,float> SpecificWorker::turn(const RoboCompLidar3D::TPoints &filtered_points)
{
    int offset = filtered_points.size() / 2 - filtered_points.size() / 5;
    auto min_elem = std::min_element(filtered_points.begin() + offset, filtered_points.end() - offset,
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);});
    std::cout<<"Distance: "<<std::hypot(min_elem->x, min_elem->y)<<std::endl;
    State state = State::TURN;

    if((std::hypot(min_elem->x, min_elem->y) > DIST_COL))
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

std::tuple<SpecificWorker::State, float, float> SpecificWorker::follow_wall(const RoboCompLidar3D::TPoints &filtered_points){

    int offset = filtered_points.size() / 2 - filtered_points.size() / 5;
    auto min_elem = std::min_element(filtered_points.begin() + offset, filtered_points.end() - offset,
                                     [](auto a, auto b) { return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);});

    std::cout<<"Distance: "<<std::hypot(min_elem->x, min_elem->y)<<std::endl;

    float adv = 0.0;
    float rot = 0.0;
    State state = State::FOLLOW_WALL;


    if((std::hypot(min_elem->x, min_elem->y) < DIST_PARED - DELTA))
    {
        state = State::FOLLOW_WALL;
        adv = MAX_SPEED;
        rot = -0.2;
        auto tuple = std::make_tuple(state, adv, rot);
        return tuple;
    }

    if((std::hypot(min_elem->x, min_elem->y) < DIST_COL))
    {
        state = State::TURN;
        adv = 100.0;
        rot = 1.0;
        auto tuple = std::make_tuple(state, adv, rot);
        return tuple;
    }

    else if(std::hypot(min_elem->x, min_elem->y) >= DIST_PARED + DELTA)
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

