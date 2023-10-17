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
//	QLoggingCategory::setFilterRules("*.debug=false\n");
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
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }
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

void SpecificWorker::follow_wall(RoboCompLidar3D::TData w, float &ad, float &ret){

    ad = 1500;

    if (w.points.size() < keep_dist){
        ad=1200;
        ret = -0.1 * keep_dist /w.points.size();
    }


}

void SpecificWorker::compute()
{
    try
    {
        auto ldata = lidar3d_proxy->getLidarData("bpearl", 0, 360, 4);
        qInfo() << ldata.points.size();
        const auto &points =ldata.points;
        //if(points.empty()) return;

        //decltype(ldata.points) filtered_points;
        RoboCompLidar3D::TPoints filtered_points;
        std::ranges::copy_if(ldata.points, std::back_inserter(filtered_points), [](auto &p) {return p.z < 2000;});
        draw_lidar(filtered_points, viewer);

        //CONTROL
        //get minimum distance point
        int offset = filtered_points.size()/2-filtered_points.size()/3;
        auto min_elem = std::min_element(points.begin()+offset, points.end()- offset, [](auto a, auto b) {return std::hypot(a.x, a.y)< std::hypot(b.x, b.y);});

        const float MIN_DISTANCE = 1000;
        qInfo()<<std::hypot(min_elem->x, min_elem->y);
        if (std::hypot(min_elem->x, min_elem->y)<MIN_DISTANCE)
        {

            //STOP THE ROBOT & START RUNNING
            try
            {
                omnirobot_proxy->setSpeedBase(0, 0, 0.5);
            }
            catch (const Ice::Exception &e)
            {
                std::cout << "Error reading from camera" << e << std::endl;
            }
        }
        else
        {
            //start the robot
            try
            {
                omnirobot_proxy->setSpeedBase(1000/1000.f,0,0);
            }
            catch(const Ice::Exception &e)
            {
                std::cout<<"Error reading from camera"<<e<<std::endl;
            }
        }
    }
    catch(const Ice::Exception &e){ std::cout << "Error reading from camera"<<e<< std::endl;}

    switch (state)
    {
        case 0:
            // STRAIGHT_LINE
            adv = 1000;
            ret = 0;
            stuck = 0;
            break;
        case 1:
            // FOLLOW_WALL
            stuck = 0;
            follow_walls(right, adv, ret);
            break;

        case 2:
            // SPIRAL
            break;
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