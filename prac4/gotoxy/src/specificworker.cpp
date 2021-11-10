/*
 *    Copyright (C) 2021 by YOUR NAME HERE
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
SpecificWorker::SpecificWorker(MapPrx& mprx, bool startup_check) : GenericWorker(mprx)
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
    QRectF dimensions(-5000,-2500,10000,5000);
    viewer = new AbstractGraphicViewer(this,dimensions);
    this->resize(900,450);
    robot_polygon = viewer->add_robot(ROBOT_LENGTH);
    laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
    laser_in_robot_polygon->setPos(0, 190);     // move this to abstract
    try
    {
        RoboCompGenericBase::TBaseState bState;
        differentialrobot_proxy->getBaseState(bState);
        last_point = QPointF(bState.x, bState.z);
    }
    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}
    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();

	}
	else
	{
		timer.start(Period);
	}
    robotState=1;
}

void SpecificWorker::compute()
{
    float threshold = 500; // millimeters
    int frente=0;
    RoboCompGenericBase::TBaseState bState;
    RoboCompLaser::TLaserData ldata;
    try
    {
        ldata = laser_proxy->getLaserData();
        frente=ldata.size()/2;
        draw_laser(ldata);
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }

    try
    {
        differentialrobot_proxy->getBaseState(bState);
        robot_polygon->setRotation(bState.alpha*180/M_PI);
        robot_polygon->setPos(bState.x, bState.z);
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
    try {
        switch(robotState) {
            case 1://IDLE. Waiting for target, when target.active = true -> move to FORWARD
                printf("IDLE............");
                if(target.activo){
                    robotState=2;
                }
            break;
            case 2://FORWARD. Straight forward movement until obstacle too close or we arrive at target. ->move to TURN OR IDLE
            {
                printf("FORWARD............");
                Eigen::Vector2f robot_eigen(bState.x, bState.z);
                Eigen::Vector2f target_eigen(target.dest.x(), target.dest.y());
                Eigen::Vector2f pr = world_to_robot(bState, robot_eigen, target_eigen);//position of the robot (pr)

                float beta = atan2(pr.x(), pr.y());
                float adv = MAX_ADV_SPEED * dist_to_target(pr) * rotation_speed(beta);
                printf ("La velocidades son: %f y %f",adv, beta);
                differentialrobot_proxy->setSpeedBase(adv, beta);

                if (ldata[frente].dist > 0 && ldata[frente].dist < threshold) {
                    differentialrobot_proxy->setSpeedBase(0, 0);
                    robotState = 3;
                }
                if (pr.norm() < 200)//si el robot ha llegado al punto marcado
                {
                    target.activo = false;//ponemos el target a false (desactivamos el punto marcado)
                    robotState = 1;
                    differentialrobot_proxy->setSpeedBase(0, 0);//detenemos el robot
                }
            }
            break;
            case 3://TURN. We turn until front is free to advance, -> move to BORDER
                differentialrobot_proxy->setSpeedBase(0,0.5);
                if(ldata[165].dist>1500 && ldata[45].dist<45){
                    differentialrobot_proxy->setSpeedBase(0,0);
                    robotState=4;
                }
            break;
            case 4://BORDER. Follow the obstacle siluette until target is in sight or line(Ax+By*C) is in sight. -> move to FORWARD
              //  float obj= dist_to_obstacle(robot_eigen,target_eigen,robot_eigen);

            break;
            }
            //differentialrobot_proxy->setSpeedBase(adv, beta);
        }
    catch (const Ice::Exception &ex) {
        std::cout << ex << std::endl;
    }
}

void SpecificWorker::new_target_slot(QPointF point) {
    qInfo()<<point;
    target.dest=point;
    target.activo=true;
}

void SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata) // robot coordinates
{
    static QGraphicsItem *laser_polygon = nullptr;
    // code to delete any existing laser graphic element
    if(laser_polygon!= nullptr){
        viewer->scene.removeItem(laser_polygon);
    }
    QPolygonF poly;
    // code to fill poly with the laser polar coordinates (angle, dist) transformed to cartesian coordinates (x,y), all in the robot's  // reference system
    poly<<QPointF(0,0);
    for (auto &p : ldata){
        float x = p.dist * sin(p.angle);
        float y = p.dist * cos(p.angle);
        poly<<QPointF(x,y);
    }

    QColor color("LightGreen");
    color.setAlpha(40);
    laser_polygon = viewer->scene.addPolygon(laser_in_robot_polygon->mapToScene(poly), QPen(QColor("DarkGreen"), 30),
                                             QBrush(color));
    laser_polygon->setZValue(3);
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

Eigen::Vector2f SpecificWorker::world_to_robot(RoboCompGenericBase::TBaseState state, Eigen::Vector2f robot, Eigen::Vector2f mundo) {
    Eigen::Matrix2f posicion;
    posicion <<cos(state.alpha),sin(state.alpha),-sin(state.alpha),cos(state.alpha);
    return posicion *(mundo-robot);//crear matriz con eigen 2f . bstate.angle
}

float SpecificWorker::dist_to_target(Eigen::Vector2f pr) {
    if(pr.norm()>1000)
        return 1;
    else return pr.norm()/1000;
}

float SpecificWorker::rotation_speed(float beta) {
    float brake;
    float l;
    l = pow(0.5,2)/log10f(0.1);
    brake = exp(pow(-beta,2)/l);
    return brake;
}

float SpecificWorker::dist_to_obstacle(Eigen::Vector2f robot, Eigen::Vector2f target, Eigen::Vector2f obstacle) {
    float A,B,C,d;
    A=robot.y()-target.y();
    B=target.x()-robot.x();
    C=((robot.x()-target.x())*robot.y())+((target.y()-robot.y())*robot.x());
    d=fabs(A*obstacle.x()+B*obstacle.y()+C)/sqrt(pow(A,2)+pow(B,2));
    //d=abs(Ax+By+c)/sqrt(A²+B²)
    return d;
}
/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

