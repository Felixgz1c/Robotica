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
    gridmap.initialize(dimensions, ROBOT_LENGTH, &viewer->scene);
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
    QRect grid_dimensions(-5000, -2500, 10000, 5000);
    grid.initialize(grid_dimensions, 200, scene, false, "", 1);
}

void SpecificWorker::compute()
{
    try {
        RoboCompGenericBase::TBaseState bState;
        RoboCompLaser::TLaserData ldata;
        ldata = laser_proxy->getLaserData();
        draw_laser(ldata);
        differentialrobot_proxy->getBaseState(bState);
        robot_polygon->setRotation(bState.alpha*180/M_PI);
        robot_polygon->setPos(bState.x, bState.z);

        switch(robotState) {
            case 1://IDLE. Waiting for target, when target.active = true -> move to FORWARD
                printf("IDLE............\n");
                if (target.activo) {
                    A=bState.z-target.dest.y();
                    B=target.dest.x()-bState.x;
                    C=((bState.x-target.dest.x())*bState.z)+((target.dest.y()-bState.z)*bState.x);
                    robotState = 2;//FORWARD
                }
                break;
            case 2://FORWARD. Straight forward movement until obstacle too close or we arrive at target. ->move to TURN OR IDLE
            {
                printf("FORWARD............\n");
                Eigen::Vector2f target_eigen(target.dest.x(), target.dest.y());
                forward(target_eigen, ldata, bState);
            }
                break;
            case 3://TURN. We turn until front is free to advance, -> move to BORDER
                printf("TURN............\n");
                turn(ldata);
                break;
            case 4://BORDER. Follow the obstacle siluette until target is in sight or line(Ax+By*C) is in sight. -> move to FORWARD
            {
                printf("BORDER............\n");
                Eigen::Vector2f robot_eigen(bState.x, bState.z);
                Eigen::Vector2f target_eigen(target.dest.x(), target.dest.y());
                border(ldata, A, B, C, bState, target_eigen);
            }
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

Eigen::Vector2f SpecificWorker::world_to_robot(RoboCompGenericBase::TBaseState bState,Eigen::Vector2f mundo) {
    Eigen::Vector2f robpos(bState.x, bState.z);
    Eigen::Matrix2f posicion;
    posicion <<cos(bState.alpha),sin(bState.alpha),-sin(bState.alpha),cos(bState.alpha);
    return posicion *(mundo-robpos);//crear matriz con eigen 2f . bstate.angle
}

Eigen::Vector2f SpecificWorker::robot_to_world(RoboCompGenericBase::TBaseState bState, Eigen::Vector2f mundo){
    Eigen::Vector2f robpos(bState.x,bState.z);
    Eigen::Matrix2f posicion;
    posicion <<cos (bState.alpha),-sin(bState.alpha) , sin(bState.alpha), cos(bState.alpha);
    return posicion.transpose().inverse()* mundo+robpos;
}

float SpecificWorker::dist_to_target(Eigen::Vector2f pr) {
    if(pr.norm()>1200)
        return 1;
    else return pr.norm()/1200;
}

float SpecificWorker::rotation_speed(float beta) {
    float brake;
    float l;
    l = pow(0.5,2)/log10f(0.1);
    brake = exp(pow(-beta,2)/l);
    return brake;
}

float SpecificWorker::dist_to_line(float &A, float &B, float &C, RoboCompGenericBase::TBaseState &bState) {

    return fabs(A*bState.x+B*bState.z+C)/sqrt(pow(A,2)+pow(B,2));
}

void SpecificWorker::turn(RoboCompLaser::TLaserData &laser){
    printf("distancia izqda: %f\n", banda_laser(laser,160,180));
    differentialrobot_proxy->setSpeedBase(0,0.5);
    if(banda_laser(laser,160,200)>1500){
        robotState=4;//BORDER
        differentialrobot_proxy->setSpeedBase(0,0);
    }
}

void SpecificWorker::forward(Eigen::Vector2f target_eigen,RoboCompLaser::TLaserData &laser,RoboCompGenericBase::TBaseState bState){
    Eigen::Vector2f pr = world_to_robot(bState, target_eigen);//position of the robot (pr)

    float beta = atan2(pr.x(), pr.y());
    float adv = MAX_ADV_SPEED * dist_to_target(pr) * rotation_speed(beta);
    printf ("La velocidades son: %f y %f\n",adv, beta);
    differentialrobot_proxy->setSpeedBase(adv, beta);

    //banda_laserlaser,160,200) > 0 &&
    if (banda_laser(laser,160,200) < 600) {
        differentialrobot_proxy->setSpeedBase(0, 0);
        robotState = 3;//TURN
    }
    if (pr.norm() < 200)//si el robot ha llegado al punto marcado
    {
        target.activo = false;//ponemos el target a false (desactivamos el punto marcado)
        robotState = 1;//IDLE
        differentialrobot_proxy->setSpeedBase(0, 0);//detenemos el robot
    }
}

void SpecificWorker::border(RoboCompLaser::TLaserData &laser, float &A, float &B, float &C, RoboCompGenericBase::TBaseState &bState, Eigen::Vector2f &mundo){

    if(dist_to_line(A,B,C,bState)<50){
        differentialrobot_proxy->setSpeedBase(0, 0);
        robotState=2;//FORWARD
    }

    else {
        if (banda_laser(laser,30,120) > 500 )
            differentialrobot_proxy->setSpeedBase(300, -0.4);
        else if (banda_laser(laser,30,120) < 500)
            differentialrobot_proxy->setSpeedBase(300, 0.4);
        else
            differentialrobot_proxy->setSpeedBase(300, 0);
    }
}

void SpecificWorker::check_free_path_to_target( const RoboCompLaser::TLaserData &ldata, RoboCompGenericBase::TBaseState &bState, Eigen::Vector2f &mundo) {
    // lambda to convert from Eigen to QPointF
    auto toQPointF = [](const Eigen::Vector2f &p) { return QPointF(p.x(), p.y()); };

    // create polyggon
    QPolygonF pol;
    pol << QPointF(0, 0);
    for (const auto &l: ldata)
        pol << QPointF(l.dist * sin(l.angle), l.dist * cos(l.angle));

    // create tube lines
    auto goal_r = world_to_robot(bState, mundo);
    Eigen::Vector2f robot(0.0, 0.0);
    // number of parts the target vector is divided into
    float parts = (goal_r).norm() / (ROBOT_LENGTH / 4);
    Eigen::Vector2f rside(220, 200);
    Eigen::Vector2f lside(-220, 200);
    if (parts < 1) return;

    QPointF p, q, r;
    for (auto l: iter::range(0.0, 1.0, 1.0 / parts)) {
        p = toQPointF(robot * (1 - l) + goal_r * l);
        q = toQPointF((robot + rside) * (1 - l) + (goal_r + rside) * l);
        r = toQPointF((robot + lside) * (1 - l) + (goal_r + lside) * l);
        if (not pol.containsPoint(p, Qt::OddEvenFill) or
            not pol.containsPoint(q, Qt::OddEvenFill) or
            not pol.containsPoint(r, Qt::OddEvenFill))
            break;
    }
    // draw
    QLineF line_center(toQPointF(robot_to_world(bState,robot)), toQPointF(robot_to_world(bState,Eigen::Vector2f(p.x(),p.y()))));
    QLineF line_right(toQPointF(robot_to_world(bState,robot+rside)), toQPointF(robot_to_world(bState,Eigen::Vector2f(q.x(),q.y()))));
    QLineF line_left(toQPointF(robot_to_world(bState,robot+lside)), toQPointF(robot_to_world(bState,Eigen::Vector2f(r.x(),r.y()))));
    static QGraphicsItem *graphics_line_center = nullptr;
    static QGraphicsItem *graphics_line_right = nullptr;
    static QGraphicsItem *graphics_line_left = nullptr;
    static QGraphicsItem *graphics_target = nullptr;
    if (graphics_line_center != nullptr)
        viewer->scene.removeItem(graphics_line_center);
    if (graphics_line_right != nullptr)
        viewer->scene.removeItem(graphics_line_right);
    if (graphics_line_left != nullptr)
        viewer->scene.removeItem(graphics_line_left);
    if (graphics_target != nullptr)
        viewer->scene.removeItem(graphics_target);
    graphics_line_center = viewer->scene.addLine(line_center, QPen(QColor("Blue"), 30));
    graphics_line_right = viewer->scene.addLine(line_right, QPen(QColor("Orange"), 30));
    graphics_line_left = viewer->scene.addLine(line_left, QPen(QColor("Magenta"), 30));
    graphics_target = viewer->scene.addEllipse(-100, -100, 200, 200, QPen(QColor("Blue")), QBrush(QColor("Blue")));
    graphics_target->setPos(mundo.x(), mundo.y());

}

float SpecificWorker::banda_laser(const RoboCompLaser::TLaserData &ldata, int min, int max)
{
    auto res = std::min_element(ldata.begin()+min, ldata.begin()+max, [](auto a, auto b){ return a.dist < b.dist;} );
    return (*res).dist;
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