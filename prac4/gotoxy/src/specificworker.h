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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <eigen3/Eigen/Eigen>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);



public slots:
	void compute();
	int startup_check();
    void new_target_slot(QPointF point);
	void initialize(int period);
    void draw_laser(const RoboCompLaser :: TLaserData &ldata);
private:
	std::shared_ptr < InnerModel > innerModel;
	bool startup_check_flag;

    struct Target{
        QPointF dest;
        bool activo;
    };
    Target target;
    const float MAX_ADV_SPEED=1000;
    AbstractGraphicViewer *viewer;
    const int ROBOT_LENGTH = 400;
    QGraphicsPolygonItem *robot_polygon;
    QGraphicsRectItem *laser_in_robot_polygon;
    QPointF last_point;

    Eigen::Vector2f world_to_robot(RoboCompGenericBase::TBaseState state, Eigen::Vector2f robot, Eigen::Vector2f mundo);

    float dist_to_target(Eigen::Vector2f pr);

    float rotation_speed(float beta);

    float dist_to_obstacle(Eigen::Vector2f robot, Eigen::Vector2f target, Eigen::Vector2f obstacle);
};

#endif
