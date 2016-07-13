/*
 *    Copyright (C) 2016 by YOUR NAME HERE
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

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	StateLeg getStateLeg();
	void move(const float x, const float y);
	bool setListIKLeg(const ListPoseLeg &ps, const bool &simu);
	bool setIKLeg(const PoseLeg &p, const bool &simu);
	bool setIKBody(const PoseBody &p, const bool &simu);
	bool setFKLeg(const AnglesLeg &al, const bool &simu);

public slots:

private:
	InnerModel *inner;
	string innerpath;
	QStringList motores;
	QString foot,floor,base,nameLeg;
	double  coxa, femur, tibia, rPitch, rRoll;
	int signleg;
	QVec pos_foot, pos_center;
	bool idel;
	RoboCompJointMotor::MotorStateMap statemap;
	QMap<string,RoboCompJointMotor::MotorParams> motorsparams;
	
	void moverangles(QVec angles,double vel);
	QVec movFoottoPoint(QVec p, bool &exito);
	void stabilize();
	
	QVec bezier3(QVec p0, QVec p1, QVec p2, float t);
	QVec bezier2(QVec p0, QVec p2, float t);
	double mapear(double x, double in_min, double in_max, double out_min, double out_max);
	void updateinner();

private slots:
//Specification slot funtions State Machine
	void fun_paso();
	void fun_empujar();

//--------------------
	
};

#endif

