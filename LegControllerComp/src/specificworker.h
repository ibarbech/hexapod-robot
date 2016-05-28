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

// #include <kdl/chain.hpp>
// #include <kdl/chainfksolver.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/frames_io.hpp>
// #include <kdl/chainiksolvervel_pinv.hpp>
// #include <kdl/chainiksolverpos_nr.hpp>
// #include <kdl/frames.hpp>
// #include <stdio.h>
// #include <iostream>

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <qt4/QtCore/QMap>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	bool setListIKLeg(const ListPoseLeg &ps, const bool &simu);
	StateLeg getStateLeg();
	bool setIKLeg(const PoseLeg &p, const bool &simu);
	bool setIKBody(const PoseBody &p, const bool &simu);
	bool setFKLeg(const AnglesLeg &al, const bool &simu);

public slots:
	void compute(); 	

private:
	InnerModel *inner;
	string innerpath;
	QStringList motores;
	QString foot,floor,base,nameLeg;
	double  coxa, femur, tibia;
	QVec pos_foot;
	int signleg;
	RoboCompJointMotor::MotorStateMap statemap;
	QMap<string,RoboCompJointMotor::MotorParams> motorsparams;
	
	
//-----------------------Funciones------------------
	void moverangles(QVec angles,double vel);
	QVec movFoottoPoint(QVec p, bool &exito);
};

#endif

