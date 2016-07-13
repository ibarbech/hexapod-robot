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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include "config.h"
#include <QtGui>
#include <stdint.h>
#include <qlog/qlog.h>

#include <CommonBehavior.h>
#include <LegController.h>
#include <JointMotor.h>



#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

typedef map <string,::IceProxy::Ice::Object*> MapPrx;

using namespace std;

using namespace RoboCompLegController;
using namespace RoboCompJointMotor;




class GenericWorker : 
public QObject
{
Q_OBJECT
public:
	GenericWorker(MapPrx& mprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);
	
	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;
	

	JointMotorPrx jointmotor_proxy;

	virtual StateLeg getStateLeg() = 0;
	virtual void move(const float x, const float y) = 0;
	virtual bool setListIKLeg(const ListPoseLeg &ps, const bool &simu) = 0;
	virtual bool setIKLeg(const PoseLeg &p, const bool &simu) = 0;
	virtual bool setIKBody(const PoseBody &p, const bool &simu) = 0;
	virtual bool setFKLeg(const AnglesLeg &al, const bool &simu) = 0;


protected:

	QTimer timer;
	int Period;

public slots:
	virtual void compute() = 0;
signals:
	void kill();
};

#endif
