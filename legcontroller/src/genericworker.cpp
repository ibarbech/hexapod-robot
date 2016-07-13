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
#include "genericworker.h"
/**
* \brief Default constructor
*/
GenericWorker::GenericWorker(MapPrx& mprx) :
QObject()
{

//Initialization State machine
	paso->addTransition(this, SIGNAL(pasotopaso()), paso);
	paso->addTransition(this, SIGNAL(pasotoempujar()), empujar);
	empujar->addTransition(this, SIGNAL(empujartoempujar()), empujar);
	empujar->addTransition(this, SIGNAL(empujartopaso()), paso);

	hexapod.addState(empujar);
	hexapod.addState(paso);

	hexapod.setInitialState(paso);

	QObject::connect(empujar, SIGNAL(entered()), this, SLOT(fun_empujar()));
	QObject::connect(paso, SIGNAL(entered()), this, SLOT(fun_paso()));

//------------------
	footpreassuresensor_proxy = (*(FootPreassureSensorPrx*)mprx["FootPreassureSensorProxy"]);
	jointmotor_proxy = (*(JointMotorPrx*)mprx["JointMotorProxy"]);
	imu_proxy = (*(IMUPrx*)mprx["IMUProxy"]);


	mutex = new QMutex(QMutex::Recursive);

	Period = BASIC_PERIOD;
// 	timer.start(Period);
}

/**
* \brief Default destructor
*/
GenericWorker::~GenericWorker()
{

}
void GenericWorker::killYourSelf()
{
	rDebug("Killing myself");
	emit kill();
}
/**
* \brief Change compute period
* @param per Period in ms
*/
void GenericWorker::setPeriod(int p)
{
	rDebug("Period changed"+QString::number(p));
	Period = p;
	timer.start(Period);
}

