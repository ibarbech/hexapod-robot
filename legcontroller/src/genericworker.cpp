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
	avanzar->addTransition(this, SIGNAL(avanzartorecibe_ofset()), recibe_ofset);
	avanzar->addTransition(this, SIGNAL(avanzartoerror_timeout()), error_timeout);
	avanzar->addTransition(this, SIGNAL(avanzartoerror_imu()), error_imu);
	recibe_ofset->addTransition(this, SIGNAL(recibe_ofsettoavanzar()), avanzar);
	error_timeout->addTransition(this, SIGNAL(error_timeouttorecibe_ofset()), recibe_ofset);
	error_imu->addTransition(this, SIGNAL(error_imutorecibe_ofset()), recibe_ofset);
	leer_imu->addTransition(this, SIGNAL(leer_imutoleer_imu()), leer_imu);
	leer_sensores->addTransition(this, SIGNAL(leer_sensorestoleer_sensores()), leer_sensores);
	calcular_obj->addTransition(this, SIGNAL(calcular_objtomoverse()), moverse);
	moverse->addTransition(this, SIGNAL(moversetomoverse()), moverse);
	moverse->addTransition(this, SIGNAL(moversetocalcular_subobj()), calcular_subobj);
	moverse->addTransition(this, SIGNAL(moversetoexit()), exit);
	calcular_subobj->addTransition(this, SIGNAL(calcular_subobjtomoverse()), moverse);

	hexapod.addState(avanzar);
	hexapod.addState(error_imu);
	hexapod.addState(error_timeout);
	hexapod.addState(recibe_ofset);
	hexapod.addState(n);

	hexapod.setInitialState(recibe_ofset);
	avanzar_principal->setInitialState(calcular_obj);

	QObject::connect(avanzar, SIGNAL(entered()), this, SLOT(fun_avanzar()));
	QObject::connect(error_imu, SIGNAL(entered()), this, SLOT(fun_error_imu()));
	QObject::connect(error_timeout, SIGNAL(entered()), this, SLOT(fun_error_timeout()));
	QObject::connect(recibe_ofset, SIGNAL(entered()), this, SLOT(fun_recibe_ofset()));
	QObject::connect(n, SIGNAL(entered()), this, SLOT(fun_n()));
	QObject::connect(leer_imu, SIGNAL(entered()), this, SLOT(fun_leer_imu()));
	QObject::connect(leer_sensores, SIGNAL(entered()), this, SLOT(fun_leer_sensores()));
	QObject::connect(avanzar_principal, SIGNAL(entered()), this, SLOT(fun_avanzar_principal()));
	QObject::connect(calcular_obj, SIGNAL(entered()), this, SLOT(fun_calcular_obj()));
	QObject::connect(exit, SIGNAL(entered()), this, SLOT(fun_exit()));
	QObject::connect(calcular_subobj, SIGNAL(entered()), this, SLOT(fun_calcular_subobj()));
	QObject::connect(moverse, SIGNAL(entered()), this, SLOT(fun_moverse()));

//------------------
	footpreassuresensor_proxy = (*(FootPreassureSensorPrx*)mprx["FootPreassureSensorProxy"]);
	jointmotor_proxy = (*(JointMotorPrx*)mprx["JointMotorProxy"]);
	imu_proxy = (*(IMUPrx*)mprx["IMUProxy"]);


	mutex = new QMutex(QMutex::Recursive);

	Period = BASIC_PERIOD;
	connect(&timer, SIGNAL(timeout()), this, SLOT(compute()));
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

