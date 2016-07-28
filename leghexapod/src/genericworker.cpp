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
	caminar->addTransition(this, SIGNAL(caminartoidel()), idel);
	caminar->addTransition(this, SIGNAL(caminartoerror_timeout()), error_timeout);
	caminar->addTransition(this, SIGNAL(caminartoerror_imu()), error_imu);
	idel->addTransition(this, SIGNAL(ideltoidel()), idel);
	idel->addTransition(this, SIGNAL(ideltocaminar()), caminar);
	error_timeout->addTransition(this, SIGNAL(error_timeouttoidel()), idel);
	error_imu->addTransition(this, SIGNAL(error_imutoerror_imu()), error_imu);
	error_imu->addTransition(this, SIGNAL(error_imutoidel()), idel);
	leer_imu->addTransition(this, SIGNAL(leer_imutoleer_imu()), leer_imu);
	leer_sensores->addTransition(this, SIGNAL(leer_sensorestoleer_sensores()), leer_sensores);
	moverse->addTransition(this, SIGNAL(moversetocalcular_subobj()), calcular_subobj);
	calcular_subobj->addTransition(this, SIGNAL(calcular_subobjtomoverse()), moverse);
	comporbar_accion->addTransition(this, SIGNAL(comporbar_acciontopaso()), paso);
	comporbar_accion->addTransition(this, SIGNAL(comporbar_acciontoempujar()), empujar);
	paso->addTransition(this, SIGNAL(pasotopaso()), paso);
	empujar->addTransition(this, SIGNAL(empujartoempujar()), empujar);

	hexapod.addState(caminar);
	hexapod.addState(error_imu);
	hexapod.addState(error_timeout);
	hexapod.addState(idel);

	hexapod.setInitialState(idel);
	avanzar->setInitialState(moverse);
	moverse->setInitialState(comporbar_accion);

	QObject::connect(caminar, SIGNAL(entered()), this, SLOT(fun_caminar()));
	QObject::connect(error_imu, SIGNAL(entered()), this, SLOT(fun_error_imu()));
	QObject::connect(error_timeout, SIGNAL(entered()), this, SLOT(fun_error_timeout()));
	QObject::connect(idel, SIGNAL(entered()), this, SLOT(fun_idel()));
	QObject::connect(leer_imu, SIGNAL(entered()), this, SLOT(fun_leer_imu()));
	QObject::connect(leer_sensores, SIGNAL(entered()), this, SLOT(fun_leer_sensores()));
	QObject::connect(avanzar, SIGNAL(entered()), this, SLOT(fun_avanzar()));
	QObject::connect(moverse, SIGNAL(entered()), this, SLOT(fun_moverse()));
	QObject::connect(calcular_subobj, SIGNAL(entered()), this, SLOT(fun_calcular_subobj()));
	QObject::connect(comporbar_accion, SIGNAL(entered()), this, SLOT(fun_comporbar_accion()));
	QObject::connect(paso, SIGNAL(entered()), this, SLOT(fun_paso()));
	QObject::connect(empujar, SIGNAL(entered()), this, SLOT(fun_empujar()));

//------------------
	imu_proxy = (*(IMUPrx*)mprx["IMUProxy"]);
	jointmotor_proxy = (*(JointMotorPrx*)mprx["JointMotorProxy"]);
	footpreassuresensor_proxy = (*(FootPreassureSensorPrx*)mprx["FootPreassureSensorProxy"]);

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

