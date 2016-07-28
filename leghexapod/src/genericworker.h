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

#include <qt4/QtCore/qstatemachine.h>
#include <qt4/QtCore/qstate.h>
#include <CommonBehavior.h>

#include <JointMotor.h>
#include <LegController.h>
#include <FootPreassureSensor.h>
#include <IMU.h>

#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

typedef map <string,::IceProxy::Ice::Object*> MapPrx;

using namespace std;

using namespace RoboCompLegController;
using namespace RoboCompIMU;
using namespace RoboCompJointMotor;
using namespace RoboCompFootPreassureSensor;




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
	

	IMUPrx imu_proxy;
	FootPreassureSensorPrx footpreassuresensor_proxy;
	JointMotorPrx jointmotor_proxy;

	virtual StateLeg getStateLeg() = 0;
	virtual void move(const float x, const float y, const string &state) = 0;
	virtual bool setListIKLeg(const ListPoseLeg &ps, const bool &simu) = 0;
	virtual bool setIKLeg(const PoseLeg &p, const bool &simu) = 0;
	virtual bool setIKBody(const PoseBody &p, const bool &simu) = 0;
	virtual bool setFKLeg(const AnglesLeg &al, const bool &simu) = 0;

protected:
//State Machine
	QStateMachine hexapod;

	QState *caminar = new QState(QState::ParallelStates);
	QState *error_imu = new QState();
	QState *error_timeout = new QState();
	QState *idel = new QState();
	QState *leer_imu = new QState(caminar);
	QState *leer_sensores = new QState(caminar);
	QState *avanzar = new QState(caminar);
	QState *calcular_subobj = new QState(avanzar);
	QState *moverse = new QState(avanzar);
	QState *paso = new QState(moverse);
	QState *empujar = new QState(moverse);
	QState *comporbar_accion = new QState(moverse);

//-------------------------

	QTimer timer;
	int Period;

private:


public slots:
//Slots funtion State Machine
	virtual void fun_caminar() = 0;
	virtual void fun_error_imu() = 0;
	virtual void fun_error_timeout() = 0;
	virtual void fun_idel() = 0;
	virtual void fun_leer_imu() = 0;
	virtual void fun_leer_sensores() = 0;
	virtual void fun_avanzar() = 0;
	virtual void fun_calcular_subobj() = 0;
	virtual void fun_moverse() = 0;
	virtual void fun_paso() = 0;
	virtual void fun_empujar() = 0;
	virtual void fun_comporbar_accion() = 0;

//-------------------------
signals:
	void kill();
//Signals for State Machine
	void caminartoidel();
	void caminartoerror_timeout();
	void caminartoerror_imu();
	void ideltoidel();
	void ideltocaminar();
	void error_timeouttoidel();
	void error_imutoerror_imu();
	void error_imutoidel();
	void leer_imutoleer_imu();
	void leer_sensorestoleer_sensores();
	void moversetocalcular_subobj();
	void calcular_subobjtomoverse();
	void comporbar_acciontopaso();
	void comporbar_acciontoempujar();
	void pasotopaso();
	void empujartoempujar();

//-------------------------
};

#endif