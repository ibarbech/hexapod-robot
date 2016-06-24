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
#include <LegController.h>
#include <JointMotor.h>
#include <FootPreassureSensor.h>



#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100

typedef map <string,::IceProxy::Ice::Object*> MapPrx;

using namespace std;

using namespace RoboCompLegController;
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
	

	FootPreassureSensorPrx footpreassuresensor_proxy;
	JointMotorPrx jointmotor_proxy;

	virtual StateLeg getStateLeg() = 0;
	virtual bool move(const float x, const float y) = 0;
	virtual bool setListIKLeg(const ListPoseLeg &ps, const bool &simu) = 0;
	virtual bool setIKLeg(const PoseLeg &p, const bool &simu) = 0;
	virtual bool setIKBody(const PoseBody &p, const bool &simu) = 0;
	virtual bool setFKLeg(const AnglesLeg &al, const bool &simu) = 0;


protected:
//State Machine
	QStateMachine hexapod;

	QState *avanzar = new QState(QState::ParallelStates);
	QState *error_imu = new QState();
	QState *error_timeout = new QState();
	QState *recibe_ofset = new QState();
	QState *leer_imu = new QState(avanzar);
	QState *leer_sensores = new QState(avanzar);
	QState *avanzar_principal = new QState(avanzar);
	QState *calcular_subobj = new QState(avanzar_principal);
	QState *moverse = new QState(avanzar_principal);
	QState *calcular_obj = new QState(avanzar_principal);
	QFinalState *exit = new QFinalState(avanzar_principal);

//-------------------------

	QTimer timer;
	int Period;

public slots:
	virtual void compute() = 0;
//Slots funtion State Machine
	virtual void fun_avanzar() = 0;
	virtual void fun_error_imu() = 0;
	virtual void fun_error_timeout() = 0;
	virtual void fun_recibe_ofset() = 0;
	virtual void fun_leer_imu() = 0;
	virtual void fun_leer_sensores() = 0;
	virtual void fun_avanzar_principal() = 0;
	virtual void fun_calcular_subobj() = 0;
	virtual void fun_moverse() = 0;
	virtual void fun_calcular_obj() = 0;
	virtual void fun_exit() = 0;

//-------------------------
signals:
	void kill();
//Signals for State Machine
	void avanzartorecibe_ofset();
	void avanzartoerror_timeout();
	void avanzartoerror_imu();
	void recibe_ofsettoavanzar();
	void error_timeouttorecibe_ofset();
	void error_imutorecibe_ofset();
	void leer_imutoleer_imu();
	void leer_sensorestoleer_sensores();
	void calcular_objtomoverse();
	void moversetomoverse();
	void moversetocalcular_subobj();
	void moversetoexit();
	void calcular_subobjtomoverse();

//-------------------------
};

#endif
