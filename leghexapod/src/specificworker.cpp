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

#include "specificworker.h"
#define printer 1
#define VEL 50
#define ANTICIPACION 30
#define SENSOR_SENSIBILITI 100
#define ALT_CENTER -20	
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{	
		string name = PROGRAM_NAME;
		nameLeg=QString::fromStdString(params[name+".name"].value);
		base=QString::fromStdString(params[name+".base"].value);
		floor=QString::fromStdString(params[name+".floor"].value);
		innerpath=params[name+".InnerModel"].value;
		inner = new InnerModel(innerpath);
		
		motores<<QString::fromStdString(params[name+".m1"].value)<<QString::fromStdString(params[name+".m2"].value)<<QString::fromStdString(params[name+".m3"].value);
		foot=QString::fromStdString(params[name+".foot"].value);
		
		signleg=QString::fromStdString(params[name+".singleg"].value.data()).toInt();

		QVec aux=inner->transform(motores.at(1),motores.at(0));
		coxa=aux.norm2();
		
		aux=inner->transform(motores.at(2),motores.at(1));
		femur=aux.norm2();
		
		aux=inner->transform(foot,motores.at(2));
		tibia=aux.norm2();
		
		
	}
	catch(std::exception e)
	{
		qFatal("Error reading config params");
	}
	for(auto name:motores)
	{
		motorsparams[name.toStdString()]=jointmotor_proxy->getMotorParams(name.toStdString());
	}
	qDebug()<<"-----------------------------";
	qDebug()<<"    InnerModel ="<<QString::fromStdString(innerpath);
	qDebug()<<"    coxa   = "<<coxa;
	qDebug()<<"    femur  = "<<femur;
	qDebug()<<"    tibia  = "<<tibia;
	qDebug()<<"    signleg = "<<signleg;
	qDebug()<<"    foot = "<<foot;
	qDebug()<<"    base = "<<base;
	qDebug()<<"    floor = "<<floor;
	qDebug()<<"    m1 = "<<motores.at(0);
	qDebug()<<"    m2 = "<<motores.at(1);
	qDebug()<<"    m2 = "<<motores.at(2);
	qDebug()<<"-----------------------------";
	float aux;
	if(signleg==1)
	{
		aux=maxq3;
		maxq3=-minq3;
		minq3=-aux;
	}
	
	QVec posini = QVec::vec3(0,0.25,-1.1);
	moverangles(posini,50);
	obfin = QVec::vec3(0,0,0);
	sleep(1);
	updateinner();
	idel=true;
// 	hexapod.start();
	timer.start(10);
	connect(&tStabilize, SIGNAL(timeout()), this, SLOT(Act_stabilize()));
//  	tStabilize.start(200);
	return true;
}

void SpecificWorker::fun_caminar()
{
	//no implement
}

void SpecificWorker::fun_error_imu()
{
	if(printer!=0)
		qDebug()<<__FUNCTION__;
	updateinner();
	RoboCompIMU::Orientation o = imu_proxy->getOrientation();
	if(fabs(o.Pitch)>0.04||fabs(o.Roll)>0.04)
	{
		stabilize(pos_center);
		emit error_imutoerror_imu();
		return;
	}
	emit error_imutoidel();
}

void SpecificWorker::fun_error_timeout()
{

}

void SpecificWorker::fun_idel()
{
	qDebug()<<__FUNCTION__<<nameLeg;
	hexapod.blockSignals(false);
	updateinner();
	idel = true;
	i=INCREMENTO;
	if(obfin != QVec::vec3(0,0,0))
	{
		idel = false;
		emit ideltocaminar();
	}
	else
		hexapod.stop();
}

void SpecificWorker::fun_leer_imu()
{
// 	if(printer!=0)
// 		qDebug()<<__FUNCTION__;
// 	updateinner();
// 	RoboCompIMU::Orientation o = imu_proxy->getOrientation();
// 	if(fabs(o.Roll)>0.3||fabs(o.Pitch)>0.3)
// 	{
// // 		go_poscenter();
// 		emit caminartoerror_imu();
// 		return;
// 	}
// 	emit leer_imutoleer_imu();
}

void SpecificWorker::fun_leer_sensores()
{
	if(printer!=0)
		qDebug()<<__FUNCTION__;
	updateinner();
	if(footpreassuresensor_proxy->readSensor(nameLeg.toStdString()) > SENSOR_SENSIBILITI && i>0.6&& act == Paso)
	{
		qDebug()<<"---------------------------------emit caminartoidel"<<nameLeg;
		obfin = QVec::vec3(0,0,0);
		emit caminartoidel();
		hexapod.blockSignals(true);
	}
	else
		emit leer_sensorestoleer_sensores();
}

void SpecificWorker::fun_avanzar()
{
	//no implement
}

void SpecificWorker::fun_calcular_subobj()
{
	if(obfin != QVec::vec3(0,0,0))
	{
		if(printer!=0)
			qDebug()<<__FUNCTION__;
		i=INCREMENTO;
		if(obfin.y()==-10)
		{
			subobje = true;
			obfin = QVec::vec3(obfin.x(),-30,obfin.z());
		}
		else
		{
			subobje = false;
			int x = rand() % (((int)obfin.x()+10 + 1) - (int)obfin.x()-10) + (int)obfin.x()-10;
			int z = rand() % (((int)obfin.z()+10 + 1) - (int)obfin.z()-10) + (int)obfin.z()-10;
			center = QVec::vec3(0,0,0);
			obfin = QVec::vec3(x,-10,z);
		}
		emit calcular_subobjtomoverse();
	}
}

void SpecificWorker::fun_moverse()
{
	// no implement
}

void SpecificWorker::fun_paso()
{
	static int intentos = 0;
	if (intentos == 10 && i>INCREMENTO)
		i-=INCREMENTO;
	if (i<=1 && act == Paso && obfin != QVec::vec3(0,0,0))
	{
		static QVec tmp = QVec::vec3(0,0,0);
		if(printer!=0)
			qDebug()<<__FUNCTION__<<(pos_foot-tmp).norm2()<<i<< intentos;
		updateinner();
		
		if(i == INCREMENTO || (pos_foot-tmp).norm2()<ANTICIPACION || intentos == 10)
		{
			intentos=0;
			if (center!=QVec::vec3(0,0,0))
				tmp=bezier3(ini,center,fin,i);
			else
				tmp=bezier2(ini,fin,i);
			RoboCompLegController::PoseLeg p;
			p.x=tmp.x();
			p.y=tmp.y();
			p.z=tmp.z();
			p.ref=base.toStdString();
			p.vel = VEL;
			
			setIKLeg(p,false);
			
			i+=INCREMENTO;
			if(i>=0.95)
			{
// 				qDebug()<<"---------------------------------emit caminartoidel"<<nameLeg;
// 				emit caminartoidel();
				emit moversetocalcular_subobj();
				return;
			}
		}
		else
			intentos++;
		emit pasotopaso();
	}
}

void SpecificWorker::fun_empujar()
{
	static int intentos = 0;
	if (intentos == 10 && i>INCREMENTO)
		i-=INCREMENTO;
	if (i<=1 && act == Empujar)
	{
		static QVec tmp = QVec::vec3(0,0,0);
		if(printer!=0)
			qDebug()<<__FUNCTION__<<(pos_foot-tmp).norm2()<<i;
		updateinner();
		if(i==INCREMENTO || (pos_foot-tmp).norm2()<ANTICIPACION || intentos == 10)
		{
			intentos=0;
			tmp=bezier2(ini,fin,i);
			RoboCompLegController::PoseLeg p;
			p.x=tmp.x();
			p.y=tmp.y();
			p.z=tmp.z();
			p.ref=base.toStdString();
			p.vel = VEL;	
			setIKLeg(p,false);
			
			i+=2*INCREMENTO;
			if(i>1)
			{
				obfin = QVec::vec3(0,0,0);
				emit caminartoidel();
// 				idel = true;
// 				hexapod.stop();
				return;
			}
		}
		else
			intentos++;
		emit empujartoempujar();
	}
}

void SpecificWorker::fun_comporbar_accion()
{
	if(printer!=0)
		qDebug()<<__FUNCTION__;
	updateinner();
	if(obfin != QVec::vec3(0,0,0))
	{
		if(act == Paso)
		{
			ini = pos_foot;
			fin = pos_center + obfin;
			if(!subobje)
				center = QVec::vec3((fin.x()+ini.x())/2,ALT_CENTER,(fin.z()+ini.z())/2);
			else
				center = QVec::vec3(0,0,0);
			emit comporbar_acciontopaso();
		}
		else
		{
			ini = pos_foot;
			fin = pos_center - obfin;
			if(pos_foot.y()>-90)
			{
				fin = QVec::vec3(fin.x(),pos_foot.y(),fin.z());
			}
			emit comporbar_acciontoempujar();
		}
	}
	return;
}

StateLeg SpecificWorker::getStateLeg()
{
// 	if(printer!=0)
// 		qDebug()<<__FUNCTION__;
	StateLeg state;
	state.ismoving=false;
	RoboCompLegController::Statemotor aux[3];
	QVec aux2=QVec::vec3();
	int i=0;
	MotorState ms;
	foreach(QString m, motores)
	{
		try
		{
			ms=jointmotor_proxy->getMotorState(m.toStdString());
			if(ms.isMoving)
				state.ismoving=true;
			aux[i].pos=ms.pos;
			aux[i].name=m.toStdString();
		}
		catch(const Ice::Exception &ex)
		{
			std::cout << ex << std::endl;
			RoboCompLegController::HardwareFailedException e;
			e.what="Hardware Failed" +m.toStdString();
			//send e
		}
		i++;
	}
	state.q1=aux[0];
	state.q2=aux[1];
	state.q3=aux[2];
	aux2=inner->transform(base,foot);
	state.x=aux2.x();
	state.y=aux2.y();
	state.z=aux2.z();
	state.ref=base.toStdString();
	state.name=nameLeg.toStdString();
	state.idel= idel;
	return state;
}

void SpecificWorker::move(const float x, const float y, const string& state)
{
	if(printer!=0)
		qDebug()<<__FUNCTION__;
	idel=false;
	if(state=="paso")
		act=Paso;
	else
		act=Empujar;
	obfin = QVec::vec3(x,-10,y);
	hexapod.start();
}

bool SpecificWorker::setListIKLeg(const ListPoseLeg &ps, const bool &simu)
{
	bool exito=true;
	for(auto p:ps)
	{
		while(getStateLeg().ismoving){}
		exito=setIKLeg(p,simu);
		if (!exito)
			break;
	}
	return exito;
}

bool SpecificWorker::setIKLeg(const PoseLeg &p, const bool &simu)
{
	bool exito;
	try
	{
		QVec posfoot=inner->transform(motores.at(0),QVec::vec3(p.x,p.y,p.z),QString::fromStdString(p.ref));
		QVec angles=movFoottoPoint(posfoot, exito);
		if(exito&&!simu)
		{
			moverangles(angles, p.vel);
		}
		if(!exito)
		{
			RoboCompLegController::ImpossiblePositionException e;
			e.what="Impossible Position";
			qDebug()<<"Error";
			//send e
		}
		return exito;
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
		RoboCompLegController::HardwareFailedException e;
		e.what="Hardware Failed";
		//send e
		return exito;
	}
}

bool SpecificWorker::setIKBody(const PoseBody &p, const bool &simu)
{
	//inicio rotar el cuerpo
// 	InnerModel *inner= new InnerModel(innerpath);
	QVec pos=inner->transform(floor, QVec::vec3(p.px,p.py,p.pz), QString::fromStdString(p.ref));
	inner->updateRotationValues(base, p.rx, p.ry, p.rz,"");
	//fin rotar el cuerpo
	pos = inner->transform(base, pos, floor);
	PoseLeg pl;
	pl.ref = base.toStdString();
	pl.x = pos.x() + p.x;
	pl.y = pos.y() + p.y;
	pl.z = pos.z() + p.z;
	pl.vel=p.vel;
	return setIKLeg(pl,simu);
}

bool SpecificWorker::setFKLeg(const AnglesLeg &al, const bool &simu)
{
	bool exito=false;
	try
	{
		QVec angles=QVec::vec3(al.q1,al.q2,al.q3);
// 		double max=M_PI/2, min=- M_PI/2;
		if((minq1<al.q1&&al.q1<maxq1)&&(minq2<al.q2*signleg&&al.q2*signleg<maxq2)&&(minq3<al.q3*signleg&&al.q3*signleg<maxq3))
			exito=true;
		if(exito&&!simu)
			moverangles(angles, al.vel);
		return exito;
	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
		RoboCompLegController::HardwareFailedException e;
		e.what="Hardware Failed";
		//send e
		return exito;
	}
}

QVec SpecificWorker::movFoottoPoint(QVec p, bool &exito)
{
	QVec angles=QVec::zeros(3);

	double q1, q2, q3,x=p.x(), y=p.y(), z=p.z(),
		r=abs(sqrt(pow(x,2)+pow(z,2))-coxa),
		cosq3=(pow(r,2)+pow(y,2)-pow(tibia,2)-pow(femur,2))/(2*tibia*femur);
	if(cosq3>1)
		cosq3=1;
	else if(cosq3<-1)
		cosq3=-1;
	double senq3=-sqrt(1-pow(cosq3,2));
	if(senq3>1)
		senq3=1;
	else if(senq3<-1)
		senq3=-1;
	double L=sqrt(pow(y,2)+pow(r,2));
	if(L<tibia+femur /*&&( x>0 || z>0)*/)
	{
		q1=atan2(x,z);
		q3=atan2(senq3,cosq3);
		q2=atan2(y,r)-atan2((tibia*senq3),(femur+(tibia*cosq3)));
		q1+=statemap[motores.at(0).toStdString()].pos;
		q2 += 0.22113;
		q3 += 0.578305;
		exito = true;
//  		double max=M_PI/2+0.15, min=- M_PI/2-0.15;
		if((minq1<q1&&q1<maxq1)&&(minq2<q2*signleg&&q2*signleg<maxq2)&&(minq3<q3*signleg&&q3*signleg<maxq3)){
			exito=true;
		}
		else
		{
			qDebug()<<"Imposible llegar: angulos sobrepasados.";
			exito=false;
		}
	}
	else
	{
		qDebug()<<"Imposible llegar: punto demasiado lejano.";
		exito=false;
	}
	
	angles(0)=q1/*+motorsparams[motores.at(0).toStdString()].offset*/;
	angles(1)=q2/*+motorsparams[motores.at(1).toStdString()].offset*/;
	angles(2)=q3/*+motorsparams[motores.at(2).toStdString()].offset*/;
	return angles;
}

void SpecificWorker::moverangles(QVec angles,double vel)
{
	if(!isnan(angles(0))&&!isnan(angles(1))&&!isnan(angles(2)))
	{
		RoboCompJointMotor::MotorGoalPositionList mg;
		RoboCompJointMotor::MotorGoalPosition p;
		RoboCompJointMotor::MotorGoalVelocityList mv;
		RoboCompJointMotor::MotorGoalVelocity v;
		double 	q1=angles(0)/* *-1*/,
				q2=angles(1) *signleg,
				q3=angles(2) *signleg;
// 		qDebug()<<"Leg: "<<foot<<" Moviendo"<<"q1 = "<<q1<<"  q2 = "<<q2<<"  q3 = "<<q3;
		MotorState m=jointmotor_proxy->getMotorState(motores.at(0).toStdString());
		v.name = p.name = motores.at(0).toStdString();
		v.velocity = vel;
		p.maxSpeed=/*fabs(q1-m.pos)**/vel;
		p.position=q1;
		mg.push_back(p);
		mv.push_back(v);
		
		m=jointmotor_proxy->getMotorState(motores.at(1).toStdString());
		v.name = p.name=motores.at(1).toStdString();
		v.velocity = vel;
		p.maxSpeed=/*fabs(q2-m.pos)**/vel;
		p.position=q2;
		mg.push_back(p);
		mv.push_back(v);
		
		m=jointmotor_proxy->getMotorState(motores.at(2).toStdString());
		v.name = p.name=motores.at(2).toStdString();
		v.velocity = vel;
		p.position=q3;
		p.maxSpeed=/*fabs(q3-m.pos)**/vel;
		mg.push_back(p);
		mv.push_back(v);
		jointmotor_proxy->setSyncVelocity(mv);
		jointmotor_proxy->setSyncPosition(mg);
// 		jointmotor_proxy->setSyncPosition(mg);
		
	}
	else
		qDebug()<< "Posicion no alcanzada";

}

void SpecificWorker::stabilize(QVec pos)
{
	updateinner();
	QVec stabilize_pos = pos;
	RoboCompIMU::Orientation o = imu_proxy->getOrientation();
	RoboCompJointMotor::MotorStateMap ms;
	jointmotor_proxy->getAllMotorState(ms);
	for(RoboCompJointMotor::MotorStateMap::iterator it = ms.begin(); it != ms.end(); it++)
		if(it->second.isMoving)
			return;
	if(fabs(o.Pitch)>0.1||fabs(o.Roll)>0.1)
	{
		RoboCompLegController::PoseBody p;
		InnerModelTransform* t = inner->getTransform(base);
		p.ref = base.toStdString();
		p.x = 0;
		p.y = 0;
		p.z = 0;
		p.rx = o.Pitch + t->getRxValue();
		p.ry = 0;
		p.rz = o.Roll + t->getRzValue();
		p.vel = 150;
		p.px = stabilize_pos.x();
		p.pz = stabilize_pos.z();
		p.py = stabilize_pos.y();
		setIKBody(p,false);
	}
}

QVec SpecificWorker::bezier2(QVec p0, QVec p1, float t)
{
	QVec diff = p1 - p0;
	return p0 + (diff * t);
}

QVec SpecificWorker::bezier3(QVec p0, QVec p1, QVec p2, float t)
{
	QVec a=bezier2(p0,p1,t);
	QVec b=bezier2(p1,p2,t);
	return bezier2(a,b,t);
}

double SpecificWorker::mapear(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void SpecificWorker::updateinner()
{
	static int i = 0;
	try{
		foreach(QString m, motores)
		{
			statemap[m.toStdString()]=jointmotor_proxy->getMotorState(m.toStdString());//robot
			inner->updateJointValue(m,statemap[m.toStdString()].pos);
		}
	}
	catch(const Ice::Exception &ex)
	{
		  std::cout << ex << std::endl;
	}	
	pos_foot = inner->transform(base,foot);
	if (i==0)
	{
		i++;
		pos_center = pos_foot;
	}
}

void SpecificWorker::go_poscenter()
{
	RoboCompLegController::PoseLeg p;
	p.vel = 50;
	p.x = pos_center.x();
	p.y = pos_center.y();
	p.z = pos_center.z();
	p.ref = base.toStdString();
	setIKLeg(p,false);
}

void SpecificWorker::Act_stabilize()
{
//  if(printer!=0)
// 		qDebug()<<__FUNCTION__;
	updateinner();
	if(idel)
		stabilize(pos_foot);
}
