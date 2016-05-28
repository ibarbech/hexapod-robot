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
#include <qt4/Qt/qlocale.h>
#include <qt4/Qt/qdebug.h>
#define tbezier 0.2
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	modovalue=0;
	vel=2;
	proxies[0]=legcontroller1_proxy;
	proxies[1]=legcontroller2_proxy;
	proxies[2]=legcontroller3_proxy;
	proxies[3]=legcontroller4_proxy;
	proxies[4]=legcontroller5_proxy;
	proxies[5]=legcontroller6_proxy;
	
	stateCrawl.enqueue(0);
	stateCrawl.enqueue(1);
	stateCrawl.enqueue(2);
	stateCrawl.enqueue(3);
	stateCrawl.enqueue(4);
	stateCrawl.enqueue(5);
	
	stateQuadruped.enqueue(0);
	stateQuadruped.enqueue(5);
	stateQuadruped.enqueue(2);
	stateQuadruped.enqueue(3);
	stateQuadruped.enqueue(1);
	stateQuadruped.enqueue(4);
	
	lini=QVec::vec3(0,0,0);
	lfin=QVec::vec3(0,0,0);
	lmed=QVec::vec3(0,70,0);
	l1[0]=0;
	l1[1]=3;
	l1[2]=4;
	l2[0]=1;
	l2[1]=2;
	l2[2]=5;
	x=0;
	y=0;
	z=0;
	X=0;
	Y=0;
	Z=0;
	X_pre=0;
	Y_pre=0;
	Z_pre=0;
	angles.q1=0;
	angles.q2=0;
	angles.q3=0;
	//Innerviewer
	osgView = new OsgView(this->frame);
	show();
	clk.start(100);
	clkupdate.start(10);
// 	connect(Start, SIGNAL(clicked()), this, SLOT(iniciar()));
// 	connect(Stop, SIGNAL(clicked()), this, SLOT(parar()));
	connect(&clkupdate, SIGNAL(timeout()), this, SLOT(updateStates()));
	connect(&clk, SIGNAL(timeout()), this, SLOT(update()));
	connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(updateState(int)));
	connect(Uphexapod, SIGNAL(clicked()), this, SLOT(stateuphexapod()));
	connect(Updatepos, SIGNAL(clicked()), this, SLOT(updateposleg()));
	connect(Reset, SIGNAL(clicked()), this, SLOT(ResetSlider()));
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	string name = PROGRAM_NAME;
	base=QString::fromStdString(params[name+".floor"].value);
	base=QString::fromStdString(params[name+".base"].value);
	inner = new InnerModel(params[name+".InnerModel"].value);
	
	
	innerViewer = new InnerModelViewer(inner, "root", osgView->getRootGroup(), false);

	
	for(int i=1;i<=6;i++)
	{
		legs<<QString::fromStdString(params[name+".nameleg"+to_string(i)].value);
	}
	qDebug()<<"-----------------------legs----------------";
	qDebug()<<base;
	qDebug()<<legs;
	qDebug()<<"-----------------------legs----------------";
	
	for(int i=0;i<6;i++){
		statelegs[i] = proxies[i]->getStateLeg();
		legsp[i]=QVec::vec3(statelegs[i].x,statelegs[i].y,statelegs[i].z);
	}
	for(auto p:legsp)
		qDebug()<<p;
	
	timer.start(10);
	
	return true;
}

void SpecificWorker::compute()
{
	switch(modovalue)
	{
		case -1:
			uphexapod();
			break;
		case 0:
			LEGS->setVisible(false);
			break;
		case 1://Alternating tripod
				ocultarAngles();
				if(caminar3x3())
				{
					lini=QVec::vec3(-X,0,-Z);
					lfin=QVec::vec3(X,0,Z);
				}
			break;
		case 2://Cuadrupedo
				ocultarAngles();
				if(Quadruped())
				{
					lini=QVec::vec3(-X,0,-Z);
					lfin=QVec::vec3(X,0,Z);
				}
			break;
		case 3://Crawl
				ocultarAngles();
				if(Crawl())
				{
					lini=QVec::vec3(-X,0,-Z);
					lfin=QVec::vec3(X,0,Z);
				}
			break;
		case 4://rotate
				ocultarAngles();
				if(rotar())
					lrot=QVec::vec3(0,0,Y);
			break;
		case 5://FK legs
				ocultarPoint();
				if(angles_pre.q1!=angles.q1||angles_pre.q2!=angles.q2||angles_pre.q3!=angles.q3)
				{
					fkLegs();
					angles_pre.q1=angles.q1;
					angles_pre.q2=angles.q2;
					angles_pre.q3=angles.q3;
				}
			break;
		case 6://IK Legs
				ocultarAngles();
				if(X_pre!=X || Y_pre!=Y || Z_pre!=Z)
				{
					ikLegs();
				
					X_pre=X;
					Y_pre=Y;
					Z_pre=Z;
			}
			break;
		case 7://IK Body
			mostrarPointandAngles();
			if(X_pre!=X || Y_pre!=Y || Z_pre!=Z||angles_pre.q1!=angles.q1||angles_pre.q2!=angles.q2||angles_pre.q3!=angles.q3)
			{
				ikBody();
				X_pre=X;
				Y_pre=Y;
				Z_pre=Z;
				angles_pre.q1=angles.q1;
				angles_pre.q2=angles.q2;
				angles_pre.q3=angles.q3;
			}
			break;
		case 8://IKonlyoneleg
				ocultarAngles();
				if(X_pre!=X || Y_pre!=Y || Z_pre!=Z)
				{
					ikonlyoneleg();
					
					X_pre=X;
					Y_pre=Y;
					Z_pre=Z;
				}
			break;
		case 9://FKonlyoneleg
				ocultarPoint();
				if(angles_pre.q1!=angles.q1||angles_pre.q2!=angles.q2||angles_pre.q3!=angles.q3)
				{
					fkonlyoneleg();
					angles_pre.q1=angles.q1;
					angles_pre.q2=angles.q2;
					angles_pre.q3=angles.q3;
				}
			break;
			
	}
}

bool SpecificWorker::caminar3x3()
{
	static float i=0;
	bool ismoving=false;
	if(i==0)
	{
		for(int k=0;k<6;k++)
			if(proxies[k]->getStateLeg().ismoving)
			{
				ismoving=true;
				break;
			}
		if(!ismoving)
			for(int i=0;i<6;i++)
				pre_statelegs[i]=statelegs[i];
	}
	if(!ismoving)	
	{
		if(lini!=QVec::vec3(0, 0, 0)&&lfin!=QVec::vec3(0,0,0))
		{
			//patas por arco
			for(int s=0;s<3;s++)
			{
				RoboCompLegController::StateLeg st=pre_statelegs[l1[s]];
				QVec posini =QVec::vec3(st.x,legsp[l1[s]].y(),st.z);
				QVec ini = posini,fin = legsp[l1[s]]+lfin,med=legsp[l1[s]];
				QVec tmp=bezier3(ini,QVec::vec3(med.x(),0,med.z()),fin,i);
				RoboCompLegController::PoseLeg p;
				p.x=tmp.x();
				p.y=tmp.y();
				p.z=tmp.z();
				p.ref=base.toStdString();
				p.vel = vel;
				proxies[l1[s]]->setIKLeg(p,false);
				
			}
		// patas por tierra
			for(int s=0;s<3;s++)
			{
				RoboCompLegController::StateLeg st=pre_statelegs[l2[s]];
				QVec posini =QVec::vec3(st.x,legsp[l2[s]].y(),st.z);
				QVec ini = posini,fin = legsp[l2[s]]+lini;
				QVec tmp=bezier2(ini,fin,i);
				RoboCompLegController::PoseLeg p;
				p.x=tmp.x();
				p.y=tmp.y();
				p.z=tmp.z();
				p.ref=base.toStdString();
				p.vel = vel;
				proxies[l2[s]]->setIKLeg(p,false);
				
			}
			i+=tbezier;
			if (i>1)
			{
				int aux[]={l1[0],l1[1],l1[2]};
				l1[0]=l2[0];
				l1[1]=l2[1];
				l1[2]=l2[2];
				l2[0]=aux[0];
				l2[1]=aux[1];
				l2[2]=aux[2];
				i=0;
				return true;
			}
			return false;
		}
	}
	return true;
}

bool SpecificWorker::rotar()
{
	static bool estado=true;
	if(lrot!=QVec::vec3(0, 0, 0))
	{
		static float i=0;
		bool ismoving=false;
		for(int k=0;k<6;k++)
			if(proxies[k]->getStateLeg().ismoving)
			{
				ismoving=true;
				break;
			}
		
		if(!ismoving)
		{
			if(i==0)
				for(int i=0;i<6;i++)
					pre_statelegs[i]=statelegs[i];
			QVec ini,fin;
			for(int j=0;j<6;j++)
			{
				RoboCompLegController::StateLeg st=pre_statelegs[j];
				QVec posini =QVec::vec3(st.x,legsp[j].y(),st.z);
				if((j==2 || j==3))
				{
					ini = posini;
					if(estado)
						fin = legsp[j]+lrot;
					else
						fin = legsp[j]-lrot;
				}
				else
				{
					ini = posini;
					if(estado)
						fin = legsp[j]-lrot;
					else
						fin = legsp[j]+lrot;
				}
				QVec tmp;
				if(estado)
					if(j==0||j==3||j==4)
						tmp=bezier3(ini,bezier2(ini,fin,0.5)+lmed,fin,i);
					else
						tmp = bezier2(ini,fin,i);
				else
					if(j==0||j==3||j==4)
						tmp = bezier2(ini,fin,i);
					else
						tmp=bezier3(ini,bezier2(ini,fin,0.5)+lmed,fin,i);
				
				RoboCompLegController::PoseLeg p;
				p.x   = tmp.x();
				p.y   = tmp.y();
				p.z   = tmp.z();
				p.ref = base.toStdString();
				p.vel = 6;
				proxies[j]->setIKLeg(p,false);
			}
			i+=tbezier;
			if (i>1)
			{
				i=0;
				estado=!estado;
				return true;
			}
		}
		return false;
	}
	else
		return true;
}

void SpecificWorker::ikLegs()
{
	RoboCompLegController::PoseLeg pos[6];
	bool simufallida=true;
	for(int i=0;i<6;i++)
	{
		pos[i].ref=base.toStdString();
		pos[i].vel=vel;
		pos[i].x=legsp[i].x()+x;
		pos[i].y=legsp[i].y()+y;
		pos[i].z=legsp[i].z()+z;
		
	}
	for(int i=0;i<6;i++)
	{	
		RoboCompLegController::PoseLeg p= pos[i];
		if(!proxies[i]->setIKLeg(pos[i],true))
		{
			simufallida=false;
			break;
		}
	}
	if(simufallida)
		for(int i=0;i<6;i++)
			proxies[i]->setIKLeg(pos[i],false);
}

void SpecificWorker::fkLegs()
{
	angles.vel=vel;
	for(int i=0;i<6;i++)
		proxies[i]->setFKLeg(angles, false);
}

void SpecificWorker::ikBody()
{
	RoboCompLegController::PoseBody pb[6];
	bool simufallida =false;
	for(int i=0;i<6;i++)
	{
		pb[i].vel = vel;
		pb[i].ref = base.toStdString();
		pb[i].rx = angles.q1/1.5;
		pb[i].ry = angles.q2/1.5;
		pb[i].rz = angles.q3/1.5;
		pb[i].px=legsp[i].x();
		pb[i].py=legsp[i].y();
		pb[i].pz=legsp[i].z();
		pb[i].x = X;
		pb[i].y = Y;
		pb[i].z = Z;
	}
	
	for(int i=0;i<6;i++)
	{
		if(!proxies[i]->setIKBody(pb[i],true))
		{
			simufallida=true;
			break;
		}
	}
	if(!simufallida)
		for(int i=0;i<6;i++)
			proxies[i]->setIKBody(pb[i],false);
}

void SpecificWorker::fkonlyoneleg()
{
	if(proxies[LEGS->currentIndex()]->setFKLeg(angles,true))
		proxies[LEGS->currentIndex()]->setFKLeg(angles,false);
}

void SpecificWorker::ikonlyoneleg()
{
	int i= LEGS->currentIndex();
	RoboCompLegController::PoseLeg pos;
	pos.ref=base.toStdString();
	pos.vel=vel;
	pos.x=legsp[i].x()+x;
	pos.y=legsp[i].y()+y;
	pos.z=legsp[i].z()+z;
	if(proxies[i]->setIKLeg(pos,true))
		proxies[i]->setIKLeg(pos,false);
}

bool SpecificWorker::Crawl()
{
	static float i=0;
	bool ismoving=false;
	if(i==0)
	{
		for(int k=0;k<6;k++)
			if(proxies[k]->getStateLeg().ismoving)
			{
				ismoving=true;
				break;
			}
		if(!ismoving)
		{
			stateCrawl.enqueue(stateCrawl.dequeue());
			for(int i=0;i<6;i++)
				pre_statelegs[i]=statelegs[i];
		}
	}
	if(!ismoving)	
	{
		if(lini!=QVec::vec3(0, 0, 0)&&lfin!=QVec::vec3(0,0,0))
		{
			RoboCompLegController::PoseLeg p;
			RoboCompLegController::StateLeg st;
			QVec ini,fin,tmp,med;
			for(int j=0;j<6;j++)
			{
				int aux=stateCrawl.dequeue();
				st=pre_statelegs[aux];
				if(j==0)
				{
					ini =QVec::vec3(st.x,legsp[aux].y(),st.z),
					fin = legsp[aux]+lfin,med=legsp[aux],
					tmp=bezier3(ini,QVec::vec3(med.x(),0,med.z()),fin,i);
				}
				else
				{
					ini = QVec::vec3(st.x,legsp[aux].y(),st.z),
					fin = legsp[aux]-lfin-(((lini-lfin)/5)*j),
					tmp = bezier2(ini,fin,i);
				}
				stateCrawl.enqueue(aux);
				p.x=tmp.x();
				p.y=tmp.y();
				p.z=tmp.z();
				p.ref=base.toStdString();
				p.vel = vel;
				proxies[aux]->setIKLeg(p,false);
			}
			i+=tbezier;
			if(i>1)
			{
				return true;
				i=0;
			}
			return false;
		}
	}
	return true;
}

bool SpecificWorker::Quadruped()
{
	static float i=0;
	bool ismoving=false;
	if(i==0)
	{
		for(int k=0;k<6;k++)
			if(proxies[k]->getStateLeg().ismoving)
			{
				ismoving=true;
				break;
			}
		if(!ismoving)
		{
			stateQuadruped.enqueue(stateQuadruped.dequeue());
			stateQuadruped.enqueue(stateQuadruped.dequeue());
			for(int i=0;i<6;i++)
				pre_statelegs[i]=statelegs[i];
		}
	}
	if(!ismoving)	
	{
		if(lini!=QVec::vec3(0, 0, 0)&&lfin!=QVec::vec3(0,0,0))
		{
			RoboCompLegController::PoseLeg p;
			RoboCompLegController::StateLeg st;
			QVec ini,fin,tmp,med;
			for(int j=0;j<6;j++)
			{
				int aux=stateQuadruped.dequeue();
				st=pre_statelegs[aux];
				if(j==0||j==1)
				{ //patas por arco
					ini =QVec::vec3(st.x,legsp[aux].y(),st.z),
					fin = legsp[aux]+lfin,med=legsp[aux],
					tmp=bezier3(ini,QVec::vec3(med.x(),0,med.z()),fin,i);
				}
				else
				{ //patas por tierra
					int a=2;
					if(j==2||j==3)
						a=1;
					ini = QVec::vec3(st.x,legsp[aux].y(),st.z),
					fin = legsp[aux]-lfin-(((lini-lfin)/3)*a),
					tmp = bezier2(ini,fin,i);
				}
				stateQuadruped.enqueue(aux);
				p.x=tmp.x();
				p.y=tmp.y();
				p.z=tmp.z();
				p.ref=base.toStdString();
				p.vel = vel;
				proxies[aux]->setIKLeg(p,false);
			}
			i+=tbezier;
			if(i>1)
			{
				i=0;
				return true;
			}
			return false;
		}
	}
	return true;
}

void SpecificWorker::uphexapod()
{
	static bool state=true;
	static float i;
	static QVec ini[6],fin[6];
	RoboCompLegController::PoseLeg p;
	p.ref=base.toStdString();
	p.vel = vel;
	if(i==0)
		for(int i=0;i<6;i++)
			pre_statelegs[i]=statelegs[i];
	if(state)
	{
		for(int s=0;s<6;s++)
		{
			if(i==0)
			{
				RoboCompLegController::StateLeg st=pre_statelegs[s];
				fin[s] = QVec::vec3(legsp[s].x(),20,legsp[s].z());
				ini[s] = QVec::vec3(st.x,st.y,st.z);
			}
			QVec tmp=bezier2(ini[s],fin[s],i);
			p.x=tmp.x();
			p.y=tmp.y();
			p.z=tmp.z();
			proxies[s]->setIKLeg(p,false);
		}
		i+=0.1;
	}
	else
	{
		for(int s=0;s<6;s++)
		{
			if(i==0)
			{
				RoboCompLegController::StateLeg st=pre_statelegs[s];
				fin[s] =legsp[s];
				ini[s] = QVec::vec3(st.x,st.y,st.z);
			}
			QVec tmp=bezier2(ini[s],fin[s],i);
			p.x=tmp.x();
			p.y=tmp.y();
			p.z=tmp.z();
			proxies[s]->setIKLeg(p,false);
		}
		i+=0.1;
	}
	if(i>1)
	{
		if(!state)
		{
			state=true;
			modovalue=comboBox->currentIndex();
		}
		else
			state=false;
		i=0;
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

void SpecificWorker::update()
{
	float value_x, value_y, value_z;
	value_x=sliderX->value();
	value_y=sliderY->value();
	value_z=sliderZ->value();
	
	x=value_x/300;
	X_pre=X;
	X=mapear(value_x,-65537,65537, -39,39);

	
	y=value_y/300;
	Y_pre=Y;
	Y=mapear(value_y,-65537,65537, 47,-47);

	
	z=value_z/300;
	Z_pre=Z;
	Z=mapear(value_z,65537,-65537, -47,47);
	
	vel=sliderVel->value();
	
	angles_pre.q1=angles.q1;
	angles_pre.q2=angles.q2;
	angles_pre.q3=angles.q3;
	float aux=sliderq1->value();
	angles.q1=(aux/35537.0);
	aux=sliderq2->value();
	angles.q2=(aux/35537.0);
	aux=sliderq3->value();
	angles.q3=(aux/35537.0);

	valueX->setText(QString::number(X));
	valueY->setText(QString::number(Y));
	valueZ->setText(QString::number(Z));
	valueq1->setText(QString::number(angles.q1));
	valueq2->setText(QString::number(angles.q2));
	valueq3->setText(QString::number(angles.q3));
	valuevel->setText(QString::number(vel));
	
}
void SpecificWorker::updateState(int state)
{
	modovalue=state;
	if(state==8||state==9)
		LEGS->setVisible(true);
	else
		LEGS->setVisible(false);
}

void SpecificWorker::stateuphexapod()
{
	modovalue=-1;
}

void SpecificWorker::updateposleg()
{
	for(int i=0;i<6;i++)
	{
		statelegs[i] = proxies[i]->getStateLeg();
		legsp[i]=QVec::vec3(statelegs[i].x,statelegs[i].y,statelegs[i].z);
	}
}

void SpecificWorker::ResetSlider()
{
	sliderq1->setSliderPosition(0);
	sliderq2->setSliderPosition(0);
	sliderq3->setSliderPosition(0);
	sliderX->setSliderPosition(0);
	sliderY->setSliderPosition(0);
	sliderZ->setSliderPosition(0);
	sliderVel->setSliderPosition(0);
	update();
}

void SpecificWorker::mostrarPointandAngles()
{
	sliderq1->setVisible(true);
	sliderq2->setVisible(true);
	sliderq3->setVisible(true);
	sliderX->setVisible(true);
	sliderY->setVisible(true);
	sliderZ->setVisible(true);
	
	valueq1->setVisible(true);
	valueq2->setVisible(true);
	valueq3->setVisible(true);
	valueX->setVisible(true);
	valueY->setVisible(true);
	valueZ->setVisible(true);
	
	labelX->setVisible(true);
	labelY->setVisible(true);
	labelZ->setVisible(true);
	labelq1->setVisible(true);
	labelq2->setVisible(true);
	labelq3->setVisible(true);
	Point->setVisible(true);
	Angles->setVisible(true);
}

void SpecificWorker::ocultarAngles()
{
	sliderq1->setVisible(false);
	sliderq2->setVisible(false);
	sliderq3->setVisible(false);
	sliderX->setVisible(true);
	sliderY->setVisible(true);
	sliderZ->setVisible(true);
	
	valueq1->setVisible(false);
	valueq2->setVisible(false);
	valueq3->setVisible(false);
	valueX->setVisible(true);
	valueY->setVisible(true);
	valueZ->setVisible(true);
	
	labelX->setVisible(true);
	labelY->setVisible(true);
	labelZ->setVisible(true);
	labelq1->setVisible(false);
	labelq2->setVisible(false);
	labelq3->setVisible(false);
	Point->setVisible(true);
	Angles->setVisible(false);
}

void SpecificWorker::ocultarPoint()
{
	sliderq1->setVisible(true);
	sliderq2->setVisible(true);
	sliderq3->setVisible(true);
	sliderX->setVisible(false);
	sliderY->setVisible(false);
	sliderZ->setVisible(false);
	
	valueq1->setVisible(true);
	valueq2->setVisible(true);
	valueq3->setVisible(true);
	valueX->setVisible(false);
	valueY->setVisible(false);
	valueZ->setVisible(false);
	
	labelX->setVisible(false);
	labelY->setVisible(false);
	labelZ->setVisible(false);
	labelq1->setVisible(true);
	labelq2->setVisible(true);
	labelq3->setVisible(true);
	Point->setVisible(false);
	Angles->setVisible(true);
}

void SpecificWorker::updateStates()
{
	for(int i=0;i<6;i++)
		statelegs[i] = proxies[i]->getStateLeg();
	for(int i=0;i<6;i++)
	{
		RoboCompLegController::StateLeg s=statelegs[i];
		inner->updateJointValue(QString::fromStdString(s.q1.name),s.q1.pos);
		inner->updateJointValue(QString::fromStdString(s.q2.name),s.q2.pos);
		inner->updateJointValue(QString::fromStdString(s.q3.name),s.q3.pos);
	}
	innerViewer->update();
	osgView->autoResize();
	osgView->frame();
}

