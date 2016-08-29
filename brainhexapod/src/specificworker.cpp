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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	proxies[0]=legcontroller1_proxy;
	proxies[1]=legcontroller2_proxy;
	proxies[2]=legcontroller3_proxy;
	proxies[3]=legcontroller4_proxy;
	proxies[4]=legcontroller5_proxy;
	proxies[5]=legcontroller6_proxy;
	pasostate[0]="paso";
	pasostate[1]="empujar";
	pasostate[2]="empujar";
	pasostate[3]="paso";
	pasostate[4]="paso";
	pasostate[5]="empujar";
	osgView = new OsgView(this->frame);
	show();
	clkupdate.start(10);
	connect(&clkupdate, SIGNAL(timeout()), this, SLOT(updatevalues()));
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
	qDebug()<<PROGRAM_NAME;
	base=QString::fromStdString(params[name+".base"].value);
	innerModel = new InnerModel(params[name+".InnerModel"].value);

	innerViewer = new InnerModelViewer(innerModel, "root", osgView->getRootGroup(), false);
	
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
	
	timer.start(Period);
	
	
	return true;
}


void SpecificWorker::compute()
{
// 	static int i = 0;
// 	if(i==6)
// 		i=0;
// 	bool menor=true;
// 	double incremento=0;
	if (allidel())
	{
// 		for(int i = 0; i<6; i++)
// 		{
// 			if(statelegs[i].y < -90)
// 			{
// 				menor=false;
// 				break;
// 			}
// 		}
// 		if(menor)
// 			for(int i = 0; i<6; i++)
// 				if(incremento < statelegs[i].y+96)
// 					incremento = statelegs[i].y+96;
// 		for(int i = 0; i<6; i++)
// 		{
// 			RoboCompLegController::PoseLeg p;
// 			p.ref =  statelegs[i].ref;
// 			p.vel = 50;
// 			p.x = statelegs[i].x;
// 			p.z = statelegs[i].z;
// 			p.y = statelegs[i].y-incremento;
// 			proxies[i]->setIKLeg(p,false);
// 		}
		for(int i = 0; i<6; i++)
		{
			qDebug()<<"envio a la pata "<< (i+1);
			proxies[i]->move(0,30,pasostate[i]);
			if(pasostate[i]=="paso")
				pasostate[i]="empujar";
			else
				pasostate[i]="paso";
		}
	}
	
}

bool SpecificWorker::allidel()
{
	bool noidel=true;
	QString idel;
	qDebug()<<"-----------------";
	for(int i=0; i<6; i++)
	{
		if(statelegs[i].idel==false)
		{
			noidel=false;
			qDebug()<<" Leg " << (i+1) << statelegs[i].idel;
		}
	}
	qDebug()<<"-----------------";
	return noidel;
}

void SpecificWorker::updatevalues()
{
	//update inner
	for(int i=0;i<6;i++)
		statelegs[i] = proxies[i]->getStateLeg();
	for(int i=0;i<6;i++)
	{
		RoboCompLegController::StateLeg s=statelegs[i];
		innerModel->updateJointValue(QString::fromStdString(s.q1.name),s.q1.pos);
		innerModel->updateJointValue(QString::fromStdString(s.q2.name),s.q2.pos);
		innerModel->updateJointValue(QString::fromStdString(s.q3.name),s.q3.pos);
	}
	innerViewer->update();
	osgView->autoResize();
	osgView->frame();
}






