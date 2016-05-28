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
#include "legcontrollerI.h"

LegControllerI::LegControllerI(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
}


LegControllerI::~LegControllerI()
{
}

bool LegControllerI::setListIKLeg(const ListPoseLeg  &ps,  bool  simu, const Ice::Current&)
{
	return worker->setListIKLeg(ps, simu);
}

StateLeg LegControllerI::getStateLeg(const Ice::Current&)
{
	return worker->getStateLeg();
}

bool LegControllerI::setIKLeg(const PoseLeg  &p,  bool  simu, const Ice::Current&)
{
	return worker->setIKLeg(p, simu);
}

bool LegControllerI::setIKBody(const PoseBody  &p,  bool  simu, const Ice::Current&)
{
	return worker->setIKBody(p, simu);
}

bool LegControllerI::setFKLeg(const AnglesLeg  &al,  bool  simu, const Ice::Current&)
{
	return worker->setFKLeg(al, simu);
}






