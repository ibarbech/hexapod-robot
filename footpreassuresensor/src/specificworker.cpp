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

	serial.setBaudRate(serial.BaudRateType::BAUD115200);
	serial.setName("/dev/ttyACM0");
	serial.open();
	if(serial.isOpen()){
		printf("¡¡EL PUERTO YA ESTA ABIERTO!!");
	}
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
	leg1 = params[name + ".leg1"].value;
	leg2 = params[name + ".leg2"].value;
	leg3 = params[name + ".leg3"].value;
	leg4 = params[name + ".leg4"].value;
	leg5 = params[name + ".leg5"].value;
	leg6 = params[name + ".leg6"].value;
	timer.start(100);
	return true;
}

void SpecificWorker::compute()
{
	timer.start(100);
	char buff[96];// = "p1 1 p2 2 p3 3 p4 4 p5 5 p6 6";
	int nb = serial.readLine(buff, 96);

	if(nb > 35)
	{
		QString b(buff);	
		QStringList ls = b.split(" ");
		QMutexLocker ml(mutex);
		this->buffer.clear();
		for(int i=0; i< ls.size(); i+=2)
		{
			string name;
			string value = ls[i].toStdString();
			if(value == "p1")
				name = leg1;
			if(value == "p2")
				name = leg2;
			if(value == "p3")
				name = leg3;
			if(value == "p4")
				name = leg4;
			if(value == "p5")
				name = leg5;
			if(value == "p6")
				name = leg6;
			this->buffer[name] = ls[i+1].toInt();
			cout << name <<" "<< ls[i+1].toInt() + '\n';
		}
	 }
	 else
		qDebug() << __FUNCTION__ << "Error reading serial port. Only " << nb << "bytes read";
	
}



Buffer SpecificWorker::readSensors()
{
	QMutexLocker ml(mutex);
	return buffer;
}

int SpecificWorker::readSensor(const string &name)
{
	QMutexLocker ml(mutex);
	if(buffer.count(name) == 1)
		return buffer[name];			
	return -1;
}






