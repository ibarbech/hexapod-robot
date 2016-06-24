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

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{


	hexapod.start();

	timer.start(Period);
	

	return true;
}

void SpecificWorker::compute()
{
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
}

void SpecificWorker::fun_avanzar()
{

}

void SpecificWorker::fun_error_imu()
{

}

void SpecificWorker::fun_error_timeout()
{

}

void SpecificWorker::fun_recibe_ofset()
{

}

void SpecificWorker::fun_leer_imu()
{

}

void SpecificWorker::fun_leer_sensores()
{

}

void SpecificWorker::fun_avanzar_principal()
{

}

void SpecificWorker::fun_calcular_subobj()
{

}

void SpecificWorker::fun_moverse()
{

}

void SpecificWorker::fun_calcular_obj()
{

}

void SpecificWorker::fun_exit()
{

}


StateLeg SpecificWorker::getStateLeg()
{

}

bool SpecificWorker::move(const float x, const float y)
{

}

bool SpecificWorker::setListIKLeg(const ListPoseLeg &ps, const bool &simu)
{

}

bool SpecificWorker::setIKLeg(const PoseLeg &p, const bool &simu)
{

}

bool SpecificWorker::setIKBody(const PoseBody &p, const bool &simu)
{

}

bool SpecificWorker::setFKLeg(const AnglesLeg &al, const bool &simu)
{

}






