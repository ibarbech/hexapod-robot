#
# Copyright (C) 2016 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import sys, os, Ice, traceback, time

from PySide import *
from genericworker import *

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
	print 'genericworker.py: ROBOCOMP environment variable not set! Exiting.'
	sys.exit()


preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"JointMotor.ice")
from RoboCompJointMotor import *
Ice.loadSlice(preStr+"FootPreassureSensor.ice")
from RoboCompFootPreassureSensor import *



class SpecificWorker(GenericWorker):
    
        listaPatas = ["arm1","arm2","arm3","arm4","arm5","arm6"]
        
        listaMotores = ["arm1motor1","arm1motor2","arm1motor3",
                        "arm2motor1","arm2motor2","arm2motor3",
                        "arm3motor1","arm3motor2","arm3motor3",
                        "arm4motor1","arm4motor2","arm4motor3",
                        "arm5motor1","arm5motor2","arm5motor3",
                        "arm6motor1","arm6motor2","arm6motor3"]
        
        listaProgress = []
        lcdNumbers = []
        
        def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 100
		self.timer.start(self.Period)
		
                self.lcdNumbers = [self.ui.lcdNumber_1,  self.ui.lcdNumber_3,  self.ui.lcdNumber_5, 
                                   self.ui.lcdNumber_2,  self.ui.lcdNumber_4,  self.ui.lcdNumber_6,
                                   self.ui.lcdNumber_7,  self.ui.lcdNumber_9,  self.ui.lcdNumber_11,
                                   self.ui.lcdNumber_8,  self.ui.lcdNumber_10, self.ui.lcdNumber_12,
                                   self.ui.lcdNumber_13, self.ui.lcdNumber_15, self.ui.lcdNumber_17,
                                   self.ui.lcdNumber_14, self.ui.lcdNumber_16, self.ui.lcdNumber_18]

	def setParams(self, params):
		#try:
		#	par = params["InnerModelPath"]
		#	innermodel_path=par.value
		#	innermodel = InnerModel(innermodel_path)
		#except:
		#	traceback.print_exc()
		#	print "Error reading config params"
		return True

	@QtCore.Slot()
	def compute(self):
		#print 'SpecificWorker.compute...'
		#try:
		#	self.differentialrobot_proxy.setSpeedBase(100, 0)
		#except Ice.Exception, e:
		#	traceback.print_exc()
		#	print e
		
		
                self.listaProgress = [self.ui.progressBar_1, self.ui.progressBar_2, self.ui.progressBar_3, self.ui.progressBar_4, self.ui.progressBar_5, self.ui.progressBar_6] 
                	
                for i in range(6):
                    sensor = self.footpreassuresensor_proxy.readSensor(self.listaPatas[i])
                    self.listaProgress[i].setValue(sensor*100/1023)

                for i in range (18):
                    try:
                        estado = self.jointmotor_proxy.getMotorState(self.listaMotores[i])
                        temperatura = estado.temperature
                        self.lcdNumbers[i].display(temperatura)
                    except Ice.Exception as e:
                        print e
                    
		return True





