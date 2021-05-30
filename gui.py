#!/usr/bin python3

import sys
import rospy
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow
from coverage import*
from time import time
from PyQt5 import*
from nodo_ROS import px4AutoFlight
from readCoords import readCoords
from CoveragePathPlanning.classes.builGrid import Grid




class gui(QMainWindow):
    def __init__(self):
        super().__init__()
        #rospy.init_node('coverage', anonymous=True)
        self.ui = Ui_Coverage()
        self.ui.setupUi(self)
        self.velo_goal = 0.0
        self.alt_goal = 0.0
        self.selected_algorith = 'BCD'
        self.PX4modes = px4AutoFlight()
        timer = QTimer(self)
        timer.timeout.connect(self.displayTime)
        timer.start(1)
        self.ui.sliderVelocidad.valueChanged.connect(self.updateDial)
        self.ui.sliderAltura.valueChanged.connect(self.updateALtura)
        self.ui.pushButtonSimulacion.clicked.connect(self.startFlight)
        self.ui.btn_CalcularRuta.clicked.connect(self.calcRoutes)
        self.ui.radioButtonBCD.toggled.connect(self.onClicked)
        self.ui.radioButtonWave.toggled.connect(self.onClicked)
        self.ui.radioButtonSTC.toggled.connect(self.onClicked)

        
    def displayTime(self):
        self.ui.lcdVelocityDrone.display(self.PX4modes.velocidad_drone)
        self.ui.lcdDistanciaDrone.display(self.PX4modes.distancia_viaje_drone)
        self.ui.lcdAltitudeDrone.display(self.PX4modes.altura_drone)
        self.ui.lcdTimeOfFly.display(self.PX4modes.tiempo_vuelo_drone)

    def updateDial(self):
        self.velo_goal = self.ui.sliderVelocidad.value()
        floatNUmer = float(self.velo_goal/10)
        self.ui.lcdNumber.display(floatNUmer)
        self.velo_goal= floatNUmer
    
    def updateALtura(self, event):
        self.alt_goal = self.ui.sliderAltura.value()
        self.ui.lcdAlturaTeorica.display(self.alt_goal)
        print(self.alt_goal)


   

    def startFlight(self):

        self.PX4modes.__init__(self.velo_goal, self.alt_goal)

        if self.PX4modes.drone_in_the_air:
            self.PX4modes.readWayPoints('ruta1.csv')
            self.PX4modes.loadMission()
            self.PX4modes.setAutoMissionMode()
        else:
            self.failsafe_status = self.PX4modes.read_failsafe()
            if (self.failsafe_status['DL'] != 0) or (self.failsafe_status['RC'] != 0):   
                self.PX4modes.remove_failsafe() 
            
            self.PX4modes.readWayPoints('ruta1.csv')
            self.PX4modes.loadMission()
            self.PX4modes.setAutoMissionMode()
            self.PX4modes.setArm()
            
    def calcRoutes(self):
        coords = readCoords()
        coords.readArchive('coords.csv')
        grid = Grid(coords.gps0, coords.gps1, coords.gps2, coords.gps3, coords.takeoffPoint, 1)

    def onClicked(self):
        btn = self.sender()
        if btn.isChecked():
            self.selected_algorith = btn.text()
            # print(self.selected_algorith)



if __name__ == "__main__":
    rospy.init_node('coverage', anonymous=True)
    import sys
    app = QApplication(sys.argv)
    my_app = gui()
    my_app.show()
    
    sys.exit(app.exec_())

