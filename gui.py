#!/usr/bin python3

import sys
import rospy
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow
from coverage import*
from time import time
from PyQt5 import*
from nodo_ROS import px4AutoFlight
from readCoords import calculateFromCoords



class gui(QMainWindow):
    def __init__(self):
        super().__init__()
        #rospy.init_node('coverage', anonymous=True)
        self.ui = Ui_Coverage()
        self.ui.setupUi(self)
        self.velo_goal = 0.0
        self.alt_goal = 0.0
        self.selected_algorith = 'BCD'
        self.teoricTimeOfFly = 0.0
        self.teoricDistanceOfFly = 0.0
        self.redundancyCalculated = 0.0
        self.coverageCalculated = 0.0
        self.distanceBetweenLines = 0.0
        self.PX4modes = px4AutoFlight()
        timer = QTimer(self)
        timer.timeout.connect(self.displayTime)
        timer.timeout.connect(self.updateStatusDrone)
        timer.start(1)
        self.ui.sliderVelocidad.valueChanged.connect(self.updateDial)
        self.ui.sliderAltura.valueChanged.connect(self.updateALtura)
        self.ui.sliderAnchoLineas.valueChanged.connect(self.updateAncho)
        self.ui.pushButtonSimulacion.clicked.connect(self.startFlight)
        self.ui.btn_CalcularRuta.clicked.connect(self.calcRoutes)
        self.ui.radioButtonBCD.toggled.connect(self.onClicked)
        self.ui.radioButtonWave.toggled.connect(self.onClicked)
        self.ui.radioButtonSTC.toggled.connect(self.onClicked)

        
    def displayTime(self):
        self.ui.velocidadDrone.setText("{:.2f}".format(self.PX4modes.velocidad_drone))
        self.ui.distanciaRecorridaDrone.setText("{:.2f}".format(self.PX4modes.distancia_viaje_drone))
        self.ui.AltitudeDrone.setText("{:.2f}".format(self.PX4modes.altura_drone))
        self.ui.TimeOfFlyDrone.setText(str(int(self.PX4modes.tiempo_vuelo_drone)))
        self.ui.currentWayPoint.setText(str(self.PX4modes.wp_actual))

    def updateStatusDrone(self):

        if self.PX4modes.extended_state == 2:
            self.ui.pushButtonSimulacion.setEnabled(False)
            self.ui.lbStatus.setText('In the Air')
            # self.ui.lbStatus.setStyleSheet()
        elif self.PX4modes.extended_state == 1:
            self.ui.pushButtonSimulacion.setEnabled(True)
            self.ui.lbStatus.setText('On the Groud')
        elif self.PX4modes.extended_state == 3:
            self.ui.lbStatus.setText('Taking Off')
        elif self.PX4modes.extended_state == 4:
            self.ui.lbStatus.setText('Landing')

    def updateDial(self):
        self.velo_goal = self.ui.sliderVelocidad.value()
        self.velo_goal = float(self.velo_goal/10)
        self.ui.velocidadParametro.setText(str(self.velo_goal))
        #self.ui.lcdNumber.display(floatNUmer)
        # self.velo_goal= floatNUmer
    
    def updateALtura(self, event):
        self.alt_goal = self.ui.sliderAltura.value()
        self.ui.AlturaParametro.setText(str(self.alt_goal))
        print(self.alt_goal)
    
    def updateAncho(self):
        self.distanceBetweenLines = self.ui.sliderAnchoLineas.value()
        self.ui.anchoLineasParametro.setText(str(self.distanceBetweenLines))
        


    def startFlight(self):
        self.PX4modes.__init__(self.velo_goal, self.alt_goal)
        self.failsafe_status = self.PX4modes.read_failsafe()
        if (self.failsafe_status['DL'] != 0) or (self.failsafe_status['RC'] != 0):   
            self.PX4modes.remove_failsafe() 
            
        self.PX4modes.readWayPoints('Route.csv')
        self.PX4modes.loadMission()
        self.PX4modes.setAutoMissionMode()
        self.PX4modes.setArm()
    
            
    def calcRoutes(self):
        # if self.velo_goal > 0 and self.alt_goal >0 and self.distanceBetweenLines>0:
        coords = calculateFromCoords(self.selected_algorith, self.velo_goal, self.alt_goal, self.distanceBetweenLines)
        coords.readArchiveAndCalculateRoute('coords.csv')
        self.coverageCalculated = coords.coveragePathPercentage
        self.redundancyCalculated = coords.coveragePathRedundancy
        self.teoricTimeOfFly = coords.timeOfFly
        self.teoricDistanceOfFly=coords.distanceOfFly
        self.displayValues()

    def displayValues(self):
        self.ui.distanciaTeorica.setText("{:.2f}".format(self.teoricDistanceOfFly))
        self.ui.tiempoTeorico.setText("{:.2f}".format(self.teoricTimeOfFly))
        self.ui.redundancia.setText("{:.2f}".format(self.redundancyCalculated*100))
        self.ui.cobertura.setText("{:.2f}".format(self.coverageCalculated*100))

    def onClicked(self):
        btn = self.sender()
        if btn.isChecked():
            self.selected_algorith = btn.text()



if __name__ == "__main__":
    rospy.init_node('coverage', anonymous=True)
    import sys
    app = QApplication(sys.argv)
    my_app = gui()
    my_app.show()
    
    sys.exit(app.exec_())

