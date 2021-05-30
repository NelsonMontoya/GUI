#!/usr/bin python3
import csv

from PyQt5.sip import assign
from nodo_ROS import main


class readCoords():
    def __init__(self):
        self.gpsCoordinates = []
        self.file ='coords.csv' 
        
        self.gps0 = (10.444601, -75.391864)
        self.gps1 = (10.444648, -75.390153)
        self.gps2 = (10.443347, -75.391815)
        self.gps3 = (10.443399, -75.390110)
        self.takeoffPoint = (10.445997, -75.391791)
        

    def readArchive(self, file):
        with open(file, 'r') as csvfile:
            for data in csv.reader(csvfile, delimiter = ','):
                # coords = (0, 0)
                coords = data
                self.gpsCoordinates.append(coords)
                #self.gpsCoordinates.append(data[0], data[1])
        del data
        
        self.assignCoords()

    def assignCoords(self):
        self.gps0 = (float(self.gpsCoordinates[0][0]), float(self.gpsCoordinates[0][1]))
        self.gps1 = (float(self.gpsCoordinates[1][0]), float(self.gpsCoordinates[1][1]))
        self.gps2 = (float(self.gpsCoordinates[2][0]), float(self.gpsCoordinates[2][1]))
        self.gps3 = (float(self.gpsCoordinates[3][0]), float(self.gpsCoordinates[3][1]))
        self.takeoffPoint = (float(self.gpsCoordinates[4][0]), float(self.gpsCoordinates[4][1]))
    


def main():
    coords = readCoords()
    coords.readArchive('coords.csv')
    # coords.assignCoords()



if __name__ == "__main__":
    main()

