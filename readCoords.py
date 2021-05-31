#!/usr/bin python3
import numpy as np
from CoveragePathPlanning.cpp_algorithms.common_helpers import get_all_area_maps, get_end_coords, get_random_coords
import csv
from CoveragePathPlanning.classes.builGrid import Grid
from CoveragePathPlanning.cpp_algorithms.bcd import bcd
from CoveragePathPlanning.cpp_algorithms.wavefront import wavefront
from CoveragePathPlanning.cpp_algorithms.stc import stc
from CoveragePathPlanning.cpp_algorithms.metrics import coverage_metrics, printer

class calculateFromCoords():
    def __init__(self, route = 'BCD', velocidad = 5, altura = 5):
        self.gpsCoordinates = []
        self.file ='coords.csv' 
        self.typeOfRoute = route
        self.coveragePathRedundancy = 0.0
        self.coveragePathPercentage = 0.0
        self.distanceOfFly = 0.0
        self.timeOfFly = 0.0
        self.gps0 = (10.444601, -75.391864)
        self.gps1 = (10.444648, -75.390153)
        self.gps2 = (10.443347, -75.391815)
        self.gps3 = (10.443399, -75.390110)
        self.takeoffPoint = (10.445997, -75.391791)
        self.coverage_path = []
        self.velocidad = velocidad
        self.altura = altura
        
        

    def readArchiveAndCalculateRoute(self, file):
        with open(file, 'r') as csvfile:
            for data in csv.reader(csvfile, delimiter = ','):
                # coords = (0, 0)
                coords = data
                self.gpsCoordinates.append(coords)
                #self.gpsCoordinates.append(data[0], data[1])
        del data
        
        self.assignCoords()
        self.calculateGrid()



    def assignCoords(self):
        self.gps0 = (float(self.gpsCoordinates[0][0]), float(self.gpsCoordinates[0][1]))
        self.gps1 = (float(self.gpsCoordinates[1][0]), float(self.gpsCoordinates[1][1]))
        self.gps3 = (float(self.gpsCoordinates[2][0]), float(self.gpsCoordinates[2][1]))
        self.gps2 = (float(self.gpsCoordinates[3][0]), float(self.gpsCoordinates[3][1]))
        self.takeoffPoint = (float(self.gpsCoordinates[4][0]), float(self.gpsCoordinates[4][1]))
    

    def calculateGrid(self):
        grid = Grid(self.velocidad, 
                    self.altura,
                    self.gps0, 
                    self.gps1, 
                    self.gps2, 
                    self.gps3, 
                    self.takeoffPoint, 
                    2)
        # grid.validatePixelstoMeters()
        # ---- Get The Map ----
        area_maps = get_all_area_maps('./CoveragePathPlanning/test_maps/')
        #area_maps = get_all_area_maps("./Cove/test_maps/")   # all area maps in the folder
        area_map = area_maps[0]
        grid.buildGrid(area_map.shape[0], area_map.shape[1])
        # ---- Calculate Coverage Path ----
        start_point = get_random_coords(area_map, 1)[0]  # returns a random coord not on an obstacle
        end_point = get_end_coords(area_map, 1)[0]
        if self.typeOfRoute == 'BCD':
            self.coverage_path = bcd(area_map, start_point)
        elif self.typeOfRoute == 'Wavefront':
            self.coverage_path = wavefront(area_map, start_point)
            self.recalculatePath()
        else:
            self.coverage_path = stc(area_map, start_point)

        # cm = coverage_metrics(area_map, self.coverage_path)
        self.coveragePathPercentage,self.coveragePathRedundancy = coverage_metrics(area_map, self.coverage_path)
        # printer(self.cm)
        grid.assignDataGrid(self.coverage_path, 'Route')
        grid.calculateStatistics()
        self.distanceOfFly = grid.distanceOfFly
        self.timeOfFly = grid.timeOfFly 



    def recalculatePath(self):
        cp = np.array(self.coverage_path)
        px, py = cp.T
        coverage_path_wavefront = []
        for i in range(len(px)):
            coverage_path_wavefront.append((px[i], py[i]))
        self.coverage_path = coverage_path_wavefront


def main():
    coords = calculateFromCoords()
    coords.readArchiveAndCalculateRoute('coords.csv')
    # coords.assignCoords()
    print('Métricas Calculadas Ruta')
    print('Longitud Ruta :', coords.distanceOfFly, 'mts')
    print('Tiempo de Vuelo :', coords.timeOfFly, 'seconds')
    print('Porcentaje Cobertura:', coords.coveragePathPercentage*100, '%')
    print('Redundancia :', coords.coveragePathRedundancy*100, '%')



if __name__ == "__main__":
    main()

