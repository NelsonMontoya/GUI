from shapely.geometry import Polygon
import numpy as np
import utm
# import networkx as nx
import csv
# import PIL
from PIL import Image

class Grid(object):

    def __init__(self, gps0=1, gps1=1, gps2=1, gps3=1, takeoff=0, step=0):
        self.gps0 = gps0
        self.gps1 = gps1
        self.gps2 = gps2
        self.gps3 = gps3
        self.step = step
        self.takeOffPoint = takeoff
        self.curvePoints = []
        self.alt0 = 5
        self.alt1 = 5
        self.theta = 270
        self.timeOfFly = 0
        self.distanceOfFly = 0
        self.velocityOfFly = 5   # m/s
        self.GPSData = np.array([gps0, gps1, gps2, gps3])
        self.UTMData = [utm.from_latlon(cords[0], cords[1]) for cords in self.GPSData]
        self.UTMCoords = np.array([[data[0], data[1]] for data in self.UTMData])
        # store coordinate information
        self.UTMZone = self.UTMData[0][2:]
        self.poly = Polygon(self.UTMCoords)
        self.x_in_meters = self.DistGPS(self.gps0, self.gps1)
        self.y_in_meters = self.DistGPS(self.gps0, self.gps2)

    def validatePixelstoMeters(self):
        n = int(self.x_in_meters/self.step)
        p = int(self.y_in_meters/self.step)
        print(n, p)
        image = Image.open("./fotos/mascara.png")
        # image.show()
        new_image = image.resize((n, p))
        # new_image.show()
        new_image.save("./test_maps/mascara.png")


    @staticmethod
    def rot2D(theta):
        # theta is in rads
        c = np.cos(theta)
        s = np.sin(theta)
        return np.array([[c, -s],
                         [s, c]])

    def rotateGrid(self):
        self.R = self.rot2D(np.radians(self.theta))
        cordsRotated = (self.R @ self.UTMCoords.T).T
        return Polygon(cordsRotated)

    def DistGPS(self, gps0, gps1):
        e0, n0, _, _ = utm.from_latlon(*gps0)
        e1, n1, _, _ = utm.from_latlon(*gps1)
        return np.linalg.norm([e0 - e1, n0 - n1, self.alt0 - self.alt1])

    def buildGrid(self, x_pixels, y_pixels):

        # rotate cords
        # rotatedPoly = self.rotateGrid()
        self.xPixels = x_pixels
        self.yPixels = y_pixels
        # minx, miny, maxx, maxy = rotatedPoly.bounds
        minx, miny, maxx, maxy = self.poly.bounds
        self.xResol = float(self.x_in_meters / self.xPixels)
        self.yResol = float(self.y_in_meters / self.yPixels)

        self.xWorld = np.arange(minx, maxx, self.xResol)
        self.yWorld = np.arange(miny, maxy, self.yResol)
        self.nX = len(self.xWorld)
        self.nY = len(self.yWorld)



    def assignDataGrid(self, coveragePath, name):
        self.pathBCD = self.calculateRouteUTM(coveragePath)
        # self.R = self.rot2D(np.radians(self.theta))
        # cords = np.array(self.pathBCD)
        # cordsRotated = (self.R @ cords.T).T
        # self.pathBCD = cordsRotated
        self.UTM2GPS(self.UTMZone)
        i = len(self.GPSCoordsBCD)
        self.generateRoute(name, self.GPSCoordsBCD)




    def calculateRouteUTM(self, coveragePath):
        path, self.curvePoints = self.steamlinePath(coveragePath)
        # lenPath = len(path)
        dataPath = []
        for node in path:
            dataPath.append([self.xWorld[node[0]], self.yWorld[node[1]]])
            # dataPath.append(self.R.T @ np.array([self.xWorld[node[0]], self.yWorld[node[1]]]))
        return dataPath

    """def generatePathGPS(self):
            self.pathGPSBCD = []
            self.pathGPSBCD.append(self.UTM2GPS(self.UTMZone)) """





    def UTM2GPS(self, zone):
        # converts all the UTM cords to GPS
        self.GPSCoordsBCD = [utm.to_latlon(*cord, *zone) for cord in self.pathBCD]




    def calculateStatistics(self):
        self.distanceOfFly = self.DistGPS(self.takeOffPoint, self.GPSCoordsBCD[0])
        self.timeOfFly = self.distanceOfFly/self.velocityOfFly

        for actual_p, next_p in zip(self.GPSCoordsBCD, self.GPSCoordsBCD[1:]):
            distance = self.DistGPS(actual_p[0:2], next_p[0:2])
            self.distanceOfFly += distance
            self.timeOfFly += distance/self.velocityOfFly

        distance = self.DistGPS(self.takeOffPoint, self.GPSCoordsBCD[len(self.GPSCoordsBCD)-1])
        self.distanceOfFly += distance
        self.timeOfFly += distance / self.velocityOfFly




    def generateRoute(self, filename, GPSCoordinates):

        name = filename+'.txt'
        name_ = filename+'.csv'
        i = len(GPSCoordinates)
        # route in a csv archive with only the gps coordinates
        with open(name_, 'w', newline='') as csvfile:
            writer_ = csv.writer(csvfile, delimiter='\t')
            writer_.writerow(self.takeOffPoint)
            for wp in range(i):
                writer_.writerow(GPSCoordinates[wp])
            writer_.writerow(self.takeOffPoint)

        # Route generated ready to use in the ROS node to simulate the CPP using QGround
        with open(name, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile, delimiter=' ')
            init = ['QGC', 'WPL', '120']
            writer.writerow(init)

            writer = csv.writer(csvfile, delimiter='\t')
            init = [1, 1, 3, 22, 0.0, 0.0, 0.0, 'nan', self.takeOffPoint[0], self.takeOffPoint[1], 5.0, 1]
            writer.writerow(init)
            for wp in range(i):
                init = [wp+2, 0, 3, 16, 0.0, 0.0, 0.0, 0.0, GPSCoordinates[wp][0], GPSCoordinates[wp][1], 5.0, 1]
                writer.writerow(init)

            init = [i+2, 0, 3, 21, 0.0, 0.0, 0.0, 'nan', self.takeOffPoint[0], self.takeOffPoint[1], 0.0, 1]
            writer.writerow(init)

    @staticmethod
    def steamlinePath(coveragePath):
        # removes points in seq that are straight line
        p = [coveragePath[0]]
        # get init heading
        h = (coveragePath[0][0]-coveragePath[0][0],
             coveragePath[0][1]-coveragePath[0][1])
        for c, n in zip(coveragePath[1:], coveragePath[2:]):
            # c current pt
            # n next pt
            # get next heading
            nh = (n[0]-c[0], n[1]-c[1])
            if nh != h:
                # if the direction changes, add the pt
                p.append(c)
            h = nh  # save heading
        # add last point
        p.append(coveragePath[-1])
        # removes sequentially duplicate points
        return [n for i, n in enumerate(p) if i == 0 or n != p[i-1]], p