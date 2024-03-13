from LD19 import LD19
from robot import Robot
import cv2
import time
import serial
import numpy as np
import math
import pygame

	
lidar = LD19('/dev/ttyAMA3', offsetdeg = 0, flip = True) #offsetddeg was -90
robot = Robot('/dev/serial0')
lidar.visualise(0, 180)
	
def find_best_fit_line(points):
    # Convert the list of points into NumPy arrays
    points = np.array(points)
    
    # Compute the centroid of the points
    centroid = np.mean(points, axis=0)
    
    # Compute the covariance matrix of the points
    covariance_matrix = np.cov(points, rowvar=False)
    
    # Compute the eigenvalues and eigenvectors of the covariance matrix
    eigenvalues, eigenvectors = np.linalg.eig(covariance_matrix)
    
    # Sort the eigenvalues and corresponding eigenvectors in descending order
    sorted_indices = np.argsort(eigenvalues)[::-1]
    sorted_eigenvalues = eigenvalues[sorted_indices]
    sorted_eigenvectors = eigenvectors[:, sorted_indices]
    
    # Choose the eigenvector corresponding to the smallest eigenvalue as the direction of the line
    direction = sorted_eigenvectors[:, 1]  # Assuming the smallest eigenvalue corresponds to the second eigenvector
    
    # Compute the coefficients of the line equation (Ax + By + C = 0) using the direction vector and the centroid
    A, B = direction
    C = -(A * centroid[0] + B * centroid[1])
    
	return A, B, C

def point_line_distance(point, A, B, C):
    # Calculate the distance from the point to the line using the formula
    distance = abs(A*point[0] + B*point[1] + C) / np.sqrt(A*2 + B*2)
    return distance

def distance(point1, point2):
    return((point1[0]-point2[0])*2 + (point1[1]-point2[1])2)*0.5

def RANSAC(laserdata, samples, maxDist, numberOfPoints=11): #samples is number of consec points to find best fit line
    lines = []
    laserdataLength = len(laserdata)
    associatedReadings = [0] * len(laserdata)
    startpoint = 0
    while startpoint< len(laserdata):
        for i in range(startpoint, laserdataLength):
            if associatedReadings[i] == 0: 
                startpoint = i
                break
            if i == laserdataLength:
                startpoint = laserdataLength
        if startpoint+ samples > laserdataLength-1:
            # print("no more samples")
            break
        a, b, c = find_best_fit_line(laserdata[startpoint:startpoint+samples]) # a b c is vector coords
        numpointsonline = 0
        linepoints = []
        for i in range(startpoint, startpoint+samples):
            if point_line_distance(laserdata[i], a, b, c) < maxDist: # check distance to line
                numpointsonline += 1
                linepoints.append(laserdata[i])
                associatedReadings[i] = 1
        if numpointsonline > numberOfPoints: # how many points u need to be on a line to be a line
            cluster = numberOfPoints
            clusterpoints = []
            tempassosciatedReadings = []
            for i in range(startpoint + samples, len(laserdata)):
                if point_line_distance(laserdata[i], a, b, c) < maxDist:
                    if distance(laserdata[i], linepoints[-1]) > 30: #if first point of new cluster, far away from old cluster
                        cluster = 0
                        clusterpoints = []
                        clusterpoints.append(laserdata[i])
                        tempassosciatedReadings.append(i)
                    else:   # if there is a new cluster, we want multiple points in close proximity 
                        cluster += 1
                        if cluster < 3: # if there is less than 3 in the cluster, it may not be considered as in the same line as the current best fit, so take notes of it temporaryily first
                            tempassosciatedReadings.append(i)
                            clusterpoints.append(laserdata[i])
                        elif cluster == 3: # if cluster == 3, it is quite confirmed that it is on the line, add points and find new best fit
                            clusterpoints.append(laserdata[i])
                            tempassosciatedReadings.append(i)
                            linepoints.extend(clusterpoints)
                            for i in tempassosciatedReadings:
                                associatedReadings[i] = 1
                            a, b, c = find_best_fit_line(linepoints)
                        else: #cluster more than 5 keep adding to line 
                            linepoints.append(laserdata[i])
                            associatedReadings[i] = 1
                            a, b, c = find_best_fit_line(linepoints)
            if distance(linepoints[0], linepoints[-1])> 50: # check distance of the line, if its long enough
                lines.append((a, b, c))
        startpoint += 2
	return lines
	
def draw_lines(self, lines, color=(255, 255, 255), linewidth=2):
        width  = 1200
        center = (0,0)
        
        for A, B, C in lines:
            if B == 0:
                # Handle the case when B is zero (line is horizontal)
                x = -C / A
                pygame.draw.line(self.map, color, (x, 0), (x, 600), linewidth)
            else:
                m = -A / B  # Slope
                b = -C / B  # y-intercept
                # Calculate two points on the line for drawing
                x1 = 0
                y1 = int(m * x1 + b)
                x2 = 1200 - 1
                y2 = int(m * x2 + b)
            
                # Draw the line on the screen
                pygame.draw.line(self.map, color, (x1 + center[0], y1 + center[1]), (x2 + center[0], y2 + center[1]), linewidth)
