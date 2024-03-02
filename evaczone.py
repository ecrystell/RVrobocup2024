from robot import*
from LD19 import LD19
import time


lidar = LD19('/dev/ttyAMA3', offsetdeg = 0, flip = True) #offsetddeg was -90
robot = Robot('/dev/serial0')
lidar.visualise(0, 180)
r = Robot('/dev/serial0')

threshold = 100

def findCenter(threshold):
    r.movedegrees(100, 100, 50)
    r.move(70, -70)
    runtime = time.time() + 3
    while time.time() < runtime:
        if lidar.getReading(90) < threshold:
            diff = lidar.getReading(90) - threshold
            r.move(70 - (0.1 * diff), -70)