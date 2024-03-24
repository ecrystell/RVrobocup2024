from robot import Robot
import time
r = Robot('/dev/serial0')
# ~ r.move(0,0)
# ~ r.grabber(180,0)

r.move(50, 100)
time.sleep(2)
r.move(100, 50)
time.sleep(2)
r.move(0,0)
time.sleep(5)
