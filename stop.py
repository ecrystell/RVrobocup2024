from robot import Robot
r = Robot('/dev/serial0')
# ~ r.move(0,0)
# ~ r.grabber(180,0)
r.movedegrees(60,60,30)
time.sleep(2)
