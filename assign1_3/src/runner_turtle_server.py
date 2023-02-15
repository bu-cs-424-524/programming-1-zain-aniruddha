import rospy
import random
from turtlesim.srv import *
from turtlesim.msg import *
from std_srvs.srv import *

def spawnNewTurtle():
	turtle_x = random.randint(0, 11)
	turtle_y = random.randint(0, 11)
	spawnTurtle(turtle_x, turtle_y, 0, "runnerTurtle")

def initScreen():
	clearStage()
	spawnNewTurtle()

if __name__ == '__main__':
	spawnTurtle = rospy.ServiceProxy('/spawn', Spawn)
	clearStage = rospy.ServiceProxy('/clear', Empty)
	
	initScreen()
