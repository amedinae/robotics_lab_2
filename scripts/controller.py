import rospy
import time
import termios, sys, os
#import genpy
# from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand

TERMIOS = termios

def getkey():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
    new[6][TERMIOS.VMIN] = 1
    new[6][TERMIOS.VTIME] = 0
    termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
    c = None
    try:
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
    return c


def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

def setTorquesLimit():
    jointCommand('', 1, 'Torque_Limit', 500, 0)
    jointCommand('', 2, 'Torque_Limit', 500, 0)
    jointCommand('', 3, 'Torque_Limit', 400, 0)
    jointCommand('', 4, 'Torque_Limit', 400, 0)

class Selector:

    def __init__(self) -> None:
        self._selected = 1
        self._homePositions = [512,512,512,512,512]
        self._targetPositions = [375,647,708,667,820]


    def getCurrentId(self):
        return self._selected

    def getHomePosition(self):
        jointId = self._selected
        if jointId>len(self._homePositions) or jointId<1:
            return -1
        return self._homePositions[jointId-1]

    def getTargetPosition(self):
        jointId = self._selected
        if jointId>len(self._homePositions) or jointId<1:
            return -1
        return self._targetPositions[jointId-1]

    def setNextId(self):
        if self._selected == 5:
            self._selected = 1
            return
        self._selected += 1

    def setPreviousId(self):
        if self._selected == 1:
            self._selected = 5
            return
        self._selected -= 1    

if __name__ == '__main__':
    selector = Selector()  
    try:
        setTorquesLimit()
        while(not rospy.is_shutdown()):
            print(f"Current ID: {selector.getCurrentId()}")
            key = str(getkey())[2].lower()
            if key == "w":
                selector.setNextId()
            if key == "s":
                selector.setPreviousId()
            if key == "a":
                toPosition = selector.getHomePosition()
                jointId = selector.getCurrentId()
                jointCommand('', jointId, 'Goal_Position', toPosition, 0.5)
                print(f"Joint {jointId} moved to {toPosition}")
            if key == "d":
                toPosition = selector.getTargetPosition()
                jointId = selector.getCurrentId()
                jointCommand('', jointId, 'Goal_Position', toPosition, 0.5)
                print(f"Joint {jointId} moved to {toPosition}")
            if key == "q":
                break
                time.sleep(0.5)
    except rospy.ROSInterruptException:
        pass