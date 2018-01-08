
import vrep
import time
import sys

print "Running the script.."
vrep.simxFinish(-1) # Close it all
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP

if clientID!=-1:
    # enable the synchronous mode on the client:
    vrep.simxSynchronous(clientID,True)
    # start the simulation:
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking)
    # Connect to kinect sensor

    # Connect to robotic arm
    _, handle = vrep.simxGetObjectHandle(clientID, 'UR5_joint1', vrep.simx_opmode_oneshot)
    _, handle2 = vrep.simxGetObjectHandle(clientID, 'UR5_joint2', vrep.simx_opmode_oneshot)
    _, handle3 = vrep.simxGetObjectHandle(clientID, 'UR5_joint3', vrep.simx_opmode_oneshot)
    _, handle4 = vrep.simxGetObjectHandle(clientID, 'UR5_joint4', vrep.simx_opmode_oneshot)

    # Now step a few times:
    for i in range(1,10):
        if sys.version_info[0] == 3:
            input('Press <enter> key to step the simulation!')
        else:
            raw_input('Press <enter> key to step the simulation!')
        vrep.simxSynchronousTrigger(clientID);

    # stop the simulation:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print "Couldn't connect to remote API :("


print "Done, bye!"
