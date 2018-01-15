
import vrep
import time
import sys
import numpy as np
from skimage import data
import matplotlib.pyplot as plt
from skimage import io, exposure, img_as_uint, img_as_float
import pickle


# Sample
def sampleAngles(n_samples = 10000, num_of_joints = 4, mu = 15.0, sigma = 7.5):
    angle_list = []
    for i in range(n_samples):
        angle_list.append(np.random.normal(mu, sigma, num_of_joints))
    with open('angles.pkl', 'wb') as f:
        pickle.dump(angle_list, f)
    return angle_list


n_samples = 5
# Create n number of frames, for sequence for one angle
interpolation_steps = 4
# Create samples
angle_list = sampleAngles()



print "Running the script.."
vrep.simxFinish(-1) # Close it all
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP





if clientID!=-1:
    # enable the synchronous mode on the client:
    vrep.simxSynchronous(clientID,True)
    # start the simulation:
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking)
    # Connect to kinect sensor
    errorCodeKinectRGB,kinectRGB=vrep.simxGetObjectHandle(clientID,'kinect_rgb',vrep.simx_opmode_oneshot_wait)
    errorCodeKinectDepth,kinectDepth=vrep.simxGetObjectHandle(clientID,'kinect_depth',vrep.simx_opmode_oneshot_wait)

    # Connect to robotic arm, we use 4 angles, if you use a differenct scene check joint  names
    _, handle = vrep.simxGetObjectHandle(clientID, 'UR5_joint1', vrep.simx_opmode_blocking)
    _, handle2 = vrep.simxGetObjectHandle(clientID, 'UR5_joint2', vrep.simx_opmode_blocking)
    _, handle3 = vrep.simxGetObjectHandle(clientID, 'UR5_joint3', vrep.simx_opmode_blocking)
    _, handle4 = vrep.simxGetObjectHandle(clientID, 'UR5_joint4', vrep.simx_opmode_blocking)

    vrep.simxSetJointTargetVelocity(clientID, handle, 1.0, vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID, handle2, 1.0, vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID, handle3, 1.0, vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID, handle4, 1.0, vrep.simx_opmode_oneshot)

    # Now step a few times:
    for i in range(1,n_samples):

        #if sys.version_info[0] == 3:
        #    input('Press <enter> key to step the simulation!')
        #else:
        #    raw_input('Press <enter> key to step the simulation!')


        for j in range(4):
            joint_angles = angle_list[i]
            print joint_angles
            vrep.simxSetJointPosition(clientID, handle, joint_angles[0],  vrep.simx_opmode_oneshot )
            vrep.simxSetJointPosition(clientID, handle2, joint_angles[1],  vrep.simx_opmode_oneshot )
            vrep.simxSetJointPosition(clientID, handle3, joint_angles[2],  vrep.simx_opmode_oneshot )
            vrep.simxSetJointPosition(clientID, handle4, joint_angles[3],  vrep.simx_opmode_oneshot )
            err, resolution, image = vrep.simxGetVisionSensorImage(clientID,kinectDepth, 0, vrep.simx_opmode_streaming)

        if err == vrep.simx_return_ok:
            print resolution
            im = np.array(image, dtype=np.uint8)
            im.resize(480,640,3)
            #im = img_as_uint(im)

            io.imsave('depth.png', im)

            plt.imshow(im)
            plt.show()
            vrep.simxSetVisionSensorImage(clientID, kinectDepth, image, 0, vrep.simx_opmode_oneshot)
        elif err == vrep.simx_return_novalue_flag:
            print "no image yet"
            pass
        else:
            print err

        print errorCodeKinectRGB
        vrep.simxSynchronousTrigger(clientID);

    # stop the simulation:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print "Couldn't connect to remote API :("


print "Done, bye!"
