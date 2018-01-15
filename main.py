
import vrep
import time
import sys
import numpy as np
from skimage import data
import matplotlib.pyplot as plt
from skimage import io, exposure, img_as_uint, img_as_float
import pickle
import time


# Sample
def sampleAngles(n_samples = 10000, num_of_joints = 4, mu = 0.0, sigma = 0.3):
    angle_list = []
    for i in range(n_samples):
        angle_list.append(np.random.normal(mu, sigma, num_of_joints))
    with open('angles.pkl', 'wb') as f:
        pickle.dump(angle_list, f)
    return angle_list


n_samples = 100
# Create n number of frames, for sequence for one angle
interpolation_steps = 4
# Create samples ?
#angle_list = sampleAngles()

# or read pickle ?
angle_list = pickle.load('angles.pkl')

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


    # First step image goes to vein :) So wait for it ..
    for i in range(n_samples + 1):


        # Interpolate here

        for j in range(4):
            if i == 0:
                print 'Skipping the first step'
                continue
            else:
                joint_angles = angle_list[i - 1]
            # Interpolate here
            angle_0 = (joint_angles[0] /  (interpolation_steps)  ) * (j + 1)
            angle_1 = (joint_angles[1] /  (interpolation_steps)  ) * (j + 1)
            angle_2 = (joint_angles[2] /  (interpolation_steps)  ) * (j + 1)
            angle_3 = (joint_angles[3] /  (interpolation_steps)  ) * (j + 1)


            vrep.simxSetJointTargetPosition(clientID, handle, angle_0,  vrep.simx_opmode_oneshot )
            vrep.simxSetJointTargetPosition(clientID, handle2, angle_1,  vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetPosition(clientID, handle3, angle_2,  vrep.simx_opmode_oneshot )
            vrep.simxSetJointTargetPosition(clientID, handle4, angle_3,  vrep.simx_opmode_oneshot )
            # Wait for the motion
            time.sleep(2)
            err, resolution, image = vrep.simxGetVisionSensorImage(clientID,kinectDepth, 0, vrep.simx_opmode_streaming)

            if err == vrep.simx_return_ok:
                im = np.array(image, dtype=np.uint8)
                im.resize(480,640,3)


                io.imsave('depth_' + str(i) +'_' + str(j)+ ' .png', im)


            elif err == vrep.simx_return_novalue_flag:
                print "no image yet"
                pass
            else:
                print err

            vrep.simxSynchronousTrigger(clientID);

    # stop the simulation:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print "Couldn't connect to remote API :("


print "Done, bye!"
