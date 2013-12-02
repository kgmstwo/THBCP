################################################################################
##                          Pose Estimator Module                             ##
################################################################################

import sys, math, math2, velocity, rone

### POSE_UPDATE_PERIOD is used to avoid multiple updates from happening at the same time
POSE_UPDATE_PERIOD = 37    # want 30 ms,use a close prime period to avoid multiple updates happening at the same time

### WHEEL_BASE is the size of the base of the robot
WHEEL_BASE = 89.2

pose_state = {}

### init method
### creates a dictionary of pose_state with keys of
### ticksL, ticksR, pose, odometer, and update_time
### @return dictionary pose_state
def init():
    global pose_state
    pose_state['ticksL'] = rone.encoder_get_ticks('l')
    pose_state['ticksR'] = rone.encoder_get_ticks('r')
    #pose_state['pose'] = (0.0, 0.0, 0.0)
    pose_state['x'] = 0.0
    pose_state['y'] = 0.0
    pose_state['theta'] = 0.0
    pose_state['odometer'] = 0.0
    pose_state['update_time'] = sys.time()

### update method
### updates the pose_state's key's values.
### @params dictionary pose_state
def update():
    global pose_state
    current_time = sys.time()
    update_time = pose_state['update_time']
    if current_time > update_time:
        update_time += POSE_UPDATE_PERIOD
        # advance time if there have been delays in calling update
        if update_time < current_time:
            update_time = current_time + POSE_UPDATE_PERIOD
        pose_state['update_time'] = update_time
        
        # Compute the linear distance traveled by each wheel in millimeters
        ticksL = rone.encoder_get_ticks('l')
        ticksR = rone.encoder_get_ticks('r')
        distL = float(velocity.encoder_delta_ticks(ticksL, pose_state['ticksL'])) * 0.0625
        distR = float(velocity.encoder_delta_ticks(ticksR, pose_state['ticksR'])) * 0.0625
        pose_state['ticksL'] = ticksL
        pose_state['ticksR'] = ticksR 
    
        # Compute the distance traveled by the center of the robot in millimeters
        dist = (distR + distL) / 2.0
        pose_state['odometer'] += abs(dist)
        
        # compute the arc angle in radians
        # use the small angle approximation: arctan(theta) ~ theta
        delta_theta = (distR - distL) / (WHEEL_BASE);
#        (x, y, theta) = pose_state['pose']
#        x = x + dist * math.cos(theta)
#        y = y + dist * math.sin(theta)
#        theta = math2.normalize_theta(theta + delta_theta)
#        pose_state['pose'] = (x, y, theta)

        theta = pose_state['theta']
        pose_state['x'] += dist * math.cos(theta)
        pose_state['y'] += dist * math.sin(theta)
        pose_state['theta'] = math2.normalize_angle(theta + delta_theta)

### get_pose method
### takes in the pose state and returns the value from the key 'pose'
### @return pose_state['pose'] = (x.x, x.x, x.x)
def get_pose():
    global pose_state
    #return pose_state['pose']
    return (pose_state['x'], pose_state['y'], pose_state['theta'])


### set_pose method
### takes in the pose state and returns the value from the key 'pose'
### @param float x
### @param float y
### @param float theta
def set_pose(x, y, theta):
    global pose_state
    #pose_state['pose'] = (x, y, theta)
    pose_state['x'] = x
    pose_state['y'] = y
    pose_state['theta'] = theta


### get_x method
### takes in the pose state and returns the x value from the key 'pose'
### @return double x
#def get_x(pose_state):
#    return pose_state['pose'][0]

### get_y method
### takes in the pose state and returns the y value from the key 'pose'
### @return double y
#def get_y(pose_state):
#    return pose_state['pose'][1]

### get_theta method
### takes in the pose state and returns the theta value from the key 'pose'
### @return double theta
def get_theta():
    global pose_state
    #return pose_state['pose'][2]
    return pose_state['theta']

### get_odometer method
### takes in the pose state and returns the odometer value from the key 'odometer'
### @return double odometer
def get_odometer():
    global pose_state
    return pose_state['odometer']



    
