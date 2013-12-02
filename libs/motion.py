################################################################################
##                         Waypoint motion Module                             ##
################################################################################

import  math, math2, pose, velocity

##    NAME
##        motion
##
##    DESCRIPTION
##        Waypoint motion module
##
##    FUNCTIONS
##        init()
##            initialize the motion_state dictionary
##
##        set_goal(motion_state, goal_pos)
##            sets the motion state to rotate only
##
##            PARAMS
##                1. motion_state - the dicitonary motion state
##                2. goal_pos     - (?)
##
##        get_goal(motion_state)
##            access the motion state dictionary to retrieve the latest goal position
##
##            PARAMS
##                1. motion_state - the dicitonary motion state
##            RETURN
##                the current goal position stored in the motion state dictionary
##
##        
##        is_done(motion_state)
##            returns the boolean of motion_state dictionary's 'motion_done' key
##
##            PARAMS
##                1. motion_state - the dicitonary motion state
##            RETURN
##                boolean of the state of the motion
##
##        move_to_goal(motion_state, pose_state, tv_max)
##            moving towards the goal position
##
##            PARAMS
##                1. motion_state   - the dicitonary motion state
##                2. pose_state     - current pose
##                3. velocity_state - current velocity
##                4. tv_max         - maximum translational velocity
##            RETURN
##                a tuple of the new translational and rotational velocity
        
        
    

MOTION_CAPTURE_DISTANCE = 16
MOTION_RELEASE_DISTANCE = 32
MOTION_CAPTURE_ANGLE = math.pi/2
MOTION_RELEASE_ANGLE = math.pi/10
MOTION_TV_MIN = 20
MOTION_TV_GAIN = 3
MOTION_RV_GAIN = 50
MOTION_RV_MAX = 100

motion_state = {}

def init():
    global motion_state
    motion_state['motion_done'] = True
    motion_state['rotate_only'] = True
    motion_state['tv_max'] = 100
    motion_state['goal_pos'] = (0.0, 0.0)


def set_goal(goal_pos, tv_max):
    global motion_state
    motion_state['motion_done'] = False    
    motion_state['rotate_only'] = True
    motion_state['goal_pos'] = goal_pos
    motion_state['tv_max'] = tv_max


#def get_goal():
#    return motion_state['goal_pos']


def is_done():
    global motion_state
    return motion_state['motion_done']


def update():
    global motion_state
    if motion_state['motion_done'] == True:
        return (0, 0)
    (x_goal, y_goal) = motion_state['goal_position']
    (x_robot, y_robot, heading_robot) = pose.get_pose()
    (distance_goal, heading_goal) = math2.topolar(x_goal - x_robot, y_goal - y_robot)
    heading_goal = math2.normalize_angle(heading_goal)

    if distance_goal < MOTION_CAPTURE_DISTANCE:
        # you are at your destination 
        motion_state['motion_done'] = True
        tv = 0
        rv = 0
    elif distance_goal > MOTION_RELEASE_DISTANCE:
        motion_state['motion_done'] = False
        
    if motion_state['motion_done'] == False:
        # Drive towards goal position
        tv = distance_goal * MOTION_TV_GAIN + MOTION_TV_MIN
        tv = velocity.clamp(tv, motion_state['tv_max'])
    
        # Rotate towards goal position
        heading_error = math2.smallest_angle_diff(heading_robot, heading_goal)
        rv = MOTION_RV_GAIN * heading_error
        rv = velocity.clamp(rv, MOTION_RV_MAX)

        if motion_state['rotate_only']:
            tv = 0
            if abs(heading_error) < MOTION_RELEASE_ANGLE:
                motion_state['rotate_only'] = False
        else:
            if abs(heading_error) > MOTION_CAPTURE_ANGLE:
                motion_state['rotate_only'] = True
    return (tv, rv)




