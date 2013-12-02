#################################################
# Behavior System File
# 
# 
#################################################

import rone, sys, math, neighbors, velocity, pose, leds, math2, motion, hba


# System parameters - don't edit
NBR_PERIOD = 330
BEH_INACTIVE = (0, 0, False)
TVRV_RATIO = 20

def get_tv(beh_arg):
    return beh_arg[0]


def get_rv(beh_arg):
    return beh_arg[1]


def get_active(beh_arg):
    return beh_arg[2]

def sum(beh1, beh2):
    tv = get_tv(beh1) + get_tv(beh2)
    rv = get_rv(beh1) + get_rv(beh2)
    active = get_active(beh1) or get_active(beh2)
    return (tv, rv, active)
    
#Takes a list of behaviors that return tuples (tv,rv,active)
#Determines which one should be run based on the hierarchy 
#Returns the behavior tuple of the determined behavior
def subsume(beh_output_list):
    beh_output = BEH_INACTIVE
    for beh_val in beh_output_list:
        if(get_active(beh_val)):
            beh_output = beh_val
    return beh_output

#Initializes all robot functionality (motors, leds, pose, motion)    
def init(kff, kff_offset, kp, ki):
    velocity.init(kff, kff_offset, kp, ki)
    leds.init()
    pose.init()
    motion.init()
    neighbors.init(NBR_PERIOD)
        

#Sets motors based on input behavior tuple (tv,rv,active)
def motion_set(beh_val):
    velocity.set_tvrv(get_tv(beh_val), get_rv(beh_val))


#Updates necessary files
def update():
    leds.update()
    pose.update()
    motion.update()
    velocity.update()
    return neighbors.update()

        
#################################
# Returns the bump angle or none if there was no bump
SENSOR_ANGLES = [0.3927, 1.1781, 1.9635, 2.7489, 3.5343, 4.3197, 5.1051, 5.8905]

def bump_angle_get():
    bumps_raw = rone.bump_sensors_get_value()
    if bumps_raw == 0x00:
        return None
    else:
        sum_x = 0
        sum_y = 0
        for i in range(8):
            pressed = ((bumps_raw >> i) & 0x01) > 0
            if pressed:
                sum_y+= math.sin(SENSOR_ANGLES[i])
                sum_x+= math.cos(SENSOR_ANGLES[i])
        bump_angle = math2.normalize_angle(math.atan2(sum_y, sum_x))        
    return bump_angle


def bump_left_get_value(bump_bits):
    return ((bump_bits & 7) > 0)

def bump_front_get_value(bump_bits):
    return (bump_bits == 129)

def bump_right_get_value(bump_bits):
    return ((bump_bits & 224) > 0)

BUMP_TIME_SIDE = 300
BUMP_TIME_ROTATE = 500
BUMP_STATE_IDLE = 0
BUMP_STATE_TURN_RIGHT = 1
BUMP_STATE_TURN_LEFT = 2
BUMP_STATE_ROTATE = 3

bump_time = sys.time()
bump_state = BUMP_STATE_IDLE

def bump_beh(tv):
    # act on the information from the message.  Note that this might be 
    # information stored from the last message we received, because message 
    # information remains active for a while
    global bump_time
    global bump_state
    bump_bits = rone.bump_sensors_get_value()

    # check bump sensors
    if bump_left_get_value(bump_bits):
        bump_state = BUMP_STATE_TURN_RIGHT
        bump_time = sys.time() + BUMP_TIME_SIDE 
    elif bump_right_get_value(bump_bits):
        bump_state = BUMP_STATE_TURN_LEFT
        bump_time = sys.time() + BUMP_TIME_SIDE 
    elif bump_front_get_value(bump_bits):
        bump_state = BUMP_STATE_ROTATE
        bump_time = sys.time() + BUMP_TIME_ROTATE
    
    if sys.time() > bump_time:
        bump_state = BUMP_STATE_IDLE

    # control the robot
    beh_out = BEH_INACTIVE
    if bump_state == BUMP_STATE_TURN_RIGHT:
        beh_out = (0, -tv * TVRV_RATIO, True)
    elif bump_state == BUMP_STATE_TURN_LEFT:
        beh_out = (0, tv * TVRV_RATIO, True)
    elif bump_state == BUMP_STATE_ROTATE:
        beh_out = (0, tv * TVRV_RATIO, True)
        
    return beh_out


RV_FOLLOW_GAIN = .66

def follow_nbr(nbr, tv):
    beh_out = BEH_INACTIVE
    if nbr != None:
        rv = neighbors.get_nbr_bearing(nbr) * tv * TVRV_RATIO * RV_FOLLOW_GAIN
        #tv = MOTION_TV
        beh_out = (tv, rv, True)
    return beh_out
    

def avoid_nbr(nbr, tv):
    beh_out = BEH_INACTIVE
    if nbr != None:
        bearing = math2.normalize_angle(neighbors.get_nbr_bearing(nbr) + math.pi)
        rv = bearing * tv * TVRV_RATIO * RV_FOLLOW_GAIN
        #tv = MOTION_TV
        beh_out = (tv, rv, True)
    return beh_out

def tvrv(tv, rv):
    return (tv, rv, True)
    


