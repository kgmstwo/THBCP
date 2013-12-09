import rone, sys, math, math2, velocity, pose, motion, leds, neighbors, beh, hba

###########################################################
##
##  Winter
##
###########################################################
#
# This is the code for the Winter season.


# Basic motion parameters - change carefully
MOTION_RV = int(1000 * math.pi * 0.3)
MOTION_TV = 100

# FSM States
STATE_IDLE = 0
STATE_DARK = 1
STATE_LIGHT = 2
STATE_EDGE_OF_LIGHT = 3

# MSG components
MSG_POS_IN_LIGHT = 0

# Other constants
LED_BRIGHTNESS = 40
CLOSENESS_CONSTANT = 4
BRIGHTNESS_CONSTANT = 300

def winter():
    global new_nbrs
    
    beh.init(0.22, 40, 0.5, 0.1)

    state = STATE_IDLE

    while True:
        # run the system updates
        new_nbrs = beh.update()
        
        nbr_list = neighbors.get_neighbors()
        
        beh_out = beh.BEH_INACTIVE
            
        # this is the main finite-state machine
        if state == STATE_IDLE:
            leds.set_pattern('r', 'circle', LED_BRIGHTNESS)
            if rone.button_get_value('r'):
                ##### This is one way to find a cutoff for being in light.
                ##### Make sure you press the 'r' button when the robot is
                ##### in the light!
                global BRIGHTNESS_CONSTANT
                brightnesses = [rone.light_sensor_get_value(dir) for \
                                dir in ['fl', 'fr', 'r']
                                ]
                BRIGHTNESS_CONSTANT = 0.8 * max(brightnesses)
                #####
                initial_time = sys.time()
                state = STATE_LIGHT
            if new_nbrs:
                print "idle"

        elif state == STATE_LIGHT:
            nbr_in_dark = get_near_nbr_in_dark(nbr_list)
            if nbr_in_dark != None:
                bearing = neighbors.get_nbr_bearing(nbr_in_dark)
                bearing = bearing - math.pi
                bearing = math2.normalize_angle(bearing)
                beh_out = move_in_dir(bearing)
            else:
                beh_out = (0, 0, True)

            if not self_in_light():
                state = STATE_DARK

        elif state == STATE_DARK:
            nbrs_in_light = get_nbrs_in_light()
            if len(nbrs_in_light) > 0:
                bearing = get_avg_bearing(nbrs_in_light)
                beh_out = move_in_dir(bearing)
            else:
                beh_out = (MOTION_TV, MOTION_RV, True)

            if self_in_light():
                state = STATE_LIGHT

        elif state == STATE_EDGE_OF_LIGHT:
            pass

        # end of the FSM

        # BUMP_BEH IS NOT HELPFUL IN WINTER
        # bump_beh_out = beh.bump_beh(MOTION_TV)
        # beh_out = beh.subsume([beh_out, bump_beh_out])

        # set the beh velocities
        beh.motion_set(beh_out)

        #set the HBA message
        msg = [0, 0, 0]
        msg[MSG_POS_IN_LIGHT] = self_in_light()
        hba.set_msg(msg[0], msg[1], msg[2])

# Helper functions

def get_nbrs_in_light():
    nbr_list = hba.get_robot_neighbors()
    nbrs_in_light = []
    for nbr in nbr_list:
        in_light = hba.get_msg_from_nbr(nbr, new_nbrs)[MSG_POS_IN_LIGHT]
        if in_light:
            nbrs_in_light.append(nbr)
    return nbrs_in_light

def get_nbrs_in_dark():
    nbr_list = hba.get_robot_neighbors()
    nbrs_in_dark = []
    for nbr in nbr_list:
        in_light = hba.get_msg_from_nbr(nbr, new_nbrs)[MSG_POS_IN_LIGHT]
        if not in_light:
            nbrs_in_dark.append(nbr)
    return nbrs_in_dark

def get_avg_bearing_to_nbrs(nbr_list):
    x = 0.0
    y = 0.0
    for nbr in nbr_list:
        bearing = neighbors.get_nbr_bearing(nbr)
        x += math.cos(bearing)
        y += math.sin(bearing)
    avg_bearing = math.atan2(y, x)
    avg_bearing = math2.normalize_angle(avg_bearing)
    return avg_bearing

def get_near_nbr_in_dark(nbr_list):
    nbrs_in_dark = get_nbrs_in_dark()
    nearest = None
    if len(nbrs_in_dark) > 0:
        nearest = nbrs_in_dark[0]
        for nbr in nbrs_in_dark:
            if neighbors.get_nbr_range_bits(nbr) > neighbors.get_nbr_range_bits(nearest):
                nearest = nbr
        if neighbors.get_nbr_range_bits(nearest) < CLOSENESS_CONSTANT:
            nearest = None
    return nearest

def move_in_dir(bearing):
    bearing = math2.normalize_angle(bearing)
    if bearing >= 0:
        if bearing > math.pi / 2:
            tv = int(MOTION_TV * (1 - bearing * 2 / math.pi))
            rv = int(MOTION_RV * (2 - bearing * 2 / math.pi))
        else:
            tv = int(MOTION_TV * (-1 + bearing * 2 / math.pi))
            rv = int(MOTION_RV * (-2 + bearing * 2 / math.pi))
    else:
        if bearing < -math.pi / 2:
            tv = int(MOTION_TV * (1 + bearing * 2 / math.pi))
            rv = int(MOTION_RV * (2 + bearing * 2 / math.pi))
        else:
            tv = int(MOTION_TV * (-1 - bearing * 2 / math.pi))
            rv = int(MOTION_RV * (-2 - bearing * 2 / math.pi))
    return tv, rv        

def self_in_light():
    any_in_light = False
    for sensor_dir in ['fr', 'fl', 'r']:
        this_one_in_light = (rone.light_sensor_get_value(sensor_dir) > BRIGHTNESS_CONSTANT)
        any_in_light = any_in_light or this_one_in_light
    return any_in_light

# Start!

winter()
