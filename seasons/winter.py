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
STATE_DEAD = 3

# MSG components
MSG_IDX_STATE = 0

# Other constants
LED_BRIGHTNESS = 40
##CLOSENESS_CONSTANT = 3
BRIGHTNESS_THRESHOLDS = {'fl':300, 'fr':300, 'r':300}
ANGLE_TOL = 0.1 * math.pi
LIFESPAN = 5000

def winter():
    beh.init(0.22, 40, 0.5, 0.1)

    state = STATE_IDLE

    manual_control = False
    
    while True:
        # run the system updates
        new_nbrs = beh.update()
        
        nbr_list = neighbors.get_neighbors()
        
        beh_out = beh.BEH_INACTIVE
            
        # this is the main finite-state machine
        if state == STATE_IDLE:
            leds.set_pattern('r', 'circle', LED_BRIGHTNESS)
            if new_nbrs:
                print "idle"

            if rone.button_get_value('r'):
                ##### This is one way to find a cutoff for being in light.
                ##### Make sure you press the 'r' button when the robot is
                ##### in the light!
                global BRIGHTNESS_THRESHOLDS
                for sensor_dir in BRIGHTNESS_THRESHOLDS.keys():
                    BRIGHTNESS_THRESHOLDS[sensor_dir] = 0.85 * rone.light_sensor_get_value(sensor_dir)
                #####
                initial_time = sys.time()
                state = STATE_LIGHT

        elif state == STATE_LIGHT:
            leds.set_pattern('g', 'circle', LED_BRIGHTNESS)
            if manual_control:
                leds.set_pattern('gr', 'group', LED_BRIGHTNESS)
            nbr_in_dark = get_nearest_nbr_in_dark(nbr_list)
            if nbr_in_dark != None:
                bearing = neighbors.get_nbr_bearing(nbr_in_dark)
                bearing = bearing - math.pi
                bearing = math2.normalize_angle(bearing)
                beh_out = move_in_dir(bearing)

            if (not self_in_light()) and (not manual_control):
                dark_start_time = sys.time()
                state = STATE_DARK
            if manual_control:
                if rone.button_get_value('b'):
                    dark_start_time = sys.time()
                    state = STATE_DARK
                elif rone.button_get_value('r'):
                    state = STATE_IDLE

        elif state == STATE_DARK:
            leds.set_pattern('b', 'circle', LED_BRIGHTNESS)
            if manual_control:
                leds.set_pattern('br', 'group', LED_BRIGHTNESS)
            nbrs_in_light = get_nbrs_in_light()
            nbrs_in_dark = get_nbrs_in_dark()
            if len(nbrs_in_light) > 0:
                bearing = get_avg_bearing_to_nbrs(nbrs_in_light)
                beh_out = move_in_dir(bearing)
            elif len(nbrs_in_dark) > 0:
                bearing = get_avg_bearing_to_nbrs(nbrs_in_dark)
                beh_out = move_in_dir(bearing)

            if not manual_control:
                if self_in_light():
                    state = STATE_LIGHT
                elif sys.time() - dark_start_time > LIFESPAN:
                    score_time = hba.winter_time_keeper(initial_time)
                    hba.winter_score_calc(score_time, LED_BRIGHTNESS)
                    state = STATE_DEAD
            else:
                if rone.button_get_value('g'):
                    state = STATE_LIGHT
                elif rone.button_get_value('r'):
                    state = STATE_IDLE

        elif state == STATE_DEAD:
            pass

        # end of the FSM

##        bump_beh_out = beh.bump_beh(MOTION_TV)
##        beh_out = beh.subsume([beh_out, bump_beh_out])

        # set the beh velocities
        beh.motion_set(beh_out)

        #set the HBA message
        msg = [0, 0, 0]
        msg[MSG_IDX_STATE] = state
        hba.set_msg(msg[0], msg[1], msg[2])

# Helper functions

def get_nbrs_in_light():
    new_nbrs = 0
    nbr_list = hba.get_robot_neighbors()
    nbrs_in_light = []
    for nbr in nbr_list:
        state = hba.get_msg_from_nbr(nbr, new_nbrs)[MSG_IDX_STATE]
        if state == STATE_LIGHT:
            nbrs_in_light.append(nbr)
    return nbrs_in_light

def get_nbrs_in_dark():
    new_nbrs = 0
    nbr_list = hba.get_robot_neighbors()
    nbrs_in_dark = []
    for nbr in nbr_list:
        state = hba.get_msg_from_nbr(nbr, new_nbrs)[MSG_IDX_STATE]
        if state == STATE_DARK:
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

def get_nearest_nbr_in_dark(nbr_list):
    nbrs_in_dark = get_nbrs_in_dark()
    nearest = None
    if len(nbrs_in_dark) > 0:
        nearest = nbrs_in_dark[0]
        for nbr in nbrs_in_dark:
            if neighbors.get_nbr_range_bits(nbr) > neighbors.get_nbr_range_bits(nearest):
                nearest = nbr
##        if neighbors.get_nbr_range_bits(nearest) < CLOSENESS_CONSTANT:
##            nearest = None
    return nearest

def move_in_dir(bearing):
    bearing = math2.normalize_angle(bearing)
    tv = math.cos(bearing) * MOTION_TV
    rv = math.sin(bearing) * MOTION_RV
    if abs(bearing) > math.pi / 2:
        rv = -rv
    tv, rv = int(tv), int(rv)
    return tv, rv

def self_in_light():
    any_in_light = False
    for sensor_dir in BRIGHTNESS_THRESHOLDS.keys():
        this_one_in_light = (rone.light_sensor_get_value(sensor_dir) > BRIGHTNESS_THRESHOLDS[sensor_dir])
        any_in_light = any_in_light or this_one_in_light
    return any_in_light

# Start!

winter()
