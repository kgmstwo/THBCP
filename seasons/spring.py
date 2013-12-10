import rone, sys, math, math2, velocity, pose, motion, leds, neighbors, beh, hba

###########################################################
##
##  Spring
##
###########################################################
#
# This is the code for the Spring season.

# Basic motion parameters - change carefully
MOTION_RV = int(1000 * math.pi * 0.3)
MOTION_TV = 100

# FSM States
STATE_IDLE = 0
STATE_QUEEN = 1
STATE_WANDER = 2
STATE_RETURN = 3
STATE_RECRUIT = 4
STATE_FOLLOW = 5
STATE_LEAD = 6
STATE_SUCCESS = 7

# MSG indexes
MSG_IDX_STATE = 0

# Other constants
LED_BRIGHTNESS = 40
RANGE_BITS_CLOSE = 3
NAV_ID = 125



def spring():
    found_tree = False

    beh.init(0.22, 40, 0.5, 0.1)

    state = STATE_IDLE
    
    while True:
        # run the system updates
        new_nbrs = beh.update()
        nbr_list = neighbors.get_neighbors()
        if new_nbrs:
            print nbr_list
        beh_out = beh.BEH_INACTIVE
            
        # this is the main finite-state machine
        if state == STATE_IDLE:
            leds.set_pattern('rb', 'circle', LED_BRIGHTNESS)
            if new_nbrs:
                print "idle"
                
            if rone.button_get_value('r'):
                state = STATE_WANDER
            elif rone.button_get_value('b'):
                state = STATE_QUEEN
                
        elif state == STATE_WANDER:
            nav = hba.find_nav_tower_nbr(NAV_ID)
            beh_out = beh.avoid_nbr(nav, MOTION_TV)
            
            if bump_front():
                found_tree = True
                state = STATE_RETURN
            elif nav is None:
                state = STATE_RETURN
       
        elif state == STATE_RETURN:
            nav_tower = hba.find_nav_tower_nbr(NAV_ID)
            queen = find_queen(nbr_list)
            if nav_tower is None:
                beh_out = (-MOTION_TV, 0, True)
            elif queen is None:
                beh_out = beh.follow_nbr(nav_tower)
            elif not close_to_nbr(queen):
                beh_out = beh.follow_nbr(queen, MOTION_TV)
            elif found_tree:
                start_time = sys.time()
                state = STATE_RECRUIT
            else:
                start_time = sys.time()
                state = STATE_FOLLOW
            
        elif state == STATE_RECRUIT:
            for nbr in hba.get_robot_neighbors():
                msg = neighbors.get_nbr_message(nbr)
                print type(msg)
                print msg
                if 'LEADER' in msg:
                    state = STATE_FOLLOW
                else:
                    neighbors.set_message('LEADER')
                    
        elif state == STATE_FOLLOW: 
            for nbr in nbrList:
                msg = neighbors.get_nbr_message(nbr)
                if 'LEADER' in msg:
                    nbr = LEADER
            nbr_bearing = neighbors.get_nbr_bearing(LEADER)
            nbr_orientation = neighbors.get_nbr_orientation(LEADER)
            nbr_heading = math2.normalize_angle(math.pi + nbr_bearing - nbr_orientation) + math.pi 
            
            if nbr_heading > 2: 
                beh_out = beh.tvrv(0, MOTION_RV)
            
            elif nbr_heading < 2:
                beh_out = beh.tvrv(0,-MOTION_RV)
            
            else:
                beh_out = beh.tvrv(MOTION_TV,0)
                
        # end of the FSM
        if state not in [STATE_RETURN, STATE_RECRUIT]:
            bump_beh_out = beh.bump_beh(MOTION_TV)
            beh_out = beh.subsume([beh_out, bump_beh_out])

        # set the beh velocities
        beh.motion_set(beh_out)

        #set the HBA message
        msg = [0, 0, 0]
        msg[MSG_IDX_STATE] = state
        hba.set_msg(msg[0], msg[1], msg[2])

# Helper functions
def light_diff():
    diff = rone.light_sensor_get_value('fl')-rone.light_sensor_get_value('fr')
    return diff #positive = right, negative = left

##def go_to_tree(diff_start):
##    diff = light_diff() - diff_start
##    if diff > 50:
##        rv = MOTION_RV
##        tv = 0
##    elif diff < -50:
##        rv = -MOTION_RV
##        tv = 0
##    else:
##        tv = MOTION_TV
##        rv = 0
##    return (tv,rv)
##
##def tree_detect(diff_start):
##    tree = False
##    if rone.bump_sensors_get_value(1) == 1:
##        tree = True

def bump_front():
    # Returns true if any of the front 6 out of 8 sensors are triggered
    bump_bits = rone.bump_sensors_get_value()
    return (bump_bits & 231) > 0

def close_to_nbr(nbr):
    return neighbors.get_nbr_range_bits(nbr) >= RANGE_BITS_CLOSE

# Start!
spring()
