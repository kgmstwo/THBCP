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
RANGE_BITS_CLOSE = 2



def spring():
    tree_found = False

    beh.init(0.22, 40, 0.5, 0.1)

    state = STATE_IDLE
    
    while True:
        # run the system updates
        new_nbrs = beh.update()
        diff_start = light_diff()
        nbrList = neighbors.get_neighbors()
        if new_nbrs:
            print nbrList
        beh_out = beh.BEH_INACTIVE
            
        # this is the main finite-state machine
        if state == STATE_IDLE:
            leds.set_pattern('r', 'circle', LED_BRIGHTNESS)
            if rone.button_get_value('r'):
                state = STATE_WANDER
            if new_nbrs:
                print "idle"
                
        elif state == STATE_WANDER:
            if tree_detect(diff_start) == True:
                Found_Tree = True
                state = STATE_RETURN
            elif pose.get_odometer() > WANDER:
                state = STATE_RETURN
            else:
                nav = hba.find_nav_tower_nbr(125)
                beh_out = beh.avoid_nbr(nav, MOTION_TV) # avoid navtower
                    # i tried.
       
        elif state == STATE_RETURN:
            nav_tower = hba.find_nav_tower_nbr(124)
            # Move towards the nav_tower until turning around distance reached
            if nav_tower != None:      # move forward
                beh_out = beh.follow_nbr(nav_tower, MOTION_TV)
                leds.set_pattern('g', 'blink_fast', LED_BRIGHTNESS)
            else:
                leds.set_pattern('g', 'circle', LED_BRIGHTNESS)
                beh_out = beh.follow_nbr(nav_tower, MOTION_TV)  
            distance_to_go = (motion_start_odo + MOVE_TO_TOWER_DISTANCE) - pose.get_odometer()
            beh.motion_set(beh_out)
            if rone.bump_sensors_get_value(1) == 1:
                if Found_Tree == True:
                    state = STATE_RECRUIT
                else:
                    state = STATE_FOLLOW   
            if distance_to_go < 0:    
                if Found_Tree == True:
                    state = STATE_RECRUIT
                else:
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
            recruiter = None
            leader = None
            followers = 1
            for nbr in nbr_list:
                nbr_state = hba.get_msg_from_nbr(nbr,new_nbrs)[0]
                if nbr_state == STATE_RECRUIT:
                    recruiter = nbr
                elif nbr_state == STATE_LEAD:
                    leader = nbr
                elif nbr_state == STATE_FOLLOW or nbr_state == STATE_WANDER:
                    followers += 1
            if recruiter == None:
                if leader == None:
                    beh_out = BEH_INACTIVE
                    if follwers == 5:
                        state = STATE_WANDER
                else:
                    beh_out = beh.follow_nbr(leader)
            else:
                beh_out = BEH_INACTIVE

        elif state == STATE_LEAD:
            beh_out = beh.tvrv(-MOTION_TV, 0)
            if bump_front():
                state = STATE_SUCCESS
        elif state == STATE_SUCCESS:
            beh_out = beh.BEH_INACTIVE
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
    lightdiff = rone.light_sensor_get_value('fl')-rone.light_sensor_get_value('fr')
    return lightdiff #positive = right, negative = left

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
def tree_detect(diff_start):
    tree = False
    if rone.bump_sensors_get_value(1) == 1:
        tree = True

# Start!
spring()
