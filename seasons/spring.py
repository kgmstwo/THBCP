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
STATE_WANDER = 1 
STATE_MOVE_TO_TREE = 2
STATE_RETURN = 3
STATE_RECRUIT = 4
STATE_FOLLOW = 5

#global data
Found_Tree = FALSE

# Other constants
LED_BRIGHTNESS = 40

def spring():
    beh.init(0.22, 40, 0.5, 0.1)

    state = STATE_IDLE
    diff_start = light_diff()
    while True:
        # run the system updates
        new_nbrs = beh.update()
        
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
            if tree_detect():
                state = STATE_MOVE_TO_TREE
            else:
                #right now just runs forward, get fancy?
                beh_out = beh.tvrv(MOTION_TV, MOTION_RV)
        elif state == STATE_MOVE_TO_TREE:
            if not beh.bump_angle_get() == None:
                motion_start_odo = pose.get_odometer()
                state = STATE_RETURN
            else:
                (tv, rv) = go_to_tree()
                beh_out = beh.tvrv(tv, rv)
                pass
        elif state == STATE_RETURN:
            nav_tower = hba.find_nav_tower_nbr(127)
            new_nbrs = beh.update()
            # Move towards the nav_tower until turning around distance reached
            if nav_tower != None:      # move forward
                beh_out = beh.follow_nbr(nav_tower, MOTION_TV)
                leds.set_pattern('g', 'blink_fast', LED_BRIGHTNESS)
            else:
                leds.set_pattern('g', 'circle', LED_BRIGHTNESS)
                beh_out = beh.follow_nbr(nav_tower, MOTION_TV)  
            distance_to_go = (motion_start_odo + MOVE_TO_TOWER_DISTANCE) - pose.get_odometer()
            beh.motion_set(beh_out)
            if bump_sensors_get_value(1) == 1
                if Found_Tree:
                    state = STATE_RECRUIT
                else:
                    state = STATE_FOLLOW   
            if distance_to_go < 0:    
                if Found_Tree:
                    state = STATE_RECRUIT
                else:
                    state = STATE_FOLLOW   
            
        elif state == STATE_RECRUIT:
            pass
        elif state == STATE_FOLLOW:
            pass
        # end of the FSM
        bump_beh_out = beh.bump_beh(MOTION_TV)
        if state != STATE_RETURN:
            beh_out = beh.subsume([beh_out, bump_beh_out])

        # set the beh velocities
        beh.motion_set(beh_out)

        #set the HBA message
        hba.set_msg(state, 0, 0)

# Helper functions
def light_diff():
    lightdiff = rone.light_sensor_get_value('fl')-rone.light_sensor_get_value('fr')
    return lightdiff #positive = right, negative = left

def light_diff():
    return rone.light_sensor_get_value('fl')+rone.light_sensor_get_value('fr')

def go_to_tree():
    diff = light_diff() - diff_start
    if diff > 50:
        rv = MOTION_RV
        tv = 0
    elif diff < -50:
        rv = -MOTION_RV
        tv = 0
    else:
        tv = MOTION_TV
        rv = 0
    return (tv,rv)


# Start!
spring()
