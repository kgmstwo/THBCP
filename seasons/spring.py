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
Found_Tree = False

# Other constants
LED_BRIGHTNESS = 40
MOVE_TO_TOWER_DISTANCE = 3000


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
            if tree_detect() == True:
                state = STATE_MOVE_TO_TREE
            else:
                #right now just runs forward and sideways, get fancy?
                beh_out = beh.tvrv(MOTION_TV, MOTION_RV)
        elif state == STATE_MOVE_TO_TREE:
            if not beh.bump_angle_get() == None:
                motion_start_odo = pose.get_odometer()
                state = STATE_RETURN
            else:
                (tv, rv) = go_to_tree(diff_start)
                beh_out = beh.tvrv(tv, rv)
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
            if rone.bump_sensors_get_value(1) == 1:
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
            for nbr in nbrList:
                msg = neighbors.get_nbr_message(nbr)
                if 'LEADER' in msg:
                    state = STATE_FOLLOW
                else:
                    neighbors.set_message('LEADER')
                    
        elif state == STATE_FOLLOW: 
            
            nbr_bearing = neighbors.get_nbr_bearing(nbr)
            nbr_orientation = neighbors.get_nbr_orientation(nbr)
            nbr_heading = math2.normalize_angle(math.pi + nbr_bearing - nbr_orientation) + math.pi 
            
            if nbr_heading > 2: 
                beh_out = beh.tvrv(0, MOTION_RV)
            
            elif nbr_heading < 2:
                beh_out = beh.tvrv(0,-MOTION_RV)
            
            else:
                beh_out = beh.tvrv(MOTION_TV,0)
                
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

def go_to_tree(diff_start):
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

def tree_detect(diff_start):
    tree = False
    light = light_diff() - diff_start
    if light > 50:
        tree = True
    elif light < -50:
        tree = True
    else:
        tree = False
    return tree

# Start!
spring()
