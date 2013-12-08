import rone, sys, math, math2, velocity, pose, motion, leds, neighbors, beh, hba

###########################################################
##
##  Fall
##
###########################################################
#
# This is the code for the Fall season.


# Basic motion parameters - change carefully
MOTION_RV = int(1000 * math.pi * 0.3)
MOTION_TV = 100

# FSM States
STATE_IDLE = 0
STATE_WANDER = 1
STATE_MOVE_TO_FLOWER = 2
STATE_COLLECT_POLLEN = 3
STATE_RETURN_TO_BASE = 4
STATE_RETURN_TO_FLOWER = 5

# Global Data
Found_Flower = False

# Other constants
LED_BRIGHTNESS = 40
COLLECT_POLLEN_TIME = 3000
#these are the time to wait at base before heading out again
RECRUIT_TIME = 10 * 1000
FOLLOW_TIME = 10 * 1000

def fall():
    beh.init(0.22, 40, 0.5, 0.1)
    state = STATE_IDLE
    motion_start_odo = pose.get_odometer()

    while True:
        new_nbrs = beh.update()
        
        nbrList = neighbors.get_neighbors()
        if new_nbrs:
            print nbrList
        beh_out = beh.BEH_INACTIVE

        #FINITE STATE MACHINE
        if state = STATE_IDLE:
            leds.set_pattern('r', 'circle', LED_BRIGHTNESS)
            if rone.button_get_value('r'):
                state = STATE_MOVE_TO_FLOWER
            if new_nbrs:
                print "idle"
        elif state = STATE_WANDER:
            #run forward, avoid direction of neighbors
            pass
        elif state = STATE_MOVE_TO_FLOWER:
            pass
        elif state = STATE_COLLECT_POLLEN:
            motion_start_odo = pose.get_odometer()
            state = STATE_RETURN_TO_BASE
            pass
        elif state = STATE_RETURN_TO_BASE:
            nav_tower = hba.find_nav_tower_nbr(127)
            
            if state == STATE_MOVE_TO_TOWER:
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
                if distance_to_go < 0:    
                    if Found_Flower:
                        state = STATE_RETURN_TO_FLOWER
                    else:
                        state = STATE_WANDER   
        elif state = STATE_RETURN_TO_FLOWER:
            pass
        #END OF FINITE STATE MACHINE 

        bump_beh_out = beh.bump_meh(MOTION_TV)
        beh_out = beh.subsume([beh_out, bump_beh_out])
        beh.motion_set(beh_out)
        hba.set_msg(0, 0, 0)

fall()
