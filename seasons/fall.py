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
STATE_RETURN_TO_BASE_FOLLOW = 4
STATE_RETURN_TO_BASE_RECRUIT = 5

# Other constants
LED_BRIGHTNESS = 40
COLLECT_POLLEN_TIME = 3000
#these are the time to wait at base before heading out again
RECRUIT_TIME = 10 * 1000
FOLLOW_TIME = 10 * 1000

def fall():

    state = STATE_IDLE

    while True:
        new_nbrs = beh.update()
        
        nbrList = neighbors.get_neighbors()
        if new_nbrs:
            print nbrList
        beh_out = beh.BEH_INACTIVE

        #FINITE STATE MACHINE OF DOOM
        if state = STATE_IDLE:
            leds.set_pattern('r', 'circle', LED_BRIGHTNESS)
            if rone.button_get_value('r'):
                state = STATE_MOVE_TO_FLOWER
            if new_nbrs:
                print "idle"
        elif state = STATE_WANDER:
            pass
        elif state = STATE_MOVE_TO_FLOWER:
            pass
        elif state = STATE_COLLECT_POLLEN:
            pass
        elif state = STATE_RETURN_TO_BASE_RECRUIT:
            pass
        elif state = STATE_RETURN_TO_BASE_RECRUIT:
            pass
        #END OF FINITE STATE MACHINE OF DOOM

fall()
