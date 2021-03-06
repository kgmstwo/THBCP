import rone
import sys
import math 
import math2 
import velocity 
import pose 
import motion 
import leds 
import neighbors 
import beh 
import hba

###########################################################
##
##  Flower Motion
##
###########################################################
#
# This demo demonstrates one possible behavior for Fall ro-bee behavior.
# When a ro-bee senses a flower it moves towards the flower rubs against it and turns
# around. Load Flower.py onto a robot and try it out.


# Basic motion parameters - change carefully
MOTION_RV = int(1000 * math.pi * 0.3)
MOTION_TV = 100

# FSM States
STATE_IDLE = 0
STATE_MOVE_TO_FLOWER = 2
STATE_COLLECT_POLLEN = 3
STATE_MOVE_AWAY_FLOWER = 4

# Other constants
LED_BRIGHTNESS = 40
COLLECT_POLLEN_TIME = 15000
BACK_UP_TIME = 1000
TURN_TIME = 1000


def flower_motion():
    beh.init(0.22, 40, 0.5, 0.1)

    state = STATE_IDLE

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
                state = STATE_MOVE_TO_FLOWER
            if new_nbrs:
                print "idle"
            
        elif state == STATE_MOVE_TO_FLOWER:
            leds.set_pattern('b', 'ramp_slow', LED_BRIGHTNESS)
            if new_nbrs:
                print "move to flower"
           
            # Move towards the flower until you bump into it
            # for this demo, assume the first robot on the list is a flower
##            flower = nbrList_getFirstRobot(nbrList)
            (color,nbr) = detflower(nbrList)
            flower = nbr
            if flower != None:
                # Stop if we get close or bump into the flower
                #if neighbors.get_nbr_close_range(flower):
                if (neighbors.get_nbr_range_bits(flower) > 6) or (beh.bump_angle_get() != None):
                    state = STATE_COLLECT_POLLEN
                    collect_pollen_start_time = sys.time()
                else:
                    # Move to the flower
                    beh_out = beh.follow_nbr(flower, MOTION_TV)
                    #print beh_out

        elif state == STATE_COLLECT_POLLEN:
            # this is where you will put your clever pollen collection code
            # we will just wait for a second, then leave. (this will not collect very much pollen)
            leds.set_pattern('g', 'blink_fast', LED_BRIGHTNESS)
            if new_nbrs:
                print "collect"
            
            # Timeout after 5 seconds
            if sys.time() > (collect_pollen_start_time + COLLECT_POLLEN_TIME):
                state = STATE_MOVE_AWAY_FLOWER
            
            elif sys.time() < (collect_pollen_start_time + BACK_UP_TIME):    
                tv = -MOTION_TV
                rv = 0
                beh_out = beh.tvrv(tv,rv) 
                turn_start_time = (collect_pollen_start_time + BACK_UP_TIME)
                
            elif sys.time() < (turn_start_time + TURN_TIME): 
                tv = 0
                rv = -MOTION_RV
            
            else: 
                tv = MOTION_TV
                rv = MOTION_RV
                beh_out = beh.tvrv(tv,rv)
            
        elif state == STATE_MOVE_AWAY_FLOWER:
            if new_nbrs:
                print "avoid flower"
            leds.set_pattern('r', 'blink_slow', LED_BRIGHTNESS)
            if rone.button_get_value('r'):
                state = STATE_MOVE_TO_FLOWER

            # Move away from the flower until it is out of range
            flower = nbrList_getFirstRobot(nbrList)
            if flower != None:
                # Point away the flower
                beh_out = beh.avoid_nbr(flower, MOTION_TV)
            else:
                state = STATE_IDLE
                
        # end of the FSM
        bump_beh_out = beh.bump_beh(MOTION_TV)

        if state != STATE_COLLECT_POLLEN:
            beh_out = beh.subsume([beh_out, bump_beh_out])

        # set the beh velocities
        beh.motion_set(beh_out)

        #set the HBA message
        hba.set_msg(0, 0, 0)

            
# helper functions
def nbrList_getFirstRobot(nbrList):
    if len(nbrList) > 0:
        return nbrList[0]
    else:
        return None

def nbrList_getRobotWithID(nbrList, nbrID):
    for nbr in nbrList:
        if neighbors.get_nbr_id(nbr) == nbrID:
            return nbr
    return None
                

def detflower(nbrList):
    for nbr in nbrList:
        (unimportant, stuff, colormsg) = hba.get_msg_from_nbr(nbr,0)
        if colormsg == 0:
            color = 'red'
        elif colormsg == 1:
            color = 'green'
        elif colormsg == 2:
            color = 'blue'
        if color != None:
            return (color, nbr)
    return (None, None)

flower_motion()
