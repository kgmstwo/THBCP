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
        if state == STATE_IDLE:
            leds.set_pattern('r', 'circle', LED_BRIGHTNESS)
            if rone.button_get_value('r'):
                state = STATE_MOVE_TO_FLOWER
            if rone.button_get_value('b') # for the robee that stays at the nav tower
                while True
                hba.set_msg(0,0,'queen')
            if new_nbrs:
                print "idle"
                
        elif state == STATE_WANDER: #run forward, avoid direction of neighbors
            nav_tower = hba.find_nav_tower_nbr(127)
            beh_out = beh.avoid_nbr(nav_tower, MOTION_TV) # possible state head out?
            for nbr in nbrList:
                beh_out = beh.avoid_nbr(nbr, MOTION_TV)
                
        elif state == STATE_MOVE_TO_FLOWER:
            leds.set_pattern('b', 'ramp_slow', LED_BRIGHTNESS)
            if new_nbrs:
                (color,nbr) = detflower(nbrList)
                flower = nbr
            if flower != None and color == 'blue': # CHANGE THIS COLOR TO WHATEVER
                # Stop if we get close or bump into the flower
                if (neighbors.get_nbr_range_bits(flower) > 6) or (beh.bump_angle_get() != None):
                    state = STATE_COLLECT_POLLEN
                    collect_pollen_start_time = sys.time()
                else:
                    # Move to the flower
                    beh_out = beh.follow_nbr(flower, MOTION_TV)
                    
        elif state = STATE_COLLECT_POLLEN:
            motion_start_odo = pose.get_odometer()
            if sys.time() > (collect_pollen_start_time + COLLECT_POLLEN_TIME):
                state = STATE_RETURN_TO_BASE
            elif sys.time() < (collect_pollen_start_time + TURN_TIME):    
                tv = 0
                rv = 60
                beh_out = beh.tvrv(tv,rv)
            else: 
                tv = MOTION_TV
                rv = MOTION_RV
                beh_out = beh.tvrv(tv,rv)
        
        elif state == STATE_RETURN_TO_BASE:
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

def detflower(nbrList):
    for nbr in nbrList:
        color = None
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

fall()
