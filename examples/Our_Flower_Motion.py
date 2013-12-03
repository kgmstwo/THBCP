import rone, sys, math, neighbors, velocity, pose, leds, math2, motion, hba

# System parameters - don't edit
NBR_PERIOD = 330

# Basic motion parameters - change carefully
MOTION_RV = int(1000 * math.pi * 0.3)
MOTION_TV = 100
ROTATE_RV_GAIN = 900
RV_FOLLOW_GAIN = 1.2

# FSM States
STATE_IDLE = 0
STATE_MOVE_TO_FLOWER = 2
STATE_COLLECT_POLLEN = 3
STATE_MOVE_AWAY_FLOWER = 4

# Other constants
LED_BRIGHTNESS = 40
COLLECT_POLLEN_TIME = 10000

def flower_motion():
    velocity.init(0.22, 40, 0.5, 0.1)
    leds.init()
    pose.init()
    motion.init()
    neighbors.init(NBR_PERIOD)

    state = STATE_IDLE

    while True:
        # Do updates
        leds.update()
        pose.update()
        velocity.update()
        new_nbrs = neighbors.update()
        
        nbrList = neighbors.get_neighbors()
        if new_nbrs:
            print nbrList
        tv = 0
        rv = 0

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
           
            # Move towards the flower for 10 seconds
            # for this demo, assume the first robot on the list is a flower
            flower = nbrList_getFirstRobot(nbrList)
            if flower != None:
                # Stop if we get close or bump into the flower
                #if neighbors.get_nbr_close_range(flower):
                if (neighbors.get_nbr_range_bits(flower) > 4) or (hba.bump_angle_get() != None):
                    state = STATE_BACK_UP
                    back_up_start_time = sys.time()
                else:
                    # Move to the flower
                    (tv, rv) = follow_nbr(flower)

        elif state == STATE_BACK_UP:
            
            
            tv = -MOTION_TV
                
            if sys.time() > (collect_pollen_start_time + COLLECT_POLLEN_TIME): 
                state = STATE_COLLECT_POLLEN
                collect_pollen_start_time = sys.time()
            
        
        elif state == STATE_COLLECT_POLLEN:
            # this is where you will put your clever pollen collection code
            
            rv = 
            tv = MOTION_TV
            # we will just wait for a second, then leave. (this will not collect very much pollen)
            if new_nbrs:
                print "collect"
            leds.set_pattern('g', 'blink_fast', LED_BRIGHTNESS)
            
            # Timeout after 1 second
            if sys.time() > (collect_pollen_start_time + COLLECT_POLLEN_TIME):
                state = STATE_MOVE_AWAY_FLOWER

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
                (tv, rv) = avoid_nbr(flower)
            else:
                state = STATE_IDLE
                
        # end of the FSM
                        
        # set the velocities
        velocity.set_tvrv(tv, rv)
        
        #set the message
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
                

def follow_nbr(nbr):
    tv = 0
    rv = 0
    if nbr != None:
        rv = neighbors.get_nbr_bearing(nbr) * MOTION_RV * RV_FOLLOW_GAIN
        tv = MOTION_TV
    return (tv, rv)
    

def avoid_nbr(nbr):
    tv = 0
    rv = 0
    if nbr != None:
        bearing = math2.normalize_angle(neighbors.get_nbr_bearing(nbr) + math.pi)
        rv = bearing * MOTION_RV * RV_FOLLOW_GAIN
        tv = MOTION_TV
    return (tv, rv)
    

flower_motion()
