import rone, sys, math, neighbors, velocity, pose, leds, math2, motion, beh, hba

# System parameters - don't edit
NBR_PERIOD = 330

# Basic motion parameters - change carefully
MOTION_RV = int(1000 * math.pi * 0.3)
MOTION_TV = 100
ROTATE_RV_GAIN = 900
RV_FOLLOW_GAIN = 1.2

# FSM States
STATE_IDLE = 0
STATE_MATCH_HEADING = 2
STATE_MOVE_TO_FLOWER = 3

# Other constants - change these as you wish
LED_BRIGHTNESS = 40
MOVE_TO_FLOWER_DISTANCE = 500
HEADING_ERROR_LIMIT = math.pi / 10

def recruit_demo():
    beh.init(0.22, 40, 0.5, 0.1)

    state = STATE_IDLE
    
    while True:
        # run the system updates
        new_nbrs = beh.update()
        
        nbrList = neighbors.get_neighbors()
        beh_out = beh.BEH_INACTIVE

        # this is the main finite-state machine
        if state == STATE_IDLE:
            leds.set_pattern('r', 'circle', LED_BRIGHTNESS)
            if rone.button_get_value('r'):
                state = STATE_MATCH_HEADING
                error_list = []
            if new_nbrs:
                print "idle"
            
        elif state == STATE_MATCH_HEADING:
            leds.set_pattern('b', 'ramp_slow', LED_BRIGHTNESS)
            if new_nbrs:
                print "match heading"
           
            # match the heading with the first robot on your nbr list
            # use the average_error_check() function to wait until you get a accurate match
            # for this demo, assume the first robot on the list is a dancing_nbr that is recruiting you to a flower
            dancing_nbr = nbrList_getFirstRobot(nbrList)
            if dancing_nbr != None:
                # rotate to face the "dancing" robot
                tv = 0
                (rv, heading_error) = match_nbr_heading(dancing_nbr)
                beh_out = beh.tvrv(tv, rv)
                small_error = hba.average_error_check(heading_error, error_list, HEADING_ERROR_LIMIT, new_nbrs)
                if new_nbrs:
                    print "error", error_list
                if small_error:
                    # We have a good heading match.  Go get pollen!
                    state = STATE_MOVE_TO_FLOWER
                    collect_pollen_start_odo = pose.get_odometer()

        elif state == STATE_MOVE_TO_FLOWER:
            # move forward
            beh_out = beh.tvrv(MOTION_TV, 0)
            
            # stop after a fixed distance_to_go
            distance_to_go = (collect_pollen_start_odo + MOVE_TO_FLOWER_DISTANCE) - pose.get_odometer() 
            if distance_to_go < 0:
                state = STATE_IDLE

            if new_nbrs:
                print "move to flower dist:", distance_to_go
            leds.set_pattern('g', 'blink_fast', LED_BRIGHTNESS)

        # end of the FSM
                        
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

def match_nbr_heading(nbr):
    nbr_brg = neighbors.get_nbr_bearing(nbr)
    nbr_ornt = neighbors.get_nbr_orientation(nbr)
    heading_error = math2.normalize_angle(math.pi + nbr_brg - nbr_ornt)  
    rv = ROTATE_RV_GAIN * heading_error
    return (rv, heading_error)


recruit_demo()
