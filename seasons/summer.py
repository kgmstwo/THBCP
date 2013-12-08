import rone, sys, math, math2, velocity, pose, motion, leds, neighbors, beh, hba

###########################################################
##
##  Summer
##
###########################################################
#
# This is the code for the Summer season.


# Basic motion parameters - change carefully
MOTION_RV = int(1000 * math.pi * 0.3)
MOTION_TV = 100

# FSM States
STATE_IDLE = 0
STATE_FIND_QUEEN = 1
STATE_BUMP_QUEEN = 2
STATE_RETURN = 3
STATE_QUEEN = 4

# MSG items
MSG_STATE = 0

# Other constants
LED_BRIGHTNESS = 40

def summer():
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
            leds.set_pattern('b', 'circle', LED_BRIGHTNESS)
            if rone.button_get_value('r'):
                state = STATE_FIND_QUEEN
            if rone.button_get_value('b'):
                state = STATE_QUEEN
            if new_nbrs:
                print "idle"
        elif state == STATE_FIND_QUEEN:
            leds.set_pattern('r', 'ramp_slow', LED_BRIGHTNESS)
            beh_out = (MOTION_TV, 0, True)
            queen = get_queen()
            if not queen == None:
                state = STATE_BUMP_QUEEN
            else:
                #go straight and hope for the best
                beh_out = beh.tvrv(MOTION_TV, MOTION_RV)
        elif state == STATE_BUMP_QUEEN:
            leds.set_pattern('r', 'ramp_slow', LED_BRIGHTNESS)
            queen = get_queen()
            if queen == None:
                state = STATE_RETURN
            else:
                if (neighbors.get_nbr_range_bits(queen) > 6) or (beh.bump_angle_get() != None):
                    state = STATE_RETURN
                else:
                    beh_out = beh.follow_nbr(queen, MOTION_TV)
        elif state == STATE_RETURN:
            leds.set_pattern('r', 'circle', LED_BRIGHTNESS)
            if rone.button_get_value('r'):
                state = STATE_FIND_QUEEN

            queen = get_queen()
            if queen == None:
                state = STATE_IDLE
            else:
                beh_out = beh.avoid_nbr(queen, MOTION_TV)
        elif state == STATE_QUEEN:
            if new_nbrs:
                print 'Ich bin die Koenigin!'

        # end of the FSM
        bump_beh_out = beh.bump_beh(MOTION_TV)
        beh_out = beh.subsume([beh_out, bump_beh_out])


        # set the beh velocities
        beh.motion_set(beh_out)

        #set the HBA message
        hba.set_msg(state, 0, 0)

# Helper functions
def get_queen():
    nbr_list = hba.get_robot_neighbors()
    new_nbrs = 0
    for nbr in nbr_list:
        is_queen = hba.get_msg_from_nbr(nbr, new_nbrs)[MSG_STATE] == STATE_QUEEN
        if is_queen:
            return nbr
    return None


# Start!

summer()
