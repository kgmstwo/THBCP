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

# Other constants
LED_BRIGHTNESS = 40

def Spring():
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

        # end of the FSM
        bump_beh_out = beh.bump_beh(MOTION_TV)

        beh_out = beh.subsume([beh_out, bump_beh_out])

        # set the beh velocities
        beh.motion_set(beh_out)

        #set the HBA message
        hba.set_msg(0, 0, 0)

# Helper functions



# Start!

Spring()
