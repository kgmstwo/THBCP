import rone, sys, math, math2, velocity, pose, motion, leds, neighbors, beh, hba

# Basic motion parameters - change carefully
#MOTION_RV = int(1000 * math.pi * 0.3)
MOTION_TV = 100

# FSM States
STATE_LEADER = 0
STATE_MINION = 1

# Other constants
LED_BRIGHTNESS = 40
LED_BRIGHTNESSER = 60


def check_button(color, count, button_old):
    button = rone.button_get_value(color)
    # look for a rising edge on the button press
    if (button == 1) and (button_old == 0):
        if count == 5:
            count = 0
        else:
            count = count + 1
        #the button was just pressed.  wait a bit for debounce
        sys.sleep(3)
    return (count, button)


def message_demo():
    beh.init(0.22, 40, 0.5, 0.1)

    state = STATE_LEADER

    num_r = 1
    num_g = 1
    num_b = 1
    button_red_old = 0
    button_green_old = 0
    button_blue_old = 0
    
    while True:
        # run the system updates
        new_nbrs = beh.update()
        nbr_list = neighbors.get_neighbors()

        # init the velocities
        beh_out = beh.BEH_INACTIVE

        # read the buttons and update the local counts
        (num_r, button_red_old) = check_button('r', num_r, button_red_old)
        (num_g, button_green_old) = check_button('g', num_g, button_green_old)
        (num_b, button_blue_old) = check_button('b', num_b, button_blue_old)

        # build this robot's neighbor message
        distance_local = num_r
        mode_local = num_g
        quality_local = num_b
        hba.set_msg(distance_local, mode_local, quality_local)
        
        # find the neighbor with the lowest ID
        nbr_leader = nbrList_getRobotWithLowID(nbr_list)
        
        # are you the leader?
        if nbr_leader == None:
            # no neighbors.  you are the leader
            state = STATE_LEADER
        else:
            low_ID = neighbors.get_nbr_id(nbr_leader)
#            if new_nbrs:
#                print 'nbr lowID=',nbr_leader
            if low_ID < rone.get_id():
                state = STATE_MINION
            else:
                state = STATE_LEADER
        
        # this is the main finite-state machine
        if state == STATE_LEADER:
            msg = (distance_local, mode_local, quality_local)
            if new_nbrs:
                print "leader:", msg
            brightness = LED_BRIGHTNESSER
            
        elif state == STATE_MINION:
            if nbr_leader != None:
                msg = hba.get_msg_from_nbr(nbr_leader, new_nbrs)
                if new_nbrs:
                    print "minion. leader = ",str(low_ID)," msg=",msg
            brightness = LED_BRIGHTNESS
            
                
        # end of the FSM
        # unpack the message and display it on the LEDs
        (distance, mode, quality) = msg
        leds.set_pattern((distance, mode, quality), 'count', brightness)
                        
        # set the velocities
        beh.motion_set(beh_out)
        

def nbrList_getRobotWithLowID(nbrList):
    nbr_lowID = None
    lowID = 1000
    for nbr in nbrList:
        if neighbors.get_nbr_id(nbr) < lowID:
            lowID = neighbors.get_nbr_id(nbr)
            nbr_lowID = nbr
    return nbr_lowID
                

message_demo()
