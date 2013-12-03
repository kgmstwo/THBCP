import rone, sys, math, math2, velocity, leds, neighbors, beh, hba
#Dvjn
FTL_TV = 100
FTL_RV = math.pi*1000/2
LED_BRIGHTNESS = 40
REMOTE_XMIT_DELAY = 50

MODE_IDLE = 0
MODE_REMOTE = 1
MODE_FLOCK = 2




def remote_motion_controller(radio_msg):
    # args: radio_msg, a string of the radio message from the remote robot
    # returns a tuple of (tv, rv) to make te leader drive around
    tv = 0 # placeholder code
    rv = 0 # placeholder code
    #print 'radio msg:', radio_msg  # debugging code - comment out when this is working

    if 'g' in radio_msg:
        # move forward
        tv = FTL_TV
    if 'r' in radio_msg:
        # rotate left when red button pressed
        rv = FTL_RV
    elif 'b' in radio_msg:
        # rotate right when blue button pressed
        rv = -FTL_RV
    return (tv, rv)



receive_msg_time = sys.time()
receive_msg_tv = 0
receive_msg_rv = 0

def receive_beh():
    global receive_msg_time, receive_msg_tv, receive_msg_rv
    radio_msg = rone.radio_get_message()
    active = True
    if radio_msg != None:
        # we have a message! put lights in active mode
        leds.set_pattern('g','blink_fast',LED_BRIGHTNESS)
        receive_msg_time = sys.time()
        (receive_msg_tv, receive_msg_rv) = remote_motion_controller(radio_msg)
    else:
        # no message. check for radio message timeout
        if sys.time() > (receive_msg_time + REMOTE_XMIT_DELAY * 3):
            # message timeout.  stop the motors
            receive_msg_tv = 0
            receive_msg_rv = 0
            active = False
        leds.set_pattern('g','circle',LED_BRIGHTNESS)
    return (receive_msg_tv, receive_msg_rv, active)


FLOCK_RV_GAIN = 900
def flock_beh():
    # act on the information from the message.  Note that this might be 
    # information stored from the last message we received, because message 
    # information remains active for a while
    nbr_list = neighbors.get_neighbors()
    if len(nbr_list) > 0:
        # any neighbor wil do.  get the first neighbor
        x = 0.0
        y = 0.0
        for nbr in nbr_list:
            nbr_bearing = neighbors.get_nbr_bearing(nbr)
            nbr_orientation = neighbors.get_nbr_orientation(nbr)
            nbr_heading = math2.normalize_angle(math.pi + nbr_bearing - nbr_orientation)
            x += math.cos(nbr_heading)
            y += math.sin(nbr_heading)
        nbr_heading_avg = math.atan2(y, x)
        beh = (0, FLOCK_RV_GAIN * nbr_heading_avg, True)
    else:
        #no neighbors. do nothing
        beh = (0, 0, False)
    return beh


def flock_demo():
    mode = MODE_IDLE

    beh.init(0.22, 40, 0.5, 0.1)

    buttons = hba.wait_for_button()
    
    if 'r' in buttons:
        mode = MODE_REMOTE
    elif 'g' in buttons:
        mode = MODE_FLOCK
    
    # Now that you know your mode, run the main loop
    while True:
        # run the system updates
        new_nbrs = beh.update()

        beh_out = beh.BEH_INACTIVE
        if mode == MODE_REMOTE:                
            buttons = beh.check_buttons()
        
            if buttons != '':
                leds.set_pattern('r','blink_fast',LED_BRIGHTNESS)
                rone.radio_send_message(buttons)
            else:
                leds.set_pattern('r','circle',LED_BRIGHTNESS)
            #sleep for a bit to avoid continuous radio transmission
            sys.sleep(REMOTE_XMIT_DELAY)
        elif mode == MODE_FLOCK:
            bump_beh_out = beh.bump_beh(FTL_TV)
            receive_beh_out = receive_beh()
            flock_beh_out = flock_beh()
        
#            if beh.get_active(bump_beh_out):
#                tv = beh.get_tv(bump_beh_out)
#                rv = beh.get_rv(bump_beh_out)
#            else:
#                tv = beh.get_tv(receive_beh_out) + beh.get_tv(flock_beh_out)
#                rv = beh.get_rv(receive_beh_out) + beh.get_rv(flock_beh_out)

            flock_beh_out = beh.sum(flock_beh_out, receive_beh_out)
            beh_out = beh.subsume([flock_beh_out, bump_beh_out])
            
        # set the beh velocities
        beh.motion_set(beh_out)

        #set the HBA message
        hba.set_msg(0, 0, 0)
            

flock_demo()
