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
STATE_FLOWER = 0 # don't actually use this
STATE_IDLE = 1
STATE_WANDER = 2
STATE_MOVE_TO_FLOWER = 3
STATE_COLLECT_POLLEN = 4
STATE_RETURN_TO_BASE = 5
STATE_FOLLOW = 6
STATE_RECRUIT = 7
STATE_QUEEN = 8
STATE_ALIGN = 9
STATE_GO = 10

# MSG items
MSG_STATE = 0

# Other constants
LED_BRIGHTNESS = 40
COLORS = ['red','green','blue']

COLLECT_POLLEN_TIME = 3000
#these are the time to wait at base before heading out again
RECRUIT_TIME = 10 * 1000
FOLLOW_TIME = 10 * 1000
BACK_UP_TIME = 1000
TURN_TIME = 1700

queen_id = 17

def fall(): 
    Found_Flower = False
    start_time = 0
    def wander():
        state = STATE_WANDER
        ri = 0
    def collect_pollen():
        state = STATE_COLLECT_POLLEN
        start_time = sys.time()
    def follow():
        state = STATE_FOLLOW
        start_time = sys.time()
    def recruit():
        state = STATE_RECRUIT
        start_time = sys.time()
    def align_with(nbr):
        state = STATE_ALIGN
        start_time = sys.time()

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
            if rone.button_get_value('b'):
                state = STATE_QUEEN
            if new_nbrs:
                print "idle"

        elif state == STATE_WANDER: #run forward, avoid direction of neighbors
            nav_tower = hba.find_nav_tower_nbr(127)
            beh_out = beh.avoid_nbr(nav_tower, MOTION_TV)

            #This might do browninan motion
            beh_out = (beh_out[0], beh_out[1] + ri)
            #ri += random.random() * 2 - 1

            (flower, color) = detflower(nbrList)
            if flower != None:
                state = STATE_MOVE_TO_FLOWER

        elif state == STATE_MOVE_TO_FLOWER:
            leds.set_pattern('b', 'ramp_slow', LED_BRIGHTNESS)
            (flower, color) = detflower(nbrList)
            if (neighbors.get_nbr_range_bits(flower) > 6) or (beh.bump_angle_get() != None):
                #collect pollen if we bump or get close
                collect_pollen()
            else:
                #otherwise keep following that flower
                beh_out = beh.follow_nbr(flower, MOTION_TV)

        elif state == STATE_COLLECT_POLLEN:
            motion_start_odo = pose.get_odometer()
            if sys.time() > (collect_pollen_start_time + COLLECT_POLLEN_TIME):
                state = STATE_RETURN_TO_BASE
                Found_Flower = True
            elif sys.time() < (collect_pollen_start_time + BACK_UP_TIME):    
                tv = -MOTION_TV
                rv = 0
                beh_out = beh.tvrv(tv,rv) 
                turn_start_time = (collect_pollen_start_time + BACK_UP_TIME)

            elif sys.time() < (turn_start_time + TURN_TIME): 
                tv = 40
                rv = -MOTION_RV
                beh_out = beh.tvrv(tv,rv)

            else: 
                tv = MOTION_TV
                rv = (MOTION_RV - 300)
                beh_out = beh.tvrv(tv,rv)

        elif state == STATE_RETURN_TO_BASE:
            nav_tower = hba.find_nav_tower_nbr(127)
            queen = find_queen(nbrList)
            new_nbrs = beh.update()
            if nav_tower == None:
                wander() #we're in trouble
            else:
                if queen == None:
                    beh_out = beh.follow_nbr(nav_tower) #just follow the nav tower
                else:
                    if get_nbr_range_bits(queen) > 2:
                        beh_out = beh.follow_nbr(nav_tower, MOTION_TV) #get closer to the queen
                    else:
                        if Found_Flower:
                            recruit()
                        else:
                            follow()

        elif state == STATE_FOLLOW:
            recruiter = find_recruiter()
            if recruiter == None:
                beh_out = beh.BEH_INACTIVE
                if sys.time() > (follow_start_time + FOLLOW_TIME):
                    wander()
            else:
                align_with(recruiter)

        elif state == STATE_GO:
            flower = detflower()
            if not flower == None:
                state = STATE_MOVE_TO_FLOWER

        elif state == STATE_RECRUIT:
            if sys.time() > (recruit_start_time + RECRUIT_TIME):
                align_with(self) #how do you do this?

        elif state == STATE_ALIGN:
            if True: #aligned with recruiter
                state = STATE_GO
            else:
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


        #END OF FINITE STATE MACHINE 

        bump_beh_out = beh.bump_beh(MOTION_TV)
        if (state != STATE_RETURN_TO_BASE) or (state !=STATE_COLLECT_POLLEN):
            beh_out = beh.subsume([beh_out, bump_beh_out])
        beh.motion_set(beh_out)
        hba.set_msg(state, 0, 0)

def match_nbr_heading(nbr):
    nbr_brg = neighbors.get_nbr_bearing(nbr)
    nbr_ornt = neighbors.get_nbr_orientation(nbr)
    heading_error = math2.normalize_angle(math.pi + nbr_brg - nbr_ornt)  
    rv = ROTATE_RV_GAIN * heading_error
    return (rv, heading_error)

def find_recruiter(nbrList):
    for nbr in nbrList:
        state = hba.get_msg_from_nbr(nbr,new_nbrs)[MSG_STATE]
        if state == STATE_RECRUIT:
            return nbr
    return None

def find_queen(nbrList):
    for nbr in nbrList:
        if not neighbors.get_nbr_id(nbr) == None:
            return nbr
    return None

def detflower(nbrList):
    for nbr in nbrList:
        (state, asdf, color) = hba.get_msg_from_nbr(nbr,new_nbr)
        if state == STATE_FLOWER:
            return (nbr, COLORS[color])
    return (None, None)

fall()
