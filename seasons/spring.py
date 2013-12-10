import rone, sys, math, math2, velocity, pose, motion, leds, neighbors, beh, hba

###########################################################
##
##  Spring
##
###########################################################
#
# This is the code for the Spring season.

# Basic motion parameters - change carefully
MOTION_RV = 1300
MOTION_TV = 100

MOTION_CAPTURE_DISTANCE = 16 * 20
MOTION_RELEASE_DISTANCE = 32 * 20
MOTION_CAPTURE_ANGLE = math.pi/2
MOTION_RELEASE_ANGLE = math.pi/10

# FSM States
STATE_IDLE = 0
STATE_QUEEN = 1
STATE_WANDER = 2
STATE_RETURN = 3
STATE_FOLLOW = 4
STATE_RECRUIT = 5
STATE_LEADER = 6
STATE_SUCCESS = 7

# MSG indexes
MSG_IDX_STATE = 0

# Other constants
LED_BRIGHTNESS = 40

RANGE_BITS_CLOSE = 2
RANGE_BITS_CLOSER = 3
NAV_ID = 14 # 125 # 127
INSURANCE_TIME = 5 * 1000
WAIT_TIME = int((1.0/6) * 60 * 1000)

def spring():
    tree_pose = None
    followers = 0
    beh.init(0.22, 40, 0.5, 0.1)
    motion.init_rv(1000, MOTION_RV, MOTION_CAPTURE_DISTANCE, MOTION_RELEASE_DISTANCE
            , MOTION_CAPTURE_ANGLE, MOTION_RELEASE_ANGLE)
    state = STATE_IDLE

    while True:
        # run the system updates
        new_nbrs = beh.update()
        nbr_list = neighbors.get_neighbors()
        if new_nbrs:
            print state
            
        beh_out = beh.BEH_INACTIVE

        # set colors, because why not do it at the top
        color_counts = [0, 0, 0]
        for i in range(3):
            color_counts[i] = min([5, (state - 5 * i)])
        if state == STATE_IDLE:
            leds.set_pattern('rb', 'group', LED_BRIGHTNESS)
        elif state == STATE_SUCCESS:
            leds.set_pattern('g', 'circle', LED_BRIGHTNESS)
        else:
            leds.set_pattern(color_counts, 'count', LED_BRIGHTNESS)

        if not state in [STATE_IDLE, STATE_LEADER, STATE_SUCCESS]:
            for nbr in nbr_list:
                nbr_state = hba.get_msg_from_nbr(nbr, new_nbrs)[MSG_IDX_STATE]
                if nbr_state in [STATE_LEADER, STATE_SUCCESS]:
                    start_time = sys.time()
                    state = STATE_FOLLOW
            
        # this is the main finite-state machine
        if state == STATE_IDLE:

            if rone.button_get_value('r'):
                pose.set_pose(0, 0, 0)
                state = STATE_WANDER
            elif rone.button_get_value('b'):
                state = STATE_QUEEN

        elif state == STATE_QUEEN:
            pass

        elif state == STATE_WANDER:
            ##            leds.set_pattern('r', 'circle', LED_BRIGHTNESS)
            nav = hba.find_nav_tower_nbr(NAV_ID)
            beh_out = beh.avoid_nbr(nav, MOTION_TV)

            if bump_front():
                tree_pose = pose.get_pose()
                motion.set_goal((0.0, 0.0), MOTION_TV)
                state = STATE_RETURN
            elif nav == None:
                motion.set_goal((0.0, 0.0), MOTION_TV)
                state = STATE_RETURN

        elif state == STATE_RETURN:
            ##            nav_tower = hba.find_nav_tower_nbr(NAV_ID)
            queen = None
            recruiter = None
            for nbr in nbr_list:
                nbr_state = hba.get_msg_from_nbr(nbr, new_nbrs)[MSG_IDX_STATE]
                if nbr_state == STATE_QUEEN:
                    queen = nbr
                elif nbr_state == STATE_RECRUIT:
                    recruiter = nbr
            if queen != None:
                if (recruiter == None) and (tree_pose != None) and \
                close_to_nbr(queen):
                    start_time = sys.time()
                    state = STATE_RECRUIT
                elif not closer_to_nbr(queen):
                    beh_out = beh.follow_nbr(queen, MOTION_TV)
                else:
                    start_time = sys.time()
                    state = STATE_FOLLOW
            else:
                (tv, rv) = motion.update()
                beh_out = beh.tvrv(tv, rv)

        elif state == STATE_RECRUIT:
            new_followers = 0
            for nbr in nbr_list:
                nbr_state = hba.get_msg_from_nbr(nbr,new_nbrs)[MSG_IDX_STATE]
                if nbr_state == STATE_RECRUIT:
                    if neighbors.get_nbr_id(nbr) > rone.get_id():
                        state = STATE_FOLLOW
                elif nbr_state in [STATE_FOLLOW, STATE_QUEEN]:
                    new_followers += 1
            if new_followers > followers:
                start_time = sys.time()
            followers = new_followers
            if followers == 4 or sys.time() > start_time + WAIT_TIME:
                tree_pos = (tree_pose[0], tree_pose[1])
                motion.set_goal(tree_pos, MOTION_TV)
                state = STATE_LEADER

        elif state == STATE_FOLLOW: 
            recruiter = None
            leader = None
            success = False
            new_followers = 1
            for nbr in nbr_list:
                nbr_state = hba.get_msg_from_nbr(nbr,new_nbrs)[MSG_IDX_STATE]
                if nbr_state == STATE_RECRUIT:
                    recruiter = nbr
                elif nbr_state == STATE_LEADER:
                    leader = nbr
                elif nbr_state == STATE_SUCCESS:
                    leader = nbr
                    success = True
                elif nbr_state in [STATE_FOLLOW, STATE_WANDER]:
                    new_followers += 1
            if new_followers > followers:
                start_time = sys.time()
            followers = new_followers

            if recruiter == None:
                if leader == None:
                    if followers == 5 or sys.time() > start_time + WAIT_TIME:
                        followers = 0
                        state = STATE_WANDER
                else:
                    if success:
                        if sys.time() > start_time + INSURANCE_TIME:
                            state = STATE_SUCCESS
                    else:
                        start_time = sys.time()
                    beh_out = beh.follow_nbr(leader, MOTION_TV)
            else:
                beh_out = beh.BEH_INACTIVE

        elif state == STATE_LEADER:
##            (tv, rv) = motion.update()
##            beh_out = beh.tvrv(tv, rv)
##            if motion.is_done():
##                state = STATE_SUCCESS
            beh_out = beh.tvrv(-MOTION_TV, 0)

            if bump():
                state = STATE_SUCCESS

        elif state == STATE_SUCCESS:
            pass
        
        # end of the FSM
        
        if state not in [STATE_IDLE, STATE_RECRUIT, STATE_LEADER, STATE_QUEEN]:
            bump_beh_out = beh.bump_beh(MOTION_TV)
            beh_out = beh.subsume([beh_out, bump_beh_out])

        # set the beh velocities
        beh.motion_set(beh_out)

        #set the HBA message
        msg = [0, 0, 0]
        msg[MSG_IDX_STATE] = state
        hba.set_msg(msg[0], msg[1], msg[2])

# Helper functions

##def light_diff():
##    diff = rone.light_sensor_get_value('fl')-rone.light_sensor_get_value('fr')
##    return diff #positive = right, negative = left

##def go_to_tree(diff_start):
##    diff = light_diff() - diff_start
##    if diff > 50:
##        rv = MOTION_RV
##        tv = 0
##    elif diff < -50:
##        rv = -MOTION_RV
##        tv = 0
##    else:
##        tv = MOTION_TV
##        rv = 0
##    return (tv,rv)
##
##def tree_detect(diff_start):
##    tree = False
##    if rone.bump_sensors_get_value(1) == 1:
##        tree = True

def bump_front():
    # Returns true if any of the front 6 out of 8 sensors are triggered
    bump_bits = rone.bump_sensors_get_value()
    return (bump_bits & 231) > 0

def bump():
    bump_bits = rone.bump_sensors_get_value()
    return bump_bits != 0x00

def close_to_nbr(nbr):
    return neighbors.get_nbr_range_bits(nbr) >= RANGE_BITS_CLOSE

def closer_to_nbr(nbr):
    return neighbors.get_nbr_range_bits(nbr) >= RANGE_BITS_CLOSER

# Start!
spring()
