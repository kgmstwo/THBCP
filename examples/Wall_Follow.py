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
STATE_LOOK_FOR_WALL = 1
STATE_WALL_FOLLOW = 2

# Other constants
LED_BRIGHTNESS = 40
WALL_TIMEOUT = 4000

def wall_follow_demo():
    velocity.init(0.22, 40, 0.5, 0.1)
    leds.init()
    pose.init()
    motion.init()
    neighbors.init(NBR_PERIOD)

    state = STATE_IDLE
    wall_time = 0
    
    while True:
        # Do updates
        leds.update()
        pose.update()
        velocity.update()
        new_nbrs = neighbors.update()
        
        nbrList = neighbors.get_neighbors()
        tv = 0
        rv = 0

        # this is the main finite-state machine
        if state == STATE_IDLE:
            leds.set_pattern('r', 'circle', LED_BRIGHTNESS)
            if new_nbrs:
                print "idle"
            if rone.button_get_value('r'):
                state = STATE_LOOK_FOR_WALL
            
        elif state == STATE_LOOK_FOR_WALL:
            leds.set_pattern('r', 'blink_fast', LED_BRIGHTNESS)
            if new_nbrs:
                print "look for wall"
            tv = MOTION_TV
            obs = neighbors.get_obstacles() 
            if (obs != None):
                state = STATE_WALL_FOLLOW                
            
        elif state == STATE_WALL_FOLLOW:
            leds.set_pattern('b', 'blink_fast', LED_BRIGHTNESS)
            if new_nbrs:
                print "wall follow"
            # follow the wall
            (tv, rv, active) = wall_follow(MOTION_TV / 2)
            if active == True:
                wall_time = sys.time()
            if sys.time() > (wall_time + WALL_TIMEOUT):
                state = STATE_LOOK_FOR_WALL
                
        # end of the FSM
                        
        # set the velocities
        velocity.set_tvrv(tv, rv)
        
        #set the message
        hba.set_msg(0, 0, 0)


# drive the robot around the obstacle clockwise, i.e. turn to the right
def wall_follow(tv):
    obs_angle = hba.obstacle_angle_get()
    active = False
    if (obs_angle != None):
        alpha = math2.normalize_angle(obs_angle + math.pi/2)
        rv = 900 * alpha
        active = True
    else:
        # no wall.  arc to the right to look for one
        rv = -1000
    return (tv, rv, active)


wall_follow_demo()
