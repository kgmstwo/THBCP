import sys, rone

## velocity_controller constants
ENCODER_MM_PER_TICKS = 0.0625
WHEEL_BASE = 78
_VEL_UPDATE_PERIOD = 29
SPEED_MAX = 300
_TV_UPDATE_PERIOD = 50
TV_RAMP_DOWN = 30 
TV_RAMP_UP = 60 

_vcstate = {}


# Get the current velocity of the given motor
def get(motor):
    # rone.validate_key(motor, 'motor', rone._motor_map)
    if (motor == 'l') or (motor == 'L'):
        return _vcstate['l']['vel']
    if (motor == 'r') or (motor == 'R'):
        return _vcstate['r']['vel']


# Set the desired goal velocity for the given motor
def set(motor, velocity):
    if (motor == 'l') or (motor == 'L'):
        _vcstate['l']['goalvel'] = velocity
    if (motor == 'r') or (motor == 'R'):
        _vcstate['r']['goalvel'] = velocity


# Set tv and rv for the robot
def set_tvrv(tv, rv):
    tv = int(tv)
    if sys.time() > _vcstate['tv_update_time']:
        _vcstate['tv_update_time'] += _TV_UPDATE_PERIOD
        if tv < _vcstate['tv_ramp']:
            _vcstate['tv_ramp'] -= TV_RAMP_DOWN
            if tv > _vcstate['tv_ramp']:
                _vcstate['tv_ramp'] = tv
        if tv > _vcstate['tv_ramp']:
            _vcstate['tv_ramp'] += TV_RAMP_UP
            if _vcstate['tv_ramp'] > tv:
                _vcstate['tv_ramp'] = tv
    tv = _vcstate['tv_ramp']
    left_vel = tv - rv * WHEEL_BASE / 2000
    right_vel = tv + rv * WHEEL_BASE / 2000
    left_vel = clamp(left_vel, SPEED_MAX)
    right_vel = clamp(right_vel, SPEED_MAX)
    set('l', left_vel)
    set('r', right_vel)


#init the velocity controller
def init(kff, kff_offset, kp, ki):
    _vcstate['l'] = {}
    _vcstate['r'] = {}

    # K terms
    _vcstate['kff'] = kff
    _vcstate['kff_offset'] = kff_offset
    _vcstate['kp'] = kp
    _vcstate['ki'] = ki
    _vcstate['update_time'] = sys.time()
    _vcstate['tv_ramp'] = 0
    _vcstate['tv_update_time'] = sys.time()

    for motor in ['l', 'r']:
        _vcstate[motor]['ticks'] = rone.encoder_get_ticks(motor) # Position
        _vcstate[motor]['time'] = sys.time() # Time
        _vcstate[motor]['iterm'] = 0.0 # Iterm
        _vcstate[motor]['goalvel'] = 0 # Goal velocity
        _vcstate[motor]['vel'] = 0 # Current velocity


# Move the robot based on the set velocity values
def update():
    if sys.time() > _vcstate['update_time']:
        _vcstate['update_time'] += _VEL_UPDATE_PERIOD
        _velocity('l')
        _velocity('r')


def _velocity(motor):
    ## vel_goal in mm/s
    ## motor either 'l' or 'r'
    ## iterm_old passed in from previous function runs (sum of the errors * ki)
    ## ticks_old is the old position of the given wheel
    ## time_old is the time of the last reading of the encoders

    vel_goal = _vcstate[motor]['goalvel']
    iterm_old = _vcstate[motor]['iterm']
    ticks_old = _vcstate[motor]['ticks']
    time_old = _vcstate[motor]['time']

    # compute distance
    ticks_new = rone.encoder_get_ticks(motor)
    distance = _compute_distance(ticks_new, ticks_old)
    
    # compute velocity_controller
    time_new = sys.time()
    time_delta = time_new - time_old 
    velocity = _compute_velocity(distance, time_delta)
    _vcstate[motor]['vel'] = velocity
    # some debug printing.  Don't leave it in, it slows down the computer
    
    # compute feedback terms
    error = vel_goal - velocity
    feedforward_term = _feedforward_compute(vel_goal)
    integral_term = _integral_compute(error, iterm_old)
    proportional_term = _proportional_compute(error)
    pwm = feedforward_term + proportional_term + integral_term
    pwm = int(pwm)
    pwm = clamp(pwm, 100)
    rone.motor_set_pwm(motor, pwm)

    #Some example debugging output.  Don't print this always, it will slow things down too much
    #print 'motor=%s ff_term=%5.1f  i_term=%5.1f  pwm=%3d' % (motor, float(feedforward_term), float(integral_term), pwm)
    
    # update old values
    _vcstate[motor]['ticks'] = ticks_new
    _vcstate[motor]['time'] = time_old = time_new
    _vcstate[motor]['iterm'] = integral_term
    

def encoder_delta_ticks(new, old):
    ## Takes two encoder values, a new one and an old one, and returns
    ## the difference.  Handles the wraparound of the encoders.
    diff = new - old
    if diff > 32768:
        diff = diff - 65536
    elif diff < -32768:
        diff = diff + 65536
    return diff


def _compute_distance(ticks_new, ticks_old):
#    pass ## to be written by the student
    ticks_delta = encoder_delta_ticks(ticks_new, ticks_old)
    distance = ENCODER_MM_PER_TICKS * ticks_delta;
    return distance
    
    
def _compute_velocity(distance, time):
    ## Takes distance in mm, time in msec
    ## returns velocity in mm/sec
    ## tricky stuff going on here with floats...
    if time == 0:
        return 0
    else:
        return float(distance * 1000)/float(time)
    
    
def clamp(val, bound):
    if val > bound:
        val = bound
    elif val < -bound:
        val = -bound
    return val


def _feedforward_compute(goal_vel):
    # returns the PWM value computed by the feed-forward term
    if goal_vel == 0:
        pwm = 0
    elif goal_vel > 0:
        pwm = _vcstate['kff_offset'] + _vcstate['kff'] * goal_vel
    else:
        pwm = -(_vcstate['kff_offset'] + _vcstate['kff'] * goal_vel)
    pwm = int(pwm)
    pwm = clamp(pwm, 100)
    return pwm
    

def _proportional_compute(error):
    return _vcstate['kp'] * error

    
def _integral_compute(error, iterm_old):
    iterm_new = iterm_old + (_vcstate['ki'] * error)
    return iterm_new 
