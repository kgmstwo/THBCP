################################################################################
##                          Additional math functions                         ##
################################################################################

import math

##    NAME
##        Math2
##
##    DESCRIPTION
##        Additional math functions
##
##    FUNCTIONS
##        topolar(x, y)
##            converting rectangular coordinates to polar coordinates in (r, theta)
##
##            PARAMS
##                1. x - the x coordinate
##                2. y - the y coordinate
##
##            RETURN
##                a tuple of the converted x and y coordinates in (r, theta)
##
##        clamp(value, value_max)
##            Bound value to lie between -value_max and value_max
##
##            PARAMS
##                1. value     - the value to clamp
##                2. value_max - the upper/lower bound for the bound
##
##            RETURN
##                the clamped integer 
##
##        normalize_angle(theta)
##            Bound theta to -pi to pi
##
##            PARMS
##                1. theta - the angle theta to normalize
##
##            RETURN
##                the normalize versin of theta that is contrained between zero to two pi.
##
##        smallest_angle_diff(current_angle, goal_angle)
##             calculates the difference between current and goal angle
##             
##            PARAMS
##               1. current_angle - the current angle the robot is facing
##               2. goal_angle    - the final angle the robot is suppose to reach
##            
##            RETURN
##               a value for the difference normalized to [-pi, pi]
##


# Subtract pose2 from pose1.  return a tuple of the difference
def pose_subtract(pose1, pose2):
    x = pose1[0] - pose2[0]
    y = pose1[1] - pose2[1]
    theta = normalize_angle(pose1[2] - pose2[2])
    return (x, y, theta)


# Convert rectangular to polar
# return a tuple of the form (r, theta) 
def topolar(x, y):
    r = math.sqrt(x*x + y*y)
    theta = math.atan2(y, x)
    return (r, theta)


# Bound value to lie between -value_max and value_max
def bound(value, value_max):
    ## Clamp value between +/- value_max
    if value > value_max:
        value = value_max
    elif value < -value_max:
        value = -value_max
    return value


# Bound theta to lie between (-pi,pi]
def normalize_angle(theta):
    while theta > math.pi:
        theta -= 2 * math.pi
    while theta <= -math.pi:
        theta += 2 * math.pi
    return theta


# Difference in angles normalized to (-pi, pi]
def smallest_angle_diff(current_angle, goal_angle):
    diff = normalize_angle(goal_angle) - normalize_angle(current_angle) 
    if diff > math.pi:
        diff -= 2 * math.pi
    if diff < (-math.pi):
        diff += 2 * math.pi
    return diff


