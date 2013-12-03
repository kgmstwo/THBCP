import rone, sys

def move_rotate_right(time):
    start=sys.time()
    while sys.time() < start+time:
         rone.motor_set_pwm('r',-70), rone.motor_set_pwm('l',70)
           
def light_diff():
    value = rone.light_sensor_get_value('fl')- rone.light_sensor_get_value('fr')
    return value
    



# Move towards Light! Use the structure below, and the movement helper functions from above
# arguments: nothing
# return: nothing
def light_follow():
    diff_start = light_diff()
    #print "diff_start", diff_start
    sys.sleep(1000)
    while True:
        diff = light_diff() - diff_start
        #print "diff", diff
        
        move_rotate_right(2000)
        if diff == 0:
            break 
            move_forward(2000)
light_follow() 
