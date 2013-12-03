import rone, sys, math, neighbors, velocity, pose, leds, math2, motion, beh, hba

# flower quality constants
TYPE_RED = 0
TYPE_GREEN = 1
TYPE_BLUE = 2

LED_BRIGHTNESS = 40

def flower():
    beh.init(0.22, 40, 0.5, 0.1)
    flower_type = TYPE_RED;
    color = 'r'
    
    # Broadcast the ID with a message saying that it is a flower
    while True:
        # Do updates
        newNbrs = beh.update()
        beh_out = beh.BEH_INACTIVE

        if rone.button_get_value('r'):
            flower_type = TYPE_RED
            color = 'r'
        if rone.button_get_value('g'):
            flower_type = TYPE_GREEN
            color = 'g'
        if rone.button_get_value('b'):
            flower_type = TYPE_BLUE
            color = 'b'

        hba.set_msg(0, 0, flower_type)
        leds.set_pattern(color, 'ramp_slow', LED_BRIGHTNESS)
        
        beh.motion_set(beh_out)
        
flower()
