################################################################################
##                          LED animation module                              ##
################################################################################

import rone, sys, math

##    NAME
##        LED
##        
##    DESCRIPTION
##        LED animation module 
##        
##    FUNCTIONS
##        init()
##            initalizing the led_state dictionary
##            
##        set(color, pattern, brightness)
##            sets the led_state dicitionary with the appropriate color, patter, and brightness level
##            
##            PARAMS
##                  2. color      - the color of the led_state
##                  3. brightness - the brightness of the leds
##                  4. pattern    - the pattern of the led to display
##
##        update()
##            Update the state of the LED lights depending on the pattern stored in the _leds_state dictionary
##            The patterns include: "manuel", "group", "ramp_slow", "blink_slow", "blink_fast", and "circle"
##
##
##        _led_set_group(color, brightness)
##            calls on rone's _led_set function and sets the indicated color of the same group to a specified brightness level.
##
##            PARAMS
##                 1. color      - the color to set
##                 2. brightness - the brightness of the led


LED_UPDATE_PERIOD = 109     # want 100, use closest prime to help prevent multiple updates on same cycle
BLINK_SLOW = 8
BLINK_FAST = 2

_leds_state = {}

def init():
    _leds_state['color'] = 'r'
    _leds_state['pattern'] = 'circle'
    _leds_state['brightness'] = 10
    _leds_state['counter'] = 0
    _leds_state['update_time'] = sys.time()


def set_pattern(color, pattern, brightness):
    _leds_state['color'] = color
    _leds_state['pattern'] = pattern
    _leds_state['brightness'] = brightness



def update():
    global old_pattern
    current_time = sys.time()
    update_time = _leds_state['update_time']
    if current_time > update_time:
        update_time += LED_UPDATE_PERIOD
        # advance time if there have been delays in calling update
        if update_time < current_time:
            update_time = current_time + LED_UPDATE_PERIOD
        _leds_state['update_time'] = update_time
        
        counter = _leds_state['counter'] + 1
        _leds_state['counter'] = counter
        color = _leds_state['color']
        pattern = _leds_state['pattern']
        brightness = _leds_state['brightness']
        
        if _leds_state['pattern'] != 'manual':
            for c in 'rgb':
                if c not in color:
                    _led_set_group(c, 0)
    
        if pattern == 'group':
            _led_set_group(color, brightness)
        elif pattern == 'ramp_slow':
            idx = counter % BLINK_SLOW
            if idx < (BLINK_SLOW / 2):
                # ramp up
                b = brightness * idx / (BLINK_SLOW / 2)
            else:
                b = brightness * (BLINK_SLOW - idx) / (BLINK_SLOW / 2)
            _led_set_group(color, b)
        elif pattern == 'blink_slow':
            idx = counter % BLINK_SLOW
            if idx < (BLINK_SLOW / 2):
                _led_set_group(color, brightness)
            else:
                _led_set_group(color, 0)
        elif pattern == 'blink_fast':
            idx = counter % BLINK_FAST
            if idx < (BLINK_FAST / 2):
                _led_set_group(color, brightness)
            else:
                _led_set_group(color, 0)
        elif pattern == 'circle':
            idx = counter % 5
            led_map = rone._led_map[color]
            for i in range(5):
                if i == idx:
                    rone._led_set(led_map[i], brightness)
                else:
                    rone._led_set(led_map[i], 0)
        elif pattern == 'count':
            # display ints as counting on the lights
            idx = 0
            for c in 'rgb':
                if color[idx] == 0:
                    _led_set_group(c, 0)
                else:
                    led_map = rone._led_map[c]
                    for i in range(5):
                        if i < color[idx]:
                            rone._led_set(led_map[i], brightness)
                        else:
                            rone._led_set(led_map[i], 0)
                idx += 1
        elif pattern == 'manual':
            pass
        
        old_pattern = pattern


# duplicate these function here to avoid the very slow "validate keys" checks in rone.py
def _led_set_group(color, brightness):
    for c in color:
        led_map = rone._led_map[c]
        for i in led_map:
            rone._led_set(i, brightness)
    


