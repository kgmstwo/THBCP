import rone, sys, math, math2, velocity, pose, motion, leds, neighbors, beh, hba

###########################################################
##
##  Nav Tower Motion
##
###########################################################
#
# This demo demonstrates one possible behavior for Fall ro-bee behavior.
# When a ro-bee senses a navigation tower it moves towards the tower until
# it gets within a certain distance, turns around, moves away and continues
# to oscillate back and forth. Load Nav_Tower.py onto a robot and try it out. 


# Basic motion parameters - change carefully
#MOTION_RV = int(1000 * math.pi * 0.3)
MOTION_TV = 100

# FSM States
STATE_MOVE_TO_TOWER = 0
STATE_MOVE_FROM_TOWER = 1

# Other constants
LED_BRIGHTNESS = 40
MOVE_TO_TOWER_DISTANCE = 3000
#find_nav_tower_nbr()

def nav_tower_motion():
    beh.init(0.22, 40, 0.5, 0.1)
    state = STATE_MOVE_TO_TOWER
    motion_start_odo = pose.get_odometer()
    while True: #while True loop just for testing, will need to call many times
        nav_tower = hba.find_nav_tower_nbr(127)
        if state == STATE_MOVE_TO_TOWER:
            new_nbrs = beh.update()
        # Move towards the nav_tower until turning around distance reached
            if nav_tower != None:      # move forward
                beh_out = beh.follow_nbr(nav_tower, MOTION_TV)
                leds.set_pattern('g', 'blink_fast', LED_BRIGHTNESS)
            else:
                leds.set_pattern('g', 'circle', LED_BRIGHTNESS)
                beh_out = beh.follow_nbr(nav_tower, MOTION_TV)  
            distance_to_go = (motion_start_odo + MOVE_TO_TOWER_DISTANCE) - pose.get_odometer()
            beh.motion_set(beh_out)
            if distance_to_go < 0:    
                state = STATE_IDLE   
    return
#     while True:
#         # run the system updates
#         new_nbrs = beh.update()
        
#         #make a nbr list        
#         nbrList = neighbors.get_neighbors()
#         if new_nbrs:
#             print nbrList
#         beh_out = beh.BEH_INACTIVE
            
#         nav_tower = hba.find_nav_tower_nbr(127)
#         # this is the main finite-state machine
#         if state == STATE_MOVE_TO_TOWER:
#             # Move towards the nav_tower until turning around distance reached
#             if nav_tower != None:
#                 # move forward
#                 beh_out = beh.follow_nbr(nav_tower, MOTION_TV)
#                 leds.set_pattern('g', 'blink_fast', LED_BRIGHTNESS)
#             else:
#                 leds.set_pattern('g', 'circle', LED_BRIGHTNESS)
                
#             # stop after a fixed distance_to_go
#             distance_to_go = (motion_start_odo + MOVE_TO_TOWER_DISTANCE) - pose.get_odometer() 
#             if distance_to_go < 0:
#                 motion_start_odo = pose.get_odometer()
#                 state = STATE_MOVE_FROM_TOWER

#             if new_nbrs:
#                 print "move to tower: dist=", distance_to_go

#         elif state == STATE_MOVE_FROM_TOWER:
#             # Move away from the nav_tower until turning around distance reached
#             if nav_tower != None:
#                 # move forward
#                 beh_out = beh.avoid_nbr(nav_tower, MOTION_TV)
#                 leds.set_pattern('r', 'blink_fast', LED_BRIGHTNESS)
#             else:
#                 leds.set_pattern('r', 'circle', LED_BRIGHTNESS)
                
#             # stop after a fixed distance_to_go
#             distance_to_go = (motion_start_odo + MOVE_TO_TOWER_DISTANCE) - pose.get_odometer() 
#             if distance_to_go < 0:
#                 motion_start_odo = pose.get_odometer()
#                 state = STATE_MOVE_TO_TOWER

#             if new_nbrs:
#                 print "move from tower: dist=", distance_to_go
                
#         # end of the FSM
#         bump_beh_out = beh.bump_beh(MOTION_TV)

#         beh_out = beh.subsume([beh_out, bump_beh_out])

#         # set the beh velocities
#         beh.motion_set(beh_out)

#         #set the HBA message
#         hba.set_msg(0, 0, 0)

            
nav_tower_motion()