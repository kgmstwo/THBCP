import rone, sys, math, neighbors, velocity, pose, leds, math2, motion

nav_tower_ids = [124,125,126,127]

def compute_avg_reciever_bearing(reciever_list):
    x = 0.0
    y = 0.0
    for r in reciever_list:
        y += math.sin(SENSOR_ANGLES[r])
        x += math.cos(SENSOR_ANGLES[r])
    return math.atan2(y, x)


# return the angle of the obstacle.  returns None if there is no obstacle
def obstacle_angle_get():
    obs = neighbors.get_obstacles() 
    if (obs != None):
        angle = compute_avg_reciever_bearing(obs)
    else:
        angle = None
    return angle
    

## These are the functions that you must run to guarantee your score is counted during
## winter. The winter_time_keeper function must be run starting after the event starts and
## the winter_score_calc will calculate your score once your bee "dies"

## winter_score_calc(score_time)

## send score_time calculated by winter_time_keeper and calculates the score and outputs it 
## visually in the form of lights 

## score_time = winter_time_keeper(initial_time)

## When winter begins, set initial_time as initial_time = sys.time()
## Send initial_time to winter_time_keeper
## Returns current score_time

  
def winter_score_calc(score_time, brightness):
    score = score_time/1000
    interval_1 = 300
    interval_2 = 180
    interval_3 = 600-interval_1-interval_2
    if score<=interval_1:
        score1=score/(interval_1/5)
        score2=0
        score3=0
    elif score<=(interval_2+interval_1):
        score1=5
        score2=(score-interval_1)/(interval_2/5)
        score3=0
    elif score<=(interval_3+interval_2+interval_1):
        score1=5
        score2=5
        score3=(score-interval_1-interval_2)/(interval_3/5)
    else:
        score1=5
        score2=5
        score3=5
    print score,score1,score2,score3
    leds.set_pattern((int(score1), int(score2), int(score3)), 'count', brightness)

def winter_time_keeper(initial_time):
    score_time = sys.time()-initial_time
    return score_time



## use this to get a msg from the nbr that contains (distance, mode, quality)
def get_msg_from_nbr(nbr, new_nbrs): # N.B. the parameter new_nbrs is never used
    if nbr != None:
        ##first get message from provided neighbor
        msg = neighbors.get_nbr_message(nbr)
    
        # If the message was none, return 0 for all the feild. This shouldn't happen but still
        if msg != '':
            message = msg.split(',')
            distance = int(message[0])
            mode = int(message[1])
            quality = int(message[2])
            return (distance, mode, quality)
        else:
            return (0, 0, 0)
    else:
        return (0, 0, 0)


## use this to set a msg to be sent out to all neighbors
## distance needs to be between 0-5000mm, mode between 0-3, and quality between 0-3
## distance is qualtized to  DISTANCE_LSB_STEP
def set_msg(distance, mode, quality):
    ##now we must convert these values to get distance (4), mode (2), quality (2)
    # The format is [Distance, mode,quality]
    msg = str(distance)+","+str(mode)+","+str(quality)

    ##set message from provided neighbor
    neighbors.set_message(msg)


## This function computes the average bearing angle over time.  It returns
## true when this average is below a fixed threshold.
## angle - desired angle
## list - initially pass in an empty list to keep track of angles
## threshold - the threshold that the average angle should be under before we decide that it
## it is close enough to follow.
LIST_SIZE = 6

def average_error_check(angle, list, threshold, newNbrs):
    if newNbrs:
        if len(list) >= LIST_SIZE:
            list.pop(0)
        list.append(angle) 

    sum = 0
    for e in list:
        sum+= abs(e)
    if len(list) > 0:
        average_error = sum/len(list)
        if (len(list) >= LIST_SIZE) and (average_error < threshold):
            return True
        else:
            return False
    else:
        return False


def leds_blink_all(brightness):
    if (sys.time() % 500) < 250:
        rone.led_set_group('r', brightness)
        rone.led_set_group('g', brightness)
        rone.led_set_group('b', brightness)
    else:
        rone.led_set_group('r', 0)
        rone.led_set_group('g', 0)
        rone.led_set_group('b', 0)


def check_buttons():
    # return a string of the buttons that are pressed 
    # return a blank string if no button is pressed
    buttons = '' # placeholder code
    for button in ['r', 'g', 'b']:
        if rone.button_get_value(button):
            buttons += button
    return buttons

LED_BRIGHTNESS = 40

def wait_for_button():
    buttons = ''
    # First, wait for the user to press a button, then select states
    # The user is already pressing a button.  wait until they release
    while check_buttons() != '':
        leds_blink_all(LED_BRIGHTNESS)
    # wait for a button press
    while check_buttons() == '':
        leds_blink_all(LED_BRIGHTNESS)    
    # finally, process the button information
    buttons = check_buttons()
    return buttons

def get_robot_neighbors():
    nbr_list_out = []
    nbr_list = neighbors.get_neighbors()
    for nbr in nbr_list:
        if neighbors.get_nbr_id(nbr) not in nav_tower_ids:
            nbr_list_out.append(nbr)
    return nbr_list_out
    
    
def find_nav_tower_nbr(id):
    nbr_list = neighbors.get_neighbors()
    # If low power ID availible, get bearing from it
    low_power_id = id
    high_power_id = id - 1
    nbr_out = None
    for nbr in nbr_list:
        if low_power_id == neighbors.get_nbr_id(nbr):
            nbr_out = nbr
    # Else take bearing from high power ID
        elif (high_power_id == neighbors.get_nbr_id(nbr)) and (nbr_out == None):
            nbr_out = nbr
    return nbr_out
        

##simple leader election-- highest ID
##otherwise, return nbr and nbr_id
def leaderElection_ID(list, includeSelf):
    leader_id = 0
    leader = 0
    if list and len(list) > 0:
        for nbr in list:
            nbr_id = neighbors.get_nbr_id(nbr)
            if (nbr_id > leader_id):
                leader = nbr
                leader_id = temp_id
    if includeSelf:
        if rone.get_id() > leader_id:
            return (0, rone.get_id())
    else:
        return (leader, leader_id)
    

def leaderElection_quality(ourQuality, ourDistance):
    ##find max quality
    leader = 0
    max_quality = 0
    distance_final = 0
    if nbrList and len(nbrList) > 0:
        for nbr in nbrList:
            (distance, mode, quality) = get_msg_from_nbr(nbr)
            if (quality >= max_quality):
                max_quality = quality
                leader = nbr
    if (ourQuality > max_quality):
        return 0
    elif (ourQuality == max_quality):
        includeSelf = True
    else:
        includeSelf = False

    ##collect all candidates with same quality value
    candidates_list = [leader]
    if nbrList and len(nbrList) > 0:
        for nbr in nbrList:
            (distance, mode, quality) = get_msg_from_nbr(nbr)
            if (quality == max_quality):
                candidates_list.append(nbr)

    (leader, leader_id) = leaderElection_ID(candidates_list, includeSelf) ##return candidate with highest ID

    ##if a nbr is the leader, get its distance, mode, and quality values; else get ours
    if (leader!= 0):
        (distance, mode, quality) = get_msg_from_nbr(leader)
    else:
        distance = ourDistance
        quality = ourQuality
        
    return (leader, quality, distance)
