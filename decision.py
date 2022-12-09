import numpy as np
import time
from enum import Enum
import typing as t 
import random
from supporting_functions import normalize_degree

# For mode 
# Go forward or stop or Go backward
class MainMode(Enum):
    FORWORD = 'forward'
    STOP = 'stop'
    BACK = 'back'
    APPROACH_ROCK = 'approach_rock'


# For submode in MainMode
class SubMode(Enum):
    NONE = -1
    
    # Approaching Rock
    TURN_TO_ROCK = 10
    FORWARD_TO_ROCK = 11
    READY_TO_PICK = 12
    PICKUP_ROCK = 13

    # Back
    BACK_TO_UNSTUCK = 20
    FORWARD_TO_UNSTUCK = 21


def set_mode(Rover, mode: MainMode, submode: SubMode):
    Rover.mode = mode
    set_submode(Rover, submode)

def set_submode(Rover, submode: SubMode):
    Rover.submode = submode
    Rover.submode_time = time.time()


# Prevent stucked rover
def check_stuck(Rover, timeout_time = None):
    if Rover.old_pos is None:
        Rover.old_pos = Rover.pos.copy()
        Rover.stucked_time = time.time()
        return
    
    # Set default timeout value
    if timeout_time is None:
        timeout_time = Rover.slipout_time_on_stucked

    dxy = (abs(Rover.pos[0] - Rover.old_pos[0]), abs(Rover.pos[1] - Rover.old_pos[1]))
    print('@ Rover.pos:', Rover.pos, Rover.old_pos, dxy)

    _thres = 0.01
    if (dxy[0] < _thres) and (dxy[1] < _thres):
        print('  position is almost same.')

        if (time.time() - Rover.stucked_time >= timeout_time):
            print("      rover is stucked, go back")

            Rover.throttle = -Rover.throttle_set
            Rover.brake = 0
            Rover.steer = 0

            set_mode(Rover, MainMode.BACK, SubMode.BACK_TO_UNSTUCK)

            Rover.stucked_time = time.time()
            Rover.found_rock = False
    else:
        Rover.stucked_time = time.time()

    Rover.old_pos = Rover.pos.copy()

# Return nearst rock data, (dist,angle)|None
def find_nearest_rock(Rover, check_detail: bool=True, debug_output: bool=False) -> t.Optional[t.Tuple[float, float]]:
    print('find_nearest_rock()')
    if Rover.rock_pos is None:
        delta = np.nan
        dist = np.nan
        angle = np.nan
    else:
        delta = (Rover.rock_pos[0] - Rover.pos[0], Rover.rock_pos[1] - Rover.pos[1])
        dist = np.sqrt(delta[0]**2 + delta[1]**2)
        angle = np.arctan2(delta[1], delta[0])

    norm_pitch = normalize_degree(Rover.pitch)
    norm_roll = normalize_degree(Rover.roll)

    # To analyze logs. DO NOT DELETE BELOW LINES.
    if debug_output:

        print('  @ nearest_rock:',
            {'pos': Rover.pos, 'rock': Rover.rock_pos,
            'dist': dist, 'angle': angle, 'degree': angle*180/np.pi,
            'yaw': Rover.yaw, 'pitch': Rover.pitch, 'roll': Rover.roll,
            'norm_pitch': norm_pitch, 'norm_roll': norm_roll,
            'delta': delta})

    if Rover.rock_pos is None:
        return None

    if check_detail and not (\
        #is_accepted_angle_of_rocks(angle) and \
        is_accepted_distance_of_rocks(dist) and \
        is_accepted_attitude(norm_pitch, norm_roll)):
        return None

    return (dist, angle)


# Check angle(yaw) between rover and rocks
def is_accepted_angle_of_rocks(angle, range_angle: float = np.pi/4) -> bool:
    # 0..2*pi -> -pi..0..pi
    angles_abs = np.abs(angle if angle <= np.pi else angle - 2*np.pi)
    # In range ?
    return (angles_abs > np.pi/2 - range_angle) and (angles_abs < np.pi/2 + range_angle)


# Check distance between rover and rocks
def is_accepted_distance_of_rocks(dist, threshold: float = 5.0) -> bool:
    return dist < threshold


# Check rover's pitch or roll angles
# Etc.
#   thre_pitch = (-0.05, 0.001)
#   thre_roll = (-0.05, 0.05)
def is_accepted_attitude(norm_pitch, norm_roll, thre_pitch = (-0.1, 0.1), thre_roll = (-0.9, 0.9)) -> bool:
    pitch_ok = (thre_pitch[0] <= norm_pitch) and (norm_pitch <= thre_pitch[1])
    roll_ok = (thre_roll[0] <= norm_roll) and (norm_roll <= thre_roll[1])
    return pitch_ok and roll_ok


# Invoke back(do unstuck)
def do_back(Rover):
    if Rover.submode == SubMode.BACK_TO_UNSTUCK:
        if (time.time() - Rover.stucked_time < 4):
            print("      go back  to unstuck! ")
            Rover.throttle = -Rover.throttle_set
            Rover.brake = 0
            Rover.steer = 20
        else:
            Rover.stucked_time = time.time()
            set_submode(Rover, SubMode.FORWARD_TO_UNSTUCK)
            Rover.throttle = Rover.throttle_set

    elif Rover.submode == SubMode.FORWARD_TO_UNSTUCK:
        if (time.time() - Rover.stucked_time < 6):
            print("      go forward to unstuck! ")

            if Rover.vel < Rover.max_vel:
                Rover.throttle = Rover.throttle_set
            else:
                Rover.throttle = 0
            
            Rover.brake = 0
            Rover.steer = -20
        else:
            Rover.stucked_time = time.time()
            if (random.randint(0,1) == 0):
                set_mode(Rover, MainMode.FORWORD, SubMode.NONE)
            else:
                set_mode(Rover, MainMode.STOP, SubMode.NONE)


    print("      :", {
        'vel': Rover.vel, 'throttle': Rover.throttle, 
        'brake': Rover.brake, 'steer': Rover.steer})


# Invoke approch rock 
def do_approch_rock_mode(Rover):
    result = find_nearest_rock(Rover, check_detail=False)
    if result is None:
        set_mode(Rover, MainMode.FORWORD, SubMode.NONE)
        return
    else:
        rock_dist, rock_angle = result

    # 0..360 -> -180..180
    yaw = Rover.yaw if Rover.yaw <= 180 else Rover.yaw -360

    rock_deg = rock_angle*180/np.pi
    # Humm... (inefficient delta)
    delta_angle = rock_deg - yaw

    # Turn to nearest rock
    if Rover.submode == SubMode.TURN_TO_ROCK:
        Rover.throttle = 0
        
        # Turn
        #_max_delta = abs(delta_angle) / 8
        _max_delta = 10 + random.randint(-5,0)
        Rover.steer = np.clip(delta_angle, -_max_delta, _max_delta)
        print('@ delta_angle:', {'delta_angle': delta_angle, '_max_delta': _max_delta})

        # Stop
        if Rover.vel > 0:
            Rover.brake = Rover.brake_set
        else:
            Rover.brake = 0

        if abs(delta_angle) < 3.0:
            Rover.steer = 0
            set_submode(Rover, SubMode.FORWARD_TO_ROCK)
    
    # Go forward
    elif Rover.submode == SubMode.FORWARD_TO_ROCK:
        if Rover.vel < Rover.max_vel:
            Rover.throttle = Rover.throttle_set / 2
        else:
            Rover.throttle = 0
        Rover.steer = 0
        Rover.brake = 0

        if abs(delta_angle) > 8.0:
            set_submode(Rover, SubMode.TURN_TO_ROCK)

        # Close enough
        if rock_dist < 0.2:
            Rover.brake = Rover.brake_set
            Rover.throttle = 0
            set_submode(Rover, SubMode.READY_TO_PICK)
            Rover.stucked_time = time.time()

    # Wait for pickup
    elif Rover.submode == SubMode.READY_TO_PICK:
        Rover.brake = Rover.brake_set
        Rover.steer = 0
        Rover.throttle = 0

        # To `near_sample` flag
        if Rover.vel == 0.0:
            Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = -20  # Turn here

    
    check_stuck(Rover, timeout_time=16)
    
    nav_angle = np.mean(Rover.nav_angles)
    print('      :', {
        'found': Rover.found_rock,
        'mean_nav_angle': [nav_angle, nav_angle*180/np.pi], 
        'len_nav_angle': len(Rover.nav_angles),
        'vel': Rover.vel, 
        'rock_dest': rock_dist,
        'rock_angle': rock_deg,
        'yaw': Rover.yaw,
        'steer': Rover.steer})


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    print("decision_step() begin...")

    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        print("  @mode:", Rover.mode, ', submode:', Rover.submode)
        if Rover.mode == MainMode.FORWORD:
            # Check the extent of navigable terrain
            if (len(Rover.nav_angles) >= Rover.stop_forward):
                print("    In throttle and turn:", len(Rover.nav_angles))

                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0

                #bias = np.std(Rover.nav_angles) * 180/np.pi # Add bias
                #bias = -random.randint(0,4)
                bias = random.randint(-5,0)
                # Set steering to average angle clipped to the range +/- x
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15 + bias, 15 + bias)

                check_stuck(Rover)

                print("      :", {
                    'vel': Rover.vel, 'throttle': Rover.throttle, 
                    'brake': Rover.brake, 'steer': Rover.steer})

            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                print("    go stop mode!!:", len(Rover.nav_angles))

                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = MainMode.STOP

                print("      :", {
                    'vel': Rover.vel, 'throttle': Rover.throttle, 
                    'brake': Rover.brake, 'steer': Rover.steer})

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == MainMode.STOP:
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                print("    more stop: vel:", Rover.vel)

                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0

            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                print("    already stoped: vel:", Rover.vel)

                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    
                    print("      stucked, and turn here:", len(Rover.nav_angles))

                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn

                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    print("      go forward!!:", len(Rover.nav_angles))

                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.stucked_time = time.time()
                    Rover.mode = MainMode.FORWORD

                    print("      :", {
                        'vel': Rover.vel, 'throttle': Rover.throttle, 
                        'brake': Rover.brake, 'steer': Rover.steer})

        elif Rover.mode == MainMode.BACK:
            do_back(Rover)

        elif Rover.mode == MainMode.APPROACH_ROCK:
            do_approch_rock_mode(Rover)

    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        print("Rover.nav_angles is NONE")
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # When found rock
    if Rover.found_rock \
            and (find_nearest_rock(Rover) is not None) \
            and Rover.mode in [ MainMode.FORWORD ]:
        print("Found rock, change mode to approach_rock!")
        set_mode(Rover, MainMode.APPROACH_ROCK, SubMode.TURN_TO_ROCK)


    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        print("begin pickup.")
        Rover.send_pickup = True

        Rover.rock_pos = None
        Rover.found_rock = False
        set_mode(Rover, MainMode.FORWORD, SubMode.NONE)

    print("@ pickup flags: ", {
        'near_sample': Rover.near_sample, 
        'picking_up': Rover.picking_up, 
        'send_pickup': Rover.send_pickup})

    find_nearest_rock(Rover, debug_output=True) # for debug
    print("decision_step() DONE.")
    
    return Rover

