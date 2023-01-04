import numpy as np

# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    ## This function deals with Rover different states by to deciding what to do given perception data ##

    # stay by the left wall when out of the starting point (after 10s) to avoid getting stuck in a circle 

    offset = 0

    if Rover.total_time > 10:
        # adjusting the offset based on the standard deviation of the angles.
        # apply small offsets on straight lines and large ones in corners and open areas.

        offset = 0.8 * np.std(Rover.nav_angles)

    # Check if we have vision data from camera to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status.
        if Rover.mode[-1] == 'forward':
            
            # if sample rock on sight (in the left side only) and relatively close
            # limited this state (rock) for rocks closer than 3 meters.
            # This was necessary to avoid the Rover to fall off its trajectory to pick a rock on the opposite wall.
            if Rover.samples_angles is not None and np.mean(Rover.samples_angles) > -0.2 and np.min(Rover.samples_dists) < 30:       
                Rover.rock_time = Rover.total_time
                Rover.mode.append('rock')

            # Check the extent of navigable terrain
            elif len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good
                # at start, if stopped status that means Rover is stuck.
                # then set mode to "stuck" and hit the brakes
                # Set brake to stored brake value
             
                if Rover.vel <= 0.1 and Rover.total_time - Rover.stuck_time > 4:
                    Rover.throttle = 0                
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode.append('stuck')
                    Rover.stuck_time = Rover.total_time

                # if velocity is below max, then throttle
                # Set throttle value to throttle setting
                elif Rover.vel < Rover.max_vel:
                    Rover.throttle = Rover.throttle_set

                else: 
                    Rover.throttle = 0
                Rover.brake = 0

                # move around Roverself by average angel +/- 15
                # stay by the left wall again

                Rover.steer = np.clip(np.mean((Rover.nav_angles+offset) * 180 / np.pi), -15, 15)


            # If there's no navigable terrain pixels then go to 'stop' mode
            # Set mode to "stop" and hit the brakes!
            # Set brake to stored brake value
            elif len(Rover.nav_angles) < Rover.stop_forward or Rover.vel <= 0:          
                    Rover.throttle = 0            
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode.append('stop')

            # If we're  in "stuck", Stay here for 1 sec
         
        elif Rover.mode[-1] == 'stuck':
                # if 1 sec passed go back to previous mode
                # Set throttle back to stored value
                # Release the brake
                # Set steer to mean angle
                # stay by left wall by setting the steer angle slightly to the left
                # returns to previous mode

            if Rover.total_time - Rover.stuck_time > 1:
                Rover.throttle = Rover.throttle_set      
                Rover.brake = 0     
                Rover.steer = np.clip(np.mean((Rover.nav_angles+offset) * 180 / np.pi), -15, 15)
                Rover.mode.pop() 

                # if stopped and we have vision data  from camera to see if there's a path forward
                # Release the brake to allow turning
                # Turn range is - 15 degrees steering to the right Since hugging left wall
                Rover.throttle = 0
                Rover.brake = 0         
                Rover.steer = -15

        #if status is rock
      
        elif Rover.mode[-1] == 'rock':
            # if sample is near, Steer torwards the sample
            mean = np.mean(Rover.samples_angles * 180 / np.pi)
            if not np.isnan(mean):
                Rover.steer = np.clip(mean, -15, 15)


            # if for any reason no rock in sight of Rover camera, Go back to previous state   
            else:
                Rover.mode.pop()

            # if 20 sec passed give up and goes back to previous mode
            if Rover.total_time - Rover.rock_time > 20:
                Rover.mode.pop()  

            # if close to the sample stop
            # Set mode to "stop" and hit the brakes!
            # Set brake to stored brake value
            if Rover.near_sample:
                Rover.throttle = 0        
                Rover.brake = Rover.brake_set

            # if got stuck go to stuck mode
            # Set brake to stored brake value
            elif Rover.vel <= 0 and Rover.total_time - Rover.stuck_time > 10:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode.append('stuck')
                Rover.stuck_time = Rover.total_time

            # else approach slowly
            else:
                slow_speed = Rover.max_vel / 2

                if Rover.vel < slow_speed:
                    Rover.throttle = 0.2
                    Rover.brake = 0
                
                # Set brake to stored brake value
                else: 
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set

        # If  in "stop" mode then make different decisions
        elif Rover.mode[-1] == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0

            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                # Release the brake to allow turning
                # Turn range is +/- 15 degrees, Since we are by left wall steering should be to the right
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    Rover.brake = 0
                    Rover.steer = -15

                # If we're stopped but see sufficient navigable terrain in front then go!
                # Set throttle back to stored value
                # Release the brake
                # Set steer to mean angle
                # stay by left wall by setting the steer angle slightly to the left
                # returns to previous mode
                if len(Rover.nav_angles) >= Rover.go_forward:
                    Rover.throttle = Rover.throttle_set   
                    Rover.brake = 0    
                    offset = 12
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi) + offset, -15, 15)
                    Rover.mode.pop()  

    #Rover do something 
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    return Rover