import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # offset in rad used to hug the left wall.
    offset = 0
    if Rover.total_time > 10:
        offset = 0.8 * np.std(Rover.nav_angles)

    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        if Rover.state[-1] == 'forward':
            # go to the rock state when the rock is on the rover's left and distance between them is small
            #failed trial: 30 and 90
            if Rover.rock_angles is not None and np.mean(Rover.rock_angles) > -0.2 and np.min(Rover.rock_distances) < 50:
                Rover.rock_time = Rover.total_time
                Rover.state.append('rock')

            # Check to see if theres a navigable path
            elif len(Rover.nav_angles) >= Rover.stop_forward:
                if Rover.vel <= 0.1 and Rover.total_time - Rover.stuck_time > 4:
                    # Switch state to "stuck" and hit the brakes
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.state.append('stuck')
                    Rover.stuck_time = Rover.total_time

                # if velocity is below max, then throttle
                elif Rover.vel < Rover.max_vel:
                    Rover.throttle = Rover.throttle_set
                else: 
                    Rover.throttle = 0
                Rover.brake = 0
                
                # Hug left wall by setting the steer angle slightly to the left
                Rover.steer = np.clip(np.mean((Rover.nav_angles+offset) * 180 / np.pi), -15, 15)

            # If there's no navigable path then switch to 'stop' state
            elif len(Rover.nav_angles) < Rover.stop_forward or Rover.vel <= 0:
                #failed trial :elif len(Rover.nav_angles) < Rover.stop_forward and len(Rover.nav_angles) >100 or Rover.vel <= 0:
                    # Set state to "stop" and hit the brakes!
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.state.append('stop')

            
        elif Rover.state[-1] == 'stuck':
            # if 1 sec passed go back to previous state
            if Rover.total_time - Rover.stuck_time > 1:
                Rover.throttle = Rover.throttle_set

                Rover.brake = 0
                # Set steer to mean angle
                # Hug left wall by setting the steer angle slightly to the left
                Rover.steer = np.clip(np.mean((Rover.nav_angles+offset) * 180 / np.pi), -15, 15)
                Rover.state.pop() 
            else:
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = -15

        elif Rover.state[-1] == 'rock': 
            # Steer torwards the rock
            mean = np.mean(Rover.rock_angles * 180 / np.pi)
            if not np.isnan(mean):
                Rover.steer = np.clip(mean, -15, 15)
            else:
                # No rock in sight anymore. Go back to previous state
                Rover.state.pop()  

            # if 15 sec passed gives up and goes back to previous state
            if Rover.total_time - Rover.rock_time > 15:
                Rover.state.append('stuck')

            # if close to the sample stop
            if Rover.near_sample:
                # Set throttle to zero and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set

            # if it got stuck, it will stop and steer back to the rock
            elif Rover.vel <= 0 and Rover.total_time - Rover.stuck_time > 5:
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = np.clip(np.mean((Rover.rock_angles+offset) * 180 / np.pi), -15, 15)

            else:
                # Approach slowly
                slow_speed = Rover.max_vel / 4
                if Rover.vel < slow_speed:
                    Rover.throttle = 0.1
                    Rover.brake = 0
                else:  # Else break
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set

        
        elif Rover.state[-1] == 'stop':
            # If we're in stop state but still moving keep hitting the brakes
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If the velocity is lower than 0.2, then do something else
            elif Rover.vel <= 0.2:
                # Now the rover isn't moving, and is checking if there is a path infront of it to start moving
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    Rover.brake = 0
                    Rover.steer = -15
                # The rover is still not moving, but has found a navigable path
                if len(Rover.nav_angles) >= Rover.go_forward:
                    Rover.throttle = Rover.throttle_set
                    Rover.brake = 0
                    offset = 12
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi) + offset, -15, 15)
                    Rover.state.pop()  # returns to previous state

    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover