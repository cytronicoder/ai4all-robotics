import time
import numpy as np

import xpc3_helper
import xpc3

def send_controls(client, controls):    
    for k in controls.keys():
        if k not in ["elev", "aileron", "rudder", "throttle"]:
            print("unrecognized control ", k)
            
    elev = np.clip(controls.get("elev", 0), -1, 1)
    aileron = np.clip(controls.get("aileron", 0), -1, 1)
    rudder = np.clip(controls.get("rudder", 0), -1, 1)
    throttle = np.clip(controls.get("throttle", 0), -1, 1)
    client.sendCTRL([elev, aileron, rudder, throttle])

def getSpins(client):
    time.sleep(0.001)
    roll_speed = client.getDREF("sim/flightmodel/position/P")[0]
    time.sleep(0.001)
    pitch_speed = client.getDREF("sim/flightmodel/position/Q")[0]
    time.sleep(0.001)
    yaw_speed = client.getDREF("sim/flightmodel/position/R")[0]
    return roll_speed, pitch_speed, yaw_speed

def getVertSpeed(client):
    time.sleep(0.001)
    vert_speed = client.getDREF("sim/flightmodel/position/vh_ind")[0]
    return vert_speed
    
def coords(client):
    time.sleep(0.001)
    lat = client.getDREF("sim/flightmodel/position/latitude")[0]
    time.sleep(0.001)
    lon = client.getDREF("sim/flightmodel/position/longitude")[0]
    time.sleep(0.001)
    el = client.getDREF("sim/flightmodel/position/elevation")[0]
    return lat, lon, el
    
    
def angles(client):
    time.sleep(0.001)
    psi = client.getDREF("sim/flightmodel/position/psi")[0]
    time.sleep(0.001)
    theta = client.getDREF("sim/flightmodel/position/theta")[0]
    time.sleep(0.001)
    phi = client.getDREF("sim/flightmodel/position/phi")[0]
    return psi, theta, phi


class TaxiController:
    def __init__(self, cte_Kp, he_Kp):
        # Define your set points
        self.speed_setpoint = 5
    
        # Gains
        self.cte_Kp = cte_Kp
        self.he_Kp = he_Kp
        self.throttle_Kp = 0.1
    
    def control(self, client, state):
        rudder = 0.008 + self.cte_Kp * state["cte"] + self.he_Kp * state["he"]
        throttle = self.throttle_Kp * (self.speed_setpoint - state["speed"])
        return {"rudder":rudder, "throttle":throttle}

class TakeoffController:
    def __init__(self, cte_Kp, he_Kp, cte_Kd, he_Kd):
        # Previous errors
        self.prev_cte = None
        self.prev_he = None
        
        # Gains
        self.cte_Kp = cte_Kp
        self.he_Kp = he_Kp
        
        self.cte_Kd = cte_Kd
        self.he_Kd = he_Kd
    
    def control(self, client, state):
        cte = state["cte"]
        he = state["he"]
        
        if self.prev_cte is None:
            self.prev_cte = cte
            self.prev_he = he
        
        
        rudder = 0.008 + self.cte_Kp * cte + self.he_Kp * he
        rudder += self.cte_Kd * (cte - self.prev_cte) + self.he_Kd * (he - self.prev_he)
        throttle = 1
        
        # Store the last errors
        self.prev_cte = cte
        self.prev_he = he
        
        return {"rudder":rudder, "throttle":throttle}


class ClimbController:
    def __init__(self, elevator_Kp, elevator_Kd):
        # Define your set points
        self.vert_speed_setpoint = 10
        
        self.prev_err = None
        
        # Gains
        self.elevator_Kp = elevator_Kp
        self.elevator_Kd = elevator_Kd
        
        # The initial elevator positioning
        self.elevator_set = 0
    
    def control(self, client, state):
        err = self.vert_speed_setpoint - state["vert_speed"]
        
        if self.prev_err is None:
            self.prev_err = err
        
        delta_elev = self.elevator_Kp * err + self.elevator_Kd * (err - self.prev_err)
        
        self.prev_err = err
        self.elevator_set += delta_elev
        
        self.elevator_set = np.clip(self.elevator_set, -0.1, 0.1)
        
        #print("elevator setting: ", self.elevator_set, " speed: ", state["vert_speed"])
        
        return {"elev": self.elevator_set, "throttle": 1}
        
            
        
def simulate_controllers(client, startCTE, startHE, startDTP, 
                         taxi_controller=TaxiController(0.015, 0.008), 
                         takeoff_controller=TakeoffController(0.07, 0.035, 0.01, 0.01),
                         climb_controller=ClimbController(0.001, 0.01),
                         simSpeed=1.0):
    """ Simulates a controller using the built-in X-Plane 11 dynamics

        Args:
            client: XPlane Client
            startCTE: Starting crosstrack error (meters)
            startHE: Starting heading error (degrees)
            startDTP: Starting downtrack position (meters)
            endDTP: Ending downtrack position (meters)
            getState: Function to estimate the current crosstrack and heading errors.
                      Takes in an XPlane client and returns the crosstrack and
                      heading error estimates
            getControl: Function to perform control based on the state
                        Takes in an XPlane client, the current crosstrack error estimate,
                        and the current heading error estimate and returns a control effort
            -------------------
            simSpeed: increase beyond 1 to speed up simulation
    """
    # Reset to the desired starting position
    client.sendDREF("sim/time/sim_speed", simSpeed)
    xpc3_helper.reset(client, cteInit = startCTE, heInit = startHE, dtpInit = startDTP)
    xpc3_helper.sendBrake(client, 0)

    time.sleep(2)  # 5 seconds to get terminal window out of the way
    client.pauseSim(False)

    time.sleep(0.001)
    init_elevation = client.getDREF("sim/flightmodel/position/elevation")[0]
    dtp = startDTP
    startTime = client.getDREF("sim/time/zulu_time_sec")[0]
    endTime = startTime
    
    # Lets start witht the taxi controller
    controller = taxi_controller

    print("Taxiing!")

    while True:
        
        # Get relevant state variables
        speed = xpc3_helper.getSpeed(client)
        cte, dtp, he = xpc3_helper.getHomeState(client)
        lat, lon, el = coords(client)
        psi, theta, phi = angles(client)
        roll_speed, pitch_speed, yaw_speed = getSpins(client)
        vert_speed = getVertSpeed(client)
        
        # Store them in a state dictionary
        state = {"speed" : speed, "cte" : cte, "he" : he,
                 "lat" : lat, "lon" : lon, "el" : el,
                 "psi" : psi, "theta" : theta, "phi" : phi,
                 "roll_speed" : roll_speed, "pitch_speed" : pitch_speed, "yaw_speed" : yaw_speed,
                 "vert_speed" : vert_speed}
        
        
        # print(state)
        
        # Set the controller here if you need to
        
        # If we are taxiing and we reach the center of the runway, lets take off!
        if controller == taxi_controller and abs(state["he"]) < 1 and abs(state["cte"]) < 1:
            print("Taking off!")
            controller = takeoff_controller
            
        if controller == takeoff_controller and abs(state["speed"]) > 30:
            print("Climbing!")
            controller = climb_controller
            
            

        # Get and send the controls from our controller
        ctrl = controller.control(client, state)
        send_controls(client, ctrl)
        
        # Wait for next timestep
        while endTime - startTime < 1:
            time.sleep(0.01)
            endTime = client.getDREF("sim/time/zulu_time_sec")[0]
            

        # Set things for next round
        time.sleep(0.01)
        startTime = client.getDREF("sim/time/zulu_time_sec")[0]
        endTime = startTime
        
        time.sleep(0.001)

    client.pauseSim(True)