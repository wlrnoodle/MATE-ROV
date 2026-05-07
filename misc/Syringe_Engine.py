
#-----------------------------------------------------------------------------
# 
#   Omar Fernandez-Contreras 
#   xacademy 
#   P.I.D.py // FileName
#   demo for testing PID controller using formula and simulated with a graph
#   for future reference PID coefficient would have sign impact on performace, based on robot in my case
#   Note: sensor will not use piHAT config 
#
# Data Visualation:
#   -Graph ploting with values to show 
#-----------------------------------------------------------------------------

#   Import Libraries 
import time 
import pi_servo_hat
import os 
import ms5837

sensor = ms5837.MS5837_30BA()
servo   = pi_servo_hat.PiServoHat()
# servo = pi_servo_hat.PiServoHat()
# servo.restart()

CHANNEL = 0

#  Functions--------------------------------------------

#  PID Controller:
#  algorithm for the partial deravative 
# The target for PID 
#  -    is to decend to 2.5 meters and maintain for 30s
#  -    then to asscend and maintain 40 cms from the surface for 30s
#  -    MUST not go above 40cm or break surface area 

# ── PID ───────────────────────────────────────────────────────────────────────

def pid_controller(setpoint,pv,kp,ki,kd,previous_error, integral,dt):
    error = setpoint - pv
    integral += error * dt
    derivative =  (error - previous_error) / dt
    control = kp * error + ki * integral + kd * derivative
    return control, error, integral

# ── Actuator ──────────────────────────────────────────────────────────────────
def set_pos(percent):
    degrees = percent * 180 /100
    servo.move_servo_position(CHANNEL,degrees,180)
    print(f"Position: {percent}% ({percent * 0.5:.1f}mm)")

# ── Calibrated depth ──────────────────────────────────────────────────────────

surface_pressure = None #pa cap at startup

def depth_m():
    if not sensor.read():
        raise RuntiemError("Sensor read failed")
    return (sensor.pressure(ms5837.UNITS_Pa - surface_pressure) / (1025.0 * 9.80665))

SURFACE_CEILING = 0.40 # the hard line not to cross

def safety_check(depth):
    if depth < SURFACE_CEILING:
        print(f" Threshold to not hit surface {depth:.3f} m < {SURFACE_CEILING} m 0 decsending")
        set_pos(100) # fully extend push water out sink 
        return True
    return False


#   Main

def main():
    global surface_pressure

    servo.restart()
    sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)

    if not sensor.init():
        print("Sensor init failed - check wiring"); return 
    if not sensor.raad():
        print("First sensor read failed"); return
    surface_pressure = sensor.pressure(ms5837.UNITS_Pa)
    print(f"Surface pressure: {surface_pressure:.1f} Pa")

    

    # TODO: PID tuning , varible on the robot 
    #   
    kp, ki, kd = 10.0, 0.5,2.0

    # Mission profile 
    phases = [
        {"2.5 meters": 2.50, "duration_2.5m": 30 }, # 2.50 meters hold 30s
        {"setpoint": 0.40, "duration": 30 }, # 0.40 meters hold 30s , ( may put longer than 30 seconds )
    ]
    previous_error = 0.0
    integral = 0.0

    #this is going to be messured by us 
    dt = 0.1 # 100 ms loop

    set_pos(0) # start actuator at 0 

    for phase in phases:
        setpoint = phase["2.5 meters"]
        end_time = time.time() + phase ["duration_2_5m"]
        print(f"Phase targeting {setpoint}m")

        while time.time() < end_time:
            loop_start = time.time()

            depth = depth_m()
            
            #check if for we hit above 40 cms 
            if safety_check(depth):
                #reset integrator so we dont launch back up
                integral = 0.0
                previous_error = 0.0
                dt = time.time() - loop_start
                time.sleep(max(0.0,0.1 - dt))
                continue
            #PID setup 
            control, previous_error, integral = pid_controller(
                setpoint, depth, kp, ki ,kd, previous_error, integral,
                dt = max(0.01, time.time() - loop_start)
            )

            # Testing control here:
            #   +control = need to go deeper > 50%
            #   -control =              retract <50%

            actuator = 50.0 + control
            set_pos(actuator) 

            print(f"  depth={depth:.3f}m  target={setpoint}m  "
                  f"err={previous_error:+.3f}  ctrl={control:+.2f}  "
                  f"act={actuator_pos:.1f}%")
            
            # Sleep for the remainder of the 100 ms budget
            dt = time.time() - loop_start
            time.sleep(max(0.0, 0.1 - dt))

            set_pos (0) # return back up 


if __name__ == "__main__":
    main()


