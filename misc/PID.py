
#-----------------------------------------------------------------------------
# 
#   Omar Fernandez-Contreras 
#   xacademy 
#   P.I.D.py // FileName
#   demo for testing PID controller using formula and simulated with a graph
#   for future reference PID coefficient would have sign impact on performace, based on robot in my case
# 
#-----------------------------------------------------------------------------

#   Importing Libraries
import time 
import matplotlib.pyplot as plt

#   PID Controller Function

def pid_controller(setpoint,pv,kp,ki,kd,previous_error, integral,dt):
    error = setpoint - pv
    integral += error * dt
    derivative =  (error - previous_error) / dt
    control = kp * error + ki * integral + kd * derivative
    return control, error, integral

def main():
    setpoint = 100 # desire setpoint 
    pv = 0 #initial process varible
    kp = 0.6 #  proportional gain
    ki = 0.3 # integral gain
    kd = 0.1 # derivative gain
    previous_error = 0
    integral = 0
    dt = 0.1 #  Time step
    #   Data Storage Initialization
    time_steps = []
    pv_values = []
    control_values = []
    setpoint_values = []

    for i in range(100):
        ctrl,err,integral = pid_controller(setpoint, pv, kp, ki, kd, previous_error, integral, dt)
        pv += ctrl * dt #update process varible based on control output
        previous_error = err
        
        time_steps.append( i * dt)
        pv_values.append(pv)
        control_values.append(ctrl)
        setpoint_values.append(setpoint)

        time.sleep(dt)

    plt.figure(figsize=(12, 6))
    
    plt.subplot(2, 1, 1)
    plt.plot(time_steps, pv_values, label='Process Variable (PV)')
    plt.plot(time_steps, setpoint_values, label='Setpoint', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.title('Process Variable vs. Setpoint')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(time_steps, control_values, label='Control Output')
    plt.xlabel('Time (s)')
    plt.ylabel('Control Output')
    plt.title('Control Output over Time')
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()