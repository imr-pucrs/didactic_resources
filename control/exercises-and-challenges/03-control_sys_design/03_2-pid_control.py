import matplotlib.pyplot as plt

# Sampling time
T = 0.001

# PID Control (ziegler-nichols and manual fine-tuning)
Kp, Ki, Kd = 25.8, 48.2, 0.803 
I = 0
lsat, usat = 0, 255     # Controller saturation limits

# System vectors
r = [0,0]
e = [0,0]
u = [0,0]
y = [0,0]
t = [0,0]

# G(s) - Based on generic DC motor model 
# (constructive parameters in slides)
def DCMotor(u):
    a = ( 20.02*(T**2) + 12*T + 1 )
    return ((2*(T**2))/a)*u + ((2+12*T)/a)*y[-1] - (1/a)*y[-2]

# C(s)
# "u" here is the last controller output, a workaround for antiwindup in python
def PID(u):
    global I

    # Error
    e.append(r[-1] - y[-1])
    
    # Integral action with anti-windup
    within_saturation = (lsat < u and u < usat)
    upper_exceeded_but_negative_error = (u > usat and e < 0)
    lower_exceeded_but_positive_error = (u < lsat and e > 0)
    if within_saturation or upper_exceeded_but_negative_error or lower_exceeded_but_positive_error:
        I += e[-1]*T
    
    # Derivative
    D = (e[-1]-e[-2])/T

    # P + I + D
    u = Kp*e[-1] + Ki*I + Kd*D 

    # Controller saturation
    if u > usat:
        u = usat
    elif u < lsat:
        u = lsat
    return u

while True:
    r.append(1)             # Step input
    u.append(PID(u[-1]))
    y.append(DCMotor(u[-1]))

    t.append(t[-1]+T)
    if len(t) > 4/T:
        break

#plt.plot(t,u,'r--',linewidth=2.0)
plt.plot(t,y,linewidth=2.0)
plt.grid()
plt.xlabel('Time (s)')
plt.ylabel('Angular Speed (rad/s)')
plt.show()