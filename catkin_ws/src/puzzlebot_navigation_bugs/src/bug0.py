#! /usr/bin/env python3

# %% [markdown]
# # Bug0
# ## Equipo 3

# %% [markdown]
# Primero se importan las librerias necesarias.

# %%
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from smach import State, StateMachine
import smach_ros
from tf.transformations import euler_from_quaternion

# %% [markdown]
# Controlador simple

# %%
class ControlPID:
    def __init__(self, rate, kp = 1, ki = 0, kd = 0):
        self.rate = rate
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i = 0
        self.ea = 0
        self.pred = 0

    def update(self, current, goal):
        ep = goal - current
        self.i = self.i + self.ki*ep*self.rate
        ed = (ep - self.ea)/self.rate
        self.ea = ep
        self.pred = current + ep*self.kp + self.i + ed*self.kd


# %% [markdown]
# Luego se declaran variables globales.

# %%
pos = Twist()
c_vel = Twist()
break_v = 0
turn_b = 0
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
rospy.init_node('bug0')
rate = rospy.Rate(10)
puntos = []
rate_dur = 25
rate = rospy.Rate(rate_dur)
rate_dur = 1/rate_dur
turnPID = ControlPID(rate_dur,kp = 0.4)
fowaPID = ControlPID(rate_dur,kp = 0.2,kd = 0.005)

# %% [markdown]
# Declaramos el callback de Odometría el cual se encarga de dar la posición del robot de acuerdo a sus cálculos de las velocidades hechas en cada motor.

# %%
def callbackOdom(msg):
    global pos
    global c_vel
    pos.linear.x = msg.pose.pose.position.x
    pos.linear.y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (_,_,pos.angular.z) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    c_vel.linear.x = msg.twist.twist.linear.x
    c_vel.angular.z = msg.twist.twist.angular.z

# %% [markdown]
# Esta función sirve para dar la diferencia de ángulo entre dos ángulos en el plano cartesiano.

# %%
def smallest_angle_diff(t,s):
    a = t - s
    a -= 2*np.pi if a > np.pi else -2*np.pi if a < -np.pi else 0
    return a

# %% [markdown]
# Función para medir distancia entre dos puntos

# %%
def dis(src, goal):
    return np.sqrt((goal[0]-src[0])**2 + (goal[1]-src[1])**2)

# %% [markdown]
# Este callback verifica con el LiDAR los obstaculos más cercanos los cuales si se detecta alguno calcula la cantidad de freno y desviación para evitar colisión.

# %%
def callbackScan(msg):
    global break_v
    global turn_b

    #Import scan
    scan = np.array(msg.ranges)

    #Get angles array
    ang = msg.angle_min - np.pi
    andf = msg.angle_max - np.pi
    inc = msg.angle_increment
    angles = np.arange(ang,andf+inc,inc)

    #Get only front detected points
    angles_f = angles[np.r_[260:460]]
    scan_f = scan[np.r_[260:460]]
    angles_f = angles_f[~np.isnan(scan_f)]
    scan_f = scan_f[~np.isnan(scan_f)]
    angles_f = angles_f[np.isfinite(scan_f)]
    scan_f = scan_f[np.isfinite(scan_f)]
    
    #Avoid collision of objects when braking and turning away from the obstacle.
    #Maximum reach 0.25 - recommended 0.37
    if scan_f[scan_f < 0.5].shape[0] > 0:
        break_v = (scan_f.min() - 0.25)/scan_f.min()
        try:
            angle = smallest_angle_diff(angles_f[scan_f == scan_f.min()],np.pi)
            turn_b = np.pi/2 if angle < 0 else -np.pi/2
        except:
            turn_b = 0
    else:
        break_v = 1
        turn_b = 0

# %% [markdown]
# Este callback recibe nuevos puntos objetivo

# %%
def callbackGoalPoint(msg):
    global puntos
    puntos.append([msg.pose.position.x, msg.pose.position.y])

# %% [markdown]
# Este es el primer estado el cual envía al robot la velocidad de giro necesaria para alcanzar el ángulo mínimo entre el turtlebot y el punto objetivo, y así iniciar haciendo un recorrido en línea recta.

# %%
class Turn(State):
    def __init__(self):
        State.__init__(self, outcomes=['turnr','fowardr','wait'])
        
    def execute(self, ud):
        global pos
        global c_vel
        global pub
        global rate
        global puntos
        global turnPID

        if len(puntos) == 0:
            return 'wait'
        goal = puntos[0]
        vel = Twist()

        c_ang = np.arctan2(goal[1]-pos.linear.y,goal[0]-pos.linear.x)
        o_ang = smallest_angle_diff(c_ang,pos.angular.z)
        o_ang = o_ang if abs(o_ang) <= np.pi/4 else (np.pi/4)*np.sign(o_ang)
        turnPID.update(c_vel.angular.z, o_ang)
        vel.angular.z = turnPID.pred
        vel.angular.z = vel.angular.z if abs(vel.angular.z) <= np.pi/4 else (np.pi/4)*np.sign(vel.angular.z)
        pub.publish(vel)
        rate.sleep()

        if abs(smallest_angle_diff(c_ang,pos.angular.z)) > 0.05:
            return 'turnr'
        else:
            while c_vel.angular.z > 0.05:
                turnPID.update(c_vel.angular.z, 0)
                vel.angular.z = turnPID.pred
                vel.angular.z = vel.angular.z if abs(vel.angular.z) <= np.pi/4 else (np.pi/4)*np.sign(vel.angular.z)
                pub.publish(vel)
                rate.sleep()
            vel.angular.z = 0
            pub.publish(vel)
            rate.sleep()
            return 'fowardr'

# %% [markdown]
# El segundo estado consiste en avanzar y corregir el rumbo si se está desviando a partir de un giro muy discreto. Si hay obstaculo aquí se aplica el freno y la desviación.

# %%
class Advance(State):
    def __init__(self):
        State.__init__(self, outcomes=['turnr','fowardr','wait'])
        
    def execute(self, ud):
        global pos
        global pub
        global rate
        global puntos
        global break_v
        global turn_b
        global turnPID
        global fowaPID

        goal = puntos[0]
        vel = Twist()

        c_ang = np.arctan2(goal[1]-pos.linear.y,goal[0]-pos.linear.x)
        o_ang = smallest_angle_diff(c_ang,pos.angular.z) if turn_b == 0 else turn_b*(1-break_v) if break_v >= 0 else 0
        o_ang = o_ang if abs(o_ang) <= np.pi/4 else (np.pi/4)*np.sign(o_ang)
        turnPID.update(c_vel.angular.z, o_ang)
        vel.angular.z = turnPID.pred
        vel.angular.z = vel.angular.z if abs(vel.angular.z) <= np.pi/4 else (np.pi/4)*np.sign(vel.angular.z)

        c_dis = dis([pos.linear.x,pos.linear.y],goal)
        o_dis = c_dis if abs(c_dis) <= 0.2 else 0.2*np.sign(c_dis)
        o_dis = c_dis*break_v if break_v > 0 else -0.2
        fowaPID.update(c_vel.linear.x, o_dis)
        vel.linear.x = fowaPID.pred
        vel.linear.x = vel.linear.x if abs(vel.linear.x) <= 0.2 else 0.2*np.sign(vel.linear.x)*abs(break_v)
        pub.publish(vel)
        rate.sleep()

        if c_dis > 0.05:
            return 'fowardr'
        else:
            puntos.pop(0)
            while c_vel.angular.z > 0.05 or c_vel.linear.x > 0.05:
                turnPID.update(c_vel.angular.z, 0)
                vel.angular.z = turnPID.pred
                vel.angular.z = vel.angular.z if abs(vel.angular.z) <= np.pi/4 else (np.pi/4)*np.sign(vel.angular.z)
                fowaPID.update(c_vel.linear.x, 0)
                vel.linear.x = fowaPID.pred
                vel.linear.x = vel.linear.x if abs(vel.linear.x) <= 0.2 else 0.2*np.sign(vel.linear.x)*abs(break_v)
                pub.publish(vel)
                rate.sleep()

            if len(puntos) == 0:
                return 'wait'
            else:
                return 'turnr'

# %% [markdown]
# El tercer estado consiste de esperar nuevos puntos objetivo

# %%
class Wait(State):
    def __init__(self):
        State.__init__(self, outcomes=['turnr','wait'])
    def execute(self, ud):
        global pub
        global rate
        vel = Twist()
        vel.angular.z = 0.0
        vel.linear.x = 0.0
        pub.publish(vel)
        rate.sleep()
        pub.publish(vel)
        rate.sleep()
        if (len(puntos)) > 0:
            return 'turnr'
        else:
            return 'wait'

# %% [markdown]
# Ésta es la función principal donde se inicializan los nodos suscriptores y la máquina de estados.

# %%
def main():
    
    odom = rospy.Subscriber('/odom',Odometry,callbackOdom)
    scanS = rospy.Subscriber('/scan',LaserScan,callbackScan)
    Gpoints = rospy.Subscriber('/move_base_simple/goal',PoseStamped,callbackGoalPoint)
    
    sm = StateMachine(outcomes=['succeeded'])
    sm.userdata.sm_input = 0

    with sm:

        StateMachine.add('WAIT', Wait(), transitions={'turnr':'TURN','wait':'WAIT'})
        StateMachine.add('TURN', Turn(), transitions={'turnr':'TURN','fowardr':'FOWARD','wait':'WAIT'})
        StateMachine.add('FOWARD', Advance(), transitions={'turnr':'TURN','fowardr':'FOWARD','wait':'WAIT'})
    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    rospy.sleep(1)
    sis.start()
    
    outcome = sm.execute()

    sis.stop()
    
    rospy.sleep(1)

# %% [markdown]
# Finalmente, para ejecutar el código oficialmente (sin antes haber ya ejecutado las celdas anteriores y lanzado el mundo de turtlebot en Gazebo), se ejecuta la siguiente celda para iniciar con el proceso:

# %%
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


