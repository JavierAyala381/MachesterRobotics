#! /usr/bin/env python3

# %% [markdown]
# # Bug2
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
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
rospy.init_node('bug2')
rate_dur = 10
rate = rospy.Rate(rate_dur)
rate_dur = 1/rate_dur
puntos = []
start = []
goal = []
temp = []

nfb = False
ilb = False
sl = 0
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
    #c_vel.linear.x = np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
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
    global nfb
    global ilb
    global sl

    #Get angles array
    scan = np.array(msg.ranges)
    ang = msg.angle_min - np.pi
    andf = msg.angle_max - np.pi
    inc = msg.angle_increment
    angles = np.arange(ang,andf+inc,inc)

    #Get only front detected points
    scan_f = scan[np.r_[270:360]]
    scan_f = scan_f[~np.isnan(scan_f)]
    scan_f = scan_f[np.isfinite(scan_f)]

    #Get only left detected points
    anglesl = angles[np.r_[450:540]]
    scan_l = scan[np.r_[450:540]]
    anglesl = anglesl[~np.isnan(scan_l)]
    scan_l = scan_l[~np.isnan(scan_l)]
    anglesl = anglesl[np.isfinite(scan_l)]
    scan_l = scan_l[np.isfinite(scan_l)]

    #Get booleans if obstacle in front and wall in its left
    nfb = not scan_f[scan_f < 0.45].shape[0] > 0 and not scan_l[scan_l < 0.3].shape[0] > 0
    ilb = scan_l[scan_l < 0.45].shape[0] > 0

    #Calculate slope with regression and give it as error
    if scan_l[scan_l < 0.6].shape[0] >= 13:
        px = scan_l*np.cos(anglesl)
        py = scan_l*np.sin(anglesl)
        sl = np.sum((px - np.average(px))*(py -np.average(py)))/np.sum((px - np.average(px))**2)

# %% [markdown]
# Este callback recibe nuevos puntos objetivo

# %%
def callbackGoalPoint(msg):
    global puntos
    puntos.append([msg.pose.position.x, msg.pose.position.y])

# %% [markdown]
# Este es el primer estado el cual envía al robot la velocidad de giro necesaria para alcanzar el ángulo mínimo entre el turtlebot y el punto objetivo, y así iniciar haciendo un recorrido en línea recta.
# 

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
        global goal
        global start

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
            start = [pos.linear.x,pos.linear.y]
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
# 

# %%
class Advance(State):
    def __init__(self):
        State.__init__(self, outcomes=['turnr','fowardr','wait','at'])
        
    def execute(self, ud):
        global pos
        global pub
        global rate
        global puntos

        global turnPID
        global fowaPID

        global nfb
        global temp

        goal = puntos[0]
        vel = Twist()

        c_ang = np.arctan2(goal[1]-pos.linear.y,goal[0]-pos.linear.x)
        o_ang = smallest_angle_diff(c_ang,pos.angular.z)
        o_ang = o_ang if abs(o_ang) <= np.pi/4 else (np.pi/4)*np.sign(o_ang)
        turnPID.update(c_vel.angular.z, o_ang)
        vel.angular.z = turnPID.pred
        vel.angular.z = vel.angular.z if abs(vel.angular.z) <= np.pi/4 else (np.pi/4)*np.sign(vel.angular.z)

        c_dis = dis([pos.linear.x,pos.linear.y],goal)
        o_dis = c_dis if abs(c_dis) <= 0.2 else 0.2*np.sign(c_dis)
        fowaPID.update(c_vel.linear.x, o_dis)
        vel.linear.x = fowaPID.pred
        vel.linear.x = vel.linear.x if abs(vel.linear.x) <= 0.2 else 0.2*np.sign(vel.linear.x)
        pub.publish(vel)
        rate.sleep()
        
        if nfb:
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
                    vel.linear.x = vel.linear.x if abs(vel.linear.x) <= 0.2 else 0.2*np.sign(vel.linear.x)
                    pub.publish(vel)
                    rate.sleep()
                if len(puntos) == 0:
                    return 'wait'
                else:
                    return 'turnr'
        else:
            temp = [pos.linear.x,pos.linear.y]
            return 'at'

# %% [markdown]
# El tercer estado consiste de esperar nuevos puntos objetivo

# %%
class Wait(State):
    def __init__(self):
        State.__init__(self, outcomes=['turnr','wait'])
    def execute(self, ud):
        global pub
        global rate
        global start
        global pos
        vel = Twist()
        vel.angular.z = 0
        vel.linear.x = 0
        pub.publish(vel)
        rate.sleep()
        pub.publish(vel)
        rate.sleep()
        if (len(puntos)) > 0:
            start = [pos.linear.x,pos.linear.y]
            return 'turnr'
        else:
            return 'wait'

# %% [markdown]
# El estado de giro solo se activa cuando tiene obstaculo enfrente.

# %%
class Turn_so(State):
    def __init__(self):
        State.__init__(self, outcomes=['a','at','t','turnr'])
    def execute(self, ud):
        global pub
        global rate
        global rate_dur
        global nfb
        global ilb
        global c_vel
        global pos

        global start
        global goal
        global temp
        
        global turnPID
        global fowaPID
        
        vel = Twist()
        turnPID.update(c_vel.angular.z, -np.pi/4)
        vel.angular.z = turnPID.pred
        vel.angular.z = vel.angular.z if abs(vel.angular.z) <= np.pi/4 else (np.pi/4)*np.sign(vel.angular.z)
        fowaPID.update(c_vel.linear.x, 0)
        vel.linear.x = fowaPID.pred
        vel.linear.x = vel.linear.x if abs(vel.linear.x) <= 0.2 else 0.2*np.sign(vel.linear.x)
        pub.publish(vel)
        rate.sleep()

        ang = np.arctan2(start[1] - pos.linear.y, start[0] - pos.linear.x) - np.arctan2(goal[1] - pos.linear.y, goal[0] - pos.linear.x)
        if ang < 0.05 and dis(temp, [pos.linear.x,pos.linear.y]) > 0.5:
            temp = []
            while c_vel.angular.z > 0.05 or c_vel.linear.x > 0.05:
                    turnPID.update(c_vel.angular.z, 0)
                    vel.angular.z = turnPID.pred
                    vel.angular.z = vel.angular.z if abs(vel.angular.z) <= np.pi/4 else (np.pi/4)*np.sign(vel.angular.z)
                    fowaPID.update(c_vel.linear.x, 0)
                    vel.linear.x = fowaPID.pred
                    vel.linear.x = vel.linear.x if abs(vel.linear.x) <= 0.2 else 0.2*np.sign(vel.linear.x)
                    pub.publish(vel)
                    rate.sleep()
            return 'turnr'
            
        if not nfb:
            return "t"
        elif ilb:
            return "a"
        else:
            return "at"

# %% [markdown]
# El estado de avance y giro se activa cuando no hay obstaculos enfrente ni a su izquierda.

# %%
class AdvanceTurn(State):
    def __init__(self):
        State.__init__(self, outcomes=['a','at','t','turnr'])
    def execute(self, ud):
        global pub
        global rate
        global nfb
        global ilb
        global c_vel
        global pos

        global start
        global goal
        global temp

        global turnPID
        global fowaPID
        
        vel = Twist()
        turnPID.update(c_vel.angular.z, np.pi/4)
        vel.angular.z = turnPID.pred
        vel.angular.z = vel.angular.z if abs(vel.angular.z) <= np.pi/4 else (np.pi/4)*np.sign(vel.angular.z)
        fowaPID.update(c_vel.linear.x, 0.1)
        vel.linear.x = fowaPID.pred
        vel.linear.x = vel.linear.x if abs(vel.linear.x) <= 0.2 else 0.2*np.sign(vel.linear.x)
        pub.publish(vel)
        rate.sleep()

        ang = np.arctan2(start[1] - pos.linear.y, start[0] - pos.linear.x) - np.arctan2(goal[1] - pos.linear.y, goal[0] - pos.linear.x)
        if ang < 0.05 and dis(temp, [pos.linear.x,pos.linear.y]) > 0.5:
            temp = []
            while c_vel.angular.z > 0.05 or c_vel.linear.x > 0.05:
                    turnPID.update(c_vel.angular.z, 0)
                    vel.angular.z = turnPID.pred
                    vel.angular.z = vel.angular.z if abs(vel.angular.z) <= np.pi/4 else (np.pi/4)*np.sign(vel.angular.z)
                    fowaPID.update(c_vel.linear.x, 0)
                    vel.linear.x = fowaPID.pred
                    vel.linear.x = vel.linear.x if abs(vel.linear.x) <= 0.2 else 0.2*np.sign(vel.linear.x)
                    pub.publish(vel)
                    rate.sleep()
            return 'turnr'
            
        if not nfb:
            return "t"
        elif ilb:
            return "a"
        else:
            return "at"

# %% [markdown]
# El estado de avance se activa cuando no hay obstáculo en frente y tiene pared a su izquierda. Para estar paralelo a la pared utiliza la pendiente como error donde 0 sería que está totalmente paralelo.
# 

# %%
class Advance_so(State):
    def __init__(self):
        State.__init__(self, outcomes=['a','at','t','turnr'])
    def execute(self, ud):
        global pub
        global rate
        global nfb
        global ilb
        global sl
        global pos

        global start
        global goal
        global temp

        global turnPID
        global fowaPID
        
        vel = Twist()
        turnPID.update(c_vel.angular.z, sl)
        vel.angular.z = turnPID.pred
        vel.angular.z = vel.angular.z if abs(vel.angular.z) <= np.pi/4 else (np.pi/4)*np.sign(vel.angular.z)
        fowaPID.update(c_vel.linear.x, 0.2)
        vel.linear.x = fowaPID.pred
        vel.linear.x = vel.linear.x if abs(vel.linear.x) <= 0.2 else 0.2*np.sign(vel.linear.x)
        pub.publish(vel)
        rate.sleep()

        ang = np.arctan2(start[1] - pos.linear.y, start[0] - pos.linear.x) - np.arctan2(goal[1] - pos.linear.y, goal[0] - pos.linear.x)
        if ang < 0.05 and dis(temp, [pos.linear.x,pos.linear.y]) > 0.5:
            temp = []
            while c_vel.angular.z > 0.05 or c_vel.linear.x > 0.05:
                    turnPID.update(c_vel.angular.z, 0)
                    vel.angular.z = turnPID.pred
                    vel.angular.z = vel.angular.z if abs(vel.angular.z) <= np.pi/4 else (np.pi/4)*np.sign(vel.angular.z)
                    fowaPID.update(c_vel.linear.x, 0)
                    vel.linear.x = fowaPID.pred
                    vel.linear.x = vel.linear.x if abs(vel.linear.x) <= 0.2 else 0.2*np.sign(vel.linear.x)
                    pub.publish(vel)
                    rate.sleep()
            return 'turnr'

        if not nfb:
            return "t"
        elif ilb:
            return "a"
        else:
            return "at"

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
        StateMachine.add('FOWARD', Advance(), transitions={'turnr':'TURN','fowardr':'FOWARD','wait':'WAIT','at':'ADVANCETURN'})
        StateMachine.add('TURN_so', Turn_so(), transitions={'t':'TURN_so','at':'ADVANCETURN','a':'ADVANCE_so','turnr':'TURN'})
        StateMachine.add('ADVANCE_so', Advance_so(), transitions={'t':'TURN_so','at':'ADVANCETURN','a':'ADVANCE_so','turnr':'TURN'})
        StateMachine.add('ADVANCETURN', AdvanceTurn(), transitions={'t':'TURN_so','at':'ADVANCETURN','a':'ADVANCE_so','turnr':'TURN'})
    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    rospy.sleep(1)
    sis.start()
    
    outcome = sm.execute()

    sis.stop()
    
    rospy.sleep(1)

# %% [markdown]
# Finalmente, para ejecutar el código oficialmente (sin antes haber ya ejecutado las celdas anteriores y lanzado el mundo de turtlebot en Gazebo), se ejecuta la siguiente celda para iniciar con el proceso:
# 

# %%
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass


