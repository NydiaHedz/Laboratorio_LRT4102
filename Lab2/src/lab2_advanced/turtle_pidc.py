#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, radians, sqrt, sin, cos, pi

class MoveTurtlePIDControl:
    def __init__(self):
        rospy.init_node('control_tortuga_xyz')
        
        # Suscribirse al topic de la posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publicar en el topic de comandos de movimiento de la tortuga
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Tasa de publicación de mensajes (10 Hz)
        self.rate = rospy.Rate(10)
        
        # Variables para guardar la posición actual
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0
        
        # Variables para almacenar errores previos
        self.last_error_x = 0
        self.last_error_y = 0
        self.last_error_theta = 0
        
        # Variables para acumulación de error (término integral)
        self.error_accumulation_x = 0
        self.error_accumulation_y = 0
        self.error_accumulation_theta = 0

    def pose_callback(self, pose):
        # Función que se ejecuta cada vez que llega una actualización de la posición de la tortuga
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta

    def move_turtle_to_position(self, desired_x, desired_y, desired_theta=None):
        # Constantes de proporcionalidad, integral y derivativa del controlador (ajustables)
        Kp_x = 1.0
        Ki_x = 0.01
        Kd_x = 0.1
        
        Kp_y = 1.0
        Ki_y = 0.01
        Kd_y = 0.1
        
        Kp_theta = 3.0
        Ki_theta = 0.01
        Kd_theta = 0.1
        
        # Si no se especifica un ángulo deseado, calcular el ángulo hacia el punto objetivo
        if desired_theta is None:
            calculate_theta = True
        else:
            calculate_theta = False
        
        while not rospy.is_shutdown():
            # Calcular los errores de posición
            error_x = desired_x - self.current_x
            error_y = desired_y - self.current_y
            
            # Si no se especificó un ángulo deseado, calcular el ángulo hacia el punto objetivo
            if calculate_theta:
                target_theta = atan2(error_y, error_x)
                error_theta = self.normalize_angle(target_theta - self.current_theta)
            else:
                error_theta = self.normalize_angle(desired_theta - self.current_theta)
            
            # Sumar los errores a la acumulación de errores (término integral)
            self.error_accumulation_x += error_x
            self.error_accumulation_y += error_y
            self.error_accumulation_theta += error_theta
            
            # Limitar la acumulación de errores para evitar el wind-up integral
            max_accumulation = 10.0
            self.error_accumulation_x = max(-max_accumulation, min(self.error_accumulation_x, max_accumulation))
            self.error_accumulation_y = max(-max_accumulation, min(self.error_accumulation_y, max_accumulation))
            self.error_accumulation_theta = max(-max_accumulation, min(self.error_accumulation_theta, max_accumulation))
            
            # Calcular las componentes del controlador PID
            # Primero para theta (orientación), ya que necesitamos orientarnos antes de movernos
            vel_theta = Kp_theta * error_theta + Ki_theta * self.error_accumulation_theta + Kd_theta * (error_theta - self.last_error_theta)
            
            # Luego para x e y (posición)
            vel_x = Kp_x * error_x + Ki_x * self.error_accumulation_x + Kd_x * (error_x - self.last_error_x)
            vel_y = Kp_y * error_y + Ki_y * self.error_accumulation_y + Kd_y * (error_y - self.last_error_y)
            
            # Transformar velocidades de coordenadas globales a locales (del robot)
            linear_x = vel_x * cos(self.current_theta) + vel_y * sin(self.current_theta)
            linear_y = -vel_x * sin(self.current_theta) + vel_y * cos(self.current_theta)
            
            # Guardar los errores actuales para usarlos en la próxima iteración
            self.last_error_x = error_x
            self.last_error_y = error_y
            self.last_error_theta = error_theta
            
            # Crear un mensaje de Twist para enviar el comando de movimiento
            twist_msg = Twist()
            twist_msg.linear.x = linear_x
            twist_msg.linear.y = linear_y  # Nota: En turtlesim, esto no tiene efecto ya que es un robot diferencial
            twist_msg.angular.z = vel_theta
            
            # Publicar el mensaje
            self.velocity_publisher.publish(twist_msg)
            
            # Imprimir la posición actual, el error y las velocidades en la terminal
            rospy.loginfo("Posición actual: x=%f, y=%f, theta=%f", self.current_x, self.current_y, self.current_theta)
            rospy.loginfo("Errores: x=%f, y=%f, theta=%f", error_x, error_y, error_theta)
            rospy.loginfo("Velocidades: lin_x=%f, lin_y=%f, ang_z=%f", linear_x, linear_y, vel_theta)
            
            # Verificar si se alcanza la posición deseada
            distance = sqrt(error_x**2 + error_y**2)
            if distance < 0.1 and (calculate_theta or abs(error_theta) < 0.1):
                rospy.loginfo("Posición deseada alcanzada")
                break
            
            # Esperar hasta la siguiente iteración
            self.rate.sleep()
    
    def normalize_angle(self, angle):
        # Normaliza el ángulo para que esté entre -pi y pi
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def get_desired_position_from_user(self):
        print("\nIngrese la posición deseada:")
        desired_x = float(input("Coordenada x: "))
        desired_y = float(input("Coordenada y: "))
        
        theta_option = input("¿Desea especificar un ángulo theta? (s/n): ").lower()
        if theta_option == 's' or theta_option == 'si' or theta_option == 'sí':
            desired_theta = float(input("Ángulo theta (en radianes): "))
            return desired_x, desired_y, desired_theta
        else:
            return desired_x, desired_y, None

    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            try:
                # Obtener la posición deseada del usuario
                position = self.get_desired_position_from_user()
                
                if len(position) == 3:
                    desired_x, desired_y, desired_theta = position
                    # Mover la tortuga a la posición deseada
                    self.move_turtle_to_position(desired_x, desired_y, desired_theta)
                else:
                    rospy.logerr("Error al obtener la posición deseada")
            except ValueError:
                rospy.logerr("Por favor, ingrese valores numéricos válidos")
            except Exception as e:
                rospy.logerr(f"Error: {str(e)}")
                
            # Preguntar si desea continuar
            cont = input("\n¿Desea mover la tortuga a otra posición? (s/n): ").lower()
            if cont != 's' and cont != 'si' and cont != 'sí':
                break

if __name__ == '__main__':
    try:
        move_turtle_pid = MoveTurtlePIDControl()
        move_turtle_pid.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
