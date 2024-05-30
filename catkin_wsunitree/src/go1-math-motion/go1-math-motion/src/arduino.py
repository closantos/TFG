#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32, Float32
import serial

def arduino_node():
    # Inicializar el nodo ROS
    rospy.init_node('Arduino', anonymous=True)
    
    # Crear los publicadores para los tópicos 'correa' y 'brujula'
    correa_pub = rospy.Publisher('correa', Int32, queue_size=10)
    brujula_pub = rospy.Publisher('brujula', Float32, queue_size=10)
    
    # Frecuencia de publicación (10 Hz)
    rate = rospy.Rate(10)
    
    # Puerto serial donde está conectado el Arduino
    port = "/dev/ttyACM0"
    
    # Velocidad de comunicación del puerto serial
    baudrate = 9600
    
    # Crear el objeto Serial para la comunicación con el Arduino
    ser = serial.Serial(port, baudrate, timeout=1)
    
    while not rospy.is_shutdown():
        # Leer datos del puerto serie del Arduino
        data = ser.readline().decode('utf-8').strip()
        
        # Dividir los datos en brújula y estado de la correa
        if data.startswith("Heading:"):
            try:
                brujula_data = float(data.split(":")[1])
                print("brujula: ", brujula_data)
                brujula_pub.publish(brujula_data)
            except ValueError:
                rospy.logwarn("No se pudo convertir el ángulo a float")
        elif data.startswith("Correa:"):
            try:
                correa_data = int(data.split(":")[1])
                print("correa: ", correa_data)
                correa_pub.publish(correa_data)
            except ValueError:
                rospy.logwarn("No se pudo convertir el estado de la correa a int")
        
        # Esperar según la frecuencia de publicación
        rate.sleep()

if __name__ == '__main__':
    try:
        arduino_node()
    except rospy.ROSInterruptException:
        pass
