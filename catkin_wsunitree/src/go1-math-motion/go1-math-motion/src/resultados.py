#!/usr/bin/env python3
import rospy
import numpy as np
import csv
import os
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Variables
dist_critica = 1500
mucha_dist = 3000
peligro_lateral = 1500
max_speed = 0.8
precaucion_speed = 0.3
contador_tiempo = 0
velocidad_angular = 0.3
izquierda = []
centro = []
derecha = []

# Ruta del archivo CSV
csv_filename = os.path.join(os.path.dirname(__file__), 'prueba1.csv')

# Crear el archivo CSV y escribir la cabecera si no existe
if not os.path.isfile(csv_filename):
    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Tiempo", "Izquierda Max", "Izquierda Min", "Centro Max", "Centro Min", "Derecha Max", "Derecha Min"])

# Leo matriz de profundidad y almaceno los minimos de cada columna en 3 vectores
def obtener_vectores_minimos(imagen):
    matriz = np.array(imagen)
    _, ancho = matriz.shape
    vector_izquierda = []
    vector_centro = []
    vector_derecha = []

    for i in range(ancho):
        columna = matriz[150:300, i]

        valores_filtrados = columna[(columna > 0) & (columna < 500)]  # Filtrar valores en el rango deseado
        num_valores_filtrados = len(valores_filtrados)

        if num_valores_filtrados > 30:
            minimo = np.mean(valores_filtrados)
        else:
            minimo = np.mean(columna)

        if 150 < i < 200:
            vector_izquierda.append(minimo)
        elif 200 < i < 440:
            vector_centro.append(minimo)
        elif 440 < i < 490:
            vector_derecha.append(minimo)

    return vector_izquierda, vector_centro, vector_derecha

# Si hay varios minimos seguidos significa que hay un obstaculo
def obstaculos(vector):
    contador = 0
    for valor in vector:
        if valor < dist_critica:
            contador += 1
            if contador >= 20:
                return True
        else:
            contador = 0
    return False

def callback_depth(data):
    global izquierda, derecha, centro
    # Convertir el mensaje de ROS a una matriz numpy
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    # Obtener los vectores minimos
    izquierda, centro, derecha = obtener_vectores_minimos(depth_image)

    # Calcular el tiempo transcurrido desde el inicio del programa
    tiempo_actual = rospy.get_time() - tiempo_inicial
    print(f"Tiempo: {tiempo_actual:.2f}")
    print(f"Izquierda - Max: {max(izquierda):.2f}, Min: {min(izquierda):.2f}")
    print(f"Centro - Max: {max(centro):.2f}, Min: {min(centro):.2f}")
    print(f"Derecha - Max: {max(derecha):.2f}, Min: {min(derecha):.2f}")

    # Guardar los resultados en el archivo CSV
    with open(csv_filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([tiempo_actual, max(izquierda), min(izquierda), max(centro), min(centro), max(derecha), min(derecha)])

    # Llamar a la funcion moverse con los valores de los vectores
    moverse(izquierda, centro, derecha)

def moverse(izquierda, centro, derecha):
    global max_speed, mucha_dist, dist_critica, contador_tiempo, precaucion_speed, velocidad_angular
    rate = rospy.Rate(100000)
    vel = Twist()

    # Velocidades que no se modifican
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0

    # NO PELIGRO de obstaculos
    if min(centro) > mucha_dist and not obstaculos(centro):
        vel.linear.x = max_speed
        contador_tiempo = 0
        print("NO HAY OBSTACULO")
    # POSIBLE PELIGRO
    elif min(centro) < mucha_dist and not obstaculos(centro):
        vel.linear.x = precaucion_speed # Bajo velocidad
        contador_tiempo = 0
        print("POSIBLE OBSTACULO")
        # Compruebo lados para ir apartandome
        if min(izquierda) > peligro_lateral and not obstaculos(izquierda) and min(izquierda) > min(derecha):
            vel.angular.z = velocidad_angular # giro izq
            print("IZQUIERDA POR SI ACASO")
        elif min(derecha) > peligro_lateral and not obstaculos(derecha) and min(izquierda) < peligro_lateral and min(derecha) > min(izquierda):
            vel.angular.z = -velocidad_angular # giro derecha
            print("DERECHA POR SI ACASO")
        else:
            vel.angular.z = 0.1 # si hay obstaculos a ambos lados decido uno aleatoriamente
            print("ALEATORIO")
    # PELIGRO = paro/evito
    elif obstaculos(centro):
        vel.linear.x = 0
        vel.angular.z = 0
        print("OBSTACULOOOOOOOOOOOOOOOOO")
        contador_tiempo += 1
        # vuelvo a comprobar delante para ver si de verdad era un obstaculo no pasajero
        if contador_tiempo >= 20:
            if min(izquierda) > peligro_lateral and not obstaculos(izquierda):
                vel.angular.z = velocidad_angular # giro izq
                print("OBSTACULO DELANTE, GIRO A LA IZQUIERDA")
            elif min(derecha) > peligro_lateral and not obstaculos(derecha) and min(izquierda) < peligro_lateral:
                vel.angular.z = -velocidad_angular # giro derecha
                print("OBSTACULO DELANTE, GIRO A LA DERECHA")
            else:
                vel.angular.z = velocidad_angular # si hay obstaculos a ambos lados decido uno aleatoriamente
                print("OBSTACULO DELANTE, GIRO ALEATORIO")

    # PELIGROS LATERALES
    # OBSTACULO IZQUIERDA
    if min(izquierda) < peligro_lateral and min(derecha) > peligro_lateral:
        vel.angular.z = -velocidad_angular # giro derecha para no chocarme
        print("OBSTACULO IZQUIERDA")
    # OBSTACULO DERECHA
    elif min(derecha) < peligro_lateral and min(izquierda) > peligro_lateral:
        vel.angular.z = velocidad_angular # giro izquierda para no chocarme
        print("OBSTACULO DERECHA")
    elif min(derecha) > peligro_lateral and min(izquierda) > peligro_lateral and not obstaculos(centro):
        vel.angular.z = 0 # recto, sin obstaculos

    pub.publish(vel)

if __name__ == '__main__':
    try:
        rospy.init_node("walk", anonymous=True)
        tiempo_inicial = rospy.get_time()
        rospy.Subscriber("/depth", Image, callback_depth)
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=2)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
