#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from std_msgs.msg import Int32
from std_msgs.msg import Float32
import json
import math

# Variables globales
dist_critica = 1500
mucha_dist = 3000
peligro_lateral = 1500
max_speed = 0.8
precaucion_speed = 0.3
contador_tiempo = 0
contador_espera_paso = 0
velocidad_angular = 0.3
paso_peatones = False
correa = False
marcha = False
latitud_actual = 0.0
longitud_actual = 0.0
indice_punto_control = 0
contador_actualizar_angulo = 0
angulo_brujula = 0.0
tolerancia_angulo = 5
gps_off = True
destino = False
punto_control_actual = {'latitud': 0.0, 'longitud': 0.0}
contador_actualizar_angulo = 0
objetos_detectados = []
vel = Twist()
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=2)

# Funcion para cargar los puntos de control desde un archivo JSON
def cargar_puntos_de_control(prueba):
    # Cargo los puntos de control solo si tengo conexion al GPS
    with open(prueba, "r") as archivo:
        puntos_de_control = json.load(archivo)
        print("puntos de control: ", puntos_de_control)
    return puntos_de_control

# Funcion para calcular la distancia entre dos puntos geograficos
def calcular_distancia(lat1, lon1, lat2, lon2):
    R = 6371.0  # Radio de la Tierra en km
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    dlon = lon2_rad - lon1_rad
    dlat = lat2_rad - lat1_rad
    a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distancia = R * c
    return distancia

# Funcion para orientar el robot hacia el proximo punto de control
def orientar_robot(punto_control, angulo_brujula):
    global latitud_actual, longitud_actual
    # Calcular la diferencia entre las coordenadas del punto de control y las coordenadas actuales
    delta_lat = punto_control['latitud'] - latitud_actual # 38.3871802  - 38.386805 
    #print(punto_control['latitud'] )
    delta_lon =  punto_control['longitud'] - longitud_actual # -0.5122838 - (-0.511371) 
    #print(punto_control['longitud'])

    # Calcular el angulo entre el punto de control y el robot
    angulo_punto_control = math.atan2(delta_lon, delta_lat) * (180 / math.pi) 

    if angulo_punto_control < 0:
        angulo_punto_control += 360
    print("Angulo a punto de control: ",angulo_punto_control)
    print("Angulo robot: ", angulo_brujula)

    # Lo normal en este caso seria girar derecha salvo que el giro sea muy amplio
    if angulo_punto_control > angulo_brujula:
        diferencia_angulo = angulo_punto_control - angulo_brujula
    # Lo normal en este caso seria girar izquierda salvo que el giro sea muy amplio 
    elif angulo_brujula > angulo_punto_control:
        diferencia_angulo = angulo_brujula - angulo_punto_control

    print("Diferencia angulo: ", diferencia_angulo)

    # Devolver la diferencia angular
    return diferencia_angulo

#Leo matriz de profundidad y almaceno los minimos de cada columna en 3 vectores
def obtener_vectores_minimos(imagen):
    matriz = np.array(imagen)
    _, ancho = matriz.shape
    vector_izquierda = []
    vector_centro = []
    vector_derecha = []

    for i in range(ancho):
        columna = matriz[150:300, i]

        valores_filtrados = columna[(columna > 0) & (columna <500)]  # Filtrar valores en el rango deseado
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

#Si hay varios minimos seguidos significa que hay un obstaculo
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
    global izquierda, derecha, centro, paso_peatones, gps_off, objetos_detectados, vel, pub
    # Convertir el mensaje de ROS a una matriz numpy
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    # Obtener los vectores minimos
    izquierda, centro, derecha = obtener_vectores_minimos(depth_image)

    # Imprimir los minimos
    #print("Vector izquierda (minimo): {:.2f}".format(min(izquierda)))
    #print("Vector centro (minimo): {:.2f}".format(min(centro)))
    #print("Vector derecha (minimo): {:.2f}".format(min(derecha)))

    # Si el entorno es INTERIOR no se recibe GPS
    if gps_off:
        print("Entorno INTERIOR")
        evitacion_obstaculos(izquierda, centro, derecha)
    # Si el entorno es EXTERIOR
    else:
        # Si no hay paso de peatones
        print("Entorno EXTERIOR")
        if not paso_peatones:
            #print("Evitacion")
            evitacion_obstaculos(izquierda, centro, derecha)
        else:
            print("Paso")
            protocolo_paso_peatones(objetos_detectados)
    #pub.publish(vel)

def sensor_callback(data):
    global correa
    #rospy.loginfo("Estado del sensor ON-OFF: %d", data.data)
    if data.data == 0:
        correa = True
    else:
        correa = False
    #print("Estado correa: ", correa)

def callback_objetos_detectados(data):
    global objetos_detectados, paso_peatones
    # Convertir el mensaje a una lista de strings
    objetos_detectados = data.data.split(',')
    # Imprimir los objetos detectados
    for objeto in objetos_detectados:
        if objeto == "paso_peatones":
            paso_peatones = True
            print("PASO DE PEATONES DETECTADO, LA FUNCION evitacion_obstaculos DEJA DE CONTROLAR AL ROBOT")

    print("Objetos detectados",objetos_detectados)

def protocolo_paso_peatones(objetos_detectados):
    global paso_peatones, precaucion_speed, contador_espera_paso, marcha, max_speed, vel
    semaforo_rojo = False
    semaforo_verde = False

    for objeto in objetos_detectados:
        if objeto == "semaforo_verde":
            semaforo_verde = True
        elif objeto == "semaforo_rojo":
            semaforo_rojo = True

    if paso_peatones and not semaforo_verde and not semaforo_rojo:
        contador_espera_paso += 1
        print("PASO PEATONES SIN SEMAFORO")
        # espero un poco a que los coches se paren para cruzar
        if contador_espera_paso >= 20:
            vel.linear.x = precaucion_speed
            marcha = True
    
    if paso_peatones and semaforo_verde:
        vel.linear.x = precaucion_speed
        marcha = True
        print("PASO PEATONES Y SEMAFORO VERDE")

    #Si la marcha ya se ha iniciado, continua. Si ademas se detecta semaforo rojo, se da mas prisa en cruzar
    if marcha:
        vel.linear.x = precaucion_speed
        print("SIGUE CRUZANDO")
        if semaforo_rojo:
            vel.linear.x = max_speed
            print("CORRE PERROOOO")

    if paso_peatones and semaforo_rojo and not marcha: 
        vel.linear.x = 0
        vel.angular.z = 0
        print("PASO PEATONES Y SEMAFORO ROJO, ESPERANDO")

def gps_callback(msg):
    global latitud_actual, longitud_actual, indice_punto_control, gps_off, destino, punto_control_actual
    # Dividir la cadena para obtener solo los valores numericos
    lat_long_str = msg.data.split(',')
    latitud_str = lat_long_str[0].split(':')[1].strip()  # Extraer el valor de la latitud
    longitud_str = lat_long_str[1].split(':')[1].strip()  # Extraer el valor de la longitud
    # Convertir las cadenas a flotantes y almacenarlas en las variables globales
    latitud_actual = float(latitud_str)
    longitud_actual = float(longitud_str)
    gps_off = False
    #print("Coordenadas GPS actuales: Latitud={}, Longitud={}".format(latitud_actual, longitud_actual))
     # Obtener el punto de control actual
    punto_control_actual = puntos_de_control[indice_punto_control]

    # Calcular la distancia entre el punto de control actual y las coordenadas actuales del robot
    distancia = calcular_distancia(latitud_actual, longitud_actual, punto_control_actual['latitud'], punto_control_actual['longitud'])

    # Si la distancia es menor que un cierto umbral, pasar al siguiente punto de control
    if distancia < 0.01:  # 10 metros
        indice_punto_control = (indice_punto_control + 1) % len(puntos_de_control)
        print("Checkpoint alcanzado. Pasando al siguiente punto de control.")
        # Verificar si es el ultimo punto de control
        if indice_punto_control == len(puntos_de_control) - 1:
            destino = True
            print("HAS LLEGADO A TU DESTINO")

def brujula_callback(msg):
    global angulo_brujula
    angulo_brujula = msg.data
    #rospy.loginfo("Angulo de la brujula: {}".format(angulo_brujula))

def evitacion_obstaculos(izquierda, centro, derecha):
    global max_speed, mucha_dist, dist_critica, contador_tiempo, contador_actualizar_angulo, correa, precaucion_speed, velocidad_angular
    global gps_off, tolerancia_angulo, punto_control_actual, angulo_brujula, contador_actualizar_angulo, contador_espera_paso, vel
    rate = rospy.Rate(100000)
    contador_espera_paso = 0
    redirigiendo = False

    # Velocidades que no se modifican
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    # girar hasta que dentro de un margen de 5 grados aprox
    if correa:
        #print("RECIBIDO CORREA")
        # NO PELIGRO de obstaculos
        if min(centro) > mucha_dist and not obstaculos(centro):
            vel.linear.x = max_speed
            contador_tiempo = 0
            #print("NO HAY OBSTACULO")

            # Si no hay obstaculos, actualizar angulo del robot al checkpoint cada cierto tiempo
            if gps_off is False and not obstaculos(centro) and min(derecha) > peligro_lateral and min(izquierda) > peligro_lateral:
                contador_actualizar_angulo += 1
                diferencia_angulo = orientar_robot(punto_control_actual, angulo_brujula)
                vel.linear.x = 0
                if contador_actualizar_angulo > 0:
                    redirigiendo = True
                    if abs(diferencia_angulo) > 15 or abs(diferencia_angulo) < 345:
                        print("Reorientando robot a checkpoint")
                        # derecha si es positivo
                        if abs(diferencia_angulo) < 180:
                            vel.angular.z = -0.2
                            print("Reorientacion DERECHA")
                        else:
                            vel.angular.z = 0.2
                            print("Reorientacion IZQUIERDA")
                    contador_actualizar_angulo = 0                
                
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
        if not redirigiendo:
            # OBSTACULO IZQUIERDA
            if min(izquierda) < peligro_lateral and min(derecha) > peligro_lateral:
                vel.angular.z = -velocidad_angular # giro derecha para no chocarme
                print("OBSTACULO IZQUIERDA")
            # OBSTACULO DERECHA
            elif min(derecha) < peligro_lateral and min(izquierda) > peligro_lateral:
                vel.angular.z = velocidad_angular # giro izquierda para no chocarme
                print("OBSTACULO DERECHA")
            #elif min(derecha) > peligro_lateral and min(izquierda) > peligro_lateral and not obstaculos(centro):
            #    print("equisde")
                #vel.angular.z = 0 # recto, sin obstaculos
    #print(vel)    
    #pub.publish(vel)

if __name__ == '__main__':
    try:
        rospy.init_node("walk", anonymous=True)
        #puntos_de_control = cargar_puntos_de_control("datos_ruta.json")
        puntos_de_control = cargar_puntos_de_control("/home/carmen/catkin_wsunitree/src/go1-math-motion/go1-math-motion/src/prueba.json")
        rospy.Subscriber("/depth", Image, callback_depth)
        rospy.Subscriber("/objetos_detectados", String, callback_objetos_detectados)
        rospy.Subscriber("/brujula", Float32, brujula_callback)
        rospy.Subscriber("/correa", Int32, sensor_callback, queue_size=1)
        rospy.Subscriber('/gps_coordinates', String, gps_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
