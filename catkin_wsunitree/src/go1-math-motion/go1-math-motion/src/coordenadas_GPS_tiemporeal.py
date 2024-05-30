import serial
import pynmea2
import rospy
from std_msgs.msg import String

def obtener_coordenadas_gps(puerto_serial='/dev/ttyACM0', baudios=9600, timeout=5):
    with serial.Serial(puerto_serial, baudios, timeout=timeout) as ser:
        while not rospy.is_shutdown():
            linea = ser.readline().decode('ascii')
            if linea.startswith('$GPGGA'):
                mensaje = pynmea2.parse(linea)
                if mensaje.latitude and mensaje.longitude:
                    return mensaje.latitude, mensaje.longitude

if __name__ == "__main__":
    try:
        rospy.init_node('gps_node', anonymous=True)
        pub = rospy.Publisher('gps_coordinates', String, queue_size=10)
        rate = rospy.Rate(1)  # 1 Hz

        while not rospy.is_shutdown():
            latitud, longitud = obtener_coordenadas_gps()
            coordenadas = "Latitud: {}, Longitud: {}".format(latitud, longitud)
            rospy.loginfo(coordenadas)
            pub.publish(coordenadas)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
