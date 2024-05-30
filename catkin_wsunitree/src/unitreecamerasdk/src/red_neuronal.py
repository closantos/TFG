import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import supervision as sv
from std_msgs.msg import String
import cv2

# Load the YOLOv8 model
model = YOLO('best.pt')

# Definir los nombres de las clases
class_names = ["paso_peatones", "semaforo_verde", "semaforo_rojo"]

def callback(data):
    # Convertir el mensaje de ROS a una imagen OpenCV
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    
    # Run YOLOv8 inference on the frame
    results = model(frame)
    detections = sv.Detections.from_yolov8(results[0])
    detections = detections[detections.confidence > 0.7]

    # Obtener las clases detectadas y asignarlas a los nombres correspondientes
    objetos_detectados = [class_names[class_id] for class_id in detections.class_id]

    # Visualize the results on the frame
    annotated_frame = results[0].plot()

    # Display the annotated frame
    cv2.imshow("YOLOv8 Inference", annotated_frame)

    # Publicar los objetos detectados
    publicar_array(objetos_detectados)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        rospy.signal_shutdown("User exit")

def publicar_array(objetos_detectados):
    # Crea un objeto Publisher. El segundo argumento es el tipo del mensaje.
    pub = rospy.Publisher('objetos_detectados', String, queue_size=10)

    # Crea un mensaje de tipo String
    msg = String()

    # Convierte los nombres de las clases detectadas en una cadena y asigna al mensaje
    msg.data = ','.join(objetos_detectados)

    # Publica el mensaje en el topic
    rospy.loginfo("Publicando en el topic: %s", msg.data)
    pub.publish(msg)

def main():
    # Inicializar el nodo de ROS
    rospy.init_node('object_detection_node', anonymous=True)
    
    # Suscribirse al topic color
    rospy.Subscriber("/color", Image, callback)

    # Spin
    rospy.spin()

    # Close OpenCV windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
