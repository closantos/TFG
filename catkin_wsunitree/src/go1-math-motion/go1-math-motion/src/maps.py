import googlemaps
import folium
import webbrowser
import math
import tkinter as tk
from tkinter import messagebox
from tkinter import simpledialog
import json

# Definir origen como variable global inicializada como None
origen = None

# Lista para almacenar los datos de la ruta
datos_ruta = []

# Nombre del archivo para almacenar las ubicaciones favoritas
ARCHIVO_FAVORITOS = "ubicaciones_favoritas.json"

# Lista para almacenar las ubicaciones favoritas
rutas_favoritas = {}

def cargar_favoritos():
    try:
        with open(ARCHIVO_FAVORITOS, "r") as file:
            return json.load(file)
    except FileNotFoundError:
        return {}

def guardar_favoritos():
    with open(ARCHIVO_FAVORITOS, "w") as file:
        json.dump(rutas_favoritas, file)

def obtener_coordenadas_desde_texto(texto):
    # Clave de API de Google Maps
    api_key = 'AIzaSyDkYGAvkhTYGkoyqwIhW05pJHd5KtMTzqQ'

    # Crear un cliente de Google Maps
    gmaps = googlemaps.Client(key=api_key)

    # Geocodificar la ubicación de destino
    geocode_result = gmaps.geocode(texto)

    # Obtener las coordenadas de la primera coincidencia
    if geocode_result:
        ubicacion = geocode_result[0]['geometry']['location']
        return ubicacion['lat'], ubicacion['lng']
    else:
        return None

def obtener_ruta(origen, destino):
    # Clave de API de Google Maps
    api_key = 'AIzaSyDkYGAvkhTYGkoyqwIhW05pJHd5KtMTzqQ'

    # Crear un cliente de Google Maps
    gmaps = googlemaps.Client(key=api_key)

    # Obtener la dirección de origen si está definida
    direccion_origen = None
    if origen:
        direccion_origen = gmaps.reverse_geocode(origen)[0]['formatted_address']

    # Calcular la ruta si hay una dirección de origen válida
    if direccion_origen:
        # Calcular la ruta
        directions_result = gmaps.directions(direccion_origen, destino, mode="walking")
        return directions_result
    else:
        return None

def obtener_angulo(p1, p2):
    lat1, lon1 = p1
    lat2, lon2 = p2
    dLon = lon2 - lon1
    x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
    y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
    bearing = math.atan2(x, y)
    bearing = math.degrees(bearing)
    bearing = (bearing + 360) % 360
    return bearing

def mostrar_mapa(ruta):
    global datos_ruta

    # Crear un mapa con la primera coordenada de la ruta como centro
    mapa = folium.Map(location=[ruta[0]['legs'][0]['start_location']['lat'], ruta[0]['legs'][0]['start_location']['lng']], zoom_start=15)

    # Reiniciar la lista de datos de la ruta
    datos_ruta = []

    # Bandera para indicar si se está en el primer checkpoint
    primer_checkpoint = True

    # Agregar la ruta al mapa y guardar los datos de la ruta
    for i, step in enumerate(ruta[0]['legs'][0]['steps']):
        folium.Marker(location=[step['start_location']['lat'], step['start_location']['lng']]).add_to(mapa)
        folium.PolyLine(locations=[(step['start_location']['lat'], step['start_location']['lng']), 
                                   (step['end_location']['lat'], step['end_location']['lng'])], color='blue').add_to(mapa)

        # Guardar las coordenadas de latitud y longitud del checkpoint en la lista de datos de la ruta
        checkpoint_data = {
            "checkpoint": i+1,
            "latitud": step['start_location']['lat'],
            "longitud": step['start_location']['lng']
        }
        datos_ruta.append(checkpoint_data)

        # Verificar si hay un cambio de dirección y guardar el giro correspondiente al siguiente checkpoint
        if i < len(ruta[0]['legs'][0]['steps']) - 1:
            next_step = ruta[0]['legs'][0]['steps'][i+1]
            current_direction = obtener_angulo((step['start_location']['lat'], step['start_location']['lng']), 
                                                (step['end_location']['lat'], step['end_location']['lng']))
            next_direction = obtener_angulo((next_step['start_location']['lat'], next_step['start_location']['lng']), 
                                            (next_step['end_location']['lat'], next_step['end_location']['lng']))

            degrees = next_direction - current_direction
            if degrees > 180:
                degrees -= 360
            elif degrees < -180:
                degrees += 360

            # Si es el primer checkpoint, no se almacena el giro
            if not primer_checkpoint:
                print(f"Girar {degrees} grados")

                # Guardar el giro en la lista de datos de la ruta
                giro_data = {
                    "checkpoint": i+1,
                    "giro": degrees
                }
                datos_ruta.append(giro_data)
        
        # Después del primer checkpoint, cambiar la bandera
        primer_checkpoint = False

    # Guardar los datos de la ruta en un archivo JSON
    with open("datos_ruta.json", "w") as file:
        json.dump(datos_ruta, file)

    # Guardar el mapa como un archivo HTML
    mapa.save('ruta.html')

    # Abrir el mapa en el navegador web
    webbrowser.open('ruta.html')




def obtener_destino():
    destino_texto = destino_entry.get()
    destino = obtener_coordenadas_desde_texto(destino_texto)
    if destino:
        return destino
    else:
        messagebox.showerror("Error", "No se pudo encontrar la ubicación de destino.")
        return None

def guardar_favorito():
    global rutas_favoritas
    nombre = simpledialog.askstring("Guardar Favorito", "Ingresa un nombre para esta ubicación:")
    if nombre:
        destino = obtener_destino()
        if destino:
            rutas_favoritas[nombre] = destino
            guardar_favoritos()  # Guardar ubicaciones favoritas en el archivo
            messagebox.showinfo("Guardar Favorito", "Ubicación guardada como favorita.")

def mostrar_favoritos():
    if rutas_favoritas:
        favoritos = "\n".join(rutas_favoritas.keys())
        seleccion = simpledialog.askstring("Favoritos", f"Ubicaciones favoritas:\n{favoritos}\nIngresa el nombre de la ubicación:")
        if seleccion and seleccion in rutas_favoritas:
            destino_entry.delete(0, tk.END)
            destino_entry.insert(0, f"{rutas_favoritas[seleccion][0]}, {rutas_favoritas[seleccion][1]}")
    else:
        messagebox.showinfo("Favoritos", "No hay ubicaciones favoritas guardadas.")

def borrar_favorito():
    if rutas_favoritas:
        favoritos = "\n".join(rutas_favoritas.keys())
        seleccion = simpledialog.askstring("Borrar Favorito", f"Ubicaciones favoritas:\n{favoritos}\nIngresa el nombre de la ubicación a borrar:")
        if seleccion and seleccion in rutas_favoritas:
            del rutas_favoritas[seleccion]
            guardar_favoritos()  # Guardar ubicaciones favoritas actualizadas en el archivo
            messagebox.showinfo("Borrar Favorito", f"Ubicación '{seleccion}' borrada correctamente.")
    else:
        messagebox.showinfo("Borrar Favorito", "No hay ubicaciones favoritas guardadas.")

def calcular_ruta():
    global origen  # Hacer referencia a la variable global origen
    destino = obtener_destino()
    if destino:
        # Actualizar origen si se ha ingresado
        origen_texto = origen_entry.get()
        if origen_texto:
            origen = obtener_coordenadas_desde_texto(origen_texto)
        # Obtener la ruta
        ruta = obtener_ruta(origen, destino)
        if ruta:
            # Mostrar el mapa con la ruta
            mostrar_mapa(ruta)

def salir():
    guardar_favoritos()  # Guardar ubicaciones favoritas al salir
    root.destroy()

if __name__ == '__main__':
    # Cargar ubicaciones favoritas desde el archivo
    rutas_favoritas = cargar_favoritos()

    # Crear la ventana principal
    root = tk.Tk()
    root.title("Calculadora de Rutas")

    # Establecer el tamaño de la ventana principal
    root.geometry("420x300")  # Modificar el tamaño de la ventana principal

    # Establecer el color de fondo
    root.configure(bg="#f0f0f0")

    # Crear los widgets
    origen_label = tk.Label(root, text="Origen:", bg="#f0f0f0", font=("Arial", 12))
    origen_label.grid(row=0, column=0, padx=10, pady=5, sticky="w")
    origen_entry = tk.Entry(root, font=("Arial", 12), width=30)
    origen_entry.grid(row=0, column=1, padx=10, pady=5, sticky="ew")
    origen_entry.insert(0, "Latitud, Longitud")

    destino_label = tk.Label(root, text="Destino:", bg="#f0f0f0", font=("Arial", 12))
    destino_label.grid(row=1, column=0, padx=10, pady=5, sticky="w")
    destino_entry = tk.Entry(root, font=("Arial", 12), width=30)
    destino_entry.grid(row=1, column=1, padx=10, pady=5, sticky="ew")

    calcular_button = tk.Button(root, text="Calcular Ruta", command=calcular_ruta, bg="#4CAF50", fg="white", font=("Arial", 12))
    calcular_button.grid(row=2, column=0, columnspan=2, padx=10, pady=5, sticky="ew")

    favoritos_button = tk.Button(root, text="Favoritos", command=mostrar_favoritos, bg="#FFD700", font=("Arial", 12))
    favoritos_button.grid(row=3, column=0, padx=10, pady=5, sticky="ew")

    guardar_button = tk.Button(root, text="Guardar Favorito", command=guardar_favorito, bg="#87CEEB", font=("Arial", 12))
    guardar_button.grid(row=3, column=1, padx=10, pady=5, sticky="ew")

    borrar_button = tk.Button(root, text="Borrar Favorito", command=borrar_favorito, bg="#FF6347", font=("Arial", 12))
    borrar_button.grid(row=4, column=0, columnspan=2, padx=10, pady=5, sticky="ew")

    salir_button = tk.Button(root, text="Salir", command=salir, bg="#f44336", fg="white", font=("Arial", 12))
    salir_button.grid(row=5, column=0, columnspan=2, padx=10, pady=5, sticky="ew")

    # Ejecutar la aplicación
    root.mainloop()