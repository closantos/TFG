import pandas as pd
import matplotlib.pyplot as plt
import os

# Ruta del archivo CSV
csv_filename = os.path.join(os.path.dirname(__file__), 'exterior 1.csv')

# Leer el archivo CSV en un DataFrame de Pandas
df = pd.read_csv(csv_filename)

# Graficar los datos
fig, axs = plt.subplots(3, 1, figsize=(15, 15))

# Gráfico para Izquierda
axs[0].plot(df['Tiempo'], df['Izquierda Max'], label='Izquierda Max', color='b')
axs[0].plot(df['Tiempo'], df['Izquierda Min'], label='Izquierda Min', color='r')
axs[0].set_title('Izquierda')
axs[0].legend()
axs[0].set_ylabel('Distancia (mm)')
axs[0].set_xlabel('Tiempo (s)')  # Etiqueta del eje x

# Gráfico para Centro
axs[1].plot(df['Tiempo'], df['Centro Max'], label='Centro Max', color='g')
axs[1].plot(df['Tiempo'], df['Centro Min'], label='Centro Min', color='m')
axs[1].set_title('Centro')
axs[1].legend()
axs[1].set_ylabel('Distancia (mm)')
axs[1].set_xlabel('Tiempo (s)')  # Etiqueta del eje x

# Gráfico para Derecha
axs[2].plot(df['Tiempo'], df['Derecha Max'], label='Derecha Max', color='c')
axs[2].plot(df['Tiempo'], df['Derecha Min'], label='Derecha Min', color='y')
axs[2].set_title('Derecha')
axs[2].legend()
axs[2].set_ylabel('Distancia (mm)')
axs[2].set_xlabel('Tiempo (s)')  # Etiqueta del eje x

# Ajustar el espacio entre los subgráficos
plt.subplots_adjust(hspace=0.5)

# Guardar la figura como PNG
plt.savefig('grafica1.png')

# Mostrar la figura
plt.show()
