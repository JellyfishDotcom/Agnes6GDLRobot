# Agnes6GDLRobot
Un robot manipulador antropomórfico con muñeca esférica (6 GDL), simulado, programado y controlado con herramientas Open Source del entorno de Python.

## Ficheros
En la carpeta "AgnesCAD" se encuentran los sólidos generados en Solid Works v2022 (licencia estudiantil).
En la carpeta Agnes se contiene un fichero python "visualize_agnes.py" que al correrlo abre el simulador Pybullet con el contenido del URDF. Además de contener
  una carpeta llamada "URDF", donde se tienen todos los .STL de los eslabones del robot y el fichero "agnes.urdf.xml" que funge como URDF de nuestro robot.
  
## Instalación de módulos necesarios (usando Anaconda)
  #### ---Para crear un ambiente de Python nuevo---
  conda create -n NombreAmbiente python=3
  
  #### ---Para activar este ambiente--- 
  conda activate NombreAmbiente
  
  #### ---Para instalar el módulo Pybullet---
  Opciones: 
    pip install pybullet
    conda install -c conda -forge pybullet
    
  #### ---Para correr el fichero "visualize_agnes.py" (teniendo la terminal en la ruta del fichero)---
  python visualize_agnes.py
    
