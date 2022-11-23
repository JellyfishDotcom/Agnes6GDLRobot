# Agnes 6DOF Robot

An anthropomorphic manipulator robot with a spherical wrist (6 DOF), simulated, programmed and controlled with Open Source tools from the Python environment.

## Directories

In the **AgnesCAD** directory are the solids generated in Solid Works v2022 (student license).
In the **Agnes** folder there is a python file "visualize_agnes.py" that, when run, opens the Pybullet simulator with the URDF content. In addition to containing
  a folder called "URDF", where we have all the .STL of the links of the robot and the file "agnes.urdf.xml" that serves as the URDF of our robot.

## Python stuff

### Creating a new Anaconda enviroment

In this project we use an Anaconda Enviroment called 'robotics'. You can create it as follows:

```
# conda create -n robotics python=3
```

### To activate this environment

```
# conda activate robotics
```

### Pybullet install

(In both options you have to have your conda enviroment activated), we recommend follow the Option 2.
Option 1:

```
# pip install pybullet
```

Option 2:

```
conda install -c conda -forge pybullet
```

### Check the correct display of the Agnes robot

In a terminal or a Conda Prompt (Windows), please activate the enviroment that you create and run the comand

```
# python visualize_agnes.py
```
