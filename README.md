# Control de robot Phantom X usando Matlab, Python y ROS

## ROS

### ROS librerias and funciones
Las librerias usadas para este respositorio incluyen la libreria de ROS para Python, la libreria time para aplicar unos delays en la respuesta de los actuadores, asi como termios,sys y os para hcer la lectura de la entrada de usario y por ultimo el import de los comandos de dynamixel.

```python
import rospy
import time
import termios, sys, os
from dynamixel_workbench_msgs.srv import DynamixelCommand
```

### Lectura del teclado
Como se vio en el laboratorio anterior se usa terminos para la lectura de la entrada del usuario.

```python
TERMIOS = termios

def getkey():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
    new[6][TERMIOS.VMIN] = 1
    new[6][TERMIOS.VTIME] = 0
    termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
    c = None
    try:
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
    return c
```

### Uso del servicio ROS

Bajo esta función se realiza el llamado del servicio de ROS para poder publicar la posicion del motor en los respectivos topicos.

```python
def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))
```

### Definicion de torques limites

Con el objetivo de hacer el codigo main menos largo, parametros de setteo como los torques limites los incluimso dentro de una función.

```python
def setTorquesLimit():
    jointCommand('', 1, 'Torque_Limit', 500, 0)
    jointCommand('', 2, 'Torque_Limit', 500, 0)
    jointCommand('', 3, 'Torque_Limit', 400, 0)
    jointCommand('', 4, 'Torque_Limit', 400, 0)
```

### Creacion de la clase Selector
Para solucionar este ejercicio una buena practica de programación es crear una clase con sus respectivos metodos que podemos usar en nuestro main code. En este caso sera el selector, que definira el motor sobre el cual se esta trabajando asi como los parametros de home y target position del mismo.

```python
class Selector:

    def __init__(self) -> None:
        self._selected = 1
        self._homePositions = [512,512,512,512,512]
        self._targetPositions = [375,647,708,667,820]
```

Esta clase esta inicializada con tres atributos:
- `selected`: Un entero que determina a que motor se esta apuntando.
- `homePositions`: Un arreglo con las posiciones de home.
- `targetPositions`: Otro arreglo con las posiciones objetivo. 

### Metodos de la clase Selector

- Metodos get

En este caso los metodos get retornan el valor ya sea del ´id´ o de la posicion home o objetivo del motor seleccionado en el momento.

```python
    def getCurrentId(self):
        return self._selected

    def getHomePosition(self):
        jointId = self._selected
        if jointId>len(self._homePositions) or jointId<1:
            return -1
        return self._homePositions[jointId-1]

    def getTargetPosition(self):
        jointId = self._selected
        if jointId>len(self._homePositions) or jointId<1:
            return -1
        return self._targetPositions[jointId-1]
```
- Metodos de navegación de Selector

El objetivo de estos metodos es cambiar el valor de selector al anterior o posterior siguiendo un loop entre 1  5.

```python
    def setNextId(self):
        if self._selected == 5:
            self._selected = 1
            return
        self._selected += 1

    def setPreviousId(self):
        if self._selected == 1:
            self._selected = 5
            return
        self._selected -= 1  
```

Con estos metodos definidos ya essssta definida totalmente la clase selector que se usara en el main code.
### Main code
```python
if __name__ == '__main__':
    selector = Selector()  
    try:
        setTorquesLimit()
        while(not rospy.is_shutdown()):
            print(f"Current ID: {selector.getCurrentId()}")
            key = str(getkey())[2].lower()
            if key == "w":
                selector.setNextId()
            if key == "s":
                selector.setPreviousId()
            if key == "a":
                toPosition = selector.getHomePosition()
                jointId = selector.getCurrentId()
                jointCommand('', jointId, 'Goal_Position', toPosition, 0.5)
                print(f"Joint {jointId} moved to {toPosition}")
            if key == "d":
                toPosition = selector.getTargetPosition()
                jointId = selector.getCurrentId()
                jointCommand('', jointId, 'Goal_Position', toPosition, 0.5)
                print(f"Joint {jointId} moved to {toPosition}")
            if key == "q":
                break
            time.sleep(0.5)
    except rospy.ROSInterruptException:
        pass
```
En el main code podemos apreciar la creacion de una instancia de la clase Selector y dentro del respectivo try para ROS podemos apreciar el set del torque y asi mismo la espera de la entrada del usuario.

Dependiendo de la entrada del usuario se llaman los distintos metodos de la clase Selector y en dado caso de necesitar enviar un comando a los motores estos se incluyen en la funcion jointCommand.

### Rviz

Se hizo uso del visualizador de ROS para poder observar en pantalla los movimientos al tiempo que estos se ejecutaban en el robot real. Para lograr esto se ejecutaron dos archivos `.launch`, los cuales son archivos de configuración XML que permiten inicializar nodos de ROS.
- `px_controllers.launch`: En este archivo se configura la conexión serial con el robot, las juntas y los topicos de ROS a utilizar:
```xml
<launch>
  <arg name="usb_port"                default="/dev/ttyUSB0"/>
  <arg name="dxl_baud_rate"           default="1000000"/>
  <arg name="namespace"               default="dynamixel_workbench"/>

  <arg name="use_moveit"              default="false"/>
  <arg name="use_joint_state"         default="true"/>
  <arg name="use_cmd_vel"             default="false"/>

  <param name="dynamixel_info"          value="$(find px_robot)/config/joints.yaml"/>

  <node name="$(arg namespace)" pkg="dynamixel_workbench_controllers" type="dynamixel_workbench_controllers"
        required="true" output="screen" args="$(arg usb_port) $(arg dxl_baud_rate)">
    <param name="use_moveit"              value="$(arg use_moveit)"/>
    <param name="use_joint_states_topic"  value="$(arg use_joint_state)"/>
    <param name="use_cmd_vel_topic"       value="$(arg use_cmd_vel)"/>
    <rosparam>
      publish_period: 0.010
      dxl_read_period: 0.010
      dxl_write_period: 0.010
      mobile_robot_config:                <!--this values will be set when 'use_cmd_vel' is true-->
        seperation_between_wheels: 0.160  <!--default value is set by reference of TB3-->
        radius_of_wheel: 0.033            <!--default value is set by reference of TB3-->
    </rosparam>
    <remap from="/dynamixel_workbench/joint_trajectory" to="joint_trajectory"/>
  </node>
</launch>
```
- `px_rviz_dyna.launch`: Este archivo carga el modelo 3D del robot en Rviz y se suscribe al topico `robot_state_publisher` para obtener la posición de cada motor del robot:
```xml
<launch>
  <!-- Argumentos -->
  <arg name="model" default="$(find px_robot)/urdf/px_collision.urdf"/>
  <arg name="gui" default="false" />
  <arg name="rvizconfig" default="$(find px_robot)/config/config.rviz" />
  
  <!-- Carga el robot -->
  <param name="robot_description" command="$(find xacro)/xacro $(arg model)"/>

  <!-- Carga el servidor de parametros para publicar el estado del rebot -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" >
    <remap from="joint_states" to="dynamixel_workbench/joint_states"/>
  </node>
  
  <!-- Carga el manejador de juntas -->
  <node if="$(arg gui)" name="joint_state_publisher" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" />

  <!-- Carga Rvix con config -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rvizconfig)" required="true" />
  <!-- <node name="rviz" pkg="rviz" type="rviz" /> -->

</launch>
```
Se debe ejecutar primero `px_controllers.launch`, luego `px_rviz_dyna.launch` y finalmente nuestro script de Python. El resultado de esta practica puede verse en el siguiente gif:


## Matlab Peter Corke Toolbox
### Creacion de una función prettify propia

Debido a que MATLAB bajo operaciones computacionales imprime 0's computacionales en una notacion muy extraña, fue necesario crear una funcion que permitiera arreglar estos errores numericos para una impresión amigable con el usuario:


```matlab
function MTH = prettify(x)
    MTH = x;
    for i = 1:numel(x);
        value = x(i);
        if isnumeric(value)
            if abs(value)<1e-10
                MTH(i) = 0;
            end
        else
            if abs(coeffs(value))<1e-10
                MTH(i) = 0;
            end
        end
    end
end

```
### Creación del robot

Para la creación del robot se uso tal como se recomienda un SerialLink de la libreria de peterCorke asi mismo se definio la matriz de transformación de la herramienta para que tenga la convecion NOA. 
```matlab
lenghts = [40.75 106 106 85];
L(1) = Link('revolute','d', lenghts(1), 'a', 0, 'alpha', pi/2);
L(2) = Link('revolute','d', 0, 'a', lenghts(2), 'alpha', 0,'offset',pi/2);
L(3) = Link('revolute','d', 0, 'a', lenghts(3), 'alpha', 0);
L(4) = Link('revolute','d', 0, 'a', lenghts(4), 'alpha', 0);
robot = SerialLink(L);
robot.tool=[0 0 1 0; 1 0 0 0;0 1 0 0;0 0 0 1];

```
### Obtencion de la MTH desde base a herramienta

Para esto una vez modelado el robot se usa la función A de la clase Link que nos permite acceder a cad auno de los MTH qu conforman la cinematica directa, los multiplicamos y comparamos con la función fkine de MATLAB de donde obtenemos la expresion del MTH

```matlab 
MTH =
 
[- cos(q1 + q2 + q3 + q4)/2 - cos(q2 - q1 + q3 + q4)/2,  sin(q1), - sin(q1 + q2 + q3 + q4)/2 - sin(q2 - q1 + q3 + q4)/2, -cos(q1)*(85*sin(q2 + q3 + q4) + 106*sin(q2 + q3) + 106*sin(q2))]
[  sin(q2 - q1 + q3 + q4)/2 - sin(q1 + q2 + q3 + q4)/2, -cos(q1),   cos(q1 + q2 + q3 + q4)/2 - cos(q2 - q1 + q3 + q4)/2, -sin(q1)*(85*sin(q2 + q3 + q4) + 106*sin(q2 + q3) + 106*sin(q2))]
[                                   -sin(q2 + q3 + q4),        0,                                     cos(q2 + q3 + q4),    85*cos(q2 + q3 + q4) + 106*cos(q2 + q3) + 106*cos(q2) + 163/4]
[                                                    0,        0,                                                     0,                                                                1]
```
 

