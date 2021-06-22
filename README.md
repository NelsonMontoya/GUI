# GUI
GUI con Nodo ROS

Este repositorio contiene los archivos necesarios para simular las rutas de cobertura del trabajo de grado IMPLEMENTACIÓN DE ALGORITMOS DE PLANIFICACIÓN DE RUTAS DE COBERTURA PARA
UN VEHÍCULO AÉREO NO TRIPULADO EN UN AMBIENTE SIMULADO.

Las carpetas Celsia y SouthWinston contienen los escenarios para mostrar en Gazebo.

La carpeta cpp_package contiene los archivos .launch y .world necesarios para simular los escenarios Celsia y SouthWinston.

La carpeta Instalación ROS contiene el archivo ubuntu_noetic_ros.sh con las instrucciones necesarias para instalar Gazebo en conjunto con ROS noetic.

La carpeta Master ROS contiene los archivos celsia-launch-common.sh y southWinston-launch-common.sh que incializan el maestro ROS que inicializa la simulación de acuedo a lo
descrito en el informe del trabajo de grado.

La carpeta mascaras contiene las máscaras de los escenarios a simular.  Dichas máscaras determinan el area a cubrir con las rutas de cobertura a calcular.


