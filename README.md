# **Projekt 1 - dokumentacja**

Celem projektu było napisanie programu sterującego robotem Velma, który spowoduje, że robot:

- chwyci obiekt object1 stojący na pierwszym stoliku table1
- przeniesie go na drugi stolik table2
- odłoży go na drugim stoliku, tj. zwolni chwyt i wycofa ramię w pozycję początkową

## Mapa otoczenia
W symulatorze Gazebo stworzyłyśmu następujący świat:
<img src="./img/Screenshot from 2022-12-21 20-58-02.png" aling="center">
Aby zbudować oktomapę otoczenia skorzystałyśmy z polecenia:
```sh
roslaunch velma_common octomap_server.launch
```
Po poruszaniu się głową oraz korpusem otrzymałyśmy mapy zajętości, czyli mapę zajętej oraz nieznanej przestrzeni.
Zapisałyśmy mapę za pomocą polecenia:

```sh
rosrun octomap_server octomap_saver <nazwa mapy>
```
Końcowa mapa środowiska wygląda następująco:
<img src="./img/Screenshot from 2022-12-21 20-56-39.png" aling="center">

## Plik launch
Zmodyfikowałyśmy podstawowy plik velma_stero_system.launch, dodając do niego takie rzeczy, jak:
- uruchomienie systemu naszej konfiguracji rviz
- publikowanie pozycji obu stołów i klocka
- uruchomienie offline octomapy
- inicjalizacja utworzonego przez nas świata

## Implementacja węzła
W pliku pr1.py znajduje się kod źródłowy programu.
Została zaimplementowana klasa __Velma__ zawierająca następujące metody:
-  __homing()__ - homing robota
- __getTable1Position()__ - funkcja do otrzymania transformaty pozycji pierwszego stoła 
- __getTable2Position()__ - funkcja do otrzymania transformaty pozycji drugiego stoła 
- __getObject1Position()__ - funkcja do otrzymania transformaty pozycji obiektu
- __getPlanner()__ - funkcja do inicjalizacji plannera z oktomapą
- __jimpCimpSwitch()__ - funkcja do zmiany trybu na jnt_imp lub cart_imp
- __getTWEForObj()__ - funkcja do obliczenia transformaty pozycji chwytaka dla obiektu
- __getTWEForTab()__ - funkcja do obliczenia transformaty pozycji chwytaka dla stoła 
- __getIK()__ - funkcja do rozwiazania zadania odwrotnej kinematyki
- __getTrajectory()__ - funkcja do wyznaczania ścieżki i ruchu
- __moveCimp()__ - funkcja do ruchu w trybie cart_imp
- __closeGripper()__ - funkcja do zamkniecia chwytaka
- __openGripper()__ - funkcja do otwarcia chwytaka
- __moveUp()__ - funkcja do podnoszenia ramiona robota powyżej początkowej pozycji kostki
- __startPos()__ - funkcja do powrótu do początkowej pozycji robota
- __moveOneJoint()__ - funkcja do ruchu jednego ostatniego złącza


Funkcja poza klasą: __getSide()__ - funkcja do wyznaczenia strony, po ktorej znajduje się obiekt 


## Uruchomienie programu 
Dla uruchomienia programu w różnych terminalach musimy wpisać nastęþujące polecenia:
```sh
roslaunch stero_manipulation velma_stero_system.launch - uruchomienie naszego pliku launch z rvizem
roslaunch velma_ros_plugin velma_planner.launch - uruchomienie planera
rosrun stero_manipulation pr1.py - uruchomienie algorytmu
roslaunch rcprg_gazebo_utils gazebo_client.launch - uruchomienie gazebo
```

## Wizualizacja w Gazebo
Ruch robota do kostki:
<img src="./img/Screenshot from 2022-12-21 22-15-48.png" aling="center">
<img src="./img/Screenshot from 2022-12-21 22-16-01.png" aling="center">
Chwycenie kostki:
<img src="./img/Screenshot from 2022-12-21 22-16-28.png" aling="center">
Przeniesienie kostki do drugiego stolika:
<img src="./img/Screenshot from 2022-12-21 22-17-17.png" aling="center">
<img src="./img/Screenshot from 2022-12-21 22-18-02.png" aling="center">
<img src="./img/Screenshot from 2022-12-21 22-18-25.png" aling="center">
<img src="./img/Screenshot from 2022-12-21 22-18-34.png" aling="center">
<img src="./img/Screenshot from 2022-12-21 22-18-44.png" aling="center">
Opuszczanie kostki i powrót do pozycji początkowej:
<img src="./img/Screenshot from 2022-12-21 22-19-47.png" aling="center">

