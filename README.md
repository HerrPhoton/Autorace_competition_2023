# Autorace competition 2023 - UnitedROS
![](https://github.com/HerrPhoton/Autorace_competition_2023/blob/main/images/%D0%9B%D0%BE%D0%B3%D0%BE.jpg)

---

<img src = "https://img.shields.io/badge/ROS2-humble-006C6B?style=for-the-badge&color=3a3b3a&labelColor=%3a3b3a&logo=ROS&logoColor=FFFFFF">  <img src = "https://img.shields.io/badge/Python 3.9-006C6B?style=for-the-badge&color=3a3b3a&labelColor=%3a3b3a&logo=python&logoColor=FFFFFF"> 

---
### Выполнили: 
+ Капитан: [Лейсле Александр](https://github.com/HerrPhoton)
+ Младший капитан: [Володина Софья](https://github.com/PiroJOJO)
+ Младший капитан: [Квас Андрей](https://github.com/kvasik3000)
### Дата: 19.12.2023г.
---

## Введение

ЕдиноROSы - это не просто команда, это семья. 
Наша дружба помогала нам справляться со всеми трудностями и невзгодами.
 Но когда в нашей жизни появился ROS, мы осознали, что это не просто испытание, а самая настоящая закалка, которая, непременно, скрепила наши узы ещё сильнее. Теперь мы не просто студенты, мы герои, мы три ЕдиноROSа, которые готовы принять любой удар судьбы. 
Ура!

---

## PID регулятор

Саша пишет >>>

---

 <img src="https://github.com/HerrPhoton/Autorace_competition_2023/blob/main/images/pid_1.jpeg" alt="1" width = 660px height = 360px >
 
 <img src="https://github.com/HerrPhoton/Autorace_competition_2023/blob/main/images/pid_2.jpeg" alt="1" width = 660px height = 360px >

---

## Детекция сигналов

Для детектирования знаков мы дообучили YOLOv5s на датасете, который содержал в себе около 910 изображений ( по 130 изображении на каждый класс). В ноде нейросеть брала изображения с камеры и тут же выдавала предсказанные классы и boundbox'ы. Для задачи была реализована детекция с условиям на площадь найденного объекта, чтобы избежать случаев ранней детекции знака.
 
---

 <img src="https://github.com/HerrPhoton/Autorace_competition_2023/blob/main/images/detect_2.jpeg" alt="1" width = 660px height = 360px >
 
 <img src="https://github.com/HerrPhoton/Autorace_competition_2023/blob/main/images/detect_3.jpeg" alt="1" width = 660px height = 360px >
 
---

## Перекресток

Когда робот подъезжает к перекрестку, он вначале замечает знак препятствия, который сигнализирует о уменьшении скорости. После этого распознается знак поворота, в направлении которого необходимо двигаться.

Тут Саша пишет >>>

---

 <img src="https://github.com/HerrPhoton/Autorace_competition_2023/blob/main/images/detect_1.jpeg" alt="1" width = 660px height = 360px >
 
 <img src="https://github.com/HerrPhoton/Autorace_competition_2023/blob/main/images/intersection.jpeg" alt="1" width = 660px height = 360px >

---
## Объезд препятствий

После детекции знака "Дорожные работы" включается лидар, с помощью которого отслеживаются стоящие впереди препятствия. При первом обнаружении робот вначале сворачивает налево, затем, зафиксировав вторую стену перед собой, сворачивает направо. Завершив последний поворот, робот возобновляет движение по PIDу.

---

 <img src="https://github.com/HerrPhoton/Autorace_competition_2023/blob/main/images/avoiding_2.jpeg" alt="1" width = 660px height = 360px >
 
 <img src="https://github.com/HerrPhoton/Autorace_competition_2023/blob/main/images/avoiding_1.jpeg" alt="1" width = 660px height = 360px >

---
## Парковка

Нода активируется при обнаружении знака парковки. Робот снижает скорость и отслеживает свое движение по двум желтым линиям. Когда робот замечает парковочную зону, он с помощью лидара определяет, с какой стороны свободно место, после чего совершает поворот в нужную сторону и останавливается. Далее робот ждет одну секунду, чтобы возобновить движение. Он отъезжает назад и снова включает ориентацию по двум желтым линиям. Завершив испытание, возвращается движение по PIDу и начальная скорость.

---

 <img src="https://github.com/HerrPhoton/Autorace_competition_2023/blob/main/images/parking_1.jpeg" alt="1" width = 660px height = 360px >
 
 <img src="https://github.com/HerrPhoton/Autorace_competition_2023/blob/main/images/parking_2.jpeg" alt="1" width = 660px height = 360px >

---
## Пешеходный переход

Для прохождения данного испытания считываются значения с лидара и, рассчитывая расстояние до объектов, определяется наличие пешехода на дороге. Если дистанция меньше определенного значения, робот останавливается, дожидаясь отсутствия на дороге препятствия, и продолжает движение. Иначе, робот проскакивает вперед перед пешеходом.

---

 <img src="https://github.com/HerrPhoton/Autorace_competition_2023/blob/main/images/pedestrian.jpeg" alt="1" width = 660px height = 360px >

---
## Туннель

Нода активируется после того, как распознается знак "Туннель". Для ориентации в пространстве используется лидар. Робот стремиться двигаться по диагонали к заданной точке. Если перед ним встречается препятствие, то, сравнивая положение объекта относительно центра обзора, робот выбирает направление, которое ему выгоднее сделать, чтобы избежать столкновение. Когда робот пришел к конечной точке, возвращается движение по PIDу.

---

 <img src="https://github.com/HerrPhoton/Autorace_competition_2023/blob/main/images/tunnel_2.jpeg" alt="1" width = 660px height = 360px >
 
 <img src="https://github.com/HerrPhoton/Autorace_competition_2023/blob/main/images/tunnel_1.jpeg" alt="1" width = 660px height = 360px >

 ---
# Управление проектом

### Установка и запуск программы

+ Клонируйте репозиторий нижеприведенной командой:
```
git clone https://github.com/HerrPhoton/Autorace_competition_2023
```
+ Установите необходимые библиотеки командой ниже:
```
pip install -r requirements.txt
```
+ Запустите испытание с помощью команды:
  
```
colcon build
. install/setup.bash
ros2 launch autorace_core_unitedROS autorace_core.launch.py
```

## Результаты работы программы

---


## Источники

---
- [Competition](https://github.com/virusapex/autorace_2023)
- [ROS2-Humble](https://docs.ros.org/en/humble/Installation.html)
- [Gazebo](https://gazebosim.org/docs/garden/install)
- [YOLOv5](https://github.com/ultralytics/yolov5)
- [OpenCV Tutorials](https://docs.opencv.org/4.x/d9/df8/tutorial_root.html)
