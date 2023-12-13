# Autorace competition 2023 - UnitedROS
![](https://github.com/HerrPhoton/Autorace_competition_2023/blob/main/images/%D0%9B%D0%BE%D0%B3%D0%BE.jpg)

---

<img src = "https://img.shields.io/badge/Python 3.9-006C6B?style=for-the-badge&color=3a3b3a&labelColor=%3a3b3a&logo=python&logoColor=FFFFFF"> <img src = 'https://img.shields.io/github/contributors/HerrPhoton/Autorace_competition_2023?style=for-the-badge&color=3a3b3a&labelColor=%3a3b3a&logo=teamspeak&logoColor=FFFFFF'>  <img src ='https://img.shields.io/github/repo-size/HerrPhoton/Autorace_competition_2023?style=for-the-badge&color=3a3b3a&labelColor=%3a3b3a&logo=weightsandbiases&logoColor=FFFFFF'>



---
### Выполнили: 
+ Капитан: [Лейсле Александр](https://github.com/HerrPhoton)
+ Младший капитан: [Володина Софья](https://github.com/PiroJOJO)
+ Младший капитан: [Квас Андрей](https://github.com/kvasik3000)
### Дата: 19.12.2023г.
---

## Введение

ЕдиноROSы - это не просто команда, это семья. Наша дружба помогала нам справляться со всеми трудностями и невзгодами. Но когда в нашей жизни появился ROS, мы осознали, что это не просто испытание, а самая настоящая закалка, которая, непременно, скрепила наши узы ещё сильнее. Теперь мы не просто студенты, мы герои, мы три ЕдиноROSа, которые готовы принять любой удар судьбы. Ура! 

---

## PID регулятор

Саша пишет >>>

---

![]()

---

## Детекция сигналов

Для детектирования знаков мы дообучили YOLOv5s на датасете, который содержал в себе около 910 изображений ( по 130 изображении на каждый класс). В ноде нейросеть брала изображения с камеры и тут же выдавала предсказанные классы и boundbox'ы. Для задачи была реализована детекция с условиям на площадь найденного объекта, чтобы избежать случаев ранней детекции знака.

---

![]()

---

## Перекресток

Тут Саня пишет >>>

---

![]()

---
## Объезд препятствий

Тут Андрей пишет >>>

---

![]()

---
## Парковка

Тут Соня пишет >>>

---

![]()

---
## Пешеходный переход

Для прохождения данного препятствия считываются значения с камеры глубины и, расчитывая расстояние до объектов, определяется наличие пешехода на дороге. Перед тем, как сделать принудительную остановку, с помощью OpenCV находится стоп линия, у которой необходимо затормозить.

---

![]()

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
тут надо написать 
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
