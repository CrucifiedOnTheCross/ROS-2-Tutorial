## Understanding parameters
Цель: узнать, как получать, устанавливать, сохранять и перезагружать параметры в ROS 2.
Параметр — это значение конфигурации узла. Под параметрами подразумеваются настройки узла. Узел может хранить параметры как целые числа, числа с плавающей точкой, логические значения, строки и списки. В ROS 2 у каждого узла есть свои собственные параметры.
### Запуск задачи
Запустим узлы /teleop_turtle и /turtlesim в разных терминалах: 

```bash
ros2 run turtlesim turtlesim_node
```

```bash
ros2 run turtlesim turtle_teleop_key
```
### Просмотр параметров текущих узлов
Чтобы посмотреть параметры текущих узлов, вводим
```bash
ros2 param list
```
Результат выполнения:
```bash
/teleop_turtle:
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  scale_angular
  scale_linear
  start_type_description_service
  use_sim_time
/turtlesim:
  background_b
  background_g
  background_r
  holonomic
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  start_type_description_service
  use_sim_time
```
Как мы видим, у узла /turtlesim параметры определяют, например, цвет фона. Также можно заметить, что у всех узлов имеется параметр use_sim_time (что не является уникальным для этих двух узлов).
### Получение параметров
Синтаксис команды получения значения параметров:
```bash
ros2 param get <node_name> <parameter_name>
```
Пример: получим значение параметра background_b у узла /turtlesim:
```bash
ros2 param get /turtlesim background_g
```
Результат выполнения команды:
```bash
Integer value is: 86
```
### Установка параметров
Синтаксис команды установки параметров:
```bash
ros2 param set <node_name> <parameter_name> <value>
```
Пример: изменим значение параметра background_b у узла turtlesim:
```bash
ros2 param set /turtlesim background_g 255
```
Результат:
![topics](image.png)
### Просмотр всех текущих параметров
Синтаксис команды просмотра всех текущих параметров узла:
```bash
ros2 param dump <node_name>
```
Пример: посмотрим все текущие параметры у узла /turtlesim:
```bash
ros2 param dump /turtlesim
```
Результат:
```bash
/turtlesim:
  ros__parameters:
    background_b: 255
    background_g: 255
    background_r: 69
    holonomic: false
    qos_overrides:
      /parameter_events:
        publisher:
          depth: 1000
          durability: volatile
          history: keep_last
          reliability: reliable
    start_type_description_service: true
    use_sim_time: false
```
### Загрузка параметров
Синтаксис команды загрузки параметров:
```bash
ros2 param load <node_name> <parameter_file>
```
Синтаксис запуска узла с прописанными в файле параметрами:
```bash
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```

