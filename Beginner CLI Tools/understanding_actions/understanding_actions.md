## Understanding actions
Действия являются одним из типов межузлового взаимодействия в ROS2 и предназначены для длительных задач. Действия состоят из трех частей: цели, обратной связи и результата. Функционал действия схож с функционалом сервиса, исключение состоит в том, что действие можно отменить. Действия также обеспечивают постоянную обратную связь, а не возвращают один ответ, как это делают сервисы.
Действия используют модель клиент-сервер, схожую с моделью издатель-подписчик. "Клиентский" узел отправляет цель "серверному" узлу, который возвращает результат и поток обратной связи.
![topics](https://docs.ros.org/en/rolling/_images/Action-SingleActionClient.gif)
### Запуск задачи
Запустим узлы /teleop_turtle и /turtlesim в разных терминалах: 

```bash
ros2 run turtlesim turtlesim_node
```

```bash
ros2 run turtlesim turtle_teleop_key
```
### Использование действий
При запуске /teleop_turtle мы увидим следующее:
```bash
Use arrow keys to move the turtle.
Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.
```
При нажатии каждой клавиш G|B|V|C|D|E|R|T /teleop_turtle отправляет цель узлу /turtlesim. При нажатии клавиши F цель отзывается.
### Информация об узлах
Посмотрим информаицю об узле /turtlesim:
```bash
ros2 node info /turtlesim
```
Результат:
```bash
/turtlesim
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
    /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
    /turtlesim/get_type_description: type_description_interfaces/srv/GetTypeDescription
    /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
    /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
    /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:
```
Обратим внимание на то, действие /turtle1/rotate_absolute относится к Action Servers. Это означает, что узел /turtlesim отправляет результат и предоставляет обратную связь действию /turtle1/rotate_absolute.
Теперь посмотрим информацию об узле /teleop_turtle:
```bash
ros2 node info /teleop_turtle
```
Результат:
```bash
/teleop_turtle
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Service Servers:
    /teleop_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /teleop_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /teleop_turtle/get_parameters: rcl_interfaces/srv/GetParameters
    /teleop_turtle/list_parameters: rcl_interfaces/srv/ListParameters
    /teleop_turtle/set_parameters: rcl_interfaces/srv/SetParameters
    /teleop_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
    /turtle1/rotate_absolute: turtlesim_msgs/action/RotateAbsolute
```
В этом случае /turtle1/rotate_absolute: относится к клиентам, что означает, что этот узел отправляет цель.
### Список действий
Чтобы вывести все действия, существует команда
```bash
ros2 action list
```
Результат:
```bash
/turtle1/rotate_absolute
```
Это действие, которое управляет вращением черепахи, является единственный на данный момент.
#### Список действий с типами
Чтобы вывести все действия с их типами, можно ввести
```bash
ros2 action list -t
```
Результат:
```bash
/turtle1/rotate_absolute [turtlesim_msgs/action/RotateAbsolute]
```
### Информация о действии
Чтобы подробно узнать информацию о действии /turtle1/rotate_absolute, можно ввести
```bash
ros2 action info /turtle1/rotate_absolute
```
Результат:
```bash
Action: /turtle1/rotate_absolute
Action clients: 1
    /teleop_turtle
Action servers: 1
    /turtlesim
```
Как видим, у этого действия есть один "клиентский" и один "серверный" узел, что мы уже видели ранее.