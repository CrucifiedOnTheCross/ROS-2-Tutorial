## Understanding servies 
Цель: изучить работу сервисов в ROS2 пользуясь средствами командной строки
Сервисы — это еще один метод связи узлов в ROS. Сервисы основаны на модели «вызов-ответ» по сравнению с моделью «издатель-подписчик», которую реализуют темы. В то время как темы позволяют узлам подписываться на потоки данных и получать постоянные обновления, сервисы предоставляют данные только тогда, когда они специально вызваны клиентом.

![topics](https://docs.ros.org/en/rolling/_images/Service-SingleServiceClient.gif)

### Запуск задачи
Запустим узлы /teleop_turtle и /turtlesim в разных терминалах: 

```bash
ros2 run turtlesim turtlesim_node
```

```bash
ros2 run turtlesim turtle_teleop_key
```

### Список сервисов
Далее, введя
```bash
ros2 service list
```
Мы увидим список активных сервисов:
```bash
/clear
/kill
/reset
/spawn
/teleop_turtle/describe_parameters
/teleop_turtle/get_parameter_types
/teleop_turtle/get_parameters
/teleop_turtle/get_type_description
/teleop_turtle/list_parameters
/teleop_turtle/set_parameters
/teleop_turtle/set_parameters_atomically
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/describe_parameters
/turtlesim/get_parameter_types
/turtlesim/get_parameters
/turtlesim/get_type_description
/turtlesim/list_parameters
/turtlesim/set_parameters
/turtlesim/set_parameters_atomically
```
### Типы сервисов
У служб есть типы, которые описывают, как структурированы данные запроса и ответа службы.
Синтаксис команды для получения типа:
```bash
ros2 service type <service_name>
```
Пример: выберем сервис /clear:
```bash
ros2 service type /clear
```
Результат выполнения команды:
```bash
std_srvs/srv/Empty
```
Empty означает, что сервис не отправляет никаких данных при выполнении запроса и не получает никаких данных при получении ответа.
#### Список сервисов с типами
Введя
```bash
ros2 service list -t
```
Мы получим список активных сервисов с их типами:
```bash
/clear [std_srvs/srv/Empty]
/kill [turtlesim/srv/Kill]
/reset [std_srvs/srv/Empty]
/spawn [turtlesim/srv/Spawn]
/teleop_turtle/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/teleop_turtle/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/teleop_turtle/get_parameters [rcl_interfaces/srv/GetParameters]
/teleop_turtle/get_type_description [type_description_interfaces/srv/GetTypeDescription]
/teleop_turtle/list_parameters [rcl_interfaces/srv/ListParameters]
/teleop_turtle/set_parameters [rcl_interfaces/srv/SetParameters]
/teleop_turtle/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/turtle1/set_pen [turtlesim/srv/SetPen]
/turtle1/teleport_absolute [turtlesim/srv/TeleportAbsolute]
/turtle1/teleport_relative [turtlesim/srv/TeleportRelative]
/turtlesim/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/turtlesim/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/turtlesim/get_parameters [rcl_interfaces/srv/GetParameters]
/turtlesim/get_type_description [type_description_interfaces/srv/GetTypeDescription]
/turtlesim/list_parameters [rcl_interfaces/srv/ListParameters]
/turtlesim/set_parameters [rcl_interfaces/srv/SetParameters]
/turtlesim/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
```
### Информация об узлах
Синтаксис команды для получения типа:
```bash
ros2 service info <service_name>
```
Пример: выберем сервис /clear:
```bash
ros2 service info /clear
```
Результат выполнения команды:
```bash
Type: std_srvs/srv/Empty
Clients count: 0
Services count: 1
```
### Как найти сервис по типу
Синтаксис команды для получения типа:
```bash
ros2 service find <type_name>
```
Пример: выберем сервис /clear:
```bash
ros2 service find std_srvs/srv/Empty
```
Результат выполнения команды:
```bash
/clear
/reset
```
### Вызов службы
Наконец, мы можем перейти к непосредственному вызову служб.
Синтаксис команды вызова службы:
```bash
ros2 service call <service_name> <service_type> <arguments>
```
arguments - не обязательный аргумент.
Пример: выберем сервис /clear:
```bash
ros2 service call /clear std_srvs/srv/Empty
```
Результат:
![topics](https://docs.ros.org/en/rolling/_images/clear.png)
### Эхо
Для демонстрации обмена данными между клиентом и сервером сервиса, можно использовать echo:
```bash
ros2 service echo <service_name | service_type> <arguments>
```
Откроем один терминал и запустим introspection_service:
```bash
ros2 launch demo_nodes_cpp introspect_services_launch.py
```
Далее, откроем второй терминал и установим следующие параметры:
```bash
ros2 param set /introspection_service service_configure_introspection contents
ros2 param set /introspection_client client_configure_introspection contents
```
В том же терминале введем ros2 service echo:
```bash
ros2 service echo --flow-style /add_two_ints
info:
  event_type: REQUEST_SENT
  stamp:
    sec: 1709408301
    nanosec: 423227292
  client_gid: [1, 15, 0, 18, 250, 205, 12, 100, 0, 0, 0, 0, 0, 0, 21, 3]
  sequence_number: 618
request: [{a: 2, b: 3}]
response: []

info:
  event_type: REQUEST_RECEIVED
  stamp:
    sec: 1709408301
    nanosec: 423601471
  client_gid: [1, 15, 0, 18, 250, 205, 12, 100, 0, 0, 0, 0, 0, 0, 20, 4]
  sequence_number: 618
request: [{a: 2, b: 3}]
response: []

info:
  event_type: RESPONSE_SENT
  stamp:
    sec: 1709408301
    nanosec: 423900744
  client_gid: [1, 15, 0, 18, 250, 205, 12, 100, 0, 0, 0, 0, 0, 0, 20, 4]
  sequence_number: 618
request: []
response: [{sum: 5}]

info:
  event_type: RESPONSE_RECEIVED
  stamp:
    sec: 1709408301
    nanosec: 424153133
  client_gid: [1, 15, 0, 18, 250, 205, 12, 100, 0, 0, 0, 0, 0, 0, 21, 3]
  sequence_number: 618
request: []
response: [{sum: 5}]
```
