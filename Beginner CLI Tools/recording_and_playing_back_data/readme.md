## recording_and_playing_back_data
Цель: запись данных о темах и сервисах для их запуска в любое удобное время
ROS2 bag - это интсрумент командной строки, позволяющий хранить данные о сервисах и воспроизводить их в удобное время. Он накапливает данные о темах и сервисах и сохраняет их в базу данных. Кроме того, это хороший инструмент для того, чтобы делиться данными.
### Установка
По традиции, запустим узел /turtlesim:
```bash
ros2 run turtlesim turtlesim_node
```
А также запустим узел /turtle_teleop:
```bash
ros2 run turtlesim turtle_teleop_key
```
Создадим отдельный каталог для хранения наших данных:
```bash
mkdir bag_files
cd bag_files
```
### Выбор темы
Чтобы увидеть темы нашей системы, введем команду:
```bash
ros2 topic list
```
Результат:
```bash
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```
Чтобы увидеть опубликованные данные /turtle1/cmd_vel, введем команду:
```bash
ros2 topic echo /turtle1/cmd_vel
```
Первоначально эта команда ничего не выведет, однако подвинув черепашку мы увидим следующее:
```bash
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
```
### Запись тем
Синтаксис команды записи одной темы:
```bash
ros2 bag record <topic_name>
```
Откроем новый терминал и перейдем в директорию bag_files. Введем команду:
```bash
ros2 bag record /turtle1/cmd_vel
```
Результат:
```bash
[INFO] [1732734261.103534777] [rosbag2_recorder]: Press SPACE for pausing/resuming
[INFO] [1732734261.114782796] [rosbag2_recorder]: Listening for topics...
[INFO] [1732734261.114829162] [rosbag2_recorder]: Event publisher thread: Starting
[WARN] [1732734261.115044524] [ROSBAG2_TRANSPORT]: Hidden topics are not recorded. Enable them with --include-hidden-topics
[INFO] [1732734261.118598435] [rosbag2_recorder]: Subscribed to topic '/turtle1/cmd_vel'
[INFO] [1732734261.118703516] [rosbag2_recorder]: Recording...
[INFO] [1732734261.118904795] [rosbag2_recorder]: All requested topics are subscribed. Stopping discovery...
```
Немного подвигаем черепашкой.
Нажмем Ctrl+C для остановки записи. Результат:
```bash
[INFO] [1732734577.718916943] [rosbag2_cpp]: Writing remaining messages from cache to the bag. It may take a while
[INFO] [1732734577.722069278] [rosbag2_recorder]: Event publisher thread: Exiting
[INFO] [1732734577.722209975] [rosbag2_recorder]: Recording stopped
[INFO] [1732734577.723327552] [rosbag2_recorder]: Recording stopped
```
Мы можем также записывать данные нескольких тем, а также устанавливать имя файла. Введем команду:
```bash
ros2 bag record -o bag /turtle1/cmd_vel /turtle1/pose
```
Результат:
```bash
[INFO] [1732735131.951735329] [rosbag2_recorder]: Press SPACE for pausing/resuming
[INFO] [1732735131.956642329] [rosbag2_recorder]: Listening for topics...
[INFO] [1732735131.956654928] [rosbag2_recorder]: Event publisher thread: Starting
[WARN] [1732735131.956869724] [ROSBAG2_TRANSPORT]: Hidden topics are not recorded. Enable them with --include-hidden-topics
[INFO] [1732735131.961427431] [rosbag2_recorder]: Subscribed to topic '/turtle1/pose'
[INFO] [1732735131.964564466] [rosbag2_recorder]: Subscribed to topic '/turtle1/cmd_vel'
[INFO] [1732735131.965034257] [rosbag2_recorder]: Recording...
[INFO] [1732735131.965218953] [rosbag2_recorder]: All requested topics are subscribed. Stopping discovery...
```
Как видим из сообщения, мы подписались на две темы.
Подвигаем еще немного черепашкой:
![alt text](image-1.png)
Потом снова нажмем Ctrl+C для остановки записи.
### Проверка данных темы
Синтаксис команды для проверки деталей темы:
```bash
ros2 bag info <bag_file_name>
```
Введем команду:
```bash
ros2 bag info bag
```
Результат:
```bash
Files:             bag_0.mcap
Bag size:          184.3 KiB
Storage id:        mcap
ROS Distro:        rolling
Duration:          72.491s
Start:             Nov 27 2024 23:39:01.307 (1732736341.307)
End:               Nov 27 2024 23:40:13.798 (1732736413.798)
Messages:          2573
Topic information: Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 20 | Serialization Format: cdr
                   Topic: /turtle1/pose | Type: turtlesim/msg/Pose | Count: 2553 | Serialization Format: cdr
```
### Запуск файла
Для запуска файла bag введем команду в терминал, в котором раньше был запущен узел teleop_key::
```bash
ros2 bag play bag
```
После этого черепашка повторит свой записанный путь:

![alt text](image-2.png)

### Запись услуг
Мы будем записывать данные сервиса между introspection_client и introspection_service, а затем отображать и воспроизводить те же данные позже. 
Откроем новый терминал и введем команду:
```bash
ros2 run demo_nodes_cpp introspection_service --ros-args -p service_configure_introspection:=contents
```
Откроем еще один терминал и введем команду:
```bash
ros2 run demo_nodes_cpp introspection_client --ros-args -p client_configure_introspection:=contents
```
### Проверка доступности услуги
ROS2 bag может записывать данные только из доступных служб. Чтобы увидеть их список, введем команду:
```bash
ros2 service list
```
Результат:
```bash
/add_two_ints
/introspection_client/describe_parameters
/introspection_client/get_parameter_types
/introspection_client/get_parameters
/introspection_client/get_type_description
/introspection_client/list_parameters
/introspection_client/set_parameters
/introspection_client/set_parameters_atomically
/introspection_service/describe_parameters
/introspection_service/get_parameter_types
/introspection_service/get_parameters
/introspection_service/get_type_description
/introspection_service/list_parameters
/introspection_service/set_parameters
/introspection_service/set_parameters_atomically
```
### Запись услуг
Синтаксис команды записи конркетной услуги:
```bash
ros2 bag record <service_names>
```
Команда для записи всех услуг:
```bash
ros2 bag record --all-services
```
Введем команду:
```bash
ros2 bag record /add_two_ints/_service_event
```
Результат:
```bash
[INFO] [1732737524.913913140] [rosbag2_recorder]: Press SPACE for pausing/resuming
[INFO] [1732737524.918384815] [rosbag2_recorder]: Listening for topics...
[INFO] [1732737524.918389415] [rosbag2_recorder]: Event publisher thread: Starting
[INFO] [1732737524.920584100] [rosbag2_recorder]: Subscribed to topic '/add_two_ints/_service_event'
[INFO] [1732737524.920676904] [rosbag2_recorder]: Recording...
[INFO] [1732737524.920816209] [rosbag2_recorder]: All requested topics are subscribed. Stopping discovery...
```
После этого прекратим запись с помощью Ctrl+C.
### Проверка данных 
Синтаксис команды проверки данных:
```bash
ros2 bag info <bag_file_name>
```
Введя команду выводится следующее:
```bash
Files:             rosbag2_2024_11_27-23_58_44_0.mcap
Bag size:          147.8 KiB
Storage id:        mcap
ROS Distro:        rolling
Duration:          175.938s
Start:             Nov 27 2024 23:58:44.931 (1732737524.931)
End:               Nov 28 2024 00:01:40.869 (1732737700.869)
Messages:          1306
Topic information: Topic: /add_two_ints/_service_event | Type: example_interfaces/srv/AddTwoInts_Event | Count: 1306 | Serialization Format: cdr
```
### Воспроизведение данных сервиса
Введем команду в терминал команду:
```bash
ros2 bag play --publish-service-requests rosbag2_2024_11_27-23_58_44
```
Результат:
```bash
[INFO] [1732738337.828639111] [rosbag2_player]: Set rate to 1
[INFO] [1732738337.835828591] [rosbag2_player]: Adding keyboard callbacks.
[INFO] [1732738337.835874193] [rosbag2_player]: Press SPACE for Pause/Resume
[INFO] [1732738337.835903094] [rosbag2_player]: Press CURSOR_RIGHT for Play Next Message
[INFO] [1732738337.835931795] [rosbag2_player]: Press CURSOR_UP for Increase Rate 10%
[INFO] [1732738337.835959996] [rosbag2_player]: Press CURSOR_DOWN for Decrease Rate 10%
[INFO] [1732738337.836075101] [rosbag2_player]: Playback until timestamp: -1
```
После выполнения команды в терминал, где был запущен клиент introspection service, будет выводится:
```bash
[INFO] [1713997478.090466075] [introspection_service]: Incoming request
a: 2 b: 3
```
Это происходит потому, что запросы на сервис отправляются из файла bag.
