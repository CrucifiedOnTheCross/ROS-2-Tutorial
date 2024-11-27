### Создание пользовательского интерфейса действия
Этот процесс позволит вам создать новый тип действия Fibonacci, который используется в сервере действий.

## Шаг 1. Создание интерфейсного пакета
1. Создайте рабочее пространство ROS 2 (если оно еще не существует):
    ```bash
    mkdir -p ./ros2_ws/src
    cd ./ros2_ws/src
    ```
2. Создайте пакет интерфейса:
    ```bash
    ros2 pkg create --license Apache-2.0 custom_action_interfaces
    ```

## Шаг 2. Определение действия
1. Перейдите в директорию проекта
    ```bash
    cd custom_action_interfaces
    ```
2. Создайте папку `action` для хранения определения действия:
    ```bash
    mkdir action
    ```
3. В папке action создайте файл `Fibonacci.action`:
    ```bash
    touch action/Fibonacci.action
    ```
4. Заполните файл `Fibonacci.action` следующим содержимым:
    ```action
    int32 order
    ---
    int32[] sequence
    ---
    int32[] partial_sequence
    ```
    * `order` — запрос на вычисление последовательности Фибоначчи заданного порядка.
    * `sequence` — конечный результат вычислений.
    * `partial_sequence` — частичные результаты, отправляемые в качестве обратной связи.

## Шаг 3. Настройка сборки

1. Откройте файл `CMakeLists.txt` пакета `custom_action_interfaces` и добавьте перед строкой `ament_package()`:
    ```cmake
    find_package(rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
    "action/Fibonacci.action"
    )
    ```
2. В файл `package.xml` добавьте зависимости:
    ```xml
    <buildtool_depend>rosidl_default_generators</buildtool_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    ```

## Шаг 4. Сборка пакета
1. Перейдите в корень рабочего пространства
    ```bash
    cd ./ros2_ws
    ```
2. Скомпилируйте рабочее пространство:
    ```bash
    colcon build
    ```
3. После сборки активируйте окружение:
    ```bash
    source install/local_setup.bash
    ```

## Шаг 5. Проверка действия
Чтобы убедиться, что действие успешно создано, выполните:
```bash
ros2 interface show custom_action_interfaces/action/Fibonacci
```
