# Задачи
1. Написание action-сервера
Мы создадим action-сервер для вычисления последовательности Фибоначчи, используя action, который был разработан в туториале "Создание action".

Ранее мы уже создавали пакеты и запускали узлы с помощью команды `ros2` run. Для упрощения в этом уроке action-сервер будет ограничен одним файлом. 

### Шаги

- Создание файла \
В домашней директории создайте новый файл и назовите его `fibonacci_action_server.py`. Затем добавьте следующий код:

```py
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from custom_action_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Fibonacci.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
```

- 2 Описание кода 
    * Класс `FibonacciActionServer`
    Определён в строке 8 и наследуется от класса `Node`. В конструкторе класса создаётся узел ROS 2 с именем `fibonacci_action_server`:

        ```python  
        super().__init__('fibonacci_action_server')
        ```
    * Инициализация action-сервера \
    В конструкторе создаётся action-сервер
        ```py
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )
        ```

    * Аргументы action-сервера: 
    1. Узел: `self` — текущий узел.
    2. Тип action: `Fibonacci` (импортирован на строке 5).
    3. Имя action: `'fibonacci;`.
    4. Функция-callback: `self.execute_callback`, которая принимает и выполняет цели.

    * Функция execute_callback \
    Определена в классе и выполняет принятые цели
        ```PYTHON
        def execute_callback(self, goal_handle):
            self.get_logger().info('Executing goal...')
            result = Fibonacci.Result()
            return result
        ```
 3. Запуск action-сервера\
    Чтобы запустить сервер, выполните следующую команду:
    ```bash
    python3 fibonacci_action_server.py
    ```
4. Обработка отправки цели и успешного завершения

    После того как вы создали и запустили сервер действий, следующим шагом будет отправка цели от клиента к серверу для выполнения. В этом примере мы отправляем задачу на вычисление последовательности Фибоначчи с заданным порядком.

    ### Отправка цели с помощью команды:
    Для этого откройте новый терминал и отправьте цель на сервер с помощью команды `ros2 action send_goal`. Команда будет выглядеть следующим образом:
    ```bash
    ros2 action send_goal fibonacci custom_action_interfaces/action/Fibonacci "{order: 5}"
    ```

    ### В этой команде:

    - `fibonacci` — это название действия, которое мы определили на сервере.
    - `custom_action_interfaces/action/Fibonacci` — это путь к нашему определению действия.
    - `{order: 5}` — это данные, которые отправляются серверу: в данном случае, мы передаем порядковый номер (order) для вычисления первых 5 чисел последовательности Фибоначчи.
    
    ### Что произойдёт на сервере?
    Когда сервер получит цель, он начнет выполнение в методе execute_callback. Сервер выведет лог-сообщение о том, что цель выполняется:
    ```bash
    Executing goal...
    ```
    После выполнения вычислений сервер должен вернуть результат, и мы используем `goal_handle.succeed()` для того, чтобы сообщить, что выполнение задачи завершено успешно.
