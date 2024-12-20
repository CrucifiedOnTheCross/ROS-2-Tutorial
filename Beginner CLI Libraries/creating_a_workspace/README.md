# Создание рабочего пространства
**Цель:** Создайте рабочее пространство и узнайте, как настроить оверлей для разработки и тестирования.
## Фон
Рабочая область — это каталог, содержащий пакеты ROS 2. Перед использованием ROS 2 необходимо установить рабочую область ROS 2 в терминале, в котором вы планируете работать. Таким образом, пакеты ROS 2 доступны для использования в этом терминале.

У вас также есть возможность найти "оверлей" - дополнительное рабочее пространство, в которое вы можете добавлять новые пакеты, не вмешиваясь в существующее рабочее пространство ROS 2, которое вы расширяете, или "подложение". Подложка должна содержать зависимости всех пакетов в оверлее. Пакеты в наложении будут переопределять пакеты в подложке. Также можно иметь несколько слоев подложек и наложений, при этом каждое последующее наложение использует пакеты родительских подложек.

### Необходимые условия
Установка ROS 2

Установка COLCON

Установка git

Установка TurtleSIM

Установите rosdep

Понимание основных команд терминала (вот руководство для Linux)

Текстовый редактор на ваш выбор

## Задачи
### 1. Исходная среда ROS 2
Ваша основная установка ROS 2 будет вашей подложкой для этого урока. (Имейте в виду, что подложка не обязательно должна быть основной установкой ROS 2.)

В зависимости от того, как вы установили ROS 2 (из исходного кода или из двоичных файлов) и на какой платформе вы работаете, ваша точная исходная команда может различаться:
```source /opt/ros/rolling/setup.bash```

### 2. Создайте новую директорию
Рекомендуется создавать новый каталог для каждого нового рабочего пространства. Название не имеет значения, но полезно, чтобы оно указывало на назначение рабочего пространства. Давайте выберем имя директории для "рабочей области разработки":ros2_ws
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Еще одна рекомендация — помещать все пакеты из рабочей области в каталог. Приведенный выше код создает каталог внутри, а затем переходит в него. src src ros2_ws

### 3. Клонирование репозитория образцов
Перед клонированием убедитесь, что вы все еще находитесь в каталоге.ros2_ws/src

В остальных руководствах для начинающих разработчиков вы будете создавать свои собственные пакеты, но сейчас вы будете практиковаться в сборке рабочего пространства с использованием существующих пакетов.

Если вы изучали уроки Beginner: CLI Tools, то наверняка знакомы с одним из пакетов в ros_tutorials.turtlesim

Репозиторий может иметь несколько ветвей. Вам нужно проверить тот, который предназначен для вашего установленного дистрибутива ROS 2. Когда вы клонируете этот репозиторий, добавьте аргумент, за которым следует эта ветвь.-b

В каталоге выполните следующую команду: ros2_ws/src

```git clone https://github.com/ros/ros_tutorials.git -b rolling```

Репозиторий клонируется в вашем рабочем пространстве. Он содержит пакет, который мы будем использовать в оставшейся части этого руководства. Другие пакеты в этом репозитории не собираются, потому что они содержат файл. ros_tutorials ros_tutorials turtlesim COLCON_IGNORE

Пока что вы заполнили свое рабочее пространство образцом пакета, но это еще не полнофункциональное рабочее пространство. Сначала необходимо разрешить зависимости, а затем создать рабочую область.

### 4. Разрешение зависимостей
Перед созданием рабочей области необходимо разрешить зависимости пакетов. Возможно, у вас уже есть все зависимости, но лучше всего проверять наличие зависимостей каждый раз при клонировании. Вы не хотите, чтобы сборка завершилась неудачей после долгого ожидания только для того, чтобы понять, что у вас отсутствуют зависимости.

В корневом каталоге рабочей области () выполните следующую команду:ros2_ws

```
cd ..
rosdep install -i --from-path src --rosdistro rolling -y
```

Если у вас уже есть все зависимости, консоль вернет:

```#All required rosdeps installed successfully```

Пакеты объявляют свои зависимости в файле package.xml (подробнее о пакетах вы узнаете в следующем руководстве). Эта команда проходит по этим объявлениям и устанавливает те, которые отсутствуют. Вы можете узнать больше об этом в другом уроке (скоро появится). rosdep

### 5. Соберите рабочее пространство с помощью colcon
В корневом каталоге рабочей области () теперь можно собрать пакеты с помощью команды: ros2_ws

```colcon build```

Консоль вернет следующее сообщение:

```
Starting >>> turtlesim_msgs
Finished <<< turtlesim_msgs [17.5s]
Starting >>> turtlesim
[Processing: turtlesim]
Finished <<< turtlesim [41.4s]
```

После завершения сборки введите команду в корневую рабочую область (): ~/ros2_ws
```
build  install  log  src
```
![kk](https://i.postimg.cc/C5HbQ4wt/2024-11-26-124731.png)

Каталог — это место, где находятся файлы настройки вашего рабочего пространства, которые вы можете использовать для источника наложения. install

### 6. Источник наложения
Прежде чем искать оверлей, очень важно открыть новый терминал, отдельный от того, на котором вы построили рабочее пространство. Использование оверлея в том же терминале, где вы построили, или в здании, где был получен оверлей, может привести к сложным проблемам.

В новом терминале используйте свою основную среду ROS 2 в качестве "подложки", чтобы вы могли построить оверлей "поверх нее":
```
source /opt/ros/rolling/setup.bash
```

Перейдите в корневой каталог вашего рабочего пространства:
```
cd ~/ros2_ws
```

В корне найдите источник оверлея:
```
source install/local_setup.bash
```
Теперь вы можете запустить пакет из оверлея:turtlesim
```ros2 run turtlesim turtlesim_node```
Но как определить, что это работает наложенный turtlesim, а не turtlesim вашей основной установки?

Давайте изменим turtlesim в оверлее, чтобы вы могли увидеть эффекты:

Вы можете изменять и перестраивать пакеты в оверлее отдельно от подложки.

Накладка имеет приоритет над подложкой.
# 7 Изменение наложения 
Вы можете изменить свой оверлей, отредактировав строку заголовка в окне turtlesim. Для этого найдите файл в формате . Откройте с помощью предпочитаемого текстового редактора.turtlesimturtle_frame.cpp~/ros2_ws/src/ros_tutorials/turtlesim/srcturtle_frame.cpp

Найдите функцию , измените значение на , и сохраните файл.setWindowTitle("TurtleSim");"TurtleSim""MyTurtleSim"

Вернитесь к первому терминалу, на котором вы бежали ранее, и запустите его снова.colcon build

Вернитесь ко второму терминалу (где находится источник оверлея) и запустите turtlesim снова:
```ros2 run turtlesim turtlesim_node```
Вы увидите, что в строке заголовка в окне turtlesim теперь написано "MyTurtleSim".
Несмотря на то, что основная среда ROS 2 была получена в этом терминале ранее, наложение среды имеет приоритет над содержимым подложки.ros2_ws

Чтобы убедиться, что подложка осталась нетронутой, откройте новый терминал и подключите только установку ROS 2. Запустите turtlesim еще раз:
```
ros2 run turtlesim turtlesim_node
```
Вы можете видеть, что изменения в наложении на самом деле ни на что не повлияли на подложку.

# Сводка
В этом руководстве вы выбрали основную установку дистрибутива ROS 2 в качестве подложки и создали оверлей путем клонирования и сборки пакетов в новой рабочей области. Наложение добавляется к траектории и имеет приоритет над подложкой, как вы видели на вашем модифицированном симуляторе turtlesim.

Использование оверлеев рекомендуется для работы с небольшим количеством пакетов, поэтому вам не придется размещать все в одном рабочем пространстве и перестраивать огромное рабочее пространство на каждой итерации.

