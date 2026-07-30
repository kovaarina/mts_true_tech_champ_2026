# Решение и ROS 2

## Решение — это КАТАЛОГ, а не один файл

На проверку берётся вся папка **`controllers/participant/`** целиком: раскладывайте код по модулям,
кладите YAML-конфиги (для Nav2 / SLAM), launch-файлы, сохранённые карты — всё, что рядом с
`participant.py`, приедет на судейство. Точка входа — **`participant.py`** (его запускает Webots).
Остальное дерево (`common/`, `worlds/`, `config/`, судья) фиксировано и берётся из эталона.

`requirements.txt` **не устанавливается** — среда фиксирована (см. ниже). Нужна библиотека сверх
списка — запрос организаторам, добавим в образ централизованно (одинаково для всех).

## ROS 2 — доступен на судействе

Образ содержит **ROS 2 Humble** (Ubuntu 22.04, Python 3.10): `rclpy`, стандартные
`geometry_msgs`/`sensor_msgs`/`nav_msgs`, `tf2_ros`, **`navigation2` (Nav2)**, **`slam_toolbox`**,
CycloneDDS. Окружение ROS уже настроено — просто `import rclpy` и пишите узлы. DDS замкнут на
loopback (`localhost`), наружу не ходит.

### Как пользоваться — свой узел (рекомендуется)

Самый надёжный путь — `enable_ros2=False` (встроенный мост выключен) + ваши собственные узлы rclpy.
Проверено на судейском стенде, работает из коробки:

```python
from go2_api import Go2
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

robot = Go2(enable_ros2=False)          # свой мост; ядро ROS доступно
rclpy.init()
node = rclpy.create_node("my_nav")
cmd_pub = node.create_publisher(Twist, "cmd_vel", 10)

scan = {"ranges": None}
node.create_subscription(LaserScan, "scan",
                         lambda m: scan.__setitem__("ranges", m.ranges), 10)

while robot.step():
    rclpy.spin_once(node, timeout_sec=0.0)   # прокачать ROS-колбэки
    lidar = robot.lidar()                    # или получайте данные напрямую из API
    # ... ваша логика / Nav2 / SLAM ...
    robot.drive(vx=0.4, vyaw=0.0)            # ехать напрямую
    # либо публикуйте /cmd_vel и подписывайтесь на него своим мостом
```

Данные сенсоров можно брать и напрямую из Python-API (`robot.lidar()`, `robot.lidar_layer()`,
`robot.pose()`, `robot.imu()`, `robot.arm`), и через свои ROS-топики — как удобно.

### Встроенный мост `enable_ros2=True`

Публикует `/scan` `/odom` `/imu`, слушает `/cmd_vel`. Мы дорабатываем его инициализацию DDS
(встроенный мост может подвиснуть на первой публикации внутри Webots). **Пока рекомендуем свой узел**
как выше — он не зависит от этой доработки. Как починим — сообщим отдельно.

### Nav2 / SLAM

`navigation2` и `slam_toolbox` установлены. Конфиги (YAML) и, если нужно, launch кладите в свой
каталог решения — они приедут на судейство. Карту (старт/финиш — как в примере, остальное
генерируется под секретный сид) стройте на лету; заранее заготовленная карта не подойдёт, трасса
на проверке другая.
