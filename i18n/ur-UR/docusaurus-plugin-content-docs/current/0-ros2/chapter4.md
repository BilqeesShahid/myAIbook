---
id: chapter4
title: "ماڈیول-0.ros2-باب-4"
sidebar_position: 4
slug: /ros2/chapter4
---

# باب 4: اپنی پہلی ROS2 میسج کی اقسام کی تعریف کرنا

جبکہ `std_msgs` بنیادی ڈیٹا اقسام جیسے `String` اور `Int32` فراہم کرتا ہے، حقیقی دنیا کے روبوٹکس ایپلی کیشنز کو اکثر زیادہ پیچیدہ اور ایپلی کیشن کے لیے مخصوص ڈیٹا ڈھانچے کی ضرورت ہوتی ہے۔ ROS2 آپ کو سادہ `.msg`، `.srv`، اور `.action` فائلوں کا استعمال کرتے ہوئے اپنی کسٹم میسج کی اقسام، سروسز اور ایکشنز کی تعریف کرنے کی اجازت دیتا ہے۔

اس باب میں، ہم نوڈز کے درمیان منظم ڈیٹا کی ترسیل کے لیے کسٹم **میسج کی اقسام** کی تعریف پر توجہ مرکوز کریں گے۔

## کسٹم میسجز کیوں؟

ایک روبوٹ پر غور کریں جسے اپنی موجودہ پوزیشن (x, y, z کوآرڈینیٹ) اور سمت (roll, pitch, yaw زاویے) کی رپورٹ کرنے کی ضرورت ہے۔ جبکہ آپ چھ الگ الگ `Float32` پیغامات بھیج سکتے ہیں، انہیں ایک واحد `RobotPose` پیغام میں پیک کرنا کہیں زیادہ موثر، پڑھنے کے قابل اور ہم آہنگی کے مسائل کا کم شکار ہے۔ یہ واضح طور پر معلومات کے ایک واحد، مربوط ٹکڑے کی تعریف کرتا ہے۔

## کسٹم میسج کی تعریف کرنا

کسٹم میسجز کو `.msg` فائلوں میں بیان کیا جاتا ہے، جو سادہ ٹیکسٹ فائلیں ہوتی ہیں جن میں فیلڈ کی قسم اور فیلڈ کے نام کے جوڑے ہوتے ہیں۔

### میسج ڈیفینیشن کا Syntax

ایک `.msg` فائل سادہ Syntax استعمال کرتی ہے:

```
field_type1 field_name1
field_type2 field_name2
...
```

*   **`field_type`**: ایک بلٹ ان ROS2 پرائمیٹو قسم (مثلاً، `int8`، `float32`، `string`، `bool`، `time`، `duration`)، ایک اور کسٹم میسج قسم، یا ان اقسام کا ایک array ہو سکتا ہے۔
*   **`field_name`**: فیلڈ کا نام، جو وضاحتی ہونا چاہیے۔

### مثال: `my_robot_pkg/msg/RobotPose.msg`

آئیے ایک `RobotPose` میسج بناتے ہیں تاکہ روبوٹ کی 3D پوزیشن اور سمت کو اسٹور کیا جا سکے۔

1.  **اپنے پیکیج کے اندر ایک `msg` ڈائرکٹری بنائیں:**

    ```bash
    mkdir -p ~/ros2_ws/src/my_robot_pkg/msg
    ```

2.  **`~/ros2_ws/src/my_robot_pkg/msg/RobotPose.msg` بنائیں:**

    ```
    # RobotPose.msg
    float32 x
    float32 y
    float32 z

    float32 roll
    float32 pitch
    float32 yaw
    ```

## کسٹم میسجز بنانا اور استعمال کرنا

ROS2 کے لیے آپ کے کسٹم میسجز کو پہچاننے اور کوڈ تیار کرنے کے لیے، آپ کو اپنے پیکیج کے بلڈ سسٹم کو کنفیگر کرنے کی ضرورت ہے۔

1.  **`package.xml` میں ترمیم کریں:**

    `~/ros2_ws/src/my_robot_pkg/package.xml` میں درج ذیل بلڈ اور ایگزیکیوشن ڈیپینڈنسیز شامل کریں:

    ```xml
    <!-- ... inside <package> tag ... -->

    <buildtool_depend>ament_cmake</buildtool_depend>
    <buildtool_depend>ament_cmake_python</buildtool_depend>
    <build_depend>rosidl_default_generators</build_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>

    <!-- ... rest of package.xml ... -->
    ```
    *نوٹ: `ament_cmake` اور `ament_cmake_python` بلڈ سسٹم کی ڈیپینڈنسیز ہیں۔ `rosidl_default_generators` آپ کے پیغامات کے لیے کوڈ تیار کرنے کے لیے ضروری ہے، اور `rosidl_default_runtime` انہیں رن ٹائم پر استعمال کرنے کے لیے ضروری ہے۔* `member_of_group` دریافت پذیری میں مدد کرتا ہے۔

2.  **`CMakeLists.txt` میں ترمیم کریں:**

    `~/ros2_ws/src/my_robot_pkg/CMakeLists.txt` کھولیں اور یہ لائنیں شامل کریں:

    *   `rosidl_default_generators` پیکیج تلاش کریں:
        ```cmake
        # ... inside find_package(ament_cmake REQUIRED)
        find_package(rosidl_default_generators REQUIRED)
        # ...
        ```
    *   اپنی میسج فائلیں شامل کریں:
        ```cmake
        # ... after ament_export_dependencies()
        rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/RobotPose.msg"
        # "srv/MyService.srv" # Uncomment and add for services
        # "action/MyAction.action" # Uncomment and add for actions
        )
        # ...
        ```
    *   ڈیپینڈنسیز ایکسپورٹ کریں (اگر `ament_cmake_python` استعمال کر رہے ہیں):
        ```cmake
        # ... after rosidl_generate_interfaces
        ament_export_dependencies(rosidl_default_runtime)
        # ...
        ```

3.  **اپنا پیکیج بنائیں:**

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_robot_pkg
    ```

4.  **اپنا ورک اسپیس سورس کریں:**

    ```bash
    source install/setup.bash
    ```

## نوڈز میں کسٹم میسجز کا استعمال کرنا

اب جب کہ کسٹم میسج `RobotPose` بنایا گیا ہے، ہم اسے اپنے پبلشر اور سبسکرائبر نوڈز میں استعمال کر سکتے ہیں۔

### `RobotPose` کا استعمال کرتے ہوئے پبلشر

آئیے `my_robot_pkg/my_robot_pkg/simple_publisher.py` میں ترمیم کریں تاکہ `String` کے بجائے `RobotPose` پیغامات پبلش کیے جا سکیں۔

```python
# ~/ros2_ws/src/my_robot_pkg/my_robot_pkg/custom_pose_publisher.py

import rclpy
from rclpy.node import Node
from my_robot_pkg.msg import RobotPose  # Import our custom message

class CustomPosePublisher(Node):

    def __init__(self):
        super().__init__('custom_pose_publisher')
        self.publisher_ = self.create_publisher(RobotPose, 'robot_pose', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def timer_callback(self):
        msg = RobotPose() # Create an instance of our custom message
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        msg.roll = self.roll
        msg.pitch = self.pitch
        msg.yaw = self.yaw

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing RobotPose: x={msg.x:.2f}, y={msg.y:.2f}, yaw={msg.yaw:.2f}')

        # Simulate movement
        self.x += 0.1
        self.y += 0.05
        self.yaw += 0.01

def main(args=None):
    rclpy.init(args=args)
    custom_pose_publisher = CustomPosePublisher()
    rclpy.spin(custom_pose_publisher)
    custom_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### `RobotPose` کا استعمال کرتے ہوئے سبسکرائبر

اسی طرح، `my_robot_pkg/my_robot_pkg/simple_subscriber.py` میں ترمیم کریں تاکہ `RobotPose` پیغامات کو سبسکرائب کیا جا سکے۔

```python
# ~/ros2_ws/src/my_robot_pkg/my_robot_pkg/custom_pose_subscriber.py

import rclpy
from rclpy.node import Node
from my_robot_pkg.msg import RobotPose  # Import our custom message

class CustomPoseSubscriber(Node):

    def __init__(self):
        super().__init__('custom_pose_subscriber')
        self.subscription = self.create_subscription(
            RobotPose,  # Subscribe to our custom message type
            'robot_pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I received RobotPose: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}, ' +
                               f'roll={msg.roll:.2f}, pitch={msg.pitch:.2f}, yaw={msg.yaw:.2f}')

def main(args=None):
    rclpy.init(args=args)
    custom_pose_subscriber = CustomPoseSubscriber()
    rclpy.spin(custom_pose_subscriber)
    custom_pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### نئے نوڈز کے لیے `setup.py` کو اپ ڈیٹ کریں

`~/ros2_ws/src/my_robot_pkg/setup.py` کو اپ ڈیٹ کرنا یاد رکھیں تاکہ نئے پبلشر اور سبسکرائبر نوڈز شامل ہوں:

```python
# ... inside setup()
entry_points={
    'console_scripts': [
        'simple_publisher = my_robot_pkg.simple_publisher:main',
        'simple_subscriber = my_robot_pkg.simple_subscriber:main',
        'custom_pose_publisher = my_robot_pkg.custom_pose_publisher:main', # Add this line
        'custom_pose_subscriber = my_robot_pkg.custom_pose_subscriber:main', # Add this line
    ],
},
# ... rest of setup()
```

اس باب نے آپ کو کسٹم ROS2 میسج کی اقسام بنانے اور استعمال کرنے میں رہنمائی کی ہے، جو زیادہ نفیس روبوٹ ایپلی کیشنز بنانے کے لیے ضروری ہے۔ اب آپ کے پاس منظم ڈیٹا کی تعریف کرنے اور نوڈز کو پیچیدہ معلومات کو مؤثر طریقے سے بات چیت کرنے کے قابل بنانے کے لیے ٹولز موجود ہیں۔ یہ ROS2 بنیادی ماڈیول کا اختتام ہے۔ اگلے ماڈیول میں، ہم روبوٹ سمولیشن کو دریافت کریں گے۔

```mermaid
graph TD
    A[Define .msg file (e.g., RobotPose.msg)] --> B{Update package.xml}
    B --> C{Update CMakeLists.txt}
    C --> D[colcon build]
    D --> E[Source install/setup.bash]
    E --> F[Import in Python/C++ Node]
    F --> G[Publish/Subscribe Custom Message]
```

### مشق: کسٹم سینسر ڈیٹا میسج کی تعریف کرنا اور استعمال کرنا

**مقصد:** زیادہ پیچیدہ سینسر ڈیٹا (مثلاً، IMU ریڈنگز) کی نمائندگی کرنے کے لیے ایک کسٹم ROS2 میسج بنائیں اور اسے پبلشر-سبسکرائبر کے جوڑے میں ضم کریں۔

**مراحل:**

1.  **کسٹم میسج `SensorData.msg` کی تعریف کریں:**
    *   `~/ros2_ws/src/my_robot_pkg/msg` میں جائیں۔
    *   درج ذیل مواد کے ساتھ `SensorData.msg` نامی ایک فائل بنائیں۔
        ```
        # SensorData.msg
        std_msgs/Header header
        float32 linear_acceleration_x
        float32 linear_acceleration_y
        float32 linear_acceleration_z
        float32 angular_velocity_x
        float32 angular_velocity_y
        float32 angular_velocity_z
        ```
        *(نوٹ: `std_msgs/Header` ٹائم اسٹیمپنگ اور فریم کی معلومات کے لیے ایک عام طور پر استعمال ہونے والی بلٹ ان میسج قسم ہے۔)*

2.  **بلڈ کنفیگریشن کو اپ ڈیٹ کریں:**
    *   یقینی بنائیں کہ `package.xml` میں ضروری بلڈ اور ایگزیکیوشن ڈیپینڈنسیز موجود ہیں (اس باب سے)۔
    *   `~/ros2_ws/src/my_robot_pkg/CMakeLists.txt` میں ترمیم کریں تاکہ `rosidl_generate_interfaces` کال میں `msg/SensorData.msg` شامل ہو۔ یقینی بنائیں کہ اگر آپ نے پہلے سے نہیں کیا ہے تو `std_msgs` بھی مل گیا ہے: `find_package(std_msgs REQUIRED)`۔ اگر آپ نے `std_msgs/Header` شامل کیا ہے، تو آپ کو `package.xml` میں `<depend>std_msgs</depend>` اور `CMakeLists.txt` میں `ament_target_dependencies(${PROJECT_NAME} std_msgs)` شامل کرنے کی ضرورت ہوگی۔

3.  **بنائین اور سورس کریں:**
    *   `~/ros2_ws` میں جائیں اور `colcon build --packages-select my_robot_pkg` کا استعمال کرتے ہوئے اپنا `my_robot_pkg` بنائیں۔
    *   اپنا ورک اسپیس سورس کریں: `source install/setup.bash`۔

4.  **کسٹم سینسر پبلشر بنائیں:**
    *   `~/ros2_ws/src/my_robot_pkg/my_robot_pkg/` میں `sensor_publisher.py` بنائیں۔
    *   ایک ROS2 نوڈ کو لاگو کریں جو `/imu_data` نامی ٹاپک پر 2 Hz کی شرح سے `my_robot_pkg/msg/SensorData` پیغامات پبلش کرتا ہے۔ فیلڈز کو مصنوعی اقدار (مثلاً، بڑھتے ہوئے نمبر) سے بھریں۔

5.  **کسٹم سینسر سبسکرائبر بنائیں:**
    *   اسی ڈائرکٹری میں، `sensor_subscriber.py` بنائیں۔
    *   ایک ROS2 نوڈ کو لاگو کریں جو `/imu_data` کو سبسکرائب کرتا ہے اور `linear_acceleration_x` اور `angular_velocity_z` کی اقدار کو پرنٹ کرتا ہے۔

6.  **`setup.py` کو اپ ڈیٹ کریں:**
    *   `~/ros2_ws/src/my_robot_pkg/setup.py` میں `sensor_publisher` اور `sensor_subscriber` کے لیے انٹری پوائنٹس شامل کریں۔

7.  **چلائیں اور تصدیق کریں:**
    *   دو ٹرمینل کھولیں، ہر ایک میں اپنا ورک اسپیس سورس کریں۔
    *   پبلشر چلائیں: `ros2 run my_robot_pkg sensor_publisher`۔
    *   سبسکرائبر چلائیں: `ros2 run my_robot_pkg sensor_subscriber`۔
    *   تصدیق کریں کہ سبسکرائبر کسٹم سینسر ڈیٹا وصول کرتا ہے اور پرنٹ کرتا ہے۔
    *   راؤ ڈیٹا دیکھنے کے لیے ایک تیسرے ٹرمینل میں `ros2 topic echo /imu_data` استعمال کریں۔

**متوقع نتیجہ:**
آپ نے کامیابی سے ایک کسٹم ROS2 میسج قسم بنائی، تعمیر کی، اور استعمال کی ہے تاکہ نوڈز کے درمیان پیچیدہ سینسر کی معلومات کا تبادلہ کیا جا سکے۔ آپ کو سبسکرائبر کو متعلقہ ڈیٹا پرنٹ کرتے ہوئے اور `ros2 topic echo` کو مکمل `SensorData` پیغامات دکھاتے ہوئے نظر آنا چاہیے۔
