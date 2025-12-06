---
id: chapter6
title: Module-1.simulation-chapter-6
sidebar_position: 2
slug: /simulation/chapter6
---

# باب 6: روبوٹ سمولیشن کے لیے Gazebo

Gazebo روبوٹکس کمیونٹی میں سب سے طاقتور اور وسیع پیمانے پر استعمال ہونے والے 3D روبوٹ سمیلیٹرز میں سے ایک ہے۔ یہ ایک مضبوط فزکس انجن، اعلیٰ معیار کے گرافکس، اور ROS/ROS2 جیسے روبوٹکس مڈل ویئر کے ساتھ انضمام کے لیے آسان انٹرفیس پیش کرتا ہے۔ Gazebo آپ کو پیچیدہ روبوٹ میکانزم، سینسر ڈیٹا، اور ماحولیاتی تعاملات کی درستگی سے نقالی کرنے کی اجازت دیتا ہے۔

## Gazebo کی اہم خصوصیات

*   **فزکس انجن**: Gazebo جدید فزکس انجنوں (جیسے ODE، Bullet، Simbody، DART) کے ساتھ مربوط ہوتا ہے تاکہ حقیقت پسندانہ سخت جسمانی حرکیات، کشش ثقل، رگڑ، اور تصادم کا پتہ لگانے کی سہولت فراہم کرے۔
*   **3D ویژولائزیشن**: اس میں روبوٹس، ماحول اور سینسر فیڈ بیک کو دیکھنے کے لیے ایک نفیس 3D رینڈرنگ انجن (OGRE) شامل ہے۔
*   **سینسر سمولیشن**: Gazebo کیمروں (RGB، depth، monochrome)، LiDAR، IMUs، فورس/ٹارک سینسرز، اور بہت کچھ سمیت سینسرز کی ایک وسیع رینج کی درستگی سے نقالی کرسکتا ہے۔
*   **روبوٹ اور ورلڈ ماڈلز**: روبوٹس اور ماحول کو SDF (Simulation Description Format) فائلوں کا استعمال کرتے ہوئے بیان کیا جاتا ہے، جو لنکس، جوائنٹس، سینسرز، مواد، اور پلگ انز کی وضاحت کر سکتے ہیں۔
*   **پلگ انز**: Gazebo کی فعالیت پلگ انز کے ذریعے انتہائی قابل توسیع ہے، جو صارفین کو فزکس، سینسرز، کنٹرول، اور یوزر انٹرفیس کو اپنی مرضی کے مطابق بنانے کی اجازت دیتی ہے۔
*   **ROS/ROS2 انضمام**: Gazebo کی ایک اہم طاقت ROS/ROS2 کے ساتھ اس کا گہرا انضمام ہے، جو مصنوعی روبوٹس اور ROS2 نوڈس کے درمیان ہموار مواصلت کو ممکن بناتا ہے۔

## Gazebo کی تنصیب (ROS2 Humble کے ساتھ)

اگر آپ نے باب 1 کی پیروی کی ہے اور Ubuntu 22.04 پر ROS2 Humble انسٹال کیا ہے، تو Gazebo Garden (جو عام طور پر Humble کے ساتھ مربوط ہوتا ہے) سیٹ اپ کرنا نسبتاً آسان ہونا چاہیے۔ یقینی بنائیں کہ آپ کا ROS2 ماحول سورس کیا گیا ہے۔

1.  **Gazebo اور ROS2 Gazebo پیکجز انسٹال کریں:**

    ```bash
    sudo apt update
    sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
    ```

    *   `ros-humble-gazebo-ros-pkgs`: ROS2 اور Gazebo کے درمیان برج فراہم کرتا ہے، جو ROS2 نوڈس کو مصنوعی روبوٹس اور سینسرز کے ساتھ تعامل کرنے کی اجازت دیتا ہے۔
    *   `ros-humble-gazebo-ros2-control`: `ros2_control` (ایک ہارڈ ویئر ایبسٹریکشن لیئر) کو Gazebo کے ساتھ استعمال کرنے کے قابل بناتا ہے، جو روبوٹ جوائنٹس کو کنٹرول کرنے کے لیے اہم ہے۔

2.  **Gazebo کی تنصیب کی تصدیق کریں:**

    آپ یہ دیکھنے کے لیے براہ راست Gazebo کا GUI لانچ کر سکتے ہیں کہ آیا یہ کام کر رہا ہے:

    ```bash
    gazebo
    ```

    اس سے Gazebo سمیلیٹر ایک خالی دنیا کے ساتھ کھلنا چاہیے۔ آگے بڑھنے کے لیے اسے بند کریں۔

## ایک سادہ Gazebo دنیا بنانا

Gazebo کی دنیا `.world` فائلوں میں SDF (Simulation Description Format) کا استعمال کرتے ہوئے بیان کی جاتی ہے۔ آئیے ایک بہت ہی بنیادی دنیا بناتے ہیں۔

1.  **اپنے `my_robot_pkg` میں ایک `worlds` ڈائریکٹری بنائیں:**

    ```bash
    mkdir -p ~/ros2_ws/src/my_robot_pkg/worlds
    ```

2.  **`~/ros2_ws/src/my_robot_pkg/worlds/empty.world` بنائیں:**

    ```xml
    <?xml version="1.0" ?>
    <sdf version="1.6">
      <world name="empty_world">
        <gravity>0 0 -9.8</gravity>
        <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
        <gui>
          <camera name="user_camera">
            <pose>0 0 5 0 0.9 0</pose>
          </camera>
        </gui>
        <light type="directional" name="sun">
          <cast_shadows>1</cast_shadows>
          <pose>0 0 10 0 -0 0</pose>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
          <attenuation>
            <range>1000</range>
            <constant>0.9</constant>
            <linear>0.01</linear>
            <quadratic>0.001</quadratic>
          </attenuation>
          <direction>-0.5 0.1 -0.9</direction>
        </light>
        <model name="ground_plane">
          <static>true</static>
          <link name="link">
            <collision name="collision">
              <geometry>
                <plane>
                  <normal>0 0 1</normal>
                  <size>100 100</size>
                </plane>
              </geometry>
              <surface>
                <friction>
                  <ode>
                    <mu>1.0</mu>
                    <mu2>1.0</mu2>
                  </ode>
                </friction>
              </surface>
            </collision>
            <visual name="visual">
              <geometry>
                <plane>
                  <normal>0 0 1</normal>
                  <size>100 100</size>
                </plane>
              </geometry>
              <material>
                <ambient>0.8 0.8 0.8 1</ambient>
                <diffuse>0.8 0.8 0.8 1</diffuse>
                <specular>0.8 0.8 0.8 1</specular>
              </material>
            </visual>
          </link>
        </model>
      </world>
    </sdf>
    ```

3.  **اپنی مرضی کی دنیا کے ساتھ Gazebo لانچ کریں:**

    ```bash
    gazebo --verbose ~/ros2_ws/src/my_robot_pkg/worlds/empty.world
    ```

    اب آپ کو Gazebo ایک فلیٹ گراؤنڈ پلین اور ایک ڈائریکشنل لائٹ کے ساتھ لانچ ہوتا ہوا نظر آنا چاہیے۔

## ROS2 کے ساتھ انضمام

Gazebo کی حقیقی طاقت ROS2 کے ساتھ اس کے انضمام سے حاصل ہوتی ہے۔ ہم Gazebo پلگ انز کا استعمال مصنوعی روبوٹ اجزاء اور سینسر ڈیٹا کو ROS2 ٹاپکس، سروسز اور ایکشنز کے طور پر ظاہر کرنے کے لیے کرتے ہیں۔

### Gazebo شروع کرنے کے لیے `ros2_launch` کا استعمال

Gazebo شروع کرنے اور روبوٹس لوڈ کرنے کے لیے `ros2_launch` فائلوں کا استعمال عام ہے۔ یہ ایک زیادہ منظم اور قابل تولید سیٹ اپ کی اجازت دیتا ہے۔

1.  **اپنے `my_robot_pkg` میں ایک `launch` ڈائریکٹری بنائیں:**

    ```bash
    mkdir -p ~/ros2_ws/src/my_robot_pkg/launch
    ```

2.  **`~/ros2_ws/src/my_robot_pkg/launch/gazebo_launch.py` بنائیں:**

    ```python
    import os

    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource


    def generate_launch_description():
        # Get the launch directory
        gazebo_ros_package_dir = get_package_share_directory('gazebo_ros')

        # Launch Gazebo
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_package_dir, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': os.path.join(get_package_share_directory('my_robot_pkg'), 'worlds', 'empty.world')}.items(),
        )

        return LaunchDescription([
            gazebo_launch
        ])
    ```

3.  **لانچ فائل انسٹال کرنے کے لیے `setup.py` کو اپ ڈیٹ کریں:**

    `~/ros2_ws/src/my_robot_pkg/setup.py` میں `data_files` فہرست میں درج ذیل شامل کریں۔ اگر `data_files` موجود نہیں ہے تو اسے بنائیں۔

    ```python
    # ... inside setup()
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add these lines:
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
    ],
    # ... rest of setup()
    ```
    *`setup.py` کے اوپری حصے میں `import os` اور `from glob import glob` شامل کرنا نہ بھولیں۔*\n\n4.  **اپنا پیکج بنائیں اور سورس کریں:**\n\n    ```bash\n    cd ~/ros2_ws\n    colcon build --packages-select my_robot_pkg\n    source install/setup.bash\n    ```\n\n5.  **`ros2_launch` کا استعمال کرتے ہوئے Gazebo لانچ کریں:**\n\n    ```bash\n    ros2 launch my_robot_pkg gazebo_launch.py\n    ```\n\n    اب یہ ROS2 لانچ سسٹم کا استعمال کرتے ہوئے آپ کی `empty.world` کے ساتھ Gazebo لانچ کرے گا۔\n\nاگلے باب میں، ہم URDF/XACRO کا استعمال کرتے ہوئے روبوٹس کی تعریف کرنا اور انہیں اپنی Gazebo دنیا میں سپون کرنا سیکھیں گے۔
