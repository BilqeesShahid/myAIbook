---
id: chapter7
title: Module-1.simulation-chapter-7
sidebar_position: 3
slug: /simulation/chapter7
---

# باب 7: Gazebo میں روبوٹس کی تعریف اور سپان کرنا

Gazebo میں کسی روبوٹ کو سیمیولیٹ کرنے کے لیے، سب سے پہلے آپ کو روبوٹ کی تفصیلی وضاحت کی ضرورت ہوتی ہے۔ ROS/ROS2 ایکو سسٹم میں، روبوٹس کو عام طور پر **URDF** (Unified Robot Description Format) یا **XACRO** (XML Macros for ROS) کا استعمال کرتے ہوئے بیان کیا جاتا ہے۔ یہ فارمیٹس آپ کو روبوٹ کی کائنمیٹکس (لنکس اور جوائنٹس)، ڈائنامکس (ماس، انرشیا)، ویژولز (تھری ڈی ماڈلز، رنگ)، اور کولیژن پراپرٹیز کی تعریف کرنے کی اجازت دیتے ہیں۔

## URDF اور XACRO کو سمجھنا

*   **URDF (یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ)**:
    *   روبوٹ ماڈلز کی نمائندگی کے لیے ایک XML فارمیٹ۔
    *   سخت باڈیز (لنکس) اور ان کے کنکشنز (جوائنٹس) کی تعریف کرتا ہے۔
    *   بصری، کولیژن، اور انرشیا کی خصوصیات شامل کر سکتا ہے۔
    *   جامد اور پیچیدہ، ماڈیولر روبوٹس کے لیے عام طور پر کم لچکدار ہوتا ہے۔

*   **XACRO (روس کے لیے XML میکروز)**:
    *   ایک XML میکرو زبان جو URDF کو وسعت دیتی ہے۔
    *   پیرامیٹرائزڈ اور ماڈیولر روبوٹ کی وضاحتوں کی اجازت دیتی ہے۔
    *   تکرار کو کم کرتی ہے اور روبوٹ ماڈلز کو منظم کرنا اور ترمیم کرنا آسان بناتی ہے۔
    *   XACRO فائلوں کو URDF فائل بنانے کے لیے پراسیس کیا جاتا ہے۔

پیچیدہ روبوٹس کے لیے، XACRO کو اس کی ماڈیولرٹی اور پڑھنے کی اہلیت کی وجہ سے ہمیشہ خام URDF پر ترجیح دی جاتی ہے۔

## ایک سادہ روبوٹ ماڈل بنانا (XACRO)

آئیے XACRO کا استعمال کرتے ہوئے ایک سادہ ڈیفرینشل ڈرائیو روبوٹ ماڈل بناتے ہیں۔ اس روبوٹ میں ایک بیس لنک اور دو وہیل لنکس ہوں گے۔

1.  **اپنے `my_robot_pkg` میں ایک `urdf` ڈائریکٹری بنائیں:**

    ```bash
    mkdir -p ~/ros2_ws/src/my_robot_pkg/urdf
    ```

2.  **`~/ros2_ws/src/my_robot_pkg/urdf/my_robot.xacro` فائل بنائیں:**

    ```xml
    <?xml version="1.0"?>
    <robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

      <xacro:macro name="default_inertial" params="mass">
        <inertial>
          <mass value="${mass}"/>
          <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        </inertial>
      </xacro:macro>

      <link name="base_link">
        <visual>
          <geometry>
            <box size="0.3 0.2 0.1"/>
          </geometry>
          <material name="blue">
            <color rgba="0 0 0.8 1"/>
          </material>
        </visual>
        <collision>
          <geometry>
            <box size="0.3 0.2 0.1"/>
          </geometry>
        </collision>
        <xacro:default_inertial mass="1.0"/>
      </link>

      <link name="left_wheel_link">
        <visual>
          <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
          <geometry>
            <cylinder radius="0.05" length="0.02"/>
          </geometry>
          <material name="black">
            <color rgba="0 0 0 1"/>
          </material>
        </visual>
        <collision>
          <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
          <geometry>
            <cylinder radius="0.05" length="0.02"/>
          </geometry>
        </collision>
        <xacro:default_inertial mass="0.1"/>
      </link>

      <link name="right_wheel_link">
        <visual>
          <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
          <geometry>
            <cylinder radius="0.05" length="0.02"/>
          </geometry>
          <material name="black">
            <color rgba="0 0 0 1"/>
          </material>
        </visual>
        <collision>
          <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
          <geometry>
            <cylinder radius="0.05" length="0.02"/>
          </geometry>
        </collision>
        <xacro:default_inertial mass="0.1"/>
      </link>

      <joint name="base_to_left_wheel" type="continuous">
        <parent link="base_link"/>
        <child link="left_wheel_link"/>
        <origin xyz="0.1 0.11 0" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
      </joint>

      <joint name="base_to_right_wheel" type="continuous">
        <parent link="base_link"/>
        <child link="right_wheel_link"/>
        <origin xyz="0.1 -0.11 0" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
      </joint>

    </robot>
    ```

## Gazebo میں روبوٹ کو سپان کرنا

اپنے روبوٹ کو Gazebo سمولیشن میں لانے کے لیے، ہم عام طور پر ایک ROS2 نوڈ استعمال کرتے ہیں جسے `robot_state_publisher` کہا جاتا ہے، تاکہ روبوٹ کے URDF کو `/robot_description` ٹاپک پر شائع کیا جا سکے، اور پھر Gazebo پلگ ان یا ایک مخصوص ROS2 پیکیج ماڈل کو سپان کرنے کے لیے استعمال کیا جاتا ہے۔

ہم اسے حاصل کرنے کے لیے اپنی `gazebo_launch.py` میں ترمیم کریں گے۔\n\n1.  **URDF فائلوں کو انسٹال کرنے کے لیے `setup.py` کو اپ ڈیٹ کریں:**\n\n    `~/ros2_ws/src/my_robot_pkg/setup.py` میں `urdf` ڈائریکٹری کو `data_files` لسٹ میں شامل کریں:\n\n    ```python\n    # ... inside setup()\n    data_files=[\n        ...\n        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),\n    ],\n    # ... rest of setup()\n    ```\n\n    *`setup.py` کے اوپری حصے میں `import os` اور `from glob import glob` شامل کرنا نہ بھولیں।*\n\n2.  **`~/ros2_ws/src/my_robot_pkg/launch/gazebo_launch.py` میں ترمیم کریں:**\n\n    ہمیں یہ کرنا ہے:\n    *   XACRO فائل کو URDF سٹرنگ میں پراسیس کریں۔\n    *   روبوٹ کی حالت کو نشر کرنے کے لیے `robot_state_publisher` لانچ کریں۔\n    *   روبوٹ کو Gazebo میں شامل کرنے کے لیے `ros_gz_sim` سے ایک `spawn_entity.py` نوڈ لانچ کریں۔\n\n    ```python\n    import os\n    from ament_index_python.packages import get_package_share_directory\n    from launch import LaunchDescription\n    from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument\n    from launch.launch_description_sources import PythonLaunchDescriptionSource\n    from launch.substitutions import LaunchConfiguration, Command\n    from launch_ros.actions import Node\n\n\n    def generate_launch_description():\n        # Declare arguments\n        pkg_name = 'my_robot_pkg'\n        world_file_name = 'empty.world'\n        robot_file_name = 'my_robot.xacro'\n\n        # Get package directory\n        pkg_share_dir = get_package_share_directory(pkg_name)\n        world_path = os.path.join(pkg_share_dir, 'worlds', world_file_name)\n        robot_path = os.path.join(pkg_share_dir, 'urdf', robot_file_name)\n\n        # Process the xacro file into a URDF string\n        robot_description_content = Command(['xacro ', robot_path])\n\n        # Robot State Publisher Node\n        robot_state_publisher_node = Node(\n            package='robot_state_publisher',\n            executable='robot_state_publisher',\n            name='robot_state_publisher',\n            output='screen',\n            parameters=[{'robot_description': robot_description_content}],\n        )\n\n        # Launch Gazebo\n        gazebo_ros_package_dir = get_package_share_directory('gazebo_ros')\n        gazebo_launch = IncludeLaunchDescription(\n            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_package_dir, 'launch', 'gazebo.launch.py')),\n            launch_arguments={'world': world_path}.items(),\n        )\n\n        # Spawn Entity Node (from ros_gz_sim package)\n        spawn_entity = Node(\n            package='ros_gz_sim',\n            executable='create',\n            arguments=['-name', 'my_robot',\n                       '-topic', 'robot_description',\n                       '-x', '0.0',\n                       '-y', '0.0',\n                       '-z', '0.1'], # Lift slightly off ground\n            output='screen',\n        )\n\n        return LaunchDescription([\n            gazebo_launch,\n            robot_state_publisher_node,\n            spawn_entity\n        ])\n    ```\n\n    *فائل کے اوپری حصے میں `from launch_ros.actions import Node` اور `from launch.substitutions import Command` شامل کرنا نہ بھولیں।*\n\n3.  **یقینی بنائیں کہ `xacro` اور `ros_gz_sim` انسٹال ہیں:**\n\n    ```bash\n    sudo apt install ros-humble-xacro ros-humble-ros-gz-sim\n    ```\n\n4.  **اپنا پیکیج بنائیں اور سورس کریں:**\n\n    ```bash\n    cd ~/ros2_ws\n    colcon build --packages-select my_robot_pkg\n    source install/setup.bash\n    ```\n\n5.  **اپنے روبوٹ کے ساتھ Gazebo لانچ کریں:**\n\n    ```bash\n    ros2 launch my_robot_pkg gazebo_launch.py\n    ```\n\n    اب آپ کو اپنا سادہ ڈیفرینشل ڈرائیو روبوٹ Gazebo کی دنیا میں سپان ہوا نظر آنا چاہیے!\n\nاس باب نے آپ کے روبوٹ ڈیزائنز کو Gazebo سمولیشن میں لانے کی بنیاد فراہم کی۔ آپ نے XACRO کا استعمال کرتے ہوئے روبوٹ کائنمیٹکس اور ویژولز کی تعریف کرنا سیکھا اور ROS2 لانچ فائلوں کا استعمال کرتے ہوئے ان ماڈلز کو لانچ اور سپان کرنا سیکھا۔ اگلے باب میں، ہم سیمیولیٹڈ روبوٹ کو کنٹرول کرنے کا طریقہ دریافت کریں گے۔\n```\n