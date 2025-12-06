---
id: chapter8
title: Module-1.simulation-chapter-8
sidebar_position: 4
slug: /simulation/chapter8
---

# باب 8: ROS2 کے ساتھ سمیولیٹڈ روبوٹس کو کنٹرول کرنا

Gazebo میں روبوٹ ماڈل کا ہونا صرف پہلا قدم ہے؛ اسے مفید کام انجام دینے کے لیے، آپ کو اس کے جوڑوں کو کنٹرول کرنے کی ضرورت ہے۔ ROS2 `ros2_control` فراہم کرتا ہے، ایک طاقتور فریم ورک جو روبوٹ ہارڈ ویئر (یا سمیولیٹڈ ہارڈ ویئر) کے ساتھ انٹرفیس کرنے اور اس کے مختلف اجزاء کو کنٹرول کرنے کا ایک معیاری طریقہ پیش کرتا ہے۔

## `ros2_control` کا تعارف

`ros2_control` پیکجز کا ایک سیٹ ہے جو روبوٹ کنٹرول کے لیے ایک عمومی اور دوبارہ قابل استعمال فن تعمیر فراہم کرتا ہے۔ یہ روبوٹ کے کنٹرول منطق کو اس کے مخصوص ہارڈ ویئر انٹرفیس سے الگ کرتا ہے، جس سے آپ کے اعلیٰ سطحی کنٹرول کوڈ کو تبدیل کیے بغیر مختلف روبوٹس یا جسمانی اور سمیولیٹڈ ماحول کے درمیان سوئچ کرنا آسان ہو جاتا ہے۔

`ros2_control` میں کلیدی تصورات:

*   **ہارڈ ویئر انٹرفیس**: ایک سافٹ ویئر جزو جو براہ راست روبوٹ کے ایکٹیویٹرز اور سینسرز (جیسے، موٹر کنٹرولرز، جوائنٹ انکوڈرز) کے ساتھ تعامل کرتا ہے۔ سمیلیشن میں، یہ Gazebo پلگ انز کے ذریعہ فراہم کیا جاتا ہے۔
*   **کنٹرولر**: ایک سافٹ ویئر جزو جو کنٹرول کی حکمت عملی کو نافذ کرتا ہے (جیسے، ایک پوزیشن کنٹرولر، ایک ویلوسٹی کنٹرولر، ایک امپیڈینس کنٹرولر)۔ کنٹرولرز `controller_manager` میں چلتے ہیں۔
*   **کنٹرولر مینیجر**: ایک مرکزی نوڈ جو کنٹرولرز کو لوڈ کرنے، شروع کرنے، روکنے اور سوئچ کرنے کا ذمہ دار ہے۔
*   **URDF/XACRO ایکسٹینشنز**: آپ کے روبوٹ کی تفصیل فائل (URDF/XACRO) کو `<ros2_control>` ٹیگز کو شامل کرنے کے لیے بڑھایا جاتا ہے، جس میں یہ بیان کیا جاتا ہے کہ ہر جوائنٹ کے لیے کون سے ہارڈ ویئر انٹرفیس اور کنٹرولرز دستیاب ہیں۔

## اپنے روبوٹ کے ساتھ `ros2_control` کو ضم کرنا

آئیے اپنے ڈفرینشل ڈرائیو روبوٹ کے پہیوں کے لیے `ros2_control` کنفیگریشن کو شامل کرنے کے لیے `my_robot.xacro` میں ترمیم کریں۔

1.  **`~/ros2_ws/src/my_robot_pkg/urdf/my_robot.xacro` میں ترمیم کریں:**

    ہمیں ایک `<ros2_control>` بلاک اور Gazebo کے مخصوص پلگ انز کو شامل کرنے کی ضرورت ہے۔

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

      <!-- ROS2 Control configuration -->
      <ros2_control name="GazeboSystem" type="system">
        <hardware>
          <plugin>gazebo_ros2_control/GazeboSystem</plugin>
        </hardware>
        <joint name="base_to_left_wheel">
          <command_interface name="velocity">
            <param name="min">-10</param>
            <param name="max">10</param>
          </command_interface>
          <state_interface name="velocity"/>
          <state_interface name="position"/>
        </joint>
        <joint name="base_to_right_wheel">
          <command_interface name="velocity">
            <param name="min">-10</param>
            <param name="max">10</param>
          </command_interface>
          <state_interface name="velocity"/>
          <state_interface name="position"/>
        </joint>
      </ros2_control>

      <!-- Gazebo Diff Drive Plugin (for physics and cmd_vel interface) -->
      <gazebo>
        <plugin name="gazebo_ros_ackermann_diff_drive" filename="libgazebo_ros_ackermann_diff_drive.so">
          <ros> <!-- ROS properties -->
            <namespace>/</namespace>
            <command_topic>cmd_vel</command_topic>
            <odom_topic>odom</odom_topic>
            <odom_frame>odom</odom_frame>
            <base_frame>base_link</base_frame>
          </ros>
          <left_joint>base_to_left_wheel</left_joint>
          <right_joint>base_to_right_wheel</right_joint>
          <wheel_separation>0.22</wheel_separation> <!-- from model geometry -->
          <wheel_radius>0.05</wheel_radius> <!-- from model geometry -->
          <publish_odom>true</publish_odom>
          <publish_odom_tf>true</publish_odom_tf>
          <publish_wheel_tf>true</publish_wheel_tf>
          <legacy_mode>false</legacy_mode>
          <max_wheel_torque>200</max_wheel_torque>
          <max_wheel_acceleration>10.0</max_wheel_acceleration>
        </plugin>
      </gazebo>

    </robot>
    ```
    *نوٹ: `gazebo_ros_ackermann_diff_drive` پلگ ان یہاں ڈفرینشل ڈرائیو روبوٹس کے لیے `cmd_vel` کو جوائنٹ کمانڈز میں ترجمہ کرنے کی ایک عام مثال کے طور پر استعمال ہوتا ہے۔ اس کے لیے آپ کو `ros-humble-gazebo-ros-pkgs` انسٹال کرنے کی ضرورت پڑ سکتی ہے۔* اگر آپ `ackermann_diff_drive` پر `diff_drive` کو ترجیح دیتے ہیں تو آپ کو اس کے مطابق `.xacro` میں پلگ ان کا نام ایڈجسٹ کرنا پڑ سکتا ہے اور `ros-humble-gazebo-ros2-control` انسٹال کرنا پڑ سکتا ہے جو `libgazebo_ros_diff_drive.so` فراہم کرے گا۔

2.  **ایک `my_robot_pkg/config/` ڈائریکٹری بنائیں:**

    ```bash
    mkdir -p ~/ros2_ws/src/my_robot_pkg/config
    ```

3.  **`~/ros2_ws/src/my_robot_pkg/config/my_robot_controllers.yaml` بنائیں:**

    یہ YAML فائل `controller_manager` کے ذریعے لوڈ ہونے والے کنٹرولرز کی وضاحت کرتی ہے۔\n\n    ```yaml\n    controller_manager:\n      ros__parameters:\n        update_rate: 100 # Hz\n\n        joint_state_broadcaster:\n          type: joint_state_broadcaster/JointStateBroadcaster\n\n        diff_drive_controller:\n          type: diff_drive_controller/DiffDriveController\n\n    diff_drive_controller:\n      ros__parameters:\n        left_wheel_names: ['base_to_left_wheel']\n        right_wheel_names: ['base_to_right_wheel']\n        wheel_separation: 0.22\n        wheel_radius: 0.05\n        publish_rate: 50.0\n        velocity_rolling_window_size: 10\n        linear_tolerance: 0.0\n        angular_tolerance: 0.0\n
        base_frame_id: base_link\n        odom_frame_id: odom\n\n        # Optional: kinematics parameters for a differential drive robot\n        # The plugin takes care of this, but it's good to define if not using plugin\n        # wheel_separation: 0.22\n        # wheel_radius: 0.05\n    ```\n\n4.  **کنفگ فائل کو انسٹال کرنے کے لیے `setup.py` کو اپ ڈیٹ کریں:**\n\n    `~/ros2_ws/src/my_robot_pkg/setup.py` میں `config` ڈائریکٹری کو `data_files` میں شامل کریں:\n\n    ```python\n    # ... setup() کے اندر\n    data_files=[\n        ...\n        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),\n    ],\n    # ... setup() کا باقی حصہ\n    ```\n\n5.  **`~/ros2_ws/src/my_robot_pkg/launch/gazebo_launch.py` میں ترمیم کریں:**\n\n    ہمیں یہ کرنے کی ضرورت ہے:\n    *   `robot_controllers.yaml` فائل لوڈ کریں۔\n    *   `controller_manager` نوڈ لانچ کریں۔\n    *   `joint_state_broadcaster` اور `diff_drive_controller` کو لوڈ اور شروع کریں۔\n\n    ```python\n    import os\n    from ament_index_python.packages import get_package_share_directory\n    from launch import LaunchDescription\n    from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument\n    from launch.launch_description_sources import PythonLaunchDescriptionSource\n    from launch.substitutions import LaunchConfiguration, Command\n    from launch_ros.actions import Node\n\n
    def generate_launch_description():\n        # Declare arguments\n        pkg_name = 'my_robot_pkg'\n        world_file_name = 'empty.world'\n        robot_file_name = 'my_robot.xacro'\n        controller_file_name = 'my_robot_controllers.yaml'\n\n        # Get package directory\n        pkg_share_dir = get_package_share_directory(pkg_name)\n        world_path = os.path.join(pkg_share_dir, 'worlds', world_file_name)\n        robot_path = os.path.join(pkg_share_dir, 'urdf', robot_file_name)\n        controller_config_path = os.path.join(pkg_share_dir, 'config', controller_file_name)\n\n        # Process the xacro file into a URDF string\n        robot_description_content = Command(['xacro ', robot_path])\n\n        # Robot State Publisher Node\n        robot_state_publisher_node = Node(\n            package='robot_state_publisher',\n            executable='robot_state_publisher',\n            name='robot_state_publisher',\n            output='screen',\n            parameters=[{'robot_description': robot_description_content}],\n        )\n\n        # Launch Gazebo\n        gazebo_ros_package_dir = get_package_share_directory('gazebo_ros')\n        gazebo_launch = IncludeLaunchDescription(\n            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_package_dir, 'launch', 'gazebo.launch.py')),\n            launch_arguments={'world': world_path}.items(),\n        )\n\n        # Spawn Entity Node (from ros_gz_sim package)\n        spawn_entity = Node(\n            package='ros_gz_sim',\n            executable='create',\n            arguments=['-name', 'my_robot',\n                       '-topic', 'robot_description',\n                       '-x', '0.0',\n                       '-y', '0.0',\n                       '-z', '0.1'],\n            output='screen',\n        )\n\n        # ROS2 Control Nodes\n        control_node = Node(\n            package='controller_manager',\n            executable='ros2_control_node',\n            parameters=[robot_description_content, controller_config_path],\n            output={'stdout': 'screen', 'stderr': 'screen'},\n        )\n\n        # Load controllers\n        load_joint_state_broadcaster = Node(\n            package='controller_manager',\n            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output={'stdout': 'screen', 'stderr': 'screen'},
        )

        load_diff_drive_controller = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
            output={'stdout': 'screen', 'stderr': 'screen'},
        )

        return LaunchDescription([
            gazebo_launch,
            robot_state_publisher_node,
            spawn_entity,
            control_node,
            load_joint_state_broadcaster,
            load_diff_drive_controller
        ])
    ```
    *ضروری امپورٹس شامل کرنا نہ بھولیں، مثلاً `from launch_ros.actions import Node`۔* اور یقینی بنائیں کہ آپ نے `ros-humble-controller-manager` اور `ros-humble-diff-drive-controller` انسٹال کر لیے ہیں (`sudo apt install ros-humble-controller-manager ros-humble-diff-drive-controller`)۔

6.  **اپنے پیکیج کو بنائیں اور سورس کریں:**

    ```bash
    cd ~/ros2_ws\n    colcon build --packages-select my_robot_pkg\n    source install/setup.bash\n    ```

7.  **کنٹرول شدہ روبوٹ کے ساتھ Gazebo لانچ کریں:**

    ```bash\n    ros2 launch my_robot_pkg gazebo_launch.py\n    ```

    آپ کو Gazebo لانچ ہوتا ہوا نظر آئے گا، اور ٹرمینل آؤٹ پٹ میں، ایسے پیغامات جو یہ ظاہر کریں گے کہ کنٹرولرز لوڈ اور شروع ہو گئے ہیں۔\n\n## روبوٹ کو کمانڈز بھیجنا\n
`diff_drive_controller` کے لوڈ ہونے کے ساتھ، آپ کا روبوٹ اب `/diff_drive_controller/cmd_vel_unstamped` ٹاپک (یا اگر آپ کے YAML میں مختلف طریقے سے کنفیگر کیا گیا ہے تو `/cmd_vel`) پر `geometry_msgs/msg/Twist` پیغامات سن رہا ہوگا۔ آپ اپنے روبوٹ کو حرکت دینے کے لیے ویلوسٹی کمانڈز بھیج سکتے ہیں۔\n\n1.  **ایک نیا ٹرمینل کھولیں اور اپنے ورک اسپیس کو سورس کریں۔**\n\n2.  **ایک `Twist` پیغام شائع کریں:**\n\n    روبوٹ کو آگے بڑھانے کے لیے:\n\n    ```bash\n    ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \\\n        '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' --once\n    ```\n\n    روبوٹ کو موڑنے کے لیے:\n\n    ```bash\n    ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \\\n        '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}' --once\n    ```\n\n    آپ اپنے روبوٹ کو Gazebo میں حرکت کرتے ہوئے دیکھ سکتے ہیں۔ کنٹرولر کے ذریعہ شائع کردہ اوڈومیٹری پیغامات دیکھنے کے لیے `ros2 topic echo /odom` استعمال کریں۔\n\nیہ باب روبوٹ سمیلیشن ماڈیول کو یہ دکھا کر مکمل کرتا ہے کہ Gazebo میں `ros2_control` کا استعمال کرتے ہوئے ایک کنٹرول شدہ روبوٹ کی وضاحت کیسے کی جائے اور ROS2 ٹاپکس کے ذریعے اسے کمانڈز کیسے بھیجی جائیں۔ اب آپ کے پاس نفیس روبوٹ سمیلیشنز قائم کرنے اور انہیں پروگرام کے ذریعے کنٹرول کرنے کا بنیادی علم ہے۔ اگلے ماڈیول میں، ہم NVIDIA Isaac کے ساتھ AI-روبوٹ برین کو دریافت کریں گے۔\n```
