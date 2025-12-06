---
id: chapter19
title: Module-4.capstone-chapter-19
sidebar_position: 3
slug: /capstone/chapter19
---

# باب 19: نقلی ماحول کا سیٹ اپ اور ہیومنائڈ ماڈل

اس سے پہلے کہ ہم اپنے ہیومنائڈ روبوٹ کے لیے خود مختار رویوں کو لاگو کر سکیں، ہمیں ایک حقیقت پسندانہ اور فعال نقلی ماحول قائم کرنے اور اس کے اندر اپنے ہیومنائڈ روبوٹ ماڈل کو مربوط کرنے کی ضرورت ہے۔ یہ باب آپ کو ایک مناسب سمولیشن ورلڈ قائم کرنے اور تعامل کے لیے ہیومنائڈ روبوٹ کو کنفیگر کرنے کے بارے میں رہنمائی کرے گا۔

## نقلی گھریلو ماحول کا سیٹ اپ

اس کیپ اسٹون کے لیے، ہم ماڈیول 1 کے علم کی بنیاد پر Gazebo یا Isaac Sim کو اپنے سمولیشن پلیٹ فارم کے طور پر استعمال کریں گے۔ ماحول کو ایک عام گھریلو ترتیب کی نمائندگی کرنی چاہیے جس میں کئی کمرے (مثلاً، رہائشی کمرہ، کچن)، فرنیچر، اور مینپولیشن کے لیے چھوٹی اشیاء ہوں۔

### Gazebo کا استعمال (مثال)

اگر Gazebo استعمال کر رہے ہیں، تو آپ عام طور پر:

1.  **ایک کسٹم `.world` فائل بنائیں**: یہ فائل (مثلاً، `~/ros2_ws/src/my_robot_pkg/worlds/home_environment.world`) آپ کے ماحول کے جامد عناصر کی وضاحت کرے گی، جیسے دیواریں، فرش، فرنیچر (میز، شیلف)، اور روشنی کے ذرائع۔ آپ سادہ پرمیٹیوز استعمال کر سکتے ہیں یا Gazebo کے ماڈل ڈیٹا بیس یا کسٹم اثاثوں سے مزید پیچیدہ ماڈلز درآمد کر سکتے ہیں۔

    *مثال ٹکڑا (`home_environment.world`)*:
    ```xml
    <?xml version="1.0" ?>
    <sdf version="1.6">
      <world name="home_world">
        <include>
          <uri>model://sun</uri>
        </include>
        <include>
          <uri>model://ground_plane</uri>
        </include>

        <!-- مثال: ایک سادہ میز -->
        <model name="table">
          <pose>2 0 0.75 0 0 0</pose>
          <static>true</static>
          <link name="table_link">
            <visual name="visual">
              <geometry><box><size>1.0 0.6 0.03</size></box></geometry>
              <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material>
            </visual>
            <collision name="collision">
              <geometry><box><size>1.0 0.6 0.03</size></box></geometry>
            </collision>
          </link>
          <link name="leg1">
            <!-- ... ٹانگ کی تعریف ... -->
          </link>
          <!-- ... دیگر ٹانگیں ... -->
        </model>

        <!-- آپ دیواروں، دیگر فرنیچر وغیرہ کے لیے مزید ماڈلز شامل کریں گے -->

      </world>
    </sdf>
    ```

2.  **اشیاء سے بھرنا**: آپ کو دنیا میں چھوٹی، قابل تعامل اشیاء (مثلاً، سوڈا کین، کتابیں، کھلونے) شامل کرنے کی ضرورت ہوگی۔ یہ سادہ Gazebo پرمیٹیوز یا درآمد شدہ میشز ہو سکتی ہیں۔ یقینی بنائیں کہ وہ جامد نہیں ہیں تاکہ روبوٹ ان کو مینپولیشن کر سکے۔

    *مثال: ایک سوڈا کین شامل کرنا*
    ```xml
    <model name="soda_can">
      <pose>2.5 0.2 0.8 0 0 0</pose>
      <link name="can_link">
        <visual name="visual"><geometry><cylinder><radius>0.03</radius><length>0.12</length></cylinder></geometry></visual>
        <collision name="collision">
          <geometry><cylinder><radius>0.03</radius><length>0.12</length></cylinder></geometry>
        </collision>
        <inertial>
          <mass>0.1</mass>
          <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.00005"/>
        </inertial>
      </link>
    </model>
    ```

### Isaac Sim کا استعمال (مثال)

اگر Isaac Sim استعمال کر رہے ہیں، تو عمل میں USD اسٹیج بنانا شامل ہے:

1.  **ایک نیا USD اسٹیج بنائیں**: Isaac Sim میں ایک نئے، خالی اسٹیج سے شروع کریں۔
2.  **ماحولیاتی اثاثے درآمد کریں**: کمروں، فرنیچر، اور آرائشی عناصر کے لیے Omniverse کی وسیع اثاثہ لائبریری کا استعمال کریں یا کسٹم 3D ماڈلز (OBJ, FBX) درآمد کریں۔ انہیں اپنے گھریلو ماحول کی شکل دینے کے لیے ترتیب دیں۔
3.  **فزکس اور مواد شامل کریں**: یقینی بنائیں کہ تمام جامد ماحولیاتی اشیاء میں `Collider` اجزاء اور حقیقت پسندانہ تعاملات کے لیے مناسب فزکس مواد موجود ہوں۔
4.  **قابل تعامل اشیاء سے بھرنا**: چھوٹی اشیاء کو ڈریگ اور ڈراپ کریں یا درآمد کریں جنہیں روبوٹ مینپولیشن کرے گا۔ ان اشیاء میں `Rigid Body` فزکس شامل کریں تاکہ وہ روبوٹ سے متاثر ہو سکیں۔
5.  **لائٹنگ اور کیمرے**: پرسیپشن کے کاموں کے لیے اچھی بصری وفاداری کو یقینی بنانے کے لیے لائٹنگ کنفیگر کریں۔ مصنوعی ڈیٹا پیدا کرنے کے لیے گراؤنڈ-ٹروتھ کیمرے شامل کریں۔

## ہیومنائڈ روبوٹ ماڈل انضمام

ایک پیچیدہ ہیومنائڈ روبوٹ ماڈل کو مربوط کرنا ایک اہم قدم ہے۔ اس کیپ اسٹون کے لیے، ہم ایک پہلے سے موجود ہیومنائڈ URDF/XACRO ماڈل (یا Isaac Sim کے لیے ایک USD ماڈل) کی دستیابی کو فرض کریں گے جو ROS2 `ros2_control` کے ساتھ مطابقت رکھتا ہے۔

### روبوٹ ماڈل کی ضروریات

*   **URDF/XACRO (Gazebo کے لیے) یا USD (Isaac Sim کے لیے)**: ہیومنائڈ کے لنکس، جوائنٹس، انرشیل خصوصیات، بصری میشز، اور تصادم میشز کی ایک مکمل تفصیل۔
*   **`ros2_control` کنفیگریشن**: ماڈل کی تفصیل میں `<ros2_control>` ٹیگ *ضرور* شامل ہونا چاہیے، جس میں ہر جوائنٹ کے لیے ہارڈویئر انٹرفیس (مثلاً، پوزیشن، ویلوسٹی، ایفرٹ انٹرفیس) اور Gazebo/Isaac Sim پلگ انز کی وضاحت کی گئی ہو جو ان کو سمیلیٹر سے جوڑتے ہیں۔
*   **سینسرز**: ماڈل میں ایک نقلی RGB-D کیمرہ (رنگین تصاویر اور گہرائی کی معلومات کے لیے) اور ایک IMU شامل ہونا چاہیے۔ یہ سینسر ROS2 ٹاپکس پر ڈیٹا شائع کریں گے۔
*   **اینڈ-ایفیکٹر/گریپر**: اشیاء کو پکڑنے کے لیے مناسب جوائنٹ کی تعریفوں اور کنٹرول انٹرفیس کے ساتھ ایک فعال گریپر ماڈل۔

### ہیومنائڈ روبوٹ کو اسپان کرنا

باب 7 کی طرح، ہم اپنے نقلی ماحول میں ہیومنائڈ روبوٹ کو اسپان کرنے کے لیے ایک ROS2 لانچ فائل استعمال کریں گے۔

1.  **`robot_description`**: ہیومنائڈ کا URDF/XACRO پروسیس کیا جائے گا (مثلاً، `xacro` کا استعمال کرتے ہوئے) اور `robot_state_publisher` کے ذریعے `/robot_description` ٹاپک پر شائع کیا جائے گا۔
2.  **`ros_gz_sim` (Gazebo) یا Isaac Sim Python API**: ایک ROS2 نوڈ (جیسے `ros_gz_sim` کا `create` ایگزیکیوٹیبل) یا Isaac Sim کی API استعمال کرنے والی ایک Python اسکرپٹ کا استعمال ہیومنائڈ ماڈل کو سمولیشن ماحول میں شامل کرنے کے لیے کیا جائے گا۔
3.  **کنٹرولر اسپانرز**: آپ کی لانچ فائل `controller_manager` اور تمام ضروری کنٹرولرز (مثلاً، `joint_state_broadcaster`، بازوؤں/ٹانگوں کے لیے `joint_trajectory_controller`، اور مخصوص گریپر کنٹرولرز) کے لیے اسپانر نوڈز بھی شروع کرے گی۔

*مثال لانچ فائل ٹکڑا (`humanoid_sim_launch.py`)*:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # ... (باب 7 میں اسی طرح کے پاتھ کی تعریفیں)
    pkg_name = 'my_robot_pkg'
    humanoid_urdf_path = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'humanoid.xacro')
    world_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'home_environment.world')
    controller_config_path = os.path.join(get_package_share_directory(pkg_name), 'config', 'humanoid_controllers.yaml')

    robot_description_content = Command(['xacro ', humanoid_urdf_path])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
    )

    # کسٹم ورلڈ کے ساتھ Gazebo لانچ کریں
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_path}.items(),
    )

    # ہیومنائڈ روبوٹ کو Gazebo میں اسپان کریں
    spawn_humanoid = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'humanoid',
                   '-topic', 'robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '1.0'], # ابتدائی پوز کو ایڈجسٹ کریں
        output='screen',
    )

    # کنٹرولر مینیجر اور اسپانر نوڈز
    # ... (باب 8 کی طرح، لیکن ہیومنائڈ کے مخصوص کنٹرولرز کے لیے)

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_humanoid,
        # ... کنٹرولر نوڈز ...
    ])
```

## ابتدائی تصدیق

اپنے نقلی ماحول اور ہیومنائڈ روبوٹ کو لانچ کرنے کے بعد، درج ذیل کی تصدیق کریں:

*   **روبوٹ کی مرئیت**: کیا آپ اپنے ہیومنائڈ روبوٹ کو سمیلیٹر میں صحیح طریقے سے دیکھ سکتے ہیں؟
*   **جوائنٹ سٹیٹس**: `ros2 topic echo /joint_states` استعمال کریں تاکہ تصدیق ہو سکے کہ جوائنٹ پوزیشن/ویلوسٹی ڈیٹا شائع ہو رہا ہے۔
*   **سینسر ڈیٹا**: `/camera/image_raw`، `/camera/depth/image_raw`، اور `/imu/data` (یا اسی طرح کے ٹاپکس) کو چیک کریں تاکہ یقینی ہو سکے کہ سینسر ڈیٹا صحیح طریقے سے بہہ رہا ہے۔
*   **TF ٹری**: `ros2 run tf2_tools view_frames` استعمال کریں تاکہ روبوٹ کے کینیمیٹک ٹری کو ویژولائز کیا جا سکے اور یقینی ہو سکے کہ تمام لنکس اور جوائنٹس صحیح طریقے سے تبدیل ہو رہے ہیں۔

اس باب نے ہمارے نقلی گھریلو ماحول کو قائم کرکے اور ہیومنائڈ روبوٹ ماڈل کو ROS2 کے ساتھ مربوط کرکے بنیادی کام انجام دیا ہے۔ اس بنیاد کے ساتھ، اب ہم بنیادی پرسیپشن اور ایکشن کی صلاحیتوں کو تیار کرنے کے لیے آگے بڑھ سکتے ہیں۔ اگلے باب میں، ہم Isaac ROS اور بصری گراؤنڈنگ تکنیکوں کا استعمال کرتے ہوئے جدید پرسیپشن پائپ لائنز کو لاگو کرنے پر توجہ دیں گے۔
