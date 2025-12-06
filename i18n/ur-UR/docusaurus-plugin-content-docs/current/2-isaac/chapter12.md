---
id: chapter12
title: Module-2.isaac-chapter-12
sidebar_position: 4
---

# باب 12: آئزک آر او ایس کے ساتھ تیز رفتار AI پرسیپشن

ذہین روبوٹکس کے سب سے اہم پہلوؤں میں سے ایک پرسیپشن ہے: روبوٹس کو سینسرز کے ذریعے اپنے ماحول کو سمجھنے کے قابل بنانا۔ NVIDIA Isaac ROS اس عمل کو نمایاں طور پر تیز کرتا ہے، عام پرسیپشن کے کاموں کے لیے ہارڈویئر-ایکسیلریٹڈ ROS2 پیکجز فراہم کرتا ہے، جو Jetson جیسے پلیٹ فارمز پر NVIDIA GPUs کی طاقت کا فائدہ اٹھاتا ہے۔

## آئزک آر او ایس کیا ہے؟

آئزک آر او ایس ROS2 پیکجز کا ایک مجموعہ ہے جو عام روبوٹکس الگورتھمز کے GPU-ایکسیلریٹڈ ورژنز کو نافذ کرتا ہے۔ اسے ROS2 ایکو سسٹم کے ساتھ بغیر کسی رکاوٹ کے کام کرنے کے لیے ڈیزائن کیا گیا ہے، جو درج ذیل شعبوں میں CPU-bound کاموں کے لیے ڈراپ-ان ریپلیسمنٹس یا بہتر اجزاء پیش کرتا ہے:

*   **امیج پروسیسنگ**: ڈی بایرنگ، ریسائزنگ، کلر کنورژن۔
*   **ڈیپتھ پرسیپشن**: سٹیریو میچنگ، پوائنٹ کلاؤڈ جنریشن۔
*   **آبجیکٹ ڈیٹیکشن اور ٹریکنگ**: YOLO، Faster R-CNN، مختلف ٹریکرز۔
*   **سیگمنٹیشن**: انسٹنس اور سیمینٹک سیگمنٹیشن۔
*   **SLAM/نیویگیشن**: ویژول اوڈومیٹری، لوکلائزیشن۔

ان کمپیوٹیشنز کو GPU پر منتقل کرکے، آئزک آر او ایس پرسیپشن پائپ لائنز کی تھرو پٹ اور لیٹنسی کو ڈرامائی طور پر بہتر بناتا ہے، جو ریئل ٹائم خود مختار روبوٹ آپریشن کے لیے ضروری ہے۔

## آئزک آر او ایس کے اہم اجزاء

*   **`isaac_ros_common`**: آئزک آر او ایس کے لیے بنیادی یوٹیلیٹیز، ڈوکر امیجز، اور بلڈ ٹولز فراہم کرتا ہے۔
*   **`isaac_ros_image_pipeline`**: تیز رفتار امیج پروسیسنگ کے کاموں کے لیے نوڈز پر مشتمل ہے۔
*   **`isaac_ros_dnn_inference`**: NVIDIA کے TensorRT کا استعمال کرتے ہوئے ڈیپ نیورل نیٹ ورک (DNN) انفرنس کی صلاحیتوں کو مربوط کرتا ہے۔
*   **`isaac_ros_nvcc`**: CUDA کمپائلیشن کے لیے ٹولز۔
*   **`isaac_ros_argus_camera`**: NVIDIA Jetson Argus کیمروں کے لیے انٹرفیس۔
*   **مختلف دیگر پیکجز**: مخصوص سینسرز، نیویگیشن، مینیپولیشن وغیرہ کے لیے۔

## آئزک آر او ایس سیٹ اپ کرنا

آئزک آر او ایس کی ترقی عام طور پر NVIDIA Jetson پلیٹ فارم یا ایک طاقتور NVIDIA GPU والے ورک سٹیشن پر ہوتی ہے۔ فزیکل روبوٹ پر تعیناتی کے لیے، عام طور پر ایک Jetson ماڈیول استعمال کیا جاتا ہے۔

1.  **جیٹسن کی تنصیب**: یقینی بنائیں کہ آپ کا NVIDIA Jetson ڈیوائس (مثلاً، Jetson Orin، Xavier) NVIDIA JetPack کے ساتھ سیٹ اپ ہے، جس میں CUDA، cuDNN، TensorRT، اور دیگر ضروری لائبریریاں شامل ہیں۔
2.  **ڈوکر انوائرمنٹ**: آئزک آر او ایس بہت زیادہ ڈوکر پر انحصار کرتا ہے۔ آپ کو اپنی ڈویلپمنٹ مشین/جیٹسن پر ڈوکر اور NVIDIA کنٹینر ٹول کٹ انسٹال کرنے کی ضرورت ہوگی۔
3.  **آئزک آر او ایس ورک اسپیس کلون کریں**: آئزک آر او ایس ریپوزٹریز کو ROS2 ورک اسپیس میں کلون کریں:\n
    ```bash\n    mkdir -p ~/isaac_ros_ws/src\n    cd ~/isaac_ros_ws/src\n    git clone https://github.com/NVIDIA-AI-IOT/isaac_ros_common.git\n    git clone https://github.com/NVIDIA-AI-IOT/isaac_ros_image_pipeline.git\n    # دیگر مطلوبہ آئزک آر او ایس پیکجز کلون کریں\n    ```\n\n4.  **ورک اسپیس بنائیں**: فراہم کردہ ڈوکر کنٹینر کے اندر `colcon` کا استعمال کرتے ہوئے ورک اسپیس بنائیں۔\n
    ```bash\n    cd ~/isaac_ros_ws\n    # ڈوکر کنٹینر کے اندر بنانے کے لیے فراہم کردہ اسکرپٹ استعمال کریں\n    ./src/isaac_ros_common/scripts/build_ros_workspace.sh\n    ```\n
    یہ اسکرپٹ ایک ڈوکر امیج بنائے گا، آپ کے ورک اسپیس کو ماؤنٹ کرے گا، اور اس کے اندر تمام ROS2 پیکجز بنائے گا۔\n
## ایک AI پرسیپشن پائپ لائن بنانا (مثال: آبجیکٹ ڈیٹیکشن)\n
آئیے ایک اعلیٰ سطحی مثال پر غور کریں کہ آپ آئزک آر او ایس کا استعمال کرتے ہوئے آبجیکٹ ڈیٹیکشن پائپ لائن کیسے بنا سکتے ہیں۔\n
1.  **کیمرہ نوڈ**: آپ کے روبوٹ کا کیمرہ ROS2 ٹاپک (`/image_raw`) پر خام امیج ڈیٹا (مثلاً، `sensor_msgs/msg/Image`) شائع کرتا ہے۔ یہ ایک فزیکل کیمرہ یا آئزک سم میں ایک سمیلیٹڈ کیمرہ سے ہو سکتا ہے۔\n
2.  **امیج پروسیسنگ (آئزک آر او ایس امیج پائپ لائن)**:\n    *   خام بایر پیٹرنز کو RGB میں تبدیل کرنے کے لیے `isaac_ros_image_proc/debayer` استعمال کریں۔\n    *   امیج کو اپنے DNN کے لیے مطلوبہ ان پٹ ڈائمینشنز میں اسکیل کرنے کے لیے `isaac_ros_image_proc/resize` استعمال کریں۔\n
3.  **DNN انفرنس (آئزک آر او ایس DNN انفرنس)**:\n    *   اپنے تربیت یافتہ AI ماڈل (مثلاً، PyTorch، TensorFlow) کو NVIDIA کے ٹولز کا استعمال کرتے ہوئے TensorRT انجن میں تبدیل کریں۔\n    *   اپنے DNN کے لیے امیج کو پری پروسیس کرنے کے لیے `isaac_ros_dnn_image_encoder` کو کنفیگر کریں۔\n    *   آبجیکٹ ڈیٹیکشن انفرنس کے لیے `isaac_ros_detectnet/detectnet` (DetectNet ماڈلز کے لیے) یا `isaac_ros_yolo/yolo` (YOLO ماڈلز کے لیے) استعمال کریں۔ یہ نوڈز پروسیس شدہ امیج ڈیٹا لیتے ہیں اور باؤنڈنگ باکسز اور کلاس لیبل آؤٹ پٹ کرتے ہیں۔\n
4.  **پوسٹ پروسیسنگ**: ٹریکنگ، فلٹرنگ، یا ویژولائزیشن جیسے کاموں کے لیے ڈیٹیکشن کے نتائج کو مزید پروسیس کریں۔\n
### مثال `ros2_launch` ٹکڑا (تصوراتی)\n
```python\n# ... imports ...\n\nfrom launch_ros.actions import ComposableNodeContainer\nfrom launch_ros.descriptions import ComposableNode\n\ndef generate_launch_description():\n    # ... دیگر لانچ سیٹ اپ ...\n\n    # امیج پروسیسنگ اور آبجیکٹ ڈیٹیکشن کنٹینر\n    container = ComposableNodeContainer(\n        name='isaac_ros_container',\n        namespace='',\n        package='rclcpp_components',\n        executable='component_container',\n        composable_node_descriptions=[\n            ComposableNode(\n                package='isaac_ros_image_proc',\n                plugin='isaac_ros_image_proc::ResizeNode',\n                name='resize_node',\n                parameters=[{'output_width': 640, 'output_height': 480}],\n                remappings=[('image', '/image_raw'), ('resize/image', '/image_resized')]\n            ),\n            ComposableNode(\n                package='isaac_ros_detectnet',\n                plugin='isaac_ros_detectnet::DetectNetNode',
                name='detectnet_node',
                parameters=[{'model_file_path': '/path/to/your/model.etlt'}],
                remappings=[('image_in', '/image_resized'), ('detections', '/detections')]
            ),\n        ],
        output='screen',
    )

    return LaunchDescription([
        # ... کیمرہ لانچ نوڈ ...
        container
    ])
```

اس باب نے آپ کو NVIDIA Isaac ROS اور روبوٹس کے لیے AI پرسیپشن کو تیز کرنے میں اس کے کردار سے متعارف کرایا ہے۔ آپ نے اس کے اہم اجزاء، سیٹ اپ کے عمل، اور یہ کیسے اعلیٰ کارکردگی والے پرسیپشن پائپ لائنز بنانے کے لیے استعمال کیا جا سکتا ہے کے بارے میں سیکھا ہے۔ یہ AI-Robot Brain ماڈیول کا اختتام ہے۔ اگلے ماڈیول میں، ہم ویژن-لینگویج-ایکشن (VLA) کو تلاش کریں گے۔