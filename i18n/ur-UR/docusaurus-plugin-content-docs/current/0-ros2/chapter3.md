---
id: ros2-chapter3
title: "ماڈیول-0.ros2-باب-3"
sidebar_position: 3
slug: /ros2/chapter3
---

# باب 3: اپنے پہلے ROS2 نوڈز لکھنا (پبلشرز اور سبسکرائبرز)

ROS2 میں، ایک روبوٹ ایپلیکیشن کے بنیادی تعمیراتی بلاکس **نوڈز** ہیں۔ نوڈز آزاد قابل عمل پراسیسز ہیں جو ایک مخصوص کام انجام دیتے ہیں، جیسے سینسر ڈیٹا پڑھنا، موٹر کو کنٹرول کرنا، یا تصویر پر کارروائی کرنا۔ نوڈز **ٹاپکس**، **سروسز**، اور **ایکشنز** کا استعمال کرتے ہوئے ایک دوسرے سے رابطہ کرتے ہیں۔

اس باب میں، ہم سب سے عام مواصلاتی پیٹرن پر توجہ مرکوز کریں گے: **ٹاپکس**، **پبلشرز** اور **سبسکرائبرز** کا استعمال کرتے ہوئے۔

## پبلشرز: ایک ٹاپک پر ڈیٹا بھیجنا

ایک پبلشر نوڈ ایک مخصوص ٹاپک پر پیغامات بھیجنے (پبلش کرنے) کا ذمہ دار ہے۔ تصور کریں کہ ایک درجہ حرارت سینسر "/temperature" ٹاپک پر درجہ حرارت کی ریڈنگز پبلش کر رہا ہے۔

آئیے اپنے `my_robot_pkg` کے اندر ایک سادہ پائتھون پبلشر نوڈ بناتے ہیں۔

1.  **اپنے پائتھون اسکرپٹس کے لیے `my_robot_pkg/my_robot_pkg/` ڈائریکٹری بنائیں:**

    ```bash
    mkdir -p ~/ros2_ws/src/my_robot_pkg/my_robot_pkg
    ```

    یہ نیسٹڈ ڈائرکٹری ڈھانچہ پیکیجز کے لیے ایک پائتھون کنونشن ہے۔

2.  **`my_robot_pkg/my_robot_pkg/simple_publisher.py` بنائیں:**

    ```python
    # ~/ros2_ws/src/my_robot_pkg/my_robot_pkg/simple_publisher.py

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class SimplePublisher(Node):

        def __init__(self):
            super().__init__('simple_publisher')
            self.publisher_ = self.create_publisher(String, 'chatter', 10)
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.i = 0

        def timer_callback(self):
            msg = String()
            msg.data = f'Hello ROS2! Count: {self.i}'
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
            self.i += 1


    def main(args=None):
        rclpy.init(args=args)

        simple_publisher = SimplePublisher()

        rclpy.spin(simple_publisher)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        simple_publisher.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()
    ```

3.  **نوڈ کو قابل عمل بنانے کے لیے `setup.py` کو اپ ڈیٹ کریں:**

    `~/ros2_ws/src/my_robot_pkg/setup.py` میں ترمیم کریں اور `setup()` فنکشن کی `entry_points` ڈکشنری کے اندر درج ذیل شامل کریں۔ اگر `entry_points` موجود نہیں ہے تو اسے بنائیں۔

    ```python
    # ... inside setup()
    entry_points={
        'console_scripts': [
            'simple_publisher = my_robot_pkg.simple_publisher:main',
        ],
    },
    # ... rest of setup()
    ```

4.  **اپنا پیکیج بنائیں:**

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_robot_pkg
    ```

5.  **اپنا ورک اسپیس سورس کریں:**

    ```bash
    source install/setup.bash
    ```

6.  **اپنا پبلشر نوڈ چلائیں:**

    ایک نیا ٹرمینل کھولیں (اور اس میں اپنا ورک اسپیس سورس کریں):

    ```bash
    ros2 run my_robot_pkg simple_publisher
    ```

    آپ کو اپنے ٹرمینل میں پیغامات پبلش ہوتے ہوئے نظر آئیں گے۔

## سبسکرائبرز: ایک ٹاپک سے ڈیٹا وصول کرنا

ایک سبسکرائبر نوڈ ایک مخصوص ٹاپک کو سننے (سبسکرائب کرنے) اور اس پر پبلش ہونے والے پیغامات کو وصول کرنے کا ذمہ دار ہے۔ ہماری مثال کی پیروی کرتے ہوئے، ایک اور نوڈ موجودہ درجہ حرارت کو ظاہر کرنے کے لیے "/temperature" ٹاپک کو سبسکرائب کر سکتا ہے۔

1.  **`my_robot_pkg/my_robot_pkg/simple_subscriber.py` بنائیں:**

    ```python
    # ~/ros2_ws/src/my_robot_pkg/my_robot_pkg/simple_subscriber.py

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class SimpleSubscriber(Node):

        def __init__(self):
            super().__init__('simple_subscriber')
            self.subscription = self.create_subscription(
                String,
                'chatter',
                self.listener_callback,
                10)
            self.subscription  # prevent unused variable warning

        def listener_callback(self, msg):
            self.get_logger().info(f'I heard: "{msg.data}"')


    def main(args=None):
        rclpy.init(args=args)

        simple_subscriber = SimpleSubscriber()

        rclpy.spin(simple_subscriber)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        simple_subscriber.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()
    ```

2.  **نئے سبسکرائبر نوڈ کے لیے `setup.py` کو اپ ڈیٹ کریں:**

    `~/ros2_ws/src/my_robot_pkg/setup.py` میں `console_scripts` لسٹ میں نیا `simple_subscriber` انٹری پوائنٹ شامل کریں:

    ```python
    # ... inside setup()
    entry_points={
        'console_scripts': [
            'simple_publisher = my_robot_pkg.simple_publisher:main',
            'simple_subscriber = my_robot_pkg.simple_subscriber:main', # Add this line
        ],
    },
    # ... rest of setup()
    ```

3.  **اپنا پیکیج دوبارہ بنائیں:**

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_robot_pkg
    ```

4.  **اپنا ورک اسپیس سورس کریں:**

    ```bash
    source install/setup.bash
    ```

5.  **اپنا سبسکرائبر نوڈ چلائیں:**

    ایک اور نیا ٹرمینل کھولیں (اور اس میں اپنا ورک اسپیس سورس کریں)۔ سبسکرائبر نوڈ کو چلائیں جبکہ پبلشر ابھی بھی پہلے ٹرمینل میں چل رہا ہے:

    ```bash
    ros2 run my_robot_pkg simple_subscriber
    ```

    اب آپ کو سبسکرائبر ٹرمینل میں `simple_publisher` کے ذریعے پبلش کیے گئے پیغامات موصول ہوتے اور پرنٹ ہوتے ہوئے نظر آئیں گے۔

## ROS2 ٹاپکس کا معائنہ کرنا

ROS2 آپ کے چلتے ہوئے سسٹم کا معائنہ کرنے کے لیے کمانڈ لائن ٹولز فراہم کرتا ہے:

*   **فعال ٹاپکس کی فہرست بنائیں:**

    ```bash
    ros2 topic list
    ```

*   **ٹاپک کی معلومات دکھائیں (قسم، پبلشرز، سبسکرائبرز):**

    ```bash
    ros2 topic info /chatter
    ```

*   **ایک ٹاپک پر پیغامات کو ایکو کریں (براہ راست ڈیٹا دیکھنے کے لیے):**

    ```bash
    ros2 topic echo /chatter
    ```

اس باب نے آپ کو اپنے پہلے ROS2 پبلشر اور سبسکرائبر نوڈز بنانے، اور ان کے درمیان بنیادی مواصلت قائم کرنے کا طریقہ بتایا ہے۔ اگلے باب میں، ہم مزید پیچیدہ ڈیٹا کے تبادلے کے لیے کسٹم پیغام کی اقسام کی تعریف کرنے کا طریقہ دریافت کریں گے۔

```mermaid
graph TD
    Publisher[پبلشر نوڈ] -- /chatter ٹاپک پر پیغامات پبلش کرتا ہے --> Topic[/chatter ٹاپک (std_msgs/msg/String)]
    Topic --> Subscriber[سبسکرائبر نوڈ]
```

### مشق: ROS2 پبلشر/سبسکرائبر کو لاگو کرنا اور معائنہ کرنا

**مقصد:** ROS2 پبلشر اور سبسکرائبر نوڈز کو لاگو کرکے اور ان کے مواصلت کا معائنہ کرنے کے لیے ROS2 کمانڈ لائن ٹولز کا استعمال کرکے ROS2 کے بارے میں اپنی سمجھ کو مضبوط کریں۔

**مراحل:**

1.  **پبلشر نوڈ بنائیں:**
    *   اپنی `~/ros2_ws/src/my_robot_pkg/my_robot_pkg/` ڈائریکٹری میں جائیں۔
    *   `custom_publisher.py` نامی ایک پائتھون فائل بنائیں۔
    *   ایک ROS2 پبلشر نوڈ کو لاگو کریں جو `/my_custom_topic` نامی ٹاپک پر 1 Hz کی شرح سے `std_msgs.msg.String` پیغامات پبلش کرتا ہے۔ پیغامات میں ایک سادہ کاؤنٹر شامل ہونا چاہیے۔

2.  **سبسکرائبر نوڈ بنائیں:**
    *   اسی ڈائریکٹری میں، `custom_subscriber.py` نامی ایک پائتھون فائل بنائیں۔
    *   ایک ROS2 سبسکرائبر نوڈ کو لاگو کریں جو `/my_custom_topic` کو سبسکرائب کرتا ہے اور موصول ہونے والے پیغامات کو کنسول پر پرنٹ کرتا ہے۔

3.  **`setup.py` کو اپ ڈیٹ کریں:**
    *   `~/ros2_ws/src/my_robot_pkg/setup.py` میں ترمیم کریں تاکہ `custom_publisher` اور `custom_subscriber` دونوں کے لیے انٹری پوائنٹس شامل ہوں۔

4.  **بنائین اور سورس کریں:**
    *   `~/ros2_ws` میں جائیں اور `colcon build --packages-select my_robot_pkg` کا استعمال کرتے ہوئے اپنا `my_robot_pkg` بنائیں۔
    *   اپنا ورک اسپیس سورس کریں: `source install/setup.bash`۔

5.  **چلائیں اور تصدیق کریں:**
    *   دو الگ الگ ٹرمینل کھولیں۔ پہلے ٹرمینل میں، اپنا پبلشر چلائیں: `ros2 run my_robot_pkg custom_publisher`۔
    *   دوسرے ٹرمینل میں، اپنا سبسکرائبر چلائیں: `ros2 run my_robot_pkg custom_subscriber`۔
    *   تصدیق کریں کہ سبسکرائبر پبلشر سے پیغامات وصول کر رہا ہے اور پرنٹ کر رہا ہے۔

6.  **ROS2 ٹولز کے ساتھ معائنہ کریں:**
    *   ایک تیسرا ٹرمینل کھولیں (اور اپنا ورک اسپیس سورس کریں)۔
    *   یہ دیکھنے کے لیے `ros2 topic list` کا استعمال کریں کہ کیا `/my_custom_topic` فعال ہے۔
    *   اس کی قسم، پبلشرز، اور سبسکرائبرز کا معائنہ کرنے کے لیے `ros2 topic info /my_custom_topic` کا استعمال کریں۔
    *   پبلش ہونے والے پیغامات کو براہ راست دیکھنے کے لیے `ros2 topic echo /my_custom_topic` کا استعمال کریں۔

**متوقع نتیجہ:**
آپ کو اپنے کسٹم پبلشر سے اپنے کسٹم سبسکرائبر تک پیغامات بہتے ہوئے نظر آئیں گے، اور آپ ROS2 کمانڈ لائن ٹولز کا استعمال کرتے ہوئے اس مواصلت کا معائنہ کر سکیں گے۔
