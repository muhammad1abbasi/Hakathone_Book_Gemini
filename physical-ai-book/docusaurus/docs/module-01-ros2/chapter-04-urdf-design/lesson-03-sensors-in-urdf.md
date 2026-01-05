---
id: lesson-03-sensors-in-urdf
title: "Lesson 3: Adding Sensors to a URDF"
sidebar_position: 3
description: Learn how to attach sensor blocks to a URDF to define the location and properties of cameras, LiDAR, and IMUs.
---

### Lesson Objective
To add a camera sensor and an IMU sensor to our humanoid URDF using the `<sensor>` tag.

### Prerequisites
- Completion of Lesson 2: "Describing Humanoid Joints".

### Concept Explanation
A URDF is not just for describing the moving parts of a robot; it's also for defining where and how the sensors are attached to the robot's body. While the basic `<link>` and `<joint>` tags don't have a place for sensor information, URDF supports extensions using custom tags. The most widely used format for this, especially for the Gazebo simulator, is the `<sensor>` tag.

A `<sensor>` element is typically placed inside the `<link>` to which it is physically attached. It has a `name` and a `type` (e.g., `camera`, `imu`, `ray` for LiDAR).

Inside the `<sensor>` tag, you define its properties:
- **`<camera>`**: You can specify the image `width` and `height`, `format`, and field of view (`hfov`).
- **`<imu>`**: You can specify noise characteristics.
- **`<pose>`**: A crucial tag that defines the sensor's position and orientation *relative to the link it's inside*. This is how you "mount" the camera on the robot's head.
- **`<plugin>`**: For simulation, a plugin is specified to actually generate the sensor data. For example, a camera sensor needs a plugin to simulate generating images.

This allows other ROS nodes and tools to look up the robot's description and find out what sensors it has and where they are.

### Real-World Analogy
Think of adding accessories to a smartphone.
- Your phone is a `<link>`.
- You buy a camera lens attachment. The attachment itself is the `<sensor>` block.
- You snap the lens attachment over your phone's existing camera. The position you place it in is the `<pose>` of the sensor.
- The software driver that your phone uses to recognize and use the new lens is the `<plugin>`.

### Hands-On Task
**Task**: Add a camera to the head of our simple humanoid URDF.

1.  **Open the File**: Open your `simple_humanoid.urdf` file.
2.  **Find the Head Link**: Locate the `<link name="head">` element.
3.  **Add the Sensor Block**: Inside the `<link name="head">` element, right after the `<visual>` block, add the `<sensor>` block from the example code below.
4.  **Analyze the Code**:
    - We've given the sensor a `name`, `head_camera_sensor`.
    - We've set its `type` to `camera`.
    - The `<pose>` tag places it slightly in front of the head's origin (`0.05` on the x-axis).
    - The `<camera>` tag defines the image properties.
    - *Note*: We are not adding a `<plugin>` yet, as that is specific to the Gazebo simulator, which we will cover in a later module. For now, we are just describing the sensor's existence.
5.  **Check the URDF**: Run `check_urdf simple_humanoid.urdf`. The checker will likely warn you that `<sensor>` is not a valid tag in the base URDF standard. This is normal! It's an extension, and tools like Gazebo and RViz know how to parse it.

### Python + ROS 2 Code Example
```xml
<!-- simple_humanoid.urdf with a camera sensor -->
<robot name="simple_humanoid">
  
  <!-- ... (torso, other links and joints) ... -->

  <!-- Head Link -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <material name="white">
        <color rgba="1.0 1.0 1.0 1.0"/>
      </material>
    </visual>

    <!-- NEW SENSOR BLOCK -->
    <sensor name="head_camera_sensor" type="camera">
      <pose>0.05 0 0.05 0 0 0</pose>
      <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
              <width>1280</width>
              <height>720</height>
              <format>R8G8B8</format>
          </image>
          <clip>
              <near>0.1</near>
              <far>100</far>
          </clip>
      </camera>
      <always_on>1</always_on>
      <update_rate>30</update_rate>
    </sensor>
    <!-- END NEW SENSOR BLOCK -->

  </link>

  <!-- ... (neck joint, etc.) ... -->

</robot>
```

### Common Mistakes & Debugging Tips
- **Mistake**: Placing the `<sensor>` tag outside of a `<link>`. A sensor must be physically attached to a part of the robot.
- **Mistake**: Forgetting the `<pose>` tag. Without it, the sensor's position defaults to the link's origin, which may not be what you want. The camera needs to be on the "front" of the head.
- **Tip**: The sensor definitions are mainly for simulation and high-level tools. The actual ROS 2 driver for the physical hardware is a separate node that might read this information for its configuration, but it's the driver node that actually publishes the sensor data on a topic. The URDF just describes *where* the sensor is supposed to be.

### Mini Assessment
1.  To add a sensor to a URDF, where should the `<sensor>` tag be placed?
    a) Inside the `<joint>` tag that the sensor is closest to.
    b) At the root of the `<robot>` tag.
    c) Inside the `<link>` tag that the sensor is physically attached to.
2.  What is the purpose of the `<pose>` tag within a `<sensor>` block?
    a) To define the sensor's position and orientation relative to its parent link.
    b) To set the color of the sensor.
    c) To define the sensor's update rate.
3.  If you want to add a LiDAR to your robot, what would you set the sensor `type` to?
    a) `camera`
    b) `imu`
    c) `ray`
4.  Why might the `check_urdf` tool give a warning about the `<sensor>` tag?
    a) Because the syntax is wrong.
    b) Because `<sensor>` is an extension to the core URDF standard, used by simulators like Gazebo.
    c) Because sensors are not allowed in URDF.
5.  Does the `<sensor>` tag in a URDF replace the need for a ROS 2 driver node for that sensor?
    a) Yes, the URDF automatically creates a driver.
    b) No, the URDF only describes the sensor's physical properties. A separate driver node is still needed to publish its data.
    c) Only for cameras, not for IMUs.