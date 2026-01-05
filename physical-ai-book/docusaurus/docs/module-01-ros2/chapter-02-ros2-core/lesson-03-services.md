---
id: lesson-03-services
title: "Lesson 3: Services - The Request/Response Pattern"
sidebar_position: 3
description: Learn to use ROS 2 Services for synchronous request/response communication.
---

### Lesson Objective
To create two nodes: a **service server** that provides a piece of functionality, and a **service client** that requests that functionality and waits for a response.

### Prerequisites
- Completion of Lesson 2: "Topics".

### Concept Explanation
While Topics are great for continuous streams of data, they aren't ideal for a simple request/response interaction. For that, ROS 2 provides **Services**.

A Service is defined by a pair of messages: a **request** and a **response**.
- A **Service Server** node advertises a service by name. It waits for a request, performs a computation, and sends back a response.
- A **Service Client** node can call the service by name. It sends a request and *waits* (blocks) until it receives the response from the server.

This is a **synchronous**, one-to-one communication pattern. It's perfect for tasks that have a clear beginning and end, like "get the robot's current position," "open the gripper," or "calculate the distance between two points."

### Real-World Analogy
A **Service** is like ordering a coffee at a cafe.
- The **Service Server** is the barista. They are waiting for someone to place an order.
- The **Service Name** is the act of ordering (e.g., "ordering_a_coffee").
- The **Request** is your specific order ("I'd like a large latte").
- The **Response** is the coffee the barista gives you.
- The **Service Client** is you, the customer. You make a request and then you wait at the counter until you get your coffee. You can't do anything else until the transaction is complete. This "waiting" is the synchronous nature of a service.

### Hands-On Task
**Task**: Create a service that adds two integers.

1.  **Define the Service**: ROS 2 has a built-in service definition for this exact purpose called `example_interfaces/srv/AddTwoInts`. The request contains two integers (`a` and `b`), and the response contains one integer (`sum`).
2.  **Create Server File**: In `ros2_ws/src`, create `simple_service_server.py` and add the server code below.
3.  **Create Client File**: In the same directory, create `simple_service_client.py` and add the client code.
4.  **Run the Nodes**:
    - Open a terminal, navigate to `ros2_ws/src`, and run the server:
      ```bash
      source /opt/ros/humble/setup.bash
      python3 simple_service_server.py
      ```
    - Open a **second terminal**, navigate to `ros2_ws/src`, and run the client. This time, we'll pass numbers as command-line arguments.
      ```bash
      source /opt/ros/humble/setup.bash
      python3 simple_service_client.py 5 10
      ```
5.  **Observe**: In the client's terminal, you will see it send the request and then print the response (the sum). The server's terminal will show that it received a request and sent a response.
6.  **Inspect (Optional)**: In a third terminal, list the available services: `ros2 service list`. You should see `/add_two_ints`.

### Python + ROS 2 Code Example

#### Service Server
```python
# simple_service_server.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__('simple_service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service server is ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        self.get_logger().info(f'Sending back response: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    simple_service_server = SimpleServiceServer()
    rclpy.spin(simple_service_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Service Client
```python
# simple_service_client.py
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class SimpleServiceClient(Node):
    def __init__(self, a, b):
        super().__init__('simple_service_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.req = AddTwoInts.Request()
        self.req.a = a
        self.req.b = b

    def send_request(self):
        self.future = self.client.call_async(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print("Usage: python3 simple_service_client.py <int1> <int2>")
        return

    a = int(sys.argv[1])
    b = int(sys.argv[2])

    simple_service_client = SimpleServiceClient(a, b)
    future = simple_service_client.send_request()

    # Spin until the future is complete
    rclpy.spin_until_future_complete(simple_service_client, future)

    try:
        response = future.result()
        simple_service_client.get_logger().info(
            f'Result of add_two_ints: for {a} + {b} = {response.sum}')
    except Exception as e:
        simple_service_client.get_logger().error(f'Service call failed {e!r}')

    simple_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Common Mistakes & Debugging Tips
- **Mistake**: The client calls the service before the server is ready. The line `self.client.wait_for_service()` is crucial to prevent this.
- **Mistake**: Trying to use a service for a long-running task. A service is synchronous; the client will be blocked, waiting for the response. If the server takes 10 seconds to respond, the client is stuck for 10 seconds. This is bad for complex systems. For long tasks, use Actions.
- **Tip**: Use `ros2 service list` to see available services and `ros2 service info /service_name` to see the service type. You can even call a service from the command line: `ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 10}" `.

### Mini Assessment
1.  A Service is best for which communication pattern?
    a) One-to-many broadcast (like a radio).
    b) One-to-one request/response (like ordering a coffee).
    c) A long-running task with continuous feedback.
2.  What does "synchronous" mean in the context of a service?
    a) The client sends a request and can immediately do other work.
    b) The client sends a request and must wait for the response before continuing.
    c) The server and client must have their clocks synchronized.
3.  In the code, what is the purpose of `self.client.wait_for_service()`?
    a) To make the server wait for the client.
    b) To pause the program for one second.
    c) To ensure the client doesn't send a request before the server is ready.
4.  What would happen if you tried to run the client script without the server running?
    a) The client would crash immediately.
    b) The client would print "Service not available, waiting again..." indefinitely.
    c) The client would start its own server.
5.  Why is a service a poor choice for a task like "navigate to the kitchen"?
    a) Navigation is too simple for a service.
    b) It's a long-running task, and the client would be blocked for the entire duration.
    c) Services can't handle numbers.