import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import asyncio
import websockets
import threading
import json
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PointStamped, PoseStamped
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs
import math
import numpy as np
from builtin_interfaces.msg import Time as BuiltinTime
from rosidl_runtime_py import message_to_ordereddict
import base64
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.task import Future

class WebSocketROS2Bridge(Node):
    def __init__(self, clients):
        super().__init__('websocket_ros2_bridge')
        self.clients1 = clients
        self.websocket_uri = "ws://0.0.0.0:8888"
        self.websocket_server = None
        # Start the WebSocket server in a separate thread
        websocket_thread = threading.Thread(target=self.start_websocket_server)
        websocket_thread.start()

        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)
        self.subscription_map  # Prevent unused variable warning

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.subscription_scan

        self.navpose_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')


        #self.publisher = self.create_publisher(PointCloud2, '/scan_pointcloud', 10)
        #self.pose_publisher = self.create_publisher(PoseStamped, '/robot_pose_in_map', 10)
    
    def send_goal(self, pose: PoseStamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.navpose_action_client.wait_for_server()

        self._send_goal_future = self.navpose_action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: Future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        

    def map_callback(self, msg):
        # Prepare map data to be sent through WebSocket
        map_data_json = self.ros2_msg_to_json('map',msg)
        asyncio.run(self.send_data_to_clients(map_data_json))

    def scan_callback(self, scan_msg):
        # Convert LaserScan to PointCloud2
        cloud_msg = self.laserscan_to_pointcloud2(scan_msg)
        current_time = rclpy.time.Time()

        # Convert rclpy.time.Time to builtin_interfaces.msg.Time
        time_msg = BuiltinTime(sec=current_time.seconds_nanoseconds()[0], nanosec=current_time.seconds_nanoseconds()[1])

        cloud_msg.header.stamp = time_msg #scan_msg.header.stamp  # Ensure correct timestamp
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",  # target frame
                cloud_msg.header.frame_id,  # source frame (laser frame)
                rclpy.time.Time.from_msg(scan_msg.header.stamp),  # time at which the transform is needed
            )
            # Robot pose in map frame
            transform_p = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())        
            # Laser scan in map frame as pointcloud2
            transformed_cloud = tf2_sensor_msgs.tf2_sensor_msgs.do_transform_cloud(cloud_msg, transform)
            

            if transformed_cloud:
                # Publish Transformed PointCloud2
                #self.publisher.publish(transformed_cloud)
        
                string = ''.join(chr(i) for i in transformed_cloud.data)
                # Convert each integer to its byte representation and concatenate
                byte_array = bytes(transformed_cloud.data)

                # Encode the byte array to Base64
                base64_string = base64.b64encode(byte_array).decode('utf-8')
      
                transformed_cloud_json = self.ros2_msg_to_json('scan_pointcloud',transformed_cloud)
                dictionary_tc = json.loads(transformed_cloud_json)
                dictionary_tc['data']['data'] = base64_string
                #print(dictionary_tc['data']['data']) 
    
                asyncio.run(self.send_data_to_clients(json.dumps(dictionary_tc)))

            
                        # Create a PoseStamped message to publish
            pose_in_map = PoseStamped()
            pose_in_map.header.stamp = self.get_clock().now().to_msg()
            pose_in_map.header.frame_id = 'map'
            # Construct the Pose message from the transform
            pose_in_map.pose.position.x = transform_p.transform.translation.x
            pose_in_map.pose.position.y = transform_p.transform.translation.y
            pose_in_map.pose.position.z = transform_p.transform.translation.z
            
            pose_in_map.pose.orientation.x = transform_p.transform.rotation.x
            pose_in_map.pose.orientation.y = transform_p.transform.rotation.y
            pose_in_map.pose.orientation.z = transform_p.transform.rotation.z
            pose_in_map.pose.orientation.w = transform_p.transform.rotation.w
  
            # Publish the pose
            #self.pose_publisher.publish(pose_in_map)
            pose_in_map_json = self.ros2_msg_to_json('robot_pose_in_map',pose_in_map)
            asyncio.run(self.send_data_to_clients(pose_in_map_json))
       
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not transform pointcloud: {e}")
            return None


    def laserscan_to_pointcloud2(self, scan_msg):
        points = []
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            if scan_msg.range_min < r < scan_msg.range_max:  # Ignore invalid ranges
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0  # Assuming a 2D scan, z is 0
                points.append([x, y, z])
            angle += scan_msg.angle_increment

        header = scan_msg.header
        cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
        return cloud_msg
    #-----------  Handle incomming message ------------   
    def convert_json_pose_to_poasestamp(self, p):
        pose_s = PoseStamped()
        pose_s.header.stamp = self.get_clock().now().to_msg()
        pose_s.header.frame_id = 'map'
        # Construct the Pose message from the transform
        pose_s.pose.position.x = float(p['position']['x'])
        pose_s.pose.position.y = float(p['position']['y'])
        pose_s.pose.position.z = float(p['position']['z'])
        
        pose_s.pose.orientation.x = float(p['orientation']['x'])
        pose_s.pose.orientation.y = float(p['orientation']['y'])
        pose_s.pose.orientation.z = float(p['orientation']['z'])
        pose_s.pose.orientation.w = float(p['orientation']['w'])
        
        return pose_s

  
    async def websocket_handler(self, websocket, path):
        self.clients1.add(websocket)
        #print("Websocket connected")
        #print(websocket)
        try:
            async for message in websocket:
                # Process incoming messages here
                #print(f"Received message: {message}")
                json_dada = json.loads(message)
                if(json_dada['type'] == "action"):
                    if(json_dada['name'] == "navtopose"):
                        print(json_dada['data'])
                        pp = self.convert_json_pose_to_poasestamp(json_dada['data'])
                        print(pp)
                        self.send_goal(pp)
                elif(json_dada['type'] == "topic"):
                    if(json_dada['name'] == "dummytopic"):
                        pass
                elif(json_dada['type'] == "service"):
                    if(json_dada['name'] == "dummyservice"):
                        pass
        
                
        except websockets.ConnectionClosed:
            pass
        finally:
            self.clients1.remove(websocket)

    def start_websocket_server(self):
        print("Start loop")
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        start_server = websockets.serve(self.websocket_handler, "0.0.0.0", 8888)
        self.loop.run_until_complete(start_server)
        self.loop.run_forever()

    async def send_data_to_clients(self, data):
        try:
            #print("Sending data to clients")  # Ensure this line is printed
            if not self.clients1:
                #print("No clients connected")
                return

            for websocket in self.clients1:
                #print(f"Sending data to client: {websocket}")
                await websocket.send(data)
                #print(f"Data sent to client: {websocket}")
        except Exception as e:
            print(f"Error in send_data_to_clients: {e}")


    def ros2_msg_to_json(self, topic_name, msg):
        # Convert the message to an ordered dictionary
        msg_dict = message_to_ordereddict(msg)

        # Create a dictionary with the topic name and message data
        msg_with_topic = {
            "topic": topic_name,
            "data": msg_dict
        }

        # Convert the dictionary to a JSON string
        return json.dumps(msg_with_topic)



def start_ros2_node(clients):
    rclpy.init()
    node = WebSocketROS2Bridge(clients)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    clients1 = set()  # Use a set to store connected WebSocket clients
    # Start the ROS 2 node in another thread or process
    ros2_thread = threading.Thread(target=start_ros2_node, args=(clients1,))
    ros2_thread.start()