import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import multiprocessing
from multiprocessing import Manager, Lock
from flask import Flask, send_from_directory, render_template
import asyncio
import websockets
import functools






class WebSocketROS2Bridge(Node):
    def __init__(self):
        super().__init__('websocket_ros2_bridge')
        self.clients1 = set()
        self.loop = asyncio.get_event_loop()
        self.websocket_server_task = self.loop.create_task(self.start_websocket_server())
 

       
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def map_callback(self, msg):
        # Prepare map data to be sent through WebSocket
        map_data = {
            'info': {
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'origin': {
                    'position': {
                        'x': msg.info.origin.position.x,
                        'y': msg.info.origin.position.y,
                        'z': msg.info.origin.position.z,
                    },
                    'orientation': {
                        'x': msg.info.origin.orientation.x,
                        'y': msg.info.origin.orientation.y,
                        'z': msg.info.origin.orientation.z,
                        'w': msg.info.origin.orientation.w,
                    }
                }
            },
            'data': list(msg.data)  # Convert array data to a list
        }
        print(self.clients1)
        asyncio.run(self.broadcast(map_data))

    async def broadcast(self, message):
        print("Trying to send")
        if self.clients1:  # Only if there are connected clients
            print("send!!!")
            #await asyncio.wait([client.send(str(message)) for client in self.client_queue])
    
    async def start_websocket_server(self):
        async def handler(websocket, path):
            print("New WebSocket connection")
            self.clients1.add(websocket)
            try:
                async for message in websocket:
                    print(f"Received message: {message}")
                    await websocket.send(message)  # Echo message back
            except websockets.ConnectionClosed:
                print("Connection closed")
            finally:
                self.clients1.remove(websocket)

        server = await websockets.serve(handler, '0.0.0.0', 8888)  # Bind to all interfaces
        await server.wait_closed()






def start_ros2_node():
    rclpy.init()
    node = WebSocketROS2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("ROS 2 node interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()




if __name__ == '__main__':
    rclpy.init()
    node = WebSocketROS2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("ROS 2 node interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()    

            
