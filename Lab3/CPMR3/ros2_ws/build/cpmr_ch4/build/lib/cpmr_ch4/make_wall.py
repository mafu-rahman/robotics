import rclpy
from gazebo_msgs.srv import SpawnEntity, DeleteEntity

def make_wall(node, id, x, y, z, length, width, height):
    WALL_MODEL = """
    <sdf version="1.6"> 				
        <world name="default">                         
            <model name="{id}"> 			
                <static>true</static> 			
                <link name="wall_link">                        
                    <collision name="wall_collision">			
                        <pose>0 0 {z} 0 0 0</pose>    		
                        <geometry>				
                            <box>                        
                                <size>{length} {width} {height}</size>             
                            </box>   			
                        </geometry>				
                    </collision>				
                    <visual name="wall_visual">			
                        <pose>0 0 {z} 0 0 0</pose>    		
                        <geometry>				
                            <box>                        
                                <size>{length} {width} {height}</size>               
                            </box>                          
                        </geometry>				
                    </visual>				
                </link>                                    
            </model>					
        </world>                                       
    </sdf>"""

    client = node.create_client(SpawnEntity, "/spawn_entity")
    node.get_logger().info(f"Connecting to /spawn_entity service for {id}...")
    client.wait_for_service()
    node.get_logger().info("...connected")
    
    request = SpawnEntity.Request()
    request.name = id
    request.initial_pose.position.x = float(x)
    request.initial_pose.position.y = float(y)
    request.initial_pose.position.z = float(0)
    request.xml = WALL_MODEL.format(id=id, length=length, width=width, height=height, z=height/2)
    
    node.get_logger().info(f"Spawning wall {id} at ({x}, {y})...")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    if not future.result().success:
        node.get_logger().info(f"Failed to spawn wall {id}: {future.result()}")
    else:
        node.get_logger().info(f"Wall {id} spawned successfully.")


def remove_wall(node, id):
    client = node.create_client(DeleteEntity, "/delete_entity")
    node.get_logger().info(f"Connecting to /delete_entity service to remove {id}...")
    client.wait_for_service()
    node.get_logger().info("...connected")
    
    request = DeleteEntity.Request()
    request.name = id
    
    node.get_logger().info(f"Removing wall {id}...")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    if not future.result().success:
        node.get_logger().info(f"Failed to remove wall {id}: {future.result()}")
    else:
        node.get_logger().info(f"Wall {id} removed successfully.")


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('bordered_environment')

    # Define dimensions of the walls and the 5x5m border layout
    wall_length = 10
    wall_width = 0.1
    wall_height = 1.0

    # Spawn 4 walls for the border
    make_wall(node, 'front_wall', 0, 5, 0, wall_length, wall_width, wall_height)  # Front
    make_wall(node, 'back_wall', 0, -5, 0, wall_length, wall_width, wall_height)  # Back
    make_wall(node, 'left_wall', -5, 0, 0, wall_width, wall_length, wall_height)  # Left
    make_wall(node, 'right_wall', 5, 0, 0, wall_width, wall_length, wall_height)  # Right

    # Clean up
    node.get_logger().info("All walls spawned.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
