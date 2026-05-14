"""ROS2 node that loads objects.yaml into the MoveIt planning scene."""
import rclpy
from rclpy.node import Node
from meca500_demos.planningscene import PlanningSceneClass


class SceneNode(Node):

    def __init__(self):
        super().__init__('scene_node')
        self.scene = PlanningSceneClass(self)
        # Delay publish so move_group has time to start
        self.timer = self.create_timer(2.0, self.publish_scene)

    def publish_scene(self):
        self.scene.load_scene('objects.yaml')
        self.get_logger().info('Planning scene published.')
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = SceneNode()
    rclpy.spin(node)
    rclpy.shutdown()
