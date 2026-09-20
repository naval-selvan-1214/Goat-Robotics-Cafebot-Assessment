#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
from own_interfaces.msg import OrderEvent

COORDS = {  "home":    (0.0, 0.0, 0.0),   
            "kitchen": (0.4, 1.25, 1.57),       ## coords wrt starting pt of robot 
            "table1":  (-1.3, 2.0, 3.14),
            "table2":  (-1.3, 1.0, 3.14),
            "table3":  (-1.3, 0.0, 3.14)  }

timeOut_value = 30.0   # in seconds 
 
def create_pose_stamped(navigator, coords):
    x, y, yaw = coords
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

class OrderManager:
    def __init__(self):
        self.nav = BasicNavigator()
        initial_pose = create_pose_stamped(self.nav, COORDS['home'])
        self.nav.setInitialPose(initial_pose)
        self.inbox = []
        self.nav.create_subscription(OrderEvent, 'order_events', self.event_callback, 10)
        self.nav.waitUntilNav2Active()
        self.orders = {}

    def event_callback(self, msg: OrderEvent):
        self.inbox.append((msg.event_type, msg.table_id))
        self.nav.get_logger().info(f"received: {msg.event_type} for {msg.table_id}")
        # print(self.inbox)
        # print(self.orders)

    # def move_robot(self, destination_name):
    #     pose = create_pose_stamped(self.nav, COORDS[destination_name])
    #     self.nav.get_logger().info(f"Navigating to {destination_name.capitalize()}")
    #     self.nav.goToPose(pose)

    #     cancel_flag = False
    #     last_log = 0.0
    #     while not self.nav.isTaskComplete():
    #         if (time.time()-last_log) > 1.0:
    #             self.nav.get_logger().info(f"On the way to {destination_name.capitalize()}....")
    #             rclpy.spin_once(self.nav, timeout_sec=0.0)
    #             for i, (event_type, ev_table) in enumerate(self.inbox):
    #                 if ev_table == destination_name and event_type == 'cancel':
    #                     self.nav.get_logger().info(f"Received CANCEL for {destination_name.capitalize()}, {self.nav.getResult()}")
    #                     self.nav.cancelTask()
    #                     self.inbox.pop(i)
    #                     cancel_flag = True
    #                     break
    #             if cancel_flag:
    #                 break                   
                        
    #             last_log = time.time()

    #     if cancel_flag:
    #         if destination_name in self.orders.keys():
    #             self.orders[destination_name]['status'] = 'cancelled'
    #     result = self.nav.getResult()
    #     self.nav.get_logger().info(f"Arrived at {destination_name.capitalize()}: {result}")
    #     return result

    def move_robot(self, destination_name):
        pose = create_pose_stamped(self.nav, COORDS[destination_name])
        self.nav.get_logger().info(f"Navigating to {destination_name.capitalize()}")
        self.nav.goToPose(pose)

        last_log = 0.0
        while not self.nav.isTaskComplete():
            if (time.time()-last_log) > 1.0:
                self.nav.get_logger().info(f"On the way to {destination_name.capitalize()}....")
                last_log = time.time()

        result = self.nav.getResult()
        self.nav.get_logger().info(f"Arrived at {destination_name.capitalize()}: {result}")
        return result

    
    def wait_for_event(self, table_id, expected_types, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self.nav, timeout_sec=0.1)
            for i, (event_type, ev_table) in enumerate(self.inbox):
                if ev_table == table_id and event_type in expected_types:
                    return self.inbox.pop(i)[0]
        return 'timeout'

    
    def wait_for_order_trigger(self, timeout=60.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self.nav, timeout_sec=0.1)
            for i, (event_type, ev_table) in enumerate(self.inbox):
                if event_type == 'order': 
                    return True
        return False


    def check_table_status(self, table_id):
        rclpy.spin_once(self.nav, timeout_sec=0.0)
        for i, (event_type, ev_table) in enumerate(self.inbox):
            if ev_table == table_id:
                if event_type == 'cancel':
                    self.inbox.pop(i)
                    return 'cancelled'
        return None


    def handle_delivery_round(self, table_ids, require_confirmation=True):
        self.nav.get_logger().info("Waiting for an order to be received...")
        bool_value = self.wait_for_order_trigger(timeout=85.0)
        if not bool_value:
            self.nav.get_logger().info("NO order received in time, so STAYING HOME")
            return self.orders

        self.move_robot('kitchen')
        if require_confirmation:
            kitchen_event = self.wait_for_event('kitchen', ['confirm', 'cancel'], timeOut_value)
        else:
            kitchen_event = 'confirm'  

        if kitchen_event != 'confirm':
            self.nav.get_logger().info(f"kitchen stage ended in '{kitchen_event}', returning home directly")
            self.move_robot('home')
            return self.orders

        for id in table_ids:
            self.orders[id]= {'status':'No Order Passed'}
            for j in range(len(self.inbox)):
                if self.inbox[j][1]==id:
                    self.orders[id] = {'status':'confirmed'}
                    continue

        any_failed = False
        for t in table_ids:
            returned_info = self.check_table_status(t)
            if returned_info == 'cancelled':
                any_failed = True
                self.nav.get_logger().info(f"{t} got CANCELLED, so skipping it !")
                self.orders[t]['status'] = returned_info
                continue
            if self.orders[t]['status'] == 'No Order Passed':
                continue

            self.move_robot(t)
            if require_confirmation:
                event = self.wait_for_event(t, ['confirm', 'cancel'], timeOut_value)
            else:
                event = 'confirm'
            if event == 'confirm':
                self.orders[t]['status'] = 'delivered'
            else:
                self.orders[t]['status'] = event  
                any_failed = True
                self.nav.get_logger().info(f"{t} ended in '{event.capitalize()}'")
        if any_failed:
            self.move_robot('kitchen') 
        self.move_robot('home')

        return self.orders

def main():
    rclpy.init()
    manager = OrderManager()
    final_states = manager.handle_delivery_round(['table1', 'table2', 'table3'])
    manager.nav.get_logger().info(f"Final Order Status of Tables : \n{final_states}")
    rclpy.shutdown()

if __name__ == '__main__':
    main()