import os
import tkinter
from enum import Enum
from threading import Thread

import customtkinter as ctk
import rclpy
import yaml
from ament_index_python import get_package_share_directory
from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action._navigate_to_pose import NavigateToPose_Feedback
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node
from robot_localization.srv import FromLL

from rovesugv_navsim import gps_utils


LEFT_BUTTON = "<ButtonPress-1>"
LINEAR_BASE_SPEED = 0.5
ANGULAR_BASE_SPEED = 0.5
ROUTE_AUTO_TO_ROBO_LAB = "Auto to Robo Lab"
ROUTE_ROBO_TO_AUTO_LAB = "Robo to Auto Lab"


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class Driving(ctk.CTkFrame):
    """Frame for manual driving of the rover."""

    def __init__(self, master: ctk.CTk):
        super().__init__(master)
        self.master = master

        self.header = ctk.CTkLabel(self, text="Manual Driving", font=master.helv16bold)
        self.header.grid(row=0, column=1)

        self.fwd_left_button = ctk.CTkButton(self, width=80, height=40, 
                                             text="Forward\nLeft")
        self.fwd_left_button.bind(LEFT_BUTTON, lambda e: self.drive(e, 1, 1))
        self.fwd_left_button.grid(row=1, column=0, padx=5, pady=5)

        self.fwd_button = ctk.CTkButton(self, width=80, height=40, 
                                        text="Forward")
        self.fwd_button.bind(LEFT_BUTTON, lambda e: self.drive(e, 1, 0))
        self.fwd_button.grid(row=1, column=1, padx=5, pady=5)

        self.fwd_right_button = ctk.CTkButton(self, width=80, height=40, 
                                              text="Forward\nRight")
        self.fwd_right_button.bind(LEFT_BUTTON, lambda e: self.drive(e, 1, -1))
        self.fwd_right_button.grid(row=1, column=2, padx=5, pady=5)

        self.rot_left_button = ctk.CTkButton(self, width=80, height=40, 
                                             text="Rotate\nLeft")
        self.rot_left_button.bind(LEFT_BUTTON, lambda e: self.drive(e, 0, 1))
        self.rot_left_button.grid(row=2, column=0, padx=5, pady=5)

        self.stop_button = ctk.CTkButton(
            self, 
            width=80, 
            height=40, 
            fg_color="#ff0000",
            hover_color="#cc0000",
            text="Stop"
        )
        self.stop_button.bind(LEFT_BUTTON, lambda e: self.drive(e, 0, 0))
        self.stop_button.grid(row=2, column=1, padx=5, pady=5)

        self.rot_right_button = ctk.CTkButton(self, width=80, height=40, 
                                              text="Rotate\nRight")
        self.rot_right_button.bind(LEFT_BUTTON, lambda e: self.drive(e, 0, -1))
        self.rot_right_button.grid(row=2, column=2, padx=5, pady=5)

        self.bkw_left_button = ctk.CTkButton(self, width=80, height=40, 
                                             text="Backward\nLeft")
        self.bkw_left_button.bind(LEFT_BUTTON, lambda e: self.drive(e, -1, -1))
        self.bkw_left_button.grid(row=3, column=0, padx=5, pady=5)

        self.bkw_button = ctk.CTkButton(self, width=80, height=40, 
                                        text="Backward")
        self.bkw_button.bind(LEFT_BUTTON, lambda e: self.drive(e, -1, 0))
        self.bkw_button.grid(row=3, column=1, padx=5, pady=5)

        self.bkw_right_button = ctk.CTkButton(self, width=80, height=40, 
                                              text="Backward\nRight")
        self.bkw_right_button.bind(LEFT_BUTTON, lambda e: self.drive(e, -1, 1))
        self.bkw_right_button.grid(row=3, column=2, padx=5, pady=5)


    def drive(self, event: tkinter.Event, linear_x: int, angular_z: int):
        """Drive the rover."""
        msg = Twist()
        msg.linear.x = linear_x * LINEAR_BASE_SPEED
        msg.angular.z = angular_z * ANGULAR_BASE_SPEED
        self.master.cmd_vel_pub.publish(msg)


class Navigation(ctk.CTkFrame):
    """Frame for giving navigation tasks to the rover."""

    def __init__(self, master: ctk.CTk):
        super().__init__(master)

        self.cancel_nav_task = False
        self.route_dict = self.get_routes()
        self.waypoint_count = 0

        self.header = ctk.CTkLabel(self, text="Navigation", font=master.helv16bold)
        self.header.grid(row=0, column=0, columnspan=2)

        self.route_label = ctk.CTkLabel(self, text="Route selection")
        self.route_label.grid(row=1, column=0)

        self.route_menu = ctk.CTkOptionMenu(
            self, 
            values=[
                ROUTE_ROBO_TO_AUTO_LAB,
                ROUTE_AUTO_TO_ROBO_LAB,
            ], 
            command=None,
        )
        self.route_menu.grid(row=1, column=1)

        self.start_button = ctk.CTkButton(
            self, 
            height=40, 
            fg_color="#009933",
            hover_color="#006600",
            text="Start\nNavigation", 
            command=self.start_navigation_task,
        )
        self.start_button.grid(row=2, column=0, padx=5, pady=5)

        self.stop_button = ctk.CTkButton(
            self, 
            height=40, 
            fg_color="#ff0000",
            hover_color="#cc0000",
            text="Stop\nNavigation", 
            command=self.cancel_navigation_task,
        )
        self.stop_button.grid(row=2, column=1, padx=5, pady=5)

        self.nav_status_label = ctk.CTkLabel(self, text="Status")
        self.nav_status_label.grid(row=3, column=0)
        self.nav_status_value = ctk.CTkLabel(self, text="INACTIVE")
        self.nav_status_value.grid(row=3, column=1, padx=30)

        self.wp_number_label = ctk.CTkLabel(self, text="Waypoint Number")
        self.wp_number_label.grid(row=4, column=0)
        self.wp_number_value = ctk.CTkLabel(self, text=f"0/{self.waypoint_count}")
        self.wp_number_value.grid(row=4, column=1, padx=30)

        self.nav_time_label = ctk.CTkLabel(self, text="Navigation time (s)")
        self.nav_time_label.grid(row=5, column=0)
        self.nav_time_value = ctk.CTkLabel(self, text="0")
        self.nav_time_value.grid(row=5, column=1, padx=30)

        self.time_rem_label = ctk.CTkLabel(self, text="Time remaining (s)")
        self.time_rem_label.grid(row=6, column=0)
        self.time_rem_value = ctk.CTkLabel(self, text="0")
        self.time_rem_value.grid(row=6, column=1, padx=30)

        self.dist_rem_label = ctk.CTkLabel(self, text="Distance remaining (m)")
        self.dist_rem_label.grid(row=7, column=0)
        self.dist_rem_value = ctk.CTkLabel(self, text="0")
        self.dist_rem_value.grid(row=7, column=1, padx=30)

        self.num_recs_label = ctk.CTkLabel(self, text="Recoveries")
        self.num_recs_label.grid(row=8, column=0)
        self.num_recs_value = ctk.CTkLabel(self, text="0")
        self.num_recs_value.grid(row=8, column=1, padx=30)

        self.nav_result_label = ctk.CTkLabel(self, text="Result")
        self.nav_result_label.grid(row=9, column=0)
        self.nav_result_value = ctk.CTkLabel(self, text="")
        self.nav_result_value.grid(row=9, column=1, padx=30)


    def cancel_navigation_task(self):
        """Cancel the navigation task"""
        self.master.cancel_nav_task = True
        self.master.navigator.cancelTask()
        self.waypoint_count = 0
        self.nav_status_value.configure(text="INACTIVE")
        self.wp_number_value.configure(text=f"0/{self.waypoint_count}")
        self.time_rem_value.configure(text="0")
        self.dist_rem_value.configure(text="0")
        self.nav_time_value.configure(text="0")
        self.num_recs_value.configure(text="0")
        self.nav_result_value.configure(text=TaskResult.CANCELED.name)


    def get_routes(self) -> dict:
        """Read GPS waypoints of the route from the YAML file."""
        share_dir = get_package_share_directory("rovesugv_navsim")
        filepath = os.path.join(share_dir, "config", "gps_waypoints.yaml")
        with open(filepath, "r") as file:
            return yaml.safe_load(file)


    def on_nav_feedback(self, msg: NavigateToPose_Feedback):
        """Called when a navigation feedback message is published."""
        nav_time = msg.navigation_time.sec + \
            msg.navigation_time.nanosec / 1_000_000_000
        time_rem = msg.estimated_time_remaining.sec + \
            msg.estimated_time_remaining.nanosec / 1_000_000_000
        dist_rem = msg.distance_remaining

        self.time_rem_value.configure(text=f"{time_rem:.1f}")
        self.dist_rem_value.configure(text=f"{dist_rem:.1f}")
        self.nav_time_value.configure(text=f"{nav_time:.1f}")
        self.num_recs_value.configure(text=f"{msg.number_of_recoveries}")


    def on_nav_goal(self, result: TaskResult):
        """Called when the navigation task is done."""
        self.time_rem_value.configure(text="0")
        self.dist_rem_value.configure(text="0")
        self.nav_time_value.configure(text="0")
        self.num_recs_value.configure(text="0")
        self.nav_result_value.configure(text=result.name)


    def start_navigation_task(self):
        """Start a new navigation task."""
        self.nav_status_value.configure(text="ACTIVE")

        selected_route = self.route_menu.get()
        if selected_route == ROUTE_AUTO_TO_ROBO_LAB:
            route_key = "auto2robo"
        elif selected_route == ROUTE_ROBO_TO_AUTO_LAB:
            route_key = "robo2auto"

        waypoints = []
        for wp in self.route_dict[route_key]:
            lat, lon, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
            waypoint = gps_utils.latLonYaw2Geopose(lat, lon, yaw)
            waypoints.append(waypoint)
        
        self.waypoint_count = len(waypoints)
        
        nav_thread = Thread(target=self.master.follow_waypoints, args=(waypoints,))
        nav_thread.start()


class Application(ctk.CTk, Node):
    """Application for teleoperating the rover on Gazebo simulator."""

    def __init__(self):
        ctk.CTk.__init__(self)
        Node.__init__(self, node_name="teleop_app")
        self.geometry(f"{310}x{510}")
        self.resizable(width=False, height=False)
        self.title("SEAMK | Teleop")

        self.cancel_nav_task = False
        self.helv16bold = ctk.CTkFont(family="Helvetica", size=16, weight="bold")

        # Frames.
        self.driving_frame = Driving(self)
        self.driving_frame.grid(row=0, column=0, padx=5, pady=5)

        self.navigation_frame = Navigation(self)
        self.navigation_frame.grid(row=1, column=0, padx=5, pady=5, rowspan=2, sticky="ns")

        # Nav2 navigator.
        self.navigator = BasicNavigator()

        # ROS 2 service clients.
        self.fromll_client = self.create_client(srv_type=FromLL, srv_name="/fromLL")

        # ROS 2 publishers.
        self.cmd_vel_pub = self.create_publisher(
            msg_type=Twist,
            topic="/cmd_vel",
            qos_profile=10,
        )


    def follow_waypoints(self, waypoints: list[GeoPose]):
        """Navigate via GPS waypoints."""
        self.navigation_frame.on_nav_goal(TaskResult.UNKNOWN)
        #poses = []
        for index, waypoint in enumerate(waypoints):
            if self.cancel_nav_task:
                return
            wp_info = f"{index+1}/{self.navigation_frame.waypoint_count}"
            self.navigation_frame.wp_number_value.configure(text=wp_info)
            req = FromLL.Request()
            req.ll_point.latitude = waypoint.position.latitude
            req.ll_point.longitude = waypoint.position.longitude
            req.ll_point.altitude = waypoint.position.altitude

            future = self.fromll_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            result: FromLL.Response = future.result()

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position = result.map_point
            goal_pose.pose.orientation = waypoint.orientation
            #poses += [goal_pose]

            self.navigator.goToPose(goal_pose)

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    self.navigation_frame.on_nav_feedback(feedback)
                if self.cancel_nav_task:
                    break
        
        #self.navigator.followWaypoints(poses)

        #while not self.navigator.isTaskComplete():
        #    feedback = self.navigator.getFeedback()

        self.navigation_frame.on_nav_goal(self.navigator.getResult())


    def on_close(self):
        """Called when the application window is closed."""
        self.close = True


def main():
    rclpy.init()
    app = Application()
    app.mainloop()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
