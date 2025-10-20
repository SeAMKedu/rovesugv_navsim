import os
from threading import Thread
from typing import Callable

import customtkinter as ctk
import rclpy
import yaml
from ament_index_python import get_package_share_directory
from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action._navigate_to_pose import NavigateToPose_Feedback
from nav2_msgs.action._follow_waypoints import FollowWaypoints_Feedback
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node
from robot_localization.srv import FromLL

from rovesugv_navsim.gps_utils import latLonYaw2Geopose

ctk.set_appearance_mode('light')
ctk.set_default_color_theme('dark-blue')

LMB_PRESS = "<ButtonPress-1>"


class WaypointRouter(Node):

    def __init__(self):
        super().__init__(node_name='gps_wp_commander')
        self.navigator = BasicNavigator()
        self.localizer = self.create_client(FromLL, '/fromLL')


    def _set_pose(self, waypoint: GeoPose) -> PoseStamped:
        """Set the pose to navigate to."""
        request = FromLL.Request()
        request.ll_point.altitude = waypoint.position.altitude
        request.ll_point.latitude = waypoint.position.latitude
        request.ll_point.longitude = waypoint.position.longitude

        future = self.localizer.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response: FromLL.Response = future.result()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = response.map_point
        pose.pose.orientation = waypoint.orientation

        return pose


    def go_to_waypoint(self, waypoint: GeoPose, on_feedback, on_target):
        """Navigate to a GPS waypoint.

        :param waypoint: Waypoint to navigate to.
        :param on_feedback: Function to call on feedback message.
        :param on_target: Function to call when navigation target is reached.

        """
        pose = self._set_pose(waypoint)
        self.navigator.goToPose(pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if not feedback:
                continue
            on_feedback(feedback)

        on_target()


    def follow_waypoints(self, geoposes: list[GeoPose], on_feedback, on_target):
        """Navigate via GPS waypoints.

        :param waypoint: List of waypoints to navigate through.
        :param on_feedback: Function to call on feedback message.
        :param on_target: Function to call when navigation target is reached.
        """
        poses = []
        for geopose in geoposes:
            pose = self._set_pose(geopose)
            poses += [pose]
        self.navigator.followWaypoints(poses)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if not feedback:
                continue
            on_feedback(feedback)

        on_target()


class WaypointParser:

    def __init__(self):
        self.share_dir = get_package_share_directory('rovesugv_navsim')
        self.path = os.path.join(self.share_dir, 'config', 'gps_waypoints.yaml')

        with open(self.path, 'r') as file:
            self.wp_dict = yaml.safe_load(file)


class Application(ctk.CTk):

    def __init__(self, wp_router: WaypointRouter, wp_parser: WaypointParser):
        super().__init__()
        self.title('SEAMK | GPS Waypoint Commander')
        self.geometry(f'{600}x{400}')

        self.wp_router = wp_router
        self.wp_parser = wp_parser

        self.is_task_running = False
        self.is_task_complete = False

        self.button1 = ctk.CTkButton(self, text="Docking Station")
        self.button1.grid(row=0, column=0, padx=5, pady=5)
        self.button1.bind(
            LMB_PRESS, 
            lambda event: self.navigate_to_target('docking_station')
        )

        self.button2 = ctk.CTkButton(self, text="Manufacturing Lab")
        self.button2.grid(row=0, column=1, padx=5, pady=5)
        self.button2.bind(
            LMB_PRESS, 
            lambda event: self.navigate_to_target('manufacturing_lab')
        )

        self.button3 = ctk.CTkButton(self, text="Robotics Lab")
        self.button3.grid(row=0, column=2, padx=5, pady=5)
        self.button3.bind(
            LMB_PRESS, 
            lambda event: self.navigate_to_target('robotics_lab')
        )

        self.button4 = ctk.CTkButton(self, text="Follow Route")
        self.button4.grid(row=0, column=3, padx=5, pady=5)
        self.button4.bind(
            LMB_PRESS, 
            lambda event: self.navigate_to_target('route')
        )
    
        self.label01 = ctk.CTkLabel(self, width=100, text="Task Running:")
        self.label01.grid(row=1, column=0, padx=5, pady=5)

        self.label02 = ctk.CTkLabel(self, width=100, text="False")
        self.label02.grid(row=1, column=1, padx=5, pady=5)

        self.label03 = ctk.CTkLabel(self, width=100, text="Task Complete:")
        self.label03.grid(row=2, column=0, padx=5, pady=5)

        self.label04 = ctk.CTkLabel(self, width=100, text="False")
        self.label04.grid(row=2, column=1, padx=5, pady=5)

        self.label05 = ctk.CTkLabel(self, width=100, text="ETA:")
        self.label05.grid(row=3, column=0, padx=5, pady=5)

        self.label06 = ctk.CTkLabel(self, width=100, text="0 s")
        self.label06.grid(row=3, column=1, padx=5, pady=5)

        self.label07 = ctk.CTkLabel(self, width=100, text="Distance remaining:")
        self.label07.grid(row=4, column=0, padx=5, pady=5)

        self.label08 = ctk.CTkLabel(self, width=100, text="0.00 m")
        self.label08.grid(row=4, column=1, padx=5, pady=5)

        self.label09 = ctk.CTkLabel(self, width=100, text="Time taken:")
        self.label09.grid(row=5, column=0, padx=5, pady=5)

        self.label10 = ctk.CTkLabel(self, width=100, text="0 s")
        self.label10.grid(row=5, column=1, padx=5, pady=5)

        self.label11 = ctk.CTkLabel(self, width=100, text="Recoveries:")
        self.label11.grid(row=6, column=0, padx=5, pady=5)

        self.label12 = ctk.CTkLabel(self, width=100, text="0")
        self.label12.grid(row=6, column=1, padx=5, pady=5)

        self.label13 = ctk.CTkLabel(self, width=100, text="Current Waypoint:")
        self.label13.grid(row=7, column=0, padx=5, pady=5)

        self.label14 = ctk.CTkLabel(self, width=100, text="0 / 0")
        self.label14.grid(row=7, column=1, padx=5, pady=5)

        self.label15 = ctk.CTkLabel(self, width=100, text="Navigation Target:")
        self.label15.grid(row=8, column=0, padx=5, pady=5)

        self.label16 = ctk.CTkLabel(self, width=100, text="")
        self.label16.grid(row=8, column=1, padx=5, pady=5)


    def _set_buttons_state(self, state: str):
        """Set the state of the buttons."""
        self.button1.configure(state=state)
        self.button2.configure(state=state)
        self.button3.configure(state=state)
        self.button4.configure(state=state)
    

    def on_waypoint_feedback(self, feedback: NavigateToPose_Feedback):
        """Called when the feedback message is received."""
        estimated_time_remaining = feedback.estimated_time_remaining.sec + \
            feedback.estimated_time_remaining.nanosec / 1_000_000_000

        navtime = feedback.navigation_time.sec + \
            feedback.navigation_time.nanosec / 1_000_000_000

        self.label06.configure(text=f'{estimated_time_remaining:.2f} s')
        self.label08.configure(text=f'{feedback.distance_remaining:.2f} m')
        self.label10.configure(text=f'{navtime:.2f} s')
        self.label12.configure(text=f'{feedback.number_of_recoveries}')


    def on_route_feedback(self, feedback: FollowWaypoints_Feedback):
        """Called when the feedback message is received."""
        self.label14.configure(text=f'{feedback.current_waypoint+1} / 4')


    def on_target(self):
        """Called when the rover has reached the navigation goal."""
        self._set_buttons_state('normal')
        self.is_task_running = False
        self.is_task_complete = True

        self.label02.configure(text=str(self.is_task_running))
        self.label04.configure(text=str(self.is_task_complete))
        self.label06.configure(text='0 s')
        self.label08.configure(text='0.00 m')
        self.label10.configure(text='0 s')
        self.label12.configure(text='0')
        self.label14.configure(text='0 / 0')
        self.label16.configure(text='')


    def navigate_to_target(self, target: str):
        """Navigate to the given GPS waypoint(s)."""
        if self.is_task_running:
            return
        self._set_buttons_state('disabled')
        self.is_task_running = True
        self.is_task_complete = False
    
        self.label02.configure(text=str(self.is_task_running))
        self.label04.configure(text=str(self.is_task_complete))
        self.label16.configure(text=target)

        thread: Thread = None

        waypoints = []
        for wp in self.wp_parser.wp_dict[target]:
            lat, lon, yaw = wp['latitude'], wp['longitude'], wp['yaw']
            waypoint = latLonYaw2Geopose(lat, lon, yaw)
            waypoints.append(waypoint)

        if target == 'route':
            thread = Thread(
                target=self.wp_router.follow_waypoints, 
                args=(waypoints, self.on_route_feedback, self.on_target),
            )
        else:
            thread = Thread(
                target=self.wp_router.go_to_waypoint, 
                args=(waypoints[0], self.on_waypoint_feedback, self.on_target),
            )

        thread.start()


def main():
    rclpy.init()
    wp_router = WaypointRouter()
    wp_parser = WaypointParser()
    app = Application(wp_router, wp_parser)
    app.mainloop()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
