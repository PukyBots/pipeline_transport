import rospy, math
import heapq
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState, SetModelState
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler

class CapsuleController:
    def __init__(self):
        rospy.init_node('capsule_controller')
        self.name = "transport_capsule"

        self.station_coords = {
            1: (-10, 0),
            2: (10, 0),
            3: (0, -10),
            4: (0, 10),
            5: (0, 0)
        }

        self.graph = {
            1: [5], 2: [5], 3: [5], 4: [5], 5: [1, 2, 3, 4]
        }

        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        rospy.wait_for_service('/gazebo/delete_model')

        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        self.marker_pub = rospy.Publisher('/capsule_marker', Marker, queue_size=10)

        # Wait for capsule to spawn
        while not rospy.is_shutdown():
            try:
                state = self.get_state(self.name, "world")
                if state.success:
                    break
            except:
                pass
            rospy.loginfo("Waiting for transport_capsule to spawn...")
            rospy.sleep(0.5)

        self._force_teleport(*self.station_coords[1])
        rospy.sleep(1)

        rospy.Subscriber('/capsule/command', Int32, self._callback)
        rospy.loginfo("Capsule controller ready.")

    def _force_teleport(self, x, y):
        state = ModelState()
        state.model_name = self.name
        state.reference_frame = 'world'
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = 1.0
        state.pose.orientation.w = 1.0
        try:
            self.set_state(state)
        except rospy.ServiceException as e:
            rospy.logerr(f"Teleport failed: {e}")

    def _callback(self, msg):
        dest = msg.data
        if dest not in self.station_coords:
            rospy.logerr("Invalid station.")
            return

        src = self._get_current_station()
        path = self._dijkstra(src, dest)

        if not path:
            rospy.logerr("No path found.")
            return

        rospy.loginfo(f"Path: {path}")
        self._highlight_path(path)

        for i in range(len(path) - 1):
            x1, y1 = self.station_coords[path[i]]
            x2, y2 = self.station_coords[path[i + 1]]
            
            # First move in x while keeping y constant
            self._move_axis('x', x1, x2, fixed=y1)
            # Then move in y while keeping x constant (now at x2)
            self._move_axis('y', y1, y2, fixed=x2)
            rospy.sleep(1.0)

        # Final marker position
        final_x, final_y = self.station_coords[dest]
        self._publish_marker(final_x, final_y)

    def _move_axis(self, axis, start, end, fixed):
        rate = rospy.Rate(30)
        speed = 3.0
        pos = start
        while abs(pos - end) > 0.05:
            pos += math.copysign(speed / 30.0, end - pos)
            state = ModelState()
            state.model_name = self.name
            state.reference_frame = "world"
            state.pose.position.x = pos if axis == 'x' else fixed
            state.pose.position.y = pos if axis == 'y' else fixed
            state.pose.position.z = 1.0
            state.pose.orientation.w = 1.0
            self.set_state(state)
            rate.sleep()


    def _publish_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "capsule"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def _get_current_station(self):
        while not rospy.is_shutdown():
            try:
                state = self.get_state(self.name, "world")
                if state.success:
                    break
            except rospy.ServiceException:
                pass
            rospy.loginfo("Waiting for transport_capsule to spawn...")
            rospy.sleep(0.5)

        pos = (round(state.pose.position.x), round(state.pose.position.y))
        for k, v in self.station_coords.items():
            if (round(v[0]), round(v[1])) == pos:
                return k
        return 5  # default to crossroad

    def _dijkstra(self, start, goal):
        dist = {node: float('inf') for node in self.graph}
        dist[start] = 0
        prev = {}
        q = [(0, start)]

        while q:
            d, u = heapq.heappop(q)
            if u == goal:
                break
            for v in self.graph[u]:
                alt = d + self._distance(u, v)
                if alt < dist[v]:
                    dist[v] = alt
                    prev[v] = u
                    heapq.heappush(q, (alt, v))

        path = []
        u = goal
        while u in prev:
            path.insert(0, u)
            u = prev[u]
        if u == start:
            path.insert(0, start)
        return path

    def _distance(self, u, v):
        x1, y1 = self.station_coords[u]
        x2, y2 = self.station_coords[v]
        return math.hypot(x2 - x1, y2 - y1)


    def _highlight_path(self, path):
        # Clean up previous path segments
        for i in range(10):  # adjust if more segments possible
            try:
                self.delete_model(f"path_segment_{i}")
            except:
                pass

        marker_sdf = """
        <sdf version='1.6'>
        <model name='{name}'>
            <static>true</static>
            <link name='link'>
            <visual name='visual'>
                <geometry>
                <cylinder>
                    <radius>0.2</radius>
                    <length>{length}</length>
                </cylinder>
                </geometry>
                <material>
                <ambient>0.1 1.0 0.1 1</ambient>
                </material>
            </visual>
            </link>
        </model>
        </sdf>
        """

        for i in range(len(path) - 1):
            x1, y1 = self.station_coords[path[i]]
            x2, y2 = self.station_coords[path[i + 1]]
            name = f"path_segment_{i}"
            length = math.hypot(x2 - x1, y2 - y1)
            pose_x = (x1 + x2) / 2
            pose_y = (y1 + y2) / 2
            yaw = math.atan2(y2 - y1, x2 - x1)
            sdf = marker_sdf.format(name=name, length=length)

            initial_pose = Pose()
            initial_pose.position.x = pose_x
            initial_pose.position.y = pose_y
            initial_pose.position.z = 0.9  # match pipe height

            q = quaternion_from_euler(0, 1.5708, yaw)
            initial_pose.orientation.x = q[0]
            initial_pose.orientation.y = q[1]
            initial_pose.orientation.z = q[2]
            initial_pose.orientation.w = q[3]

            try:
                self.spawn_model(name, sdf, "", initial_pose, "world")
            except rospy.ServiceException as e:
                rospy.logwarn(f"Failed to spawn marker {name}: {e}")

if __name__ == '__main__':
    try:
        CapsuleController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
