"""Wait for the spawned model or frame its spawn point in the Gazebo GUI."""
import argparse
import math
import os
import sys
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.gui_camera_pb2 import GUICamera
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.scene_pb2 import Scene
from gz.transport13 import Node


def wait_for_model(node, world):
    service = f"/world/{world}/scene/info"
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        ok, scene = node.request(service, Empty(), Empty, Scene, 1500)
        if ok:
            for model in scene.model:
                if model.name != "lk1209_robot":
                    continue
                visuals = sum(v.HasField("geometry") for link in model.link for v in link.visual)
                if visuals:
                    print(f"Gazebo scene {world}: {model.name}, entity {model.id}, {visuals} visuals")
                    return
        time.sleep(0.25)
    sys.exit(f"Timed out waiting for lk1209_robot visual geometry on {service}")


def focus_camera(node, x, y, gui_pid=None):
    # An oblique view from inside the room, relative to the robot in this world.
    target = (x - 0.70, y + 0.45, 1.05)
    pitch = 0.85
    yaw = -0.5713
    request = GUICamera()
    position = request.pose.position
    position.x, position.y, position.z = target
    rotation = request.pose.orientation
    sp, cp = math.sin(pitch / 2), math.cos(pitch / 2)
    sy, cy = math.sin(yaw / 2), math.cos(yaw / 2)
    rotation.x, rotation.y, rotation.z, rotation.w = -sp * sy, sp * cy, cp * sy, cp * cy
    camera = None

    def observe(pose):
        nonlocal camera
        camera = pose

    node.subscribe(Pose, "/gui/camera/pose", observe)
    deadline = time.monotonic() + 35
    next_request = 0
    while time.monotonic() < deadline:
        if gui_pid is not None:
            try:
                os.kill(gui_pid, 0)
            except ProcessLookupError:
                sys.exit("Gazebo GUI closed before camera setup.")
        pose = camera
        if pose is not None:
            p, q = pose.position, pose.orientation
            dot = abs(q.x * rotation.x + q.y * rotation.y + q.z * rotation.z + q.w * rotation.w)
            if math.dist((p.x, p.y, p.z), target) < 0.05 and dot > 0.999:
                print("Gazebo GUI: camera at robot spawn view")
                return
            if time.monotonic() >= next_request:
                node.request("/gui/move_to/pose", request, GUICamera, Boolean, 1000)
                next_request = time.monotonic() + 3
        time.sleep(0.1)
    sys.exit("Could not confirm the Gazebo camera pose; double-click lk1209_robot in Entity Tree.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--world", metavar="WORLD")
    mode.add_argument("--spawn", nargs=2, type=float, metavar=("X", "Y"))
    parser.add_argument("--gui-pid", type=int, help="stop waiting if this GUI exits")
    args = parser.parse_args()
    node = Node()
    if args.world:
        wait_for_model(node, args.world)
    else:
        focus_camera(node, *args.spawn, gui_pid=args.gui_pid)
