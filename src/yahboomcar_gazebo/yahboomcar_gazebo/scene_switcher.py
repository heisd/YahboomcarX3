#!/usr/bin/env python3
"""Hot-swap Gazebo scenes (sets of built-in models) at runtime.

The node reads scene definitions from a YAML file (see config/scenes.yaml),
then spawns / deletes them through the gazebo_ros factory services
(`/spawn_entity`, `/delete_entity`). Switching a scene deletes whatever the
node spawned before and spawns the new set, so the world can be reshaped
without restarting Gazebo.

Interfaces
----------
sub  /scene_cmd     std_msgs/String   payload = scene name | "next" | "clear" | "list"
srv  /scene/next    std_srvs/Trigger  cycle to the next scene
srv  /scene/clear   std_srvs/Trigger  remove all spawned models
srv  /scene/reload  std_srvs/Trigger  respawn the current scene

Parameters
----------
scenes_file   (str)   absolute path to the scenes YAML
default_scene (str)   scene loaded once on startup ('' = none)
spawn_timeout (float) seconds to wait for each spawn/delete service call
"""

import math
import time

import rclpy
import yaml
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


def yaw_to_quat(roll, pitch, yaw):
    """Convert roll/pitch/yaw (rad) to a geometry_msgs quaternion tuple."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


def model_sdf(model_uri):
    """Wrap a built-in model reference in a spawnable SDF document.

    Matches the format gazebo_ros' own ``spawn_entity.py --database`` sends to
    the ``/spawn_entity`` service: the ``<include>`` lives inside a
    ``<world>`` element, which the gazebo_ros factory knows how to unpack.
    """
    return (
        '<sdf version="1.6">'
        '<world name="default">'
        '<include><uri>model://{}</uri></include>'
        '</world>'
        '</sdf>'
    ).format(model_uri)


def _geometry_xml(item):
    """Return the SDF <geometry> body for a primitive item, or None."""
    if 'box' in item:
        sx, sy, sz = (float(v) for v in item['box'])
        return '<box><size>{} {} {}</size></box>'.format(sx, sy, sz)
    if 'cylinder' in item:
        radius, length = (float(v) for v in item['cylinder'])
        return ('<cylinder><radius>{}</radius><length>{}</length>'
                '</cylinder>').format(radius, length)
    if 'sphere' in item:
        radius = float(item['sphere'][0])
        return '<sphere><radius>{}</radius></sphere>'.format(radius)
    return None


def primitive_sdf(name, item):
    """Build a static-primitive SDF model from a scene item, or None."""
    geom = _geometry_xml(item)
    if geom is None:
        return None
    color = item.get('color', [0.6, 0.6, 0.6])
    r, g, b = (float(c) for c in color)
    return (
        '<sdf version="1.6">'
        '<model name="{name}">'
        '<static>true</static>'
        '<link name="link">'
        '<collision name="collision"><geometry>{geom}</geometry></collision>'
        '<visual name="visual">'
        '<geometry>{geom}</geometry>'
        '<material>'
        '<ambient>{r} {g} {b} 1</ambient>'
        '<diffuse>{r} {g} {b} 1</diffuse>'
        '</material>'
        '</visual>'
        '</link>'
        '</model>'
        '</sdf>'
    ).format(name=name, geom=geom, r=r, g=g, b=b)


def entity_sdf(name, item):
    """Build the spawnable SDF for a scene item (primitive or model://)."""
    if 'model' in item:
        return model_sdf(item['model'])
    return primitive_sdf(name, item)


class SceneSwitcher(Node):

    def __init__(self):
        super().__init__('scene_switcher')

        self.declare_parameter('scenes_file', '')
        self.declare_parameter('default_scene', '')
        self.declare_parameter('spawn_timeout', 10.0)

        self._scenes_file = self.get_parameter('scenes_file').value
        self._default_scene = self.get_parameter('default_scene').value
        self._timeout = float(self.get_parameter('spawn_timeout').value)

        self._scenes = self._load_scenes(self._scenes_file)
        self._scene_names = list(self._scenes.keys())
        self._current_scene = None
        self._spawned = []  # entity names currently in the world

        cb = ReentrantCallbackGroup()
        self._spawn_cli = self.create_client(
            SpawnEntity, 'spawn_entity', callback_group=cb)
        self._delete_cli = self.create_client(
            DeleteEntity, 'delete_entity', callback_group=cb)

        self.create_subscription(
            String, 'scene_cmd', self._on_cmd, 10, callback_group=cb)
        self.create_service(
            Trigger, 'scene/next', self._on_next, callback_group=cb)
        self.create_service(
            Trigger, 'scene/clear', self._on_clear, callback_group=cb)
        self.create_service(
            Trigger, 'scene/reload', self._on_reload, callback_group=cb)

        self.get_logger().info(
            'scene_switcher ready. Scenes: {}'.format(
                ', '.join(self._scene_names) or '(none)'))

        # apply the default scene once the executor is spinning
        self._startup_timer = self.create_timer(
            1.0, self._apply_default, callback_group=cb)

    # ----- loading ---------------------------------------------------------
    def _load_scenes(self, path):
        if not path:
            self.get_logger().warn('No scenes_file parameter set.')
            return {}
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:  # noqa: BLE001 - report and continue empty
            self.get_logger().error(
                'Failed to read scenes file "{}": {}'.format(path, exc))
            return {}
        scenes = data.get('scenes', {})
        if not isinstance(scenes, dict):
            self.get_logger().error('"scenes" must be a mapping.')
            return {}
        return scenes

    def _apply_default(self):
        self._startup_timer.cancel()
        if self._default_scene:
            self.get_logger().info(
                'Loading default scene "{}"'.format(self._default_scene))
            self.switch_to(self._default_scene)

    # ----- gazebo service helpers -----------------------------------------
    def _wait_services(self):
        for cli, name in ((self._spawn_cli, 'spawn_entity'),
                          (self._delete_cli, 'delete_entity')):
            if not cli.wait_for_service(timeout_sec=self._timeout):
                self.get_logger().error(
                    'Gazebo service "{}" unavailable.'.format(name))
                return False
        return True

    def _call(self, client, request):
        future = client.call_async(request)
        deadline = time.time() + self._timeout
        while rclpy.ok() and not future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not future.done():
            return None
        return future.result()

    def _spawn(self, name, xml, pose):
        req = SpawnEntity.Request()
        req.name = name
        req.xml = xml
        req.initial_pose = pose
        req.reference_frame = 'world'
        res = self._call(self._spawn_cli, req)
        if res is None:
            self.get_logger().error('spawn "{}" timed out'.format(name))
            return False
        if not res.success:
            self.get_logger().warn(
                'spawn "{}" failed: {}'.format(name, res.status_message))
            return False
        self._spawned.append(name)
        return True

    def _delete(self, name):
        req = DeleteEntity.Request()
        req.name = name
        res = self._call(self._delete_cli, req)
        if res is None:
            self.get_logger().error('delete "{}" timed out'.format(name))
            return False
        if not res.success:
            self.get_logger().warn(
                'delete "{}" failed: {}'.format(name, res.status_message))
        return True

    # ----- scene operations ------------------------------------------------
    def clear(self):
        if not self._spawned:
            self._current_scene = None
            return
        if not self._wait_services():
            return
        for name in list(self._spawned):
            self._delete(name)
        self._spawned = []
        self._current_scene = None
        self.get_logger().info('Scene cleared.')

    def switch_to(self, scene_name):
        if scene_name not in self._scenes:
            self.get_logger().warn(
                'Unknown scene "{}". Available: {}'.format(
                    scene_name, ', '.join(self._scene_names) or '(none)'))
            return False
        if not self._wait_services():
            return False

        items = self._scenes[scene_name]
        if not isinstance(items, list):
            self.get_logger().error(
                'Scene "{}" must be a list of models, got {}.'.format(
                    scene_name, type(items).__name__))
            return False

        self.clear()

        spawned = 0
        for idx, item in enumerate(items):
            if not isinstance(item, dict):
                self.get_logger().warn(
                    'Scene "{}" item #{} is not a mapping, skipping.'.format(
                        scene_name, idx))
                continue
            entity = '{}_{}'.format(scene_name, idx)
            xml = entity_sdf(entity, item)
            if xml is None:
                self.get_logger().warn(
                    'Scene "{}" item #{} has no model/box/cylinder/sphere, '
                    'skipping.'.format(scene_name, idx))
                continue
            pose_vals = item.get('pose', [0.0] * 6)
            pose = Pose()
            pose.position.x = float(pose_vals[0])
            pose.position.y = float(pose_vals[1])
            pose.position.z = float(pose_vals[2])
            qx, qy, qz, qw = yaw_to_quat(
                float(pose_vals[3]), float(pose_vals[4]), float(pose_vals[5]))
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            if self._spawn(entity, xml, pose):
                spawned += 1

        self._current_scene = scene_name
        self.get_logger().info(
            'Switched to scene "{}" ({} objects).'.format(scene_name, spawned))
        return True

    def next_scene(self):
        if not self._scene_names:
            return False
        if self._current_scene in self._scene_names:
            idx = (self._scene_names.index(self._current_scene) + 1) % \
                len(self._scene_names)
        else:
            idx = 0
        return self.switch_to(self._scene_names[idx])

    # ----- callbacks -------------------------------------------------------
    def _on_cmd(self, msg):
        cmd = msg.data.strip()
        if cmd in ('clear', 'empty'):
            self.clear()
        elif cmd == 'next':
            self.next_scene()
        elif cmd == 'list':
            self.get_logger().info(
                'Available scenes: {}'.format(
                    ', '.join(self._scene_names) or '(none)'))
        elif cmd:
            self.switch_to(cmd)

    def _on_next(self, request, response):
        ok = self.next_scene()
        response.success = ok
        response.message = (
            'Now in scene "{}"'.format(self._current_scene) if ok
            else 'No scenes available')
        return response

    def _on_clear(self, request, response):
        self.clear()
        response.success = True
        response.message = 'Scene cleared'
        return response

    def _on_reload(self, request, response):
        if self._current_scene is None:
            response.success = False
            response.message = 'No active scene to reload'
            return response
        scene = self._current_scene
        ok = self.switch_to(scene)
        response.success = ok
        response.message = 'Reloaded "{}"'.format(scene) if ok else 'Reload failed'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SceneSwitcher()
    # >=2 threads is required: service calls busy-wait inside callbacks, so a
    # separate thread must stay free to process the responses (a 1-thread
    # executor would deadlock on a single-core host).
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
