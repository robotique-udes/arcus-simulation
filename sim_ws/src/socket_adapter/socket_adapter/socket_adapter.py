#!/usr/bin/env python3

# Topics leaving Docker -> heading to Physical Robot
TOPICS_TO_ROBOT = [
    ('/controller/drive', 'ackermann_msgs/msg/AckermannDriveStamped', AckermannDriveStamped),
    ('/deadman_active', 'std_msgs/msg/Bool', Bool)
]

# Topics leaving Robot -> arriving into Docker
TOPICS_FROM_ROBOT = [
    ('/master_error_code', 'arcus_msgs/msg/ErrorCode', ErrorCode),
    ('/master_hearbeat', 'std_msgs/msg/Bool', Bool)
]
"""Bidirectional adapter between a foxglove_bridge WebSocket and local ROS 2.

remote -> local : subscribes to channels advertised by the bridge (CDR encoding)
                  and republishes the raw bytes on local ROS 2 topics.
local -> remote : subscribes to local ROS 2 topics (raw/serialized) and publishes
                  them to the bridge using the "clientPublish" capability.

Speaks the Foxglove WebSocket protocol (foxglove.websocket.v1) directly with the
`websockets` package.

Parameters:
  url:
    WebSocket URL, e.g. ws://robot-ip:8765

  from_remote:
    List of regex patterns for remote topics to mirror locally.
    Patterns use full-match semantics.
    Default: [".*"]

  to_remote:
    List of TOPIC:TYPE definitions for local topics to send remotely.
    Example:
      ["/cmd_vel:geometry_msgs/msg/Twist"]

  local_prefix:
    Prefix added to mirrored remote topics locally.
    Example: "/robot"

Example YAML:

  foxglove_ros_adapter:
    ros__parameters:
      url: "ws://robot-ip:8765"
      from_remote:
        - "/scan"
        - "/odom"
        - "/camera/.*"
      to_remote:
        - "/cmd_vel:geometry_msgs/msg/Twist"
      local_prefix: "/robot"
"""

import asyncio
import json
import re
import struct
import threading

import rclpy
import websockets
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


SUBPROTOCOL = "foxglove.websocket.v1"

OP_MESSAGE_DATA = 0x01
OP_CLIENT_MESSAGE = 0x01

HEADER = struct.Struct("<BIQ")


class Adapter:
    def __init__(self, node: Node, loop: asyncio.AbstractEventLoop):
        self.node = node
        self.loop = loop
        self.log = node.get_logger()

        # ------------------------------------------------------------
        # ROS 2 parameters
        # ------------------------------------------------------------

        node.declare_parameter(
            "url",
            "",
        )

        node.declare_parameter(
            "from_remote",
            [".*"],
        )

        node.declare_parameter(
            "to_remote",
            [],
        )

        node.declare_parameter(
            "local_prefix",
            "",
        )

        self.url = node.get_parameter("url").value
        self.from_remote = node.get_parameter("from_remote").value
        self.to_remote_specs = node.get_parameter("to_remote").value
        self.local_prefix = node.get_parameter("local_prefix").value

        if not self.url:
            raise RuntimeError(
                "Parameter 'url' must be specified, e.g. "
                "url:=ws://robot-ip:8765"
            )

        # Compile remote topic regexes.
        self.patterns = [
            re.compile(pattern)
            for pattern in self.from_remote
        ]

        self.out_queue: asyncio.Queue = asyncio.Queue(maxsize=1000)

        # local -> remote definitions:
        # topic -> (type string, client channel id)
        self.to_remote = {}

        for i, spec in enumerate(self.to_remote_specs, start=1):
            topic, separator, type_str = spec.partition(":")

            if not separator or not topic or not type_str:
                raise RuntimeError(
                    "Each 'to_remote' entry must be TOPIC:TYPE, "
                    f"got '{spec}'"
                )

            self.to_remote[topic] = (type_str, i)

        # Never mirror back a topic that we already send to the remote.
        # This avoids loops.
        self.blocked_remote_topics = set(self.to_remote)

        self.local_pubs = {}
        self.local_subs = []

        # Per-connection:
        # subscription ID -> local publisher
        self.sub_to_pub = {}

        self.remote_can_publish = False

        self._setup_local_subscriptions()

    # ------------------------------------------------------------
    # local -> remote
    # ------------------------------------------------------------

    def _setup_local_subscriptions(self):
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        for topic, (type_str, chan_id) in self.to_remote.items():
            msg_cls = get_message(type_str)

            def cb(raw, chan_id=chan_id):
                # rclpy thread -> asyncio thread
                self.loop.call_soon_threadsafe(
                    self._enqueue,
                    chan_id,
                    bytes(raw),
                )

            self.local_subs.append(
                self.node.create_subscription(
                    msg_cls,
                    topic,
                    cb,
                    qos,
                    raw=True,
                )
            )

            self.log.info(
                f"local -> remote: {topic} ({type_str})"
            )

    def _enqueue(self, chan_id: int, payload: bytes):
        if self.out_queue.full():
            self.out_queue.get_nowait()

        self.out_queue.put_nowait((chan_id, payload))

    async def _sender(self, ws):
        while True:
            chan_id, payload = await self.out_queue.get()

            if self.remote_can_publish:
                await ws.send(
                    struct.pack(
                        "<BI",
                        OP_CLIENT_MESSAGE,
                        chan_id,
                    ) + payload
                )

    async def _advertise_client_channels(self, ws):
        channels = [
            {
                "id": chan_id,
                "topic": topic,
                "encoding": "cdr",
                "schemaName": type_str,
            }
            for topic, (type_str, chan_id) in self.to_remote.items()
        ]

        if channels:
            await ws.send(
                json.dumps(
                    {
                        "op": "advertise",
                        "channels": channels,
                    }
                )
            )

    # ------------------------------------------------------------
    # remote -> local
    # ------------------------------------------------------------

    def _wanted(self, topic: str) -> bool:
        if topic in self.blocked_remote_topics:
            return False

        return any(
            pattern.fullmatch(topic)
            for pattern in self.patterns
        )

    async def _handle_advertise(
        self,
        ws,
        channels,
        next_sub_id,
    ):
        subs = []

        for ch in channels:
            topic = ch["topic"]

            if (
                ch.get("encoding") != "cdr"
                or not self._wanted(topic)
            ):
                continue

            try:
                msg_cls = get_message(ch["schemaName"])
            except Exception as e:
                self.log.warning(
                    f"skip {topic}: cannot load "
                    f"{ch['schemaName']} ({e})"
                )
                continue

            local_topic = self.local_prefix + topic

            if local_topic not in self.local_pubs:
                qos = QoSProfile(
                    depth=10,
                    reliability=ReliabilityPolicy.RELIABLE,
                )

                self.local_pubs[local_topic] = (
                    self.node.create_publisher(
                        msg_cls,
                        local_topic,
                        qos,
                    )
                )

            sub_id = next_sub_id()

            self.sub_to_pub[sub_id] = (
                self.local_pubs[local_topic]
            )

            subs.append(
                {
                    "id": sub_id,
                    "channelId": ch["id"],
                }
            )

            self.log.info(
                f"remote -> local: "
                f"{topic} -> {local_topic}"
            )

        if subs:
            await ws.send(
                json.dumps(
                    {
                        "op": "subscribe",
                        "subscriptions": subs,
                    }
                )
            )

    def _handle_binary(self, data: bytes):
        if (
            len(data) < HEADER.size
            or data[0] != OP_MESSAGE_DATA
        ):
            return

        _, sub_id, _timestamp = HEADER.unpack_from(data)

        pub = self.sub_to_pub.get(sub_id)

        if pub is not None:
            pub.publish(data[HEADER.size:])

    # ------------------------------------------------------------
    # connection loop
    # ------------------------------------------------------------

    async def run(self):
        delay = 1.0

        while True:
            try:
                async with websockets.connect(
                    self.url,
                    subprotocols=[SUBPROTOCOL],
                    max_size=None,
                ) as ws:

                    self.log.info(
                        f"connected to {self.url}"
                    )

                    delay = 1.0

                    await self._session(ws)

            except (
                OSError,
                websockets.WebSocketException,
            ) as e:

                self.log.warning(
                    f"connection lost: {e}; "
                    f"retrying in {delay:.0f}s"
                )

            await asyncio.sleep(delay)

            delay = min(delay * 2, 10.0)

    async def _session(self, ws):
        self.sub_to_pub.clear()
        self.remote_can_publish = False

        counter = iter(range(1, 1 << 31))

        def next_sub_id():
            return next(counter)

        sender = asyncio.create_task(
            self._sender(ws)
        )

        try:
            async for message in ws:
                if isinstance(message, bytes):
                    self._handle_binary(message)
                    continue

                msg = json.loads(message)
                op = msg.get("op")

                if op == "serverInfo":
                    self.remote_can_publish = (
                        "clientPublish"
                        in msg.get("capabilities", [])
                    )

                    if (
                        self.to_remote
                        and not self.remote_can_publish
                    ):
                        self.log.warning(
                            "bridge lacks clientPublish capability"
                        )
                    else:
                        await self._advertise_client_channels(
                            ws
                        )

                elif op == "advertise":
                    await self._handle_advertise(
                        ws,
                        msg["channels"],
                        next_sub_id,
                    )

                elif op == "status":
                    self.log.info(
                        f"bridge status: "
                        f"{msg.get('message')}"
                    )

        finally:
            sender.cancel()


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node(
        "foxglove_ros_adapter"
    )

    loop = asyncio.new_event_loop()

    try:
        adapter = Adapter(node, loop)
    except Exception:
        node.destroy_node()
        rclpy.shutdown()
        raise

    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )

    spin_thread.start()

    try:
        loop.run_until_complete(
            adapter.run()
        )

    except KeyboardInterrupt:
        pass

    finally:
        loop.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
