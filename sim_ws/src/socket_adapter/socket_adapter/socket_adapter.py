#!/usr/bin/env python3
"""Bidirectional adapter between a foxglove_bridge WebSocket and local ROS 2.

BIG PICTURE
-----------
Two ROS 2 systems are involved:

  * the ROBOT (e.g. a Jetson) which runs `foxglove_bridge`. That program
    exposes the robot's ROS 2 topics through a WebSocket (default port 8765)
    using the "Foxglove WebSocket protocol".
  * THIS machine (e.g. a Docker container) which runs this script and has its
    own, separate ROS 2 graph.

This script sits in the middle and copies messages in both directions:

    robot ROS 2  <->  foxglove_bridge  <->  WebSocket  <->  [this script]  <->  local ROS 2

remote -> local : subscribes to channels advertised by the bridge (CDR encoding)
                  and republishes the raw bytes on local ROS 2 topics.
local -> remote : subscribes to local ROS 2 topics (raw/serialized) and publishes
                  them to the bridge using the "clientPublish" capability.

Messages are never decoded: they are forwarded as raw CDR bytes (the binary
serialization format ROS 2 uses on the wire). That is fast and means the script
does not need to know the content of a message, only its type name.

Speaks the Foxglove WebSocket protocol (foxglove.websocket.v1) directly with the
`websockets` package.

Parameters (ROS 2 parameters, set with `--ros-args -p name:=value`):
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

  latched_topics:
    List of regex patterns (full-match, on the remote topic name) for mirrored
    topics to republish with TRANSIENT_LOCAL durability ("latched"), so that
    subscribers that ask for it (RViz, robot_state_publisher users, ...) get
    the last message even if they start late.
    Default: ["/tf_static", "/map", ".*_description"]
"""

# --- Standard library --------------------------------------------------------
import asyncio    # event loop used for the WebSocket connection
import json       # the protocol's control messages are JSON text
import re         # regular expressions to select which remote topics to mirror
import struct     # pack/unpack the binary message headers of the protocol
import threading  # rclpy.spin runs in its own thread next to the asyncio loop

# --- Third-party / ROS 2 -----------------------------------------------------
import rclpy
import websockets
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
# get_message("pkg/msg/Type") returns the Python class for a ROS 2 message type
from rosidl_runtime_py.utilities import get_message


# --- Foxglove WebSocket protocol constants -----------------------------------

# Name of the WebSocket sub-protocol. The server refuses clients that do not
# announce it during the handshake.
SUBPROTOCOL = "foxglove.sdk.v1"

# First byte of every BINARY message identifies its kind.
# Server -> client: 0x01 = "message data" (a ROS message from a subscribed topic)
OP_MESSAGE_DATA = 0x01
# Client -> server: 0x01 = "client message data" (a message we publish)
OP_CLIENT_MESSAGE = 0x01

# Layout of a server -> client binary message (little-endian):
#   B = 1 byte  opcode
#   I = 4 bytes subscription id (which subscription this data belongs to)
#   Q = 8 bytes timestamp in nanoseconds
# followed directly by the message payload (the raw CDR bytes).
HEADER = struct.Struct("<BIQ")


class Adapter:
    """Holds all the state and logic of the bridge <-> ROS 2 adapter.

    Two worlds run side by side and must be kept separate:
      * the asyncio world  : WebSocket traffic (single thread, event loop)
      * the rclpy world    : ROS 2 callbacks (run in the spin thread)
    Data crosses from rclpy to asyncio only through `call_soon_threadsafe`.
    """

    def __init__(self, node: Node, loop: asyncio.AbstractEventLoop):
        self.node = node
        self.loop = loop              # the asyncio loop, needed for thread hand-off
        self.log = node.get_logger()  # ROS 2 logger (shows up in the console)

        # ------------------------------------------------------------
        # ROS 2 parameters
        # ------------------------------------------------------------
        # Declare the parameters with their default values. They can then be
        # overridden from the command line or a launch file.

        node.declare_parameter(
            "url",
            "ws://localhost:8765",
        )

        node.declare_parameter(
            "from_remote",
            [".*"],
        )

        node.declare_parameter(
            "to_remote",
            Parameter.Type.STRING_ARRAY,
        )

        node.declare_parameter(
            "local_prefix",
            "",
        )

        node.declare_parameter(
            "latched_topics",
            ["/tf_static", "/map", ".*_description"],
        )

        # Read the final values (defaults or overrides) into plain attributes.
        self.url = node.get_parameter("url").value
        self.from_remote = node.get_parameter("from_remote").value
        self.to_remote_specs = node.get_parameter("to_remote").value
        self.local_prefix = node.get_parameter("local_prefix").value
        self.latched_topics = node.get_parameter("latched_topics").value

        # Fail early with a clear message if the URL was set to an empty string.
        if not self.url:
            raise RuntimeError(
                "Parameter 'url' must be specified, e.g. "
                "url:=ws://robot-ip:8765"
            )

        # Compile remote topic regexes (done once, reused for every topic).
        self.patterns = [
            re.compile(pattern)
            for pattern in self.from_remote
        ]
        self.latched_patterns = [
            re.compile(pattern)
            for pattern in self.latched_topics
        ]

        # Queue of messages waiting to be sent to the robot.
        # Items are (channel_id, payload_bytes). It is bounded (1000) so that a
        # slow or broken connection cannot make memory grow forever.
        self.out_queue: asyncio.Queue = asyncio.Queue(maxsize=1000)

        # local -> remote definitions:
        # topic -> (type string, client channel id)
        self.to_remote = {}

        # Turn each "TOPIC:TYPE" string into an entry of self.to_remote.
        # The channel id is just a unique number we choose (1, 2, 3, ...); the
        # bridge uses it to know which topic each binary message belongs to.
        for i, spec in enumerate(self.to_remote_specs, start=1):
            # "/cmd_vel:geometry_msgs/msg/Twist".partition(":")
            #   -> ("/cmd_vel", ":", "geometry_msgs/msg/Twist")
            topic, separator, type_str = spec.partition(":")

            if not separator or not topic or not type_str:
                raise RuntimeError(
                    "Each 'to_remote' entry must be TOPIC:TYPE, "
                    f"got '{spec}'"
                )

            self.to_remote[topic] = (type_str, i)

        # Never mirror back a topic that we already send to the remote.
        # This avoids loops.
        # (Without this: we send /cmd_vel to the robot, the bridge advertises
        #  /cmd_vel back to us, we republish it locally, our own subscription
        #  sees it, sends it again... forever.)
        self.blocked_remote_topics = set(self.to_remote)

        # Local publishers created for mirrored topics, keyed by local topic
        # name. Kept across reconnections so we never create duplicates.
        self.local_pubs = {}
        # Keeps references to the local subscriptions so they are not
        # garbage-collected (a subscription that is dropped stops working).
        self.local_subs = []

        # Per-connection (cleared every time we reconnect):
        # subscription ID -> local publisher
        self.sub_to_pub = {}

        # Becomes True only if the bridge says it accepts messages from clients.
        self.remote_can_publish = False

        # Start listening to the local topics that must be sent to the robot.
        self._setup_local_subscriptions()

    # ------------------------------------------------------------
    # local -> remote
    # ------------------------------------------------------------
    # Flow: local ROS topic -> rclpy callback -> out_queue -> _sender -> WebSocket

    def _setup_local_subscriptions(self):
        """Subscribe to every local topic listed in `to_remote`."""
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        for topic, (type_str, chan_id) in self.to_remote.items():
            # Resolve "pkg/msg/Type" to the Python class. Fails right away (at
            # startup) if the message package is not installed on this machine.
            msg_cls = get_message(type_str)

            # `chan_id=chan_id` freezes the current value in the closure.
            # Without it every callback would use the id of the LAST topic of
            # the loop (classic Python late-binding pitfall).
            def cb(raw, chan_id=chan_id):
                # Runs in the rclpy thread. asyncio objects must not be touched
                # from another thread, so we ask the event loop to run
                # _enqueue for us.
                self.loop.call_soon_threadsafe(
                    self._enqueue,
                    chan_id,
                    bytes(raw),  # `raw` is already-serialized CDR data
                )

            self.local_subs.append(
                self.node.create_subscription(
                    msg_cls,
                    topic,
                    cb,
                    qos,
                    raw=True,  # give us the serialized bytes, don't deserialize
                )
            )

            self.log.info(
                f"local -> remote: {topic} ({type_str})"
            )

    def _enqueue(self, chan_id: int, payload: bytes):
        """Put a message in the outgoing queue (runs in the asyncio thread)."""
        if self.out_queue.full():
            # Queue full = we are producing faster than the network can send.
            # Drop the OLDEST message so the freshest data always gets through.
            self.out_queue.get_nowait()

        self.out_queue.put_nowait((chan_id, payload))

    async def _sender(self, ws):
        """Background task: send queued messages to the bridge, one by one."""
        while True:
            # Wait (without blocking the loop) until a message is available.
            chan_id, payload = await self.out_queue.get()

            # If the bridge does not accept client messages, silently discard.
            if self.remote_can_publish:
                await ws.send(
                    # Binary frame = [opcode: 1 byte][channel id: 4 bytes][payload]
                    # "<BI" = little-endian, unsigned byte, unsigned 32-bit int.
                    struct.pack(
                        "<BI",
                        OP_CLIENT_MESSAGE,
                        chan_id,
                    ) + payload
                )

    async def _advertise_client_channels(self, ws):
        """Tell the bridge which topics we are going to publish to it.

        The bridge must know the channel (id, topic, encoding, type) BEFORE it
        accepts binary messages on it. It then republishes them into the
        robot's ROS 2 graph.
        """
        channels = [
            {
                "id": chan_id,             # our chosen number for this channel
                "topic": topic,            # topic name on the robot
                "encoding": "cdr",         # payloads are ROS 2 CDR bytes
                "schemaName": type_str,    # e.g. "std_msgs/msg/Bool"
            }
            for topic, (type_str, chan_id) in self.to_remote.items()
        ]

        # Nothing to advertise if `to_remote` is empty.
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
    # Flow: bridge "advertise" -> subscribe -> binary data -> local publisher

    def _wanted(self, topic: str) -> bool:
        """Decide whether a remote topic should be mirrored locally."""
        # Loop protection (see blocked_remote_topics above).
        if topic in self.blocked_remote_topics:
            return False

        # fullmatch: the regex must match the WHOLE topic name.
        # ".*" matches everything, "/odom" only "/odom", "/camera/.*" every
        # topic under /camera/.
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
        """React to the bridge announcing topics ("channels") it can send us.

        For each channel we want, create a local publisher and ask the bridge
        to start streaming it to us.
        """
        subs = []  # the subscriptions we will request, in a single message

        for ch in channels:
            topic = ch["topic"]

            # Skip channels that are not CDR-encoded (we only forward raw CDR)
            # and topics that don't match our filters.
            if (
                ch.get("encoding") != "cdr"
                or not self._wanted(topic)
            ):
                continue

            # Find the Python class of the message type, e.g.
            # "std_msgs/msg/Bool". It fails if the package that defines the
            # type is not installed locally; then we skip the topic.
            try:
                msg_cls = get_message(ch["schemaName"])
            except Exception as e:
                self.log.warning(
                    f"skip {topic}: cannot load "
                    f"{ch['schemaName']} ({e})"
                )
                continue

            # Name of the topic on THIS machine (optionally prefixed).
            local_topic = self.local_prefix + topic

            # Create the local publisher only once, even if the bridge
            # re-advertises the same topic or we reconnect.
            if local_topic not in self.local_pubs:
                # Topics like /tf_static, /map or robot descriptions are
                # published once and "latched" (TRANSIENT_LOCAL) on the robot.
                # Their subscribers require TRANSIENT_LOCAL too: a VOLATILE
                # publisher is incompatible and they would receive nothing.
                latched = any(
                    pattern.fullmatch(topic)
                    for pattern in self.latched_patterns
                )
                qos = QoSProfile(
                    depth=10,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=(
                        DurabilityPolicy.TRANSIENT_LOCAL
                        if latched
                        else DurabilityPolicy.VOLATILE
                    ),
                )

                self.local_pubs[local_topic] = (
                    self.node.create_publisher(
                        msg_cls,
                        local_topic,
                        qos,
                    )
                )

            # Subscription id: a number we pick. When binary data arrives, its
            # header carries this id so we know which publisher to use.
            sub_id = next_sub_id()

            self.sub_to_pub[sub_id] = (
                self.local_pubs[local_topic]
            )

            subs.append(
                {
                    "id": sub_id,           # our subscription number
                    "channelId": ch["id"],  # the bridge's number for the channel
                }
            )

            self.log.info(
                f"remote -> local: "
                f"{topic} -> {local_topic}"
                + (" (latched)" if latched else "")
            )

        # One single "subscribe" request for all the channels collected above.
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
        """Handle one binary frame received from the bridge (= one ROS message)."""
        # Ignore frames that are too short or not of the "message data" kind.
        if (
            len(data) < HEADER.size
            or data[0] != OP_MESSAGE_DATA
        ):
            return

        # Read the 13-byte header; we don't use the timestamp.
        _, sub_id, _timestamp = HEADER.unpack_from(data)

        # Which local publisher belongs to this subscription?
        pub = self.sub_to_pub.get(sub_id)

        if pub is not None:
            # Everything after the header is the raw CDR message. rclpy accepts
            # serialized bytes directly in publish().
            pub.publish(data[HEADER.size:])

    # ------------------------------------------------------------
    # connection loop
    # ------------------------------------------------------------

    async def run(self):
        """Connect to the bridge forever, reconnecting if the link drops."""
        delay = 1.0  # seconds to wait before the next reconnection attempt

        while True:
            try:
                async with websockets.connect(
                    self.url,
                    subprotocols=[SUBPROTOCOL],  # required by foxglove_bridge
                    max_size=None,               # no limit on message size
                                                 # (images/point clouds are big)
                ) as ws:

                    self.log.info(
                        f"connected to {self.url}"
                    )

                    # Connected: reset the backoff delay.
                    delay = 1.0

                    # Stays here until the connection closes.
                    await self._session(ws)

            except (
                OSError,                         # network down, host unreachable...
                websockets.WebSocketException,   # protocol / closed connection
            ) as e:

                self.log.warning(
                    f"connection lost at {self.url}: {e}; "
                    f"retrying in {delay:.0f}s"
                )

            await asyncio.sleep(delay)

            # Exponential backoff: 1s, 2s, 4s, 8s, then capped at 10s.
            delay = min(delay * 2, 10.0)

    async def _session(self, ws):
        """Handle one connection, from handshake to disconnection."""
        # A new connection starts from a clean state: the old subscription ids
        # are no longer valid on the bridge side.
        self.sub_to_pub.clear()
        self.remote_can_publish = False

        # Generator of unique subscription ids: 1, 2, 3, ...
        counter = iter(range(1, 1 << 31))

        def next_sub_id():
            return next(counter)

        # Start the task that sends queued local messages to the bridge.
        sender = asyncio.create_task(
            self._sender(ws)
        )

        try:
            # Read incoming WebSocket messages until the connection closes.
            async for message in ws:
                # Binary frame = ROS message data.
                if isinstance(message, bytes):
                    self._handle_binary(message)
                    continue

                # Text frame = JSON control message; "op" says what it is.
                msg = json.loads(message)
                op = msg.get("op")

                if op == "serverInfo":
                    # First message sent by the bridge. It lists its
                    # capabilities. We need "clientPublish" to send data.
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
                        # Declare the channels we will publish to the bridge.
                        await self._advertise_client_channels(
                            ws
                        )

                elif op == "advertise":
                    # The bridge lists the topics it can stream to us (sent
                    # after serverInfo, and again whenever topics appear).
                    await self._handle_advertise(
                        ws,
                        msg["channels"],
                        next_sub_id,
                    )

                elif op == "status":
                    # Informational messages and errors from the bridge.
                    self.log.info(
                        f"bridge status: "
                        f"{msg.get('message')}"
                    )

        finally:
            # Connection over: stop the sender task so it doesn't linger.
            sender.cancel()


def main(args=None):
    # Initialize the ROS 2 client library, then create the node.
    rclpy.init(args=args)

    node = rclpy.create_node(
        "socket_adapter"
    )

    # Dedicated asyncio event loop for the WebSocket side.
    loop = asyncio.new_event_loop()

    try:
        adapter = Adapter(node, loop)
    except Exception:
        # Bad parameters or missing message package: clean up, then re-raise
        # so the error message is visible.
        node.destroy_node()
        rclpy.shutdown()
        raise

    # ROS 2 callbacks (local subscriptions) need rclpy.spin running. It blocks,
    # so it lives in its own daemon thread (daemon = dies with the program),
    # while the main thread runs the asyncio loop below.
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )

    spin_thread.start()

    try:
        # Runs the connect / reconnect loop until Ctrl+C.
        loop.run_until_complete(
            adapter.run()
        )

    except KeyboardInterrupt:
        pass

    finally:
        # Orderly shutdown.
        loop.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()