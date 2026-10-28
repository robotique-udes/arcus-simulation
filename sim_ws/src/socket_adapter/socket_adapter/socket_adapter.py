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
services        : every service advertised by the bridge gets a local "proxy"
                  service with the same name and type. A local call is
                  forwarded to the robot ("services" capability) and the
                  robot's answer is returned to the local caller.
parameters      : ROS 2 parameters are just services (/<node>/get_parameters,
                  /<node>/set_parameters, ...), so they are forwarded like any
                  other service. On top of that, an empty local "stand-in" node
                  is created for each remote node, because `ros2 param` refuses
                  to talk to a node it cannot see in the local graph. Result:
                    ros2 param list / get / set / dump /<robot_node> ...
                  work from this machine as if the node were local.

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

  services_from_remote:
    List of regex patterns (full-match) for remote services to make callable
    locally. They get the same local_prefix as topics. A remote service is
    skipped if a local service with the same name already exists (e.g. the
    same node runs on both machines), to avoid two servers on one name.
    Default: [".*"]

  param_nodes:
    If true, create a local stand-in node for every remote node whose
    parameter services are forwarded, so that `ros2 param` finds it.
    Default: true

  service_timeout:
    Seconds to wait for the robot to answer a forwarded service call. On
    timeout or error a warning is logged and the caller gets an empty
    (default) response, because ROS 2 services have no way to return an error.
    Default: 5.0
"""

# --- Standard library --------------------------------------------------------
import asyncio    # event loop used for the WebSocket connection
import concurrent.futures  # result of a coroutine started from another thread
import itertools  # endless counter for service call ids
import json       # the protocol's control messages are JSON text
import re         # regular expressions to select which remote topics to mirror
import struct     # pack/unpack the binary message headers of the protocol
import threading  # rclpy.spin runs in its own thread next to the asyncio loop

# --- Third-party / ROS 2 -----------------------------------------------------
import rclpy
import websockets
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
# Convert ROS messages to/from CDR bytes (needed for services, which rclpy
# cannot handle in raw form like topics)
from rclpy.serialization import deserialize_message, serialize_message
# get_message("pkg/msg/Type") returns the Python class for a ROS 2 message type
# get_service("pkg/srv/Type") does the same for a service type
from rosidl_runtime_py.utilities import get_message, get_service


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

# Client -> server: 0x02 = "service call request"
OP_SERVICE_CALL_REQUEST = 0x02
# Server -> client: 0x03 = "service call response"
OP_SERVICE_CALL_RESPONSE = 0x03

# Layout of a service call request AND response (little-endian):
#   B = 1 byte  opcode
#   I = 4 bytes service id (the bridge's number for the service)
#   I = 4 bytes call id (our number, to match a response to its request)
#   I = 4 bytes length of the encoding string that follows
# followed by the encoding string (b"cdr") and then the CDR payload.
SERVICE_HEADER = struct.Struct("<BIII")
CDR = b"cdr"

# Every node has these parameter services; we use get_parameters to spot the
# remote nodes that need a local stand-in.
GET_PARAMETERS_SUFFIX = "/get_parameters"


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

        node.declare_parameter(
            "services_from_remote",
            [".*"],
        )

        node.declare_parameter(
            "param_nodes",
            True,
        )

        node.declare_parameter(
            "service_timeout",
            5.0,
        )

        # Read the final values (defaults or overrides) into plain attributes.
        self.url = node.get_parameter("url").value
        self.from_remote = node.get_parameter("from_remote").value
        self.to_remote_specs = node.get_parameter("to_remote").value
        self.local_prefix = node.get_parameter("local_prefix").value
        self.latched_topics = node.get_parameter("latched_topics").value
        self.services_from_remote = (
            node.get_parameter("services_from_remote").value
        )
        self.param_nodes_enabled = node.get_parameter("param_nodes").value
        self.service_timeout = node.get_parameter("service_timeout").value

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
        self.service_patterns = [
            re.compile(pattern)
            for pattern in self.services_from_remote
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

        # The WebSocket of the current connection (None while disconnected).
        # Service calls need it to send their request.
        self.ws = None

        # Service proxies. All of these are only touched from the asyncio
        # thread, so they need no lock.
        #   local service name -> (local rclpy service, bridge service id)
        self.local_srvs = {}
        #   bridge service id -> local service name
        self.remote_services = {}
        #   get_parameters local service name -> local stand-in node
        self.param_nodes = {}
        #   call id -> asyncio future that receives the response bytes
        self.pending_calls = {}
        # Unique call ids: 1, 2, 3, ...
        self.call_ids = itertools.count(1)

        # Proxy service callbacks wait (block) for the robot's answer. A
        # reentrant group lets the multi-threaded executor run several of them
        # at once, and keeps a slow call from blocking topic callbacks.
        self.service_group = ReentrantCallbackGroup()

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

            # Topics like /tf_static, /map or robot descriptions are
            # published once and "latched" (TRANSIENT_LOCAL) on the robot.
            # Their subscribers require TRANSIENT_LOCAL too: a VOLATILE
            # publisher is incompatible and they would receive nothing.
            # (Computed outside the `if` below: the log line needs it even
            # when the publisher already exists, e.g. after a reconnection.)
            latched = any(
                pattern.fullmatch(topic)
                for pattern in self.latched_patterns
            )

            # Create the local publisher only once, even if the bridge
            # re-advertises the same topic or we reconnect.
            if local_topic not in self.local_pubs:
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
        """Handle one binary frame received from the bridge.

        It is either one ROS message of a subscribed topic, or the answer to
        one of our service calls.
        """
        if not data:
            return

        if data[0] == OP_SERVICE_CALL_RESPONSE:
            self._handle_service_response(data)
            return

        # Ignore frames that are too short or of an unknown kind.
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
    # services (and parameters) remote -> local
    # ------------------------------------------------------------
    # Flow: bridge "advertiseServices" -> local proxy service
    #       local call -> proxy callback (rclpy thread) -> _call_remote
    #       (asyncio thread) -> WebSocket -> robot -> response frame ->
    #       _handle_service_response -> proxy callback returns the response

    def _service_wanted(self, name: str) -> bool:
        """Decide whether a remote service should be callable locally."""
        return any(
            pattern.fullmatch(name)
            for pattern in self.service_patterns
        )

    def _handle_advertise_services(self, services):
        """React to the bridge announcing services the robot offers."""
        # Services that exist locally without us. Mirroring one of them would
        # put two servers on the same name, and a caller would get whichever
        # answers first. Typical case: the same node runs on both machines,
        # so both have e.g. /joy/get_parameters.
        foreign_services = {
            name
            for name, _types in self.node.get_service_names_and_types()
            if name not in self.local_srvs
        }

        for s in services:
            name = s["name"]
            type_str = s["type"]
            local_name = self.local_prefix + name

            # Only CDR is forwarded (same as topics). Older bridges do not
            # give the encoding; it is CDR for ROS 2 then.
            encoding = s.get("request", {}).get("encoding", "cdr")
            if encoding != "cdr" or not self._service_wanted(name):
                continue

            # Same service advertised again (it can happen when the robot's
            # graph changes): just remember the new id.
            if local_name in self.local_srvs:
                srv, old_id = self.local_srvs[local_name]
                self.remote_services.pop(old_id, None)
                self.local_srvs[local_name] = (srv, s["id"])
                self.remote_services[s["id"]] = local_name
                continue

            if local_name in foreign_services:
                self.log.warning(
                    f"skip service {name}: {local_name} already exists "
                    f"locally (set local_prefix to mirror it)"
                )
                continue

            # Python class of the service type, e.g. std_srvs/srv/SetBool.
            # Skip the service if its package is not installed locally.
            try:
                srv_cls = get_service(type_str)
            except Exception as e:
                self.log.warning(
                    f"skip service {name}: cannot load {type_str} ({e})"
                )
                continue

            # `local_name=...` and `srv_cls=...` freeze the current values
            # (same late-binding pitfall as in _setup_local_subscriptions).
            def cb(request, response, local_name=local_name, srv_cls=srv_cls):
                # Runs in an rclpy executor thread. Start the remote call in
                # the asyncio loop and block this thread until it is done.
                future = asyncio.run_coroutine_threadsafe(
                    self._call_remote(
                        local_name,
                        serialize_message(request),
                    ),
                    self.loop,
                )
                try:
                    payload = future.result(timeout=self.service_timeout)
                    return deserialize_message(payload, srv_cls.Response)
                except concurrent.futures.TimeoutError:
                    future.cancel()
                    self.log.warning(
                        f"service {local_name}: no answer from the robot "
                        f"after {self.service_timeout}s"
                    )
                except Exception as e:
                    self.log.warning(f"service {local_name} failed: {e}")
                # ROS 2 services cannot return an error: send an empty
                # (default) response so the caller is not left hanging.
                return response

            srv = self.node.create_service(
                srv_cls,
                local_name,
                cb,
                callback_group=self.service_group,
            )
            self.local_srvs[local_name] = (srv, s["id"])
            self.remote_services[s["id"]] = local_name

            self.log.info(f"service remote -> local: {name} -> {local_name}")

            if (
                self.param_nodes_enabled
                and local_name.endswith(GET_PARAMETERS_SUFFIX)
            ):
                self._add_param_node(local_name)

    def _add_param_node(self, get_parameters_name: str):
        """Create an empty local node named like the remote node.

        `ros2 param` first checks that the node exists in the graph, and only
        then calls its parameter services. The stand-in node makes that check
        pass; the actual work is done by the forwarded parameter services.
        """
        # "/robot/controller/get_parameters" -> "/robot/controller"
        full_name = get_parameters_name[:-len(GET_PARAMETERS_SUFFIX)]
        namespace, _, name = full_name.rpartition("/")
        namespace = namespace or "/"

        local_nodes = {
            namespace_.rstrip("/") + "/" + name_
            for name_, namespace_ in self.node.get_node_names_and_namespaces()
        }
        if full_name in local_nodes:
            # A real local node already has that name (should not happen,
            # since its services would have blocked the forwarding).
            return

        try:
            self.param_nodes[get_parameters_name] = rclpy.create_node(
                name,
                namespace=namespace,
                context=self.node.context,
                # It must not create its own parameter services: they would
                # clash with the forwarded ones.
                start_parameter_services=False,
                enable_rosout=False,
            )
        except Exception as e:
            self.log.warning(f"cannot create stand-in node {full_name}: {e}")
            return

        self.log.info(f"parameters remote -> local: node {full_name}")

    def _remove_service(self, local_name: str):
        """Destroy a proxy service (and its stand-in node, if any)."""
        srv, remote_id = self.local_srvs.pop(local_name)
        self.remote_services.pop(remote_id, None)
        self.node.destroy_service(srv)

        param_node = self.param_nodes.pop(local_name, None)
        if param_node is not None:
            param_node.destroy_node()

    def _handle_unadvertise_services(self, service_ids):
        """The robot no longer offers these services: remove the proxies."""
        for service_id in service_ids:
            local_name = self.remote_services.get(service_id)
            if local_name is not None:
                self._remove_service(local_name)
                self.log.info(f"service removed: {local_name}")

    async def _call_remote(self, local_name: str, request: bytes) -> bytes:
        """Send one service request to the bridge and wait for the answer.

        Runs in the asyncio thread. Returns the CDR bytes of the response, or
        raises if the bridge reports a failure or the connection drops.
        """
        entry = self.local_srvs.get(local_name)
        if entry is None or self.ws is None:
            raise ConnectionError("not connected to the bridge")
        _srv, service_id = entry

        call_id = next(self.call_ids)
        future = self.loop.create_future()
        self.pending_calls[call_id] = future

        try:
            await self.ws.send(
                SERVICE_HEADER.pack(
                    OP_SERVICE_CALL_REQUEST,
                    service_id,
                    call_id,
                    len(CDR),
                ) + CDR + request
            )
            # Completed by _handle_service_response / serviceCallFailure.
            return await future
        finally:
            # Also runs on cancellation (timeout in the rclpy thread).
            self.pending_calls.pop(call_id, None)

    def _handle_service_response(self, data: bytes):
        """Hand a service response frame to the call waiting for it."""
        if len(data) < SERVICE_HEADER.size:
            return

        _, _service_id, call_id, encoding_len = (
            SERVICE_HEADER.unpack_from(data)
        )
        future = self.pending_calls.get(call_id)

        if future is not None and not future.done():
            # Skip the header and the encoding string; the rest is CDR.
            future.set_result(data[SERVICE_HEADER.size + encoding_len:])

    def _handle_service_failure(self, msg):
        """The bridge could not perform one of our calls."""
        future = self.pending_calls.get(msg.get("callId"))

        if future is not None and not future.done():
            future.set_exception(
                RuntimeError(msg.get("message", "service call failed"))
            )

    def _clear_services(self):
        """Connection lost: fail pending calls and remove every proxy.

        The bridge gives new service ids on the next connection and will
        advertise the services again, so the proxies are rebuilt then.
        """
        for future in self.pending_calls.values():
            if not future.done():
                future.set_exception(
                    ConnectionError("connection to the bridge lost")
                )
        self.pending_calls.clear()

        for local_name in list(self.local_srvs):
            self._remove_service(local_name)

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
        self.ws = ws

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

                elif op == "advertiseServices":
                    # The bridge lists the services the robot offers
                    # (including every node's parameter services).
                    self._handle_advertise_services(msg["services"])

                elif op == "unadvertiseServices":
                    self._handle_unadvertise_services(msg["serviceIds"])

                elif op == "serviceCallFailure":
                    self._handle_service_failure(msg)

                elif op == "status":
                    # Informational messages and errors from the bridge.
                    self.log.info(
                        f"bridge status: "
                        f"{msg.get('message')}"
                    )

        finally:
            # Connection over: stop the sender task so it doesn't linger.
            sender.cancel()
            self.ws = None
            self._clear_services()


def main(args=None):
    # Initialize the ROS 2 client library, then create the node.
    # rclpy's own Ctrl+C handler only shuts ROS down: the asyncio loop would
    # keep running and the program would never exit. Without it, Python raises
    # KeyboardInterrupt in the main thread and the shutdown code below runs.
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )

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

    # ROS 2 callbacks (local subscriptions, proxy services) need an executor
    # spinning. Multi-threaded, because a proxy service callback blocks while
    # it waits for the robot's answer. Spinning blocks, so it lives in its own
    # daemon thread (daemon = dies with the program), while the main thread
    # runs the asyncio loop below.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(
        target=executor.spin,
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
        executor.shutdown()
        loop.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
