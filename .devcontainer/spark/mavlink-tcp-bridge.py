#!/usr/bin/env python3
"""MAVLink UDP <-> TCP bridge for QGroundControl on the developer's laptop.

With bridge networking the sim's MAVLink never leaves the dev container,
and a VS Code / SSH port forward carries TCP only. So this bridge:

  * owns UDP 127.0.0.1:14550, where PX4 SITL sends its GCS stream (from
    port 18570, see /opt/px4-gazebo/etc/init.d-posix/px4-rc.mavlink), and
    where `track` sends its ROI messages for QGC;
  * serves a TCP stream on MAVLINK_TCP_PORT (default 5760) that VS Code
    forwards to the laptop. QGC: Comm Links -> TCP -> 127.0.0.1:5760.

Design points, each one a bug the earlier socat bridge had:

  * Listens dual-stack (IPv4 + IPv6). "localhost" often resolves to ::1
    first, both for VS Code's forwarder in here and for QGC on Windows.
  * One long-lived UDP socket, independent of TCP clients. Every datagram
    goes to every connected client, so a stale connection (half-open
    through the tunnel) can't starve the next one, and QGC reconnecting
    just works.
  * Client -> vehicle bytes go to PX4's GCS port only. MAVLink is
    self-framing and PX4 parses its UDP input as a byte stream, so TCP
    segment boundaries don't matter.
  * Slow clients get a bounded buffer and are dropped when it overflows,
    instead of stalling everyone else.

Stdlib only; run under a restart loop by start-desktop.sh. Logs to stdout.
"""
import os
import selectors
import socket
import time

TCP_PORT = int(os.environ.get("MAVLINK_TCP_PORT", "5760"))
UDP_PORT = int(os.environ.get("MAVLINK_UDP_PORT", "14550"))
PX4 = ("127.0.0.1", int(os.environ.get("PX4_GCS_PORT", "18570")))
MAX_BUFFER = 1 << 20  # per-client backlog before the client is dropped


def log(msg):
    print(time.strftime("%Y-%m-%dT%H:%M:%S"), msg, flush=True)


def tcp_listener(port):
    """Dual-stack listener; falls back to IPv4 if IPv6 is unavailable."""
    try:
        s = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
        s.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("::", port))
        stack = "IPv4+IPv6"
    except OSError:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", port))
        stack = "IPv4 only"
    s.listen(8)
    s.setblocking(False)
    return s, stack


def main():
    sel = selectors.DefaultSelector()

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp.bind(("127.0.0.1", UDP_PORT))
    udp.setblocking(False)
    sel.register(udp, selectors.EVENT_READ, "udp")

    listener, stack = tcp_listener(TCP_PORT)
    sel.register(listener, selectors.EVENT_READ, "listen")
    log(f"bridge up: udp 127.0.0.1:{UDP_PORT} <-> tcp :{TCP_PORT} ({stack}), "
        f"vehicle at {PX4[0]}:{PX4[1]}")

    clients = {}  # socket -> (peer label, pending bytes)
    px4_seen = False

    def drop(c, why):
        label, _ = clients.pop(c)
        sel.unregister(c)
        c.close()
        log(f"client {label} gone ({why}); {len(clients)} connected")

    def flush(c):
        label, buf = clients[c]
        try:
            n = c.send(buf)
            del buf[:n]
        except BlockingIOError:
            pass
        except OSError as e:
            drop(c, e.strerror or type(e).__name__)
            return
        events = selectors.EVENT_READ | (selectors.EVENT_WRITE if buf else 0)
        sel.modify(c, events, "client")

    while True:
        for key, mask in sel.select(1.0):
            tag = key.data

            if tag == "listen":
                try:
                    c, addr = listener.accept()
                except OSError:
                    continue
                c.setblocking(False)
                c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                c.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                label = f"{addr[0]}:{addr[1]}"
                clients[c] = (label, bytearray())
                sel.register(c, selectors.EVENT_READ, "client")
                log(f"client {label} connected; {len(clients)} connected"
                    + ("" if px4_seen else " (no data from PX4 yet: is `fly` running?)"))

            elif tag == "udp":
                while True:
                    try:
                        data, src = udp.recvfrom(65535)
                    except (BlockingIOError, InterruptedError):
                        break
                    except OSError:
                        break
                    if src == PX4 and not px4_seen:
                        px4_seen = True
                        log("PX4 is sending")
                    for c in list(clients):
                        if c not in clients:
                            continue
                        buf = clients[c][1]
                        if len(buf) + len(data) > MAX_BUFFER:
                            drop(c, "not reading, backlog full")
                            continue
                        buf += data
                        flush(c)

            elif tag == "client":
                c = key.fileobj
                if c not in clients:
                    continue
                if mask & selectors.EVENT_READ:
                    try:
                        data = c.recv(65536)
                    except BlockingIOError:
                        data = None
                    except OSError as e:
                        drop(c, e.strerror or type(e).__name__)
                        continue
                    if data == b"":
                        drop(c, "closed")
                        continue
                    if data:
                        try:
                            udp.sendto(data, PX4)
                        except OSError:
                            pass  # PX4 not up yet; QGC retries on its own
                if c in clients and mask & selectors.EVENT_WRITE:
                    flush(c)


if __name__ == "__main__":
    main()
