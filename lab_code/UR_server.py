from __future__ import annotations

import json
import math
import multiprocessing as mp
import socket
import threading
import time
import os
from dataclasses import dataclass
from typing import Any, Optional, Tuple


@dataclass(frozen=True)
class URControllerConfig:
    # UDP Server Configuration
    udp_ip: str = "0.0.0.0"
    udp_port: int = 5005

    # Robot Configuration
    robot_ip: str = "192.168.50.168"
    acceleration: float = 40.0
    hz: float = 125.0

    # Client Connection Management
    client_timeout_s: float = 2.0

    # Joint velocity clamp (rad/s)
    qdot_limit: float = math.pi

    # UDP socket recv timeout; keep smaller than dt
    socket_timeout_s: float = 0.01


class URControllerProcess:
    """Run the UR RTDE velocity server in a separate process (Windows-safe spawn).

    Typical usage from another program:

        from UR_controller import URControllerProcess

        server = URControllerProcess(auto_start=True)
        # ... start other programs / threads ...
        server.join()  # optional
    """

    def __init__(
        self,
        config: Optional[URControllerConfig] = None,
        *,
        auto_start: bool = True,
        daemon: bool = True,
        exit_on_child_exit: bool = True,
    ) -> None:
        self.config = config or URControllerConfig()
        self._ctx = mp.get_context("spawn")
        self._stop_event = self._ctx.Event()
        self._process: Optional[mp.Process] = None
        self._daemon = daemon
        self._exit_on_child_exit = exit_on_child_exit
        self._watchdog_thread: Optional[threading.Thread] = None

        if auto_start:
            self.start()
            time.sleep(3)

    @property
    def pid(self) -> Optional[int]:
        return self._process.pid if self._process is not None else None

    @property
    def exitcode(self) -> Optional[int]:
        return self._process.exitcode if self._process is not None else None

    def start(self) -> None:
        if self._process is not None and self._process.is_alive():
            return

        self._stop_event.clear()
        self._process = self._ctx.Process(
            target=_child_main,
            args=(self.config, self._stop_event),
            daemon=self._daemon,
        )
        self._process.start()

        if self._exit_on_child_exit:
            self._start_watchdog()


    def _start_watchdog(self) -> None:
        if self._process is None:
            return
        if self._watchdog_thread is not None and self._watchdog_thread.is_alive():
            return

        def _watch() -> None:
            assert self._process is not None
            self._process.join()

            # If we requested stop, don't kill the main program.
            if self._stop_event.is_set():
                return

            exitcode = self._process.exitcode
            print(f"\n[URControllerProcess] Child process exited (exitcode={exitcode}); exiting main process.")
            os._exit(1 if exitcode is None else int(exitcode))

        self._watchdog_thread = threading.Thread(target=_watch, daemon=True)
        self._watchdog_thread.start()

    def stop(self, timeout_s: float = 2.0) -> None:
        self._stop_event.set()
        if self._process is None:
            return
        self._process.join(timeout=timeout_s)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=timeout_s)

    def join(self, timeout_s: Optional[float] = None) -> Optional[int]:
        if self._process is None:
            return None
        self._process.join(timeout=timeout_s)
        return self._process.exitcode


def _fatal_rtde_error(context: str, exc: Exception) -> None:
    # RTDE 报错通常意味着与 UR 的连接已断开或不可用；按需求直接终止（子进程）。
    print(f"\n[RTDE ERROR] {context}: {exc}")
    raise SystemExit(1)


def _rtde_health_check(rtde_c: Any) -> None:
    """Best-effort RTDE health checks (depends on ur_rtde version)."""
    try:
        if rtde_c is None:
            _fatal_rtde_error("RTDE control interface is None", RuntimeError("rtde_c is None"))
        if hasattr(rtde_c, "isConnected") and (not rtde_c.isConnected()):
            _fatal_rtde_error("RTDE control interface disconnected", RuntimeError("isConnected() == False"))
        if hasattr(rtde_c, "isProgramRunning") and (not rtde_c.isProgramRunning()):
            _fatal_rtde_error(
                "RTDE control script not running",
                RuntimeError("isProgramRunning() == False"),
            )
    except SystemExit:
        raise
    except Exception as e:
        _fatal_rtde_error("RTDE health check failed", e)


def _child_main(config: URControllerConfig, stop_event: mp.synchronize.Event) -> None:
    # Import RTDE modules inside the child process (Windows spawn-safe).
    import rtde_control
    import rtde_receive

    dt = 1.0 / float(config.hz)
    q_range = (-float(config.qdot_limit), float(config.qdot_limit))

    active_client: Optional[Tuple[str, int]] = None
    last_received_time: Optional[float] = None
    qdot = [0.0] * 6

    print(f"Connecting to robot at {config.robot_ip}...")
    try:
        rtde_c = rtde_control.RTDEControlInterface(config.robot_ip)
        rtde_r = rtde_receive.RTDEReceiveInterface(config.robot_ip)
        print("Robot connected successfully")
    except Exception as e:
        _fatal_rtde_error("Failed to connect to robot", e)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((config.udp_ip, int(config.udp_port)))
    sock.settimeout(float(config.socket_timeout_s))

    print(f"UDP Server listening on {config.udp_ip}:{config.udp_port}")
    print("Waiting for velocity commands...")
    print("Expected format: [q1, q2, q3, q4, q5, q6] (JSON array)")

    try:
        while not stop_event.is_set():
            _rtde_health_check(rtde_c)

            try:
                t_start = rtde_c.initPeriod()
                if t_start is None:
                    _fatal_rtde_error("initPeriod returned None", RuntimeError("initPeriod() returned None"))
            except Exception as e:
                _fatal_rtde_error("initPeriod failed (connection lost?)", e)

            current_time = time.time()

            # Check client timeout
            if active_client is not None and last_received_time is not None:
                if current_time - last_received_time > float(config.client_timeout_s):
                    print(f"Client {active_client} timed out, releasing connection")
                    active_client = None
                    last_received_time = None
                    qdot = [0.0] * 6

            # UDP receive
            try:
                data, addr = sock.recvfrom(1024)

                if active_client is None:
                    active_client = addr
                    last_received_time = current_time
                    print(f"New client connected: {addr}")
                elif active_client != addr:
                    print(f"Rejected connection from {addr} - Client {active_client} is already connected")
                else:
                    received_qdot = json.loads(data.decode("utf-8"))
                    if isinstance(received_qdot, list) and len(received_qdot) == 6:
                        qdot = received_qdot
                        last_received_time = current_time
                    else:
                        print(f"Invalid data format from {addr}: {data}")
            except socket.timeout:
                pass
            except json.JSONDecodeError:
                print(f"JSON decode error: {data}")
            except Exception:
                pass

            # Speed control
            qdot = [max(q_range[0], min(q_range[1], float(v))) for v in qdot]
            try:
                ok = rtde_c.speedJ(qdot, float(config.acceleration), dt)
                # Some failures return False and only print: "RTDE control script is not running!"
                if ok is False:
                    _fatal_rtde_error(
                        "speedJ returned False (RTDE control script is not running?)",
                        RuntimeError("speedJ() returned False"),
                    )
            except Exception as e:
                _fatal_rtde_error("speedJ failed (connection lost?)", e)

            # Feedback to active client (UDP issues are non-fatal)
            if active_client is not None:
                try:
                    q_current = rtde_r.getActualQ()
                    tcp_pose = rtde_r.getActualTCPPose()
                except Exception as e:
                    _fatal_rtde_error("RTDE receive failed (connection lost?)", e)

                response_data = {"q": q_current, "tcp_pose": tcp_pose}
                try:
                    sock.sendto(json.dumps(response_data).encode("utf-8"), active_client)
                except Exception as e:
                    print(f"Error sending feedback to client: {e}")

            try:
                ok = rtde_c.waitPeriod(t_start)
                if ok is False:
                    _fatal_rtde_error("waitPeriod returned False", RuntimeError("waitPeriod() returned False"))
            except Exception as e:
                _fatal_rtde_error("waitPeriod failed (connection lost?)", e)

    except KeyboardInterrupt:
        print("\nReceived interrupt, stopping robot...")
    finally:
        print("Stopping robot...")
        try:
            if "rtde_c" in locals() and rtde_c is not None:
                rtde_c.speedStop()
                rtde_c.stopScript()
        except Exception as e:
            print(f"Cleanup warning (RTDE stop): {e}")
        try:
            sock.close()
        except Exception:
            pass
        print("Server shutdown complete")


if __name__ == "__main__":
    # When run directly, still start the server, but in a child process (so this file
    # can be imported safely without side effects).
    server = URControllerProcess(auto_start=True, daemon=False)
    server.join()