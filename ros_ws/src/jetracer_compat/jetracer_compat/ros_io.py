# -*- coding: utf-8 -*-
"""rclpy のノードを 1 つだけ作り、バックグラウンドで spin する (Jupyter からも使えるように)。

spin は spin_once のループにして、終了時 (atexit) にループを止めてスレッドを join してから rclpy を閉じる。
spin 中のスレッドを残したまま終了すると C++ 側で "terminate called without an active exception" になる。
"""
import atexit
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor

_lock = threading.Lock()
_node = None
_exec = None
_thread = None
_running = False


def _spin():
    while _running and rclpy.ok():
        try:
            _exec.spin_once(timeout_sec=0.05)
        except Exception:  # noqa: BLE001  (終了処理中の例外は無視)
            if not _running:
                break
            raise


def node():
    global _node, _exec, _thread, _running
    with _lock:
        if _node is None:
            if not rclpy.ok():
                rclpy.init()
            _node = rclpy.create_node('jetracer_compat')
            _exec = SingleThreadedExecutor()
            _exec.add_node(_node)
            _running = True
            _thread = threading.Thread(target=_spin, daemon=True)
            _thread.start()
            atexit.register(shutdown)
        return _node


def shutdown():
    global _node, _exec, _thread, _running
    with _lock:
        _running = False
        if _thread is not None:
            _thread.join(timeout=1.0)
        if _exec is not None:
            _exec.shutdown(timeout_sec=0.5)
        if _node is not None:
            _node.destroy_node()
        _node = _exec = _thread = None
    rclpy.try_shutdown()
