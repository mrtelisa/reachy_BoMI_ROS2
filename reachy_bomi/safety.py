#!/usr/bin/env python3
"""Quit/shutdown safety net for the whole pipeline (bomi_teleop.py,
reachy_detection.py, reachy_control.py, tests/test_grasp.py) -- no
dependencies on any of them, so it can be imported from all without risk
of circular imports.

Two layers, because a single cv2.waitKey-based check only ever fires while
a cv2 window has OS focus and the caller's own loop is actively polling
it -- neither holds while e.g. a matplotlib plot has focus, or the main
thread is blocked inside arm.goto(wait=True):
  - quit_requested: local, in-loop check (Q/ESC while a cv2 window has
    focus, or that window closed) for graceful "go back a step" UI flow.
  - start_global_quit_watcher / start_terminal_quit_watcher: OS-level
    watchers on a background thread, so ESC/Q stops the robot regardless
    of what the main thread is doing or which window has focus.
"""

import os
import select
import sys
import termios
import threading
import tty

import cv2
from reachy2_sdk import ReachySDK


def quit_requested(key: int, window_name: str) -> bool:
    """True if Q/ESC was pressed, or the window was closed with the X button."""
    if key in (ord('q'), ord('Q'), 27):
        return True
    try:
        return cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1
    except cv2.error:
        return False


def start_global_quit_watcher(on_quit):
    """Hooks ESC/Q system-wide via pynput, so quitting works with no cv2
    window focused. Returns the listener, or None if pynput is unavailable."""
    try:
        from pynput import keyboard
    except ImportError:
        print("[WARN] pynput not installed: ESC/Q only quits with a window focused.")
        return None

    def _on_press(key) -> None:
        if key == keyboard.Key.esc or getattr(key, "char", None) in ("q", "Q"):
            on_quit()

    try:
        listener = keyboard.Listener(on_press=_on_press)
        listener.daemon = True
        listener.start()
        return listener
    except Exception as exc:
        print(f"[WARN] Could not start the global ESC/Q watcher ({exc}).")
        return None


def start_terminal_quit_watcher(on_quit):
    """Watches this process's own terminal for ESC/Q, for when the terminal
    (not a cv2 window, not the global pynput hook under Wayland) is what
    actually has keyboard focus. Returns a stop() to restore the terminal,
    or None if stdin isn't an interactive terminal."""
    if not sys.stdin.isatty():
        return None

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    stop_flag = threading.Event()

    def _restore() -> None:
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except Exception:
            pass

    def _watch() -> None:
        tty.setcbreak(fd)
        try:
            while not stop_flag.is_set():
                ready, _, _ = select.select([sys.stdin], [], [], 0.2)
                if ready and sys.stdin.read(1) in ("q", "Q", "\x1b"):
                    _restore()
                    on_quit()
                    return
        finally:
            _restore()

    threading.Thread(target=_watch, daemon=True).start()

    def stop() -> None:
        stop_flag.set()
        _restore()

    return stop


def safe_robot_shutdown(reachy: ReachySDK, mobile_base=None) -> None:
    """Stop the base, then power down smoothly (falls back to a hard
    turn_off). Swallows exceptions since this also runs on the emergency
    quit path, where raising would block the process from exiting."""
    if mobile_base is not None:
        try:
            mobile_base.set_goal_speed(vx=0, vy=0, vtheta=0)
            mobile_base.send_speed_command()
        except Exception:
            pass

    try:
        reachy.turn_off_smoothly()
    except Exception:
        try:
            reachy.turn_off()
        except Exception:
            pass

    if mobile_base is not None:
        try:
            mobile_base.turn_off()
        except Exception:
            pass


def emergency_shutdown(reachy: ReachySDK, mobile_base=None) -> None:
    """safe_robot_shutdown + disconnect + close every cv2 window, then a
    hard process exit. Meant as the on_quit callback for the watchers
    above, so ESC/Q stops the robot no matter what the main thread is
    currently blocked doing (a plt.pause() loop, an arm.goto(wait=True), ...)."""
    print("\n[QUIT] ESC/Q pressed — stopping the robot and exiting.")
    try:
        safe_robot_shutdown(reachy, mobile_base)
        reachy.disconnect()
        cv2.destroyAllWindows()
    finally:
        os._exit(0)
