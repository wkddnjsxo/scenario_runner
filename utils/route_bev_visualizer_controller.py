import os
import subprocess
import sys


class RouteBEVVisualizerController:
    def __init__(
        self,
        *,
        workspace_root,
        runner_root,
        route_json,
        use_imshow=True,
        publish_image=True,
        window_title="AIM Route",
        state_source="grpc",
        grpc_host="127.0.0.1",
        grpc_port=7789,
        grpc_client_key="aim_scenario_runner",
        grpc_src=None,
        python_executable=None,
    ):
        self.workspace_root = os.path.abspath(workspace_root)
        self.runner_root = os.path.abspath(runner_root)
        self.route_json = os.path.abspath(route_json)
        self.use_imshow = bool(use_imshow)
        self.publish_image = bool(publish_image)
        self.window_title = str(window_title)
        self.state_source = str(state_source).lower()
        self.grpc_host = str(grpc_host)
        self.grpc_port = int(grpc_port)
        self.grpc_client_key = str(grpc_client_key)
        if grpc_src is None:
            runner_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            grpc_src = os.path.join(runner_root, "grpc_inha_univ", "src")
        self.grpc_src = os.path.abspath(os.path.expanduser(os.path.expandvars(grpc_src)))
        self.python_executable = python_executable or sys.executable
        self.process = None

    def start(self):
        if self.is_running():
            return

        script = os.path.join(
            self.workspace_root,
            "src",
            "learning_by_cheating",
            "scripts",
            "lbc_bev_visualizer.py",
        )
        cmd = [
            self.python_executable,
            script,
            f"_aim_ws_root:={self.workspace_root}",
            f"_route_overlay_enabled:=true",
            f"_route_json:={self.route_json}",
            f"_window_title:={self.window_title}",
            f"_use_imshow:={'true' if self.use_imshow else 'false'}",
            f"_draw_ego_imshow:=true",
            f"_publish_images:={'true' if self.publish_image else 'false'}",
            f"_save_snapshots:=false",
            f"_skip_overrun_frames:=true",
            f"_pixels_ahead:=0",
            f"_filter_objects_radius_m:=80",
            f"_object_bev_margin_px:=80",
            f"_max_vehicles:=80",
            f"_max_pedestrians:=40",
            f"_state_source:={self.state_source}",
            f"_grpc_host:={self.grpc_host}",
            f"_grpc_port:={self.grpc_port}",
            f"_grpc_client_key:={self.grpc_client_key}",
            f"_grpc_src:={self.grpc_src}",
        ]

        env = os.environ.copy()
        self.process = subprocess.Popen(cmd, cwd=self.workspace_root, env=env)
        print(
            f"[LBCBEV-Viz] visualizer started: pid={self.process.pid}, "
            f"route={self.route_json}"
        )

    def stop(self):
        if self.process is None:
            return

        if self.process.poll() is None:
            print(f"[LBCBEV-Viz] stopping visualizer: pid={self.process.pid}")
            self.process.terminate()
            try:
                self.process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3.0)

        self.process = None

    def is_running(self):
        return self.process is not None and self.process.poll() is None
