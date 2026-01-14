"""
Simple XYZ/TMS map tile server backed by Project AirSim imagery.

Example:
  python map_tile_server.py --port 8080
  http://127.0.0.1:8080/tiles/{z}/{x}/{y}.png
"""

import argparse
import math
import threading
from collections import OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

from projectairsim import ProjectAirSimClient, Drone
from projectairsim.types import ImageType, Quaternion
from projectairsim.utils import load_scene_config_as_dict, projectairsim_log, unpack_image

EARTH_RADIUS_M = 6378137.0


class TileCache:
    def __init__(self, max_items: int):
        self.max_items = max_items
        self._lock = threading.Lock()
        self._data = OrderedDict()

    def get(self, key):
        with self._lock:
            value = self._data.get(key)
            if value is not None:
                self._data.move_to_end(key)
            return value

    def set(self, key, value):
        with self._lock:
            self._data[key] = value
            self._data.move_to_end(key)
            if len(self._data) > self.max_items:
                self._data.popitem(last=False)


class SceneConfigWorld:
    def __init__(self, scene_config_path: str):
        config_dict, _ = load_scene_config_as_dict(scene_config_path, sim_config_path="")
        self.scene_config = config_dict
        scene_id = config_dict.get("id", "SceneBasicDrone")
        self.parent_topic = f"/Sim/{scene_id}"
        self.home_geo_point = config_dict.get("home-geo-point", {})

    def get_configuration(self):
        return self.scene_config


def tile_xy_to_latlon(tile_x: float, tile_y: float, zoom: int):
    n = 2.0 ** zoom
    lon = tile_x / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * tile_y / n)))
    return math.degrees(lat_rad), lon


def meters_per_pixel(lat_deg: float, zoom: int):
    return (
        math.cos(math.radians(lat_deg))
        * 2.0
        * math.pi
        * EARTH_RADIUS_M
        / (256 * (2**zoom))
    )


def tile_size_meters(lat_deg: float, zoom: int, tile_size_px: int):
    return meters_per_pixel(lat_deg, zoom) * tile_size_px


def compute_camera_height_m(
    lat_deg: float, zoom: int, tile_size_px: int, fov_deg: float, height_margin: float
):
    tile_size_m = tile_size_meters(lat_deg, zoom, tile_size_px)
    fov_rad = math.radians(fov_deg)
    if fov_rad <= 0.0:
        raise ValueError("Camera FOV must be > 0")
    height = (tile_size_m * 0.5) / math.tan(fov_rad * 0.5)
    return max(height * height_margin, 1.0)


def find_camera_settings(scene_config, robot_name: str, camera_id: str):
    for actor in scene_config.get("actors", []):
        if actor.get("name") != robot_name:
            continue
        robot_config = actor.get("robot-config", {})
        for sensor in robot_config.get("sensors", []):
            if sensor.get("type") != "camera":
                continue
            if sensor.get("id") != camera_id:
                continue
            settings = sensor.get("capture-settings", [])
            for setting in settings:
                if setting.get("image-type") == 0:
                    return (
                        setting.get("width"),
                        setting.get("height"),
                        setting.get("fov-degrees"),
                    )
    raise ValueError(f"Camera '{camera_id}' not found on actor '{robot_name}'.")


def normalize_image(image: np.ndarray, tile_size_px: int):
    if image is None or image.size == 0:
        raise ValueError("Empty image data received from simulator.")
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if image.shape[2] > 3:
        image = image[:, :, :3]
    if image.shape[0] != image.shape[1]:
        min_dim = min(image.shape[0], image.shape[1])
        start_y = (image.shape[0] - min_dim) // 2
        start_x = (image.shape[1] - min_dim) // 2
        image = image[start_y : start_y + min_dim, start_x : start_x + min_dim]
    if image.shape[0] != tile_size_px:
        image = cv2.resize(image, (tile_size_px, tile_size_px), interpolation=cv2.INTER_AREA)
    return image


class MapTileService:
    def __init__(
        self,
        scene_config_path: str,
        robot_name: str,
        camera_id: str,
        tile_size_px: int,
        height_margin: float,
        min_altitude_m: float,
        projection_mode: str,
        cache_size: int,
        tms_default: bool,
        sim_address: str,
        topics_port: int,
        services_port: int,
    ):
        self.scene_config_path = scene_config_path
        self.robot_name = robot_name
        self.camera_id = camera_id
        self.tile_size_px = tile_size_px
        self.height_margin = height_margin
        self.min_altitude_m = min_altitude_m
        self.projection_mode = projection_mode
        self.tms_default = tms_default
        self.cache = TileCache(cache_size)
        self._capture_lock = threading.Lock()
        self._projection_initialized = False
        self._last_ortho_width = None

        self.client = ProjectAirSimClient(
            address=sim_address, port_topics=topics_port, port_services=services_port
        )
        self.client.connect()

        self.world = SceneConfigWorld(scene_config_path)
        self.home_geo_point = self.world.home_geo_point
        self.drone = Drone(self.client, self.world, robot_name)

        self.camera_width, self.camera_height, self.camera_fov = find_camera_settings(
            self.world.get_configuration(), robot_name, camera_id
        )
        if (
            self.camera_width is not None
            and self.camera_height is not None
            and (
                self.camera_width < self.tile_size_px
                or self.camera_height < self.tile_size_px
            )
        ):
            projectairsim_log().warning(
                "Map camera resolution is smaller than tile size; tiles may be upscaled."
            )

        if not self.home_geo_point:
            raise ValueError("home-geo-point missing from scene config.")

        if self.camera_fov is None and self.projection_mode == "perspective":
            raise ValueError("Camera fov-degrees missing from config.")

        self.identity_rotation = Quaternion({"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0})

    def shutdown(self):
        self.client.disconnect()

    def _ensure_projection_settings(self, ortho_width_m: float):
        if self.projection_mode == "orthographic":
            if not self._projection_initialized:
                self.drone.set_projection_mode(
                    self.camera_id, ImageType.SCENE, projection_mode=1
                )
                self._projection_initialized = True
            if (
                self._last_ortho_width is None
                or abs(self._last_ortho_width - ortho_width_m) > 0.01
            ):
                self.drone.set_ortho_width(
                    self.camera_id, ImageType.SCENE, ortho_width=ortho_width_m
                )
                self._last_ortho_width = ortho_width_m
        else:
            if not self._projection_initialized:
                self.drone.set_projection_mode(
                    self.camera_id, ImageType.SCENE, projection_mode=0
                )
                self._projection_initialized = True

    def render_tile_png(self, zoom: int, x: int, y: int, tms: bool):
        cache_key = (zoom, x, y, tms)
        cached = self.cache.get(cache_key)
        if cached is not None:
            return cached

        with self._capture_lock:
            if tms:
                y = (2**zoom - 1 - y)

            lat, lon = tile_xy_to_latlon(x + 0.5, y + 0.5, zoom)
            tile_width_m = tile_size_meters(lat, zoom, self.tile_size_px)
            self._ensure_projection_settings(tile_width_m)

            if self.projection_mode == "orthographic":
                height_m = max(tile_width_m, self.min_altitude_m)
            else:
                height_m = compute_camera_height_m(
                    lat, zoom, self.tile_size_px, self.camera_fov, self.height_margin
                )

            altitude_m = self.home_geo_point["altitude"] + height_m

            self.drone.set_geo_pose(lat, lon, altitude_m, self.identity_rotation)

            images = self.drone.get_images(self.camera_id, [ImageType.SCENE])
            image = unpack_image(images[ImageType.SCENE])
            image = normalize_image(image, self.tile_size_px)

            success, encoded = cv2.imencode(".png", image)
            if not success:
                raise RuntimeError("Failed to encode PNG.")

            png_bytes = encoded.tobytes()
            self.cache.set(cache_key, png_bytes)
            return png_bytes


class TileRequestHandler(BaseHTTPRequestHandler):
    server_version = "ProjectAirSimTileServer/0.1"

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path in ("/", "/health"):
            self._write_text(200, "OK")
            return

        try:
            tile = self._parse_tile_path(parsed.path)
            if tile is None:
                self._write_text(404, "Not Found")
                return

            zoom, x, y = tile
            params = parse_qs(parsed.query)
            tms = self.server.tile_service.tms_default
            if "tms" in params:
                tms_val = params["tms"][0].lower()
                tms = tms_val in ("1", "true", "yes")

            png_bytes = self.server.tile_service.render_tile_png(
                zoom=zoom, x=x, y=y, tms=tms
            )
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Length", str(len(png_bytes)))
            self.end_headers()
            try:
                self.wfile.write(png_bytes)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                return
        except Exception as exc:
            if isinstance(
                exc, (BrokenPipeError, ConnectionResetError, ConnectionAbortedError)
            ):
                return
            projectairsim_log().error(f"Tile request failed: {exc}")
            try:
                self._write_text(500, "Tile generation failed")
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                return

    def log_message(self, format, *args):
        return

    def _parse_tile_path(self, path: str):
        parts = [p for p in path.split("/") if p]
        if not parts:
            return None
        if parts[0].lower() == "tiles":
            parts = parts[1:]
        if len(parts) < 3:
            return None
        zoom_str, x_str, y_str = parts[0], parts[1], parts[2]
        if y_str.endswith(".png"):
            y_str = y_str[:-4]
        try:
            return int(zoom_str), int(x_str), int(y_str)
        except ValueError:
            return None

    def _write_text(self, status: int, message: str):
        encoded = message.encode("utf-8")
        try:
            self.send_response(status)
            self.send_header("Content-Type", "text/plain")
            self.send_header("Content-Length", str(len(encoded)))
            self.end_headers()
            self.wfile.write(encoded)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            return


class TileHTTPServer(ThreadingHTTPServer):
    def __init__(self, server_address, RequestHandlerClass, tile_service: MapTileService):
        super().__init__(server_address, RequestHandlerClass)
        self.tile_service = tile_service


def parse_args():
    default_scene = (
        Path(__file__).resolve().parent / "sim_config" / "scene_ardu_quadrotor.jsonc"
    )
    parser = argparse.ArgumentParser(description="Project AirSim map tile server.")
    parser.add_argument(
        "--scene-config",
        default=str(default_scene),
        help="Path to the scene config JSONC used by the sim.",
    )
    parser.add_argument(
        "--robot-name",
        default="MapCameraRig",
        help="Name of the map camera actor in the scene config.",
    )
    parser.add_argument(
        "--camera-id",
        default="MapCamera",
        help="Camera sensor id to use for tiles.",
    )
    parser.add_argument("--port", type=int, default=8080, help="HTTP port to serve.")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind address.")
    parser.add_argument(
        "--sim-address",
        default="127.0.0.1",
        help="Project AirSim server address.",
    )
    parser.add_argument(
        "--topics-port",
        type=int,
        default=8989,
        help="Project AirSim topics port.",
    )
    parser.add_argument(
        "--services-port",
        type=int,
        default=8990,
        help="Project AirSim services port.",
    )
    parser.add_argument("--tile-size", type=int, default=256, help="Tile size in px.")
    parser.add_argument(
        "--height-margin",
        type=float,
        default=1.05,
        help="Scale height to avoid tile edge clipping.",
    )
    parser.add_argument(
        "--min-altitude",
        type=float,
        default=50.0,
        help="Minimum camera height in meters (used for orthographic projection).",
    )
    parser.add_argument(
        "--projection",
        choices=["perspective", "orthographic"],
        default="orthographic",
        help="Camera projection mode for tiles.",
    )
    parser.add_argument(
        "--cache-size",
        type=int,
        default=256,
        help="Max number of tiles to keep in memory.",
    )
    parser.add_argument(
        "--tms",
        action="store_true",
        help="Use TMS Y-flip by default (override with ?tms=0).",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    tile_service = MapTileService(
        scene_config_path=args.scene_config,
        robot_name=args.robot_name,
        camera_id=args.camera_id,
        tile_size_px=args.tile_size,
        height_margin=args.height_margin,
        min_altitude_m=args.min_altitude,
        projection_mode=args.projection,
        cache_size=args.cache_size,
        tms_default=args.tms,
        sim_address=args.sim_address,
        topics_port=args.topics_port,
        services_port=args.services_port,
    )
    server = TileHTTPServer((args.host, args.port), TileRequestHandler, tile_service)
    projectairsim_log().info(
        f"Serving tiles at http://{args.host}:{args.port}/tiles/{{z}}/{{x}}/{{y}}.png"
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        tile_service.shutdown()


if __name__ == "__main__":
    main()
