from __future__ import annotations

from typing import Any, Iterable

import mujoco as mj
import numpy as np
from mjviser.scene import ViserMujocoScene
from viser import ViserServer

from .params import resolve_robot_xml_path


class MJViserViewer:
    """Browser viewer for the retargeted robot and source human pose."""

    def __init__(
        self,
        target_robot: str,
        *,
        host: str = "0.0.0.0",
        port: int = 8080,
    ) -> None:
        self.target_robot = str(target_robot).strip().lower()
        self.model = mj.MjModel.from_xml_path(str(resolve_robot_xml_path(self.target_robot)))
        self._set_robot_opacity(0.35)
        self.data = mj.MjData(self.model)
        self._qpos_template = np.asarray(self.model.qpos0, dtype=np.float64).copy()

        self.server = ViserServer(
            host=str(host),
            port=int(port),
            label=f"teleop-{self.target_robot}",
            verbose=True,
        )
        self.mj_scene = ViserMujocoScene(self.server, self.model, num_envs=1)
        self.mj_scene.create_visualization_gui(camera_distance=3.0)

        self._body_id_cache: dict[tuple[str, ...], np.ndarray] = {}
        self._axes_handles: dict[str, Any] = {}
        self._point_handles: dict[str, Any] = {}
        self._scene_offset = np.zeros(3, dtype=np.float32)

    def _set_robot_opacity(self, opacity: float) -> None:
        opacity = float(np.clip(opacity, 0.0, 1.0))
        material_ids: set[int] = set()
        for geom_id in range(self.model.ngeom):
            if int(self.model.geom_type[geom_id]) == int(mj.mjtGeom.mjGEOM_PLANE):
                continue
            if float(self.model.geom_rgba[geom_id, 3]) > 0.0:
                self.model.geom_rgba[geom_id, 3] = min(
                    float(self.model.geom_rgba[geom_id, 3]),
                    opacity,
                )
            material_id = int(self.model.geom_matid[geom_id])
            if material_id >= 0:
                material_ids.add(material_id)
        for material_id in material_ids:
            if float(self.model.mat_rgba[material_id, 3]) > 0.0:
                self.model.mat_rgba[material_id, 3] = min(
                    float(self.model.mat_rgba[material_id, 3]),
                    opacity,
                )

    @staticmethod
    def _positions(values: np.ndarray | Iterable[Iterable[float]]) -> np.ndarray:
        arr = np.asarray(values, dtype=np.float32)
        if arr.size == 0:
            return np.zeros((0, 3), dtype=np.float32)
        if arr.ndim == 1:
            arr = arr.reshape(1, 3)
        if arr.ndim != 2 or arr.shape[1] != 3:
            raise ValueError(f"positions must have shape (N, 3), got {arr.shape}")
        return arr

    @staticmethod
    def _quaternions(values: np.ndarray | Iterable[Iterable[float]]) -> np.ndarray:
        arr = np.asarray(values, dtype=np.float32)
        if arr.size == 0:
            return np.zeros((0, 4), dtype=np.float32)
        if arr.ndim == 1:
            arr = arr.reshape(1, 4)
        if arr.ndim != 2 or arr.shape[1] != 4:
            raise ValueError(f"quaternions must have shape (N, 4), got {arr.shape}")
        return arr

    def _set_axes(
        self,
        path: str,
        positions: np.ndarray,
        quaternions_wxyz: np.ndarray,
        *,
        length: float,
        radius: float,
    ) -> None:
        handle = self._axes_handles.get(path)
        if positions.shape[0] == 0:
            if handle is not None:
                handle.visible = False
            return
        if handle is None:
            self._axes_handles[path] = self.server.scene.add_batched_axes(
                path,
                batched_wxyzs=quaternions_wxyz,
                batched_positions=positions,
                axes_length=float(length),
                axes_radius=float(radius),
                visible=True,
            )
            return
        handle.batched_positions = positions
        handle.batched_wxyzs = quaternions_wxyz
        handle.visible = True

    def _set_points(self, path: str, positions: np.ndarray) -> None:
        handle = self._point_handles.get(path)
        if positions.shape[0] == 0:
            if handle is not None:
                handle.visible = False
            return
        colors = np.broadcast_to(
            np.asarray((255, 209, 26), dtype=np.uint8),
            positions.shape,
        )
        if handle is None:
            self._point_handles[path] = self.server.scene.add_point_cloud(
                path,
                points=positions,
                colors=colors,
                point_size=0.012,
                point_shape="circle",
                precision="float32",
                visible=True,
            )
            return
        handle.points = positions
        handle.colors = colors
        handle.visible = True

    def update_qpos(self, qpos: np.ndarray) -> None:
        q = np.asarray(qpos, dtype=np.float64).reshape(-1)
        if q.shape[0] > self.model.nq:
            raise ValueError(f"qpos is longer than model.nq ({q.shape[0]} > {self.model.nq})")
        self.data.qpos[:] = self._qpos_template
        self.data.qpos[: q.shape[0]] = q
        mj.mj_forward(self.model, self.data)
        self.mj_scene.update_from_mjdata(self.data)
        self._scene_offset = np.asarray(
            getattr(self.mj_scene, "_scene_offset", np.zeros(3)),
            dtype=np.float32,
        ).copy()

    def draw_human_data(
        self,
        human_positions: np.ndarray | None,
        human_rotations_wxyz: np.ndarray | None,
    ) -> None:
        positions = self._positions(
            np.zeros((0, 3), dtype=np.float32)
            if human_positions is None
            else human_positions
        )
        positions = positions + self._scene_offset
        if human_rotations_wxyz is None:
            self._set_axes(
                "/teleop/human/axes",
                np.zeros((0, 3), dtype=np.float32),
                np.zeros((0, 4), dtype=np.float32),
                length=0.12,
                radius=0.01,
            )
            self._set_points("/teleop/human/points", positions)
            return
        quaternions = self._quaternions(human_rotations_wxyz)
        if positions.shape[0] != quaternions.shape[0]:
            raise ValueError("human positions and rotations have different lengths")
        self._set_points("/teleop/human/points", np.zeros((0, 3), dtype=np.float32))
        self._set_axes(
            "/teleop/human/axes",
            positions,
            quaternions,
            length=0.12,
            radius=0.01,
        )

    def draw_robot_axes(self, body_names: list[str] | tuple[str, ...]) -> None:
        names = tuple(str(name) for name in body_names)
        body_ids = self._body_id_cache.get(names)
        if body_ids is None:
            resolved = []
            for name in names:
                body_id = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, name)
                if body_id < 0:
                    raise ValueError(f"Unknown body name for '{self.target_robot}': {name}")
                resolved.append(body_id)
            body_ids = np.asarray(resolved, dtype=np.int32)
            self._body_id_cache[names] = body_ids
        positions = np.asarray(self.data.xpos[body_ids], dtype=np.float32) + self._scene_offset
        quaternions = np.asarray(self.data.xquat[body_ids], dtype=np.float32)
        self._set_axes(
            "/teleop/robot/axes",
            positions,
            quaternions,
            length=0.06,
            radius=0.004,
        )

    def close(self) -> None:
        try:
            self.server.stop()
        except Exception:
            pass
