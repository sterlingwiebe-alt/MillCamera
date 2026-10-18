"""Recipe persistence.

A recipe is a folder under recipes/:
    recipes/<name>/recipe.json      ROIs + settings
    recipes/<name>/tpl_<roi>.png    taught template images
"""

from __future__ import annotations

import json
import re
import shutil
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from .inspection import Roi


def _safe_name(name: str) -> str:
    name = re.sub(r"[^A-Za-z0-9 _\-]", "", name).strip()
    if not name:
        raise ValueError("recipe name is empty or invalid")
    return name


class Recipe:
    def __init__(self, name: str, rois: Optional[list] = None,
                 camera_settings: Optional[dict] = None,
                 reference_image: str = "") -> None:
        self.name = name
        self.rois: list[Roi] = rois or []
        self.camera_settings: dict = camera_settings or {}
        # optional path (relative to the project root) of a still image this
        # recipe was authored against -- lets Setup show the right scene
        # even when the live camera looks at something else
        self.reference_image = reference_image
        self.modified: float = time.time()

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "rois": [r.to_dict() for r in self.rois],
            "camera_settings": self.camera_settings,
            "reference_image": self.reference_image,
            "modified": self.modified,
        }

    @staticmethod
    def from_dict(d: dict) -> "Recipe":
        r = Recipe(
            name=d["name"],
            rois=[Roi.from_dict(x) for x in d.get("rois", [])],
            camera_settings=d.get("camera_settings", {}),
            reference_image=d.get("reference_image", ""),
        )
        r.modified = d.get("modified", 0.0)
        return r


class RecipeStore:
    def __init__(self, root: str) -> None:
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)

    def folder(self, name: str) -> Path:
        return self.root / _safe_name(name)

    def list_names(self) -> list[str]:
        return sorted(p.name for p in self.root.iterdir()
                      if p.is_dir() and (p / "recipe.json").exists())

    def load(self, name: str) -> Recipe:
        path = self.folder(name) / "recipe.json"
        with open(path, "r", encoding="utf-8") as f:
            return Recipe.from_dict(json.load(f))

    def save(self, recipe: Recipe) -> None:
        recipe.modified = time.time()
        folder = self.folder(recipe.name)
        folder.mkdir(parents=True, exist_ok=True)
        tmp = folder / "recipe.json.tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(recipe.to_dict(), f, indent=2)
        tmp.replace(folder / "recipe.json")

    def delete(self, name: str) -> None:
        folder = self.folder(name)
        if folder.is_dir():
            shutil.rmtree(folder)

    def save_template(self, recipe_name: str, roi_name: str,
                      image_bgr: np.ndarray) -> str:
        """Save a taught template crop; returns the file name stored in the ROI."""
        folder = self.folder(recipe_name)
        folder.mkdir(parents=True, exist_ok=True)
        fname = f"tpl_{_safe_name(roi_name).replace(' ', '_')}.png"
        cv2.imwrite(str(folder / fname), image_bgr)
        return fname
