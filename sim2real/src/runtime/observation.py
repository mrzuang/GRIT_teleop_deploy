from abc import ABC, abstractmethod

import numpy as np


class Observation(ABC):
    """Interface implemented by GRIT observation terms."""

    @property
    @abstractmethod
    def size(self) -> int:
        raise NotImplementedError

    def reset(self) -> None:
        return

    def update(self) -> None:
        return

    @abstractmethod
    def compute(self) -> np.ndarray:
        raise NotImplementedError
