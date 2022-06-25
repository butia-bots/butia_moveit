from panda_gym.envs.tasks.reach import Reach
import numpy as np

class DoRISReach(Reach):
    def _create_scene(self) -> None:
        """Create the scene."""
        self.sim.create_plane(z_offset=-0.4)
        self.sim.create_box(
            body_name="target",
            half_extents=np.ones(3) * 0.05 / 2,
            mass=0.0,
            ghost=True,
            position=np.array([0.0, 0.0, 0.05]),
            rgba_color=np.array([0.1, 0.9, 0.1, 0.3]),
        )