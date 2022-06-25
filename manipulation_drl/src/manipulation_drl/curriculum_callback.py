from stable_baselines3.common.callbacks import BaseCallback

class CurriculumCallback(BaseCallback):
    def __init__(self, threshold: float, verbose: int = 0):
        self.threshold = threshold
        self.level = 0
        super().__init__(verbose)
    
    def _on_rollout_end(self) -> None:
        success_buffer = self.model.ep_success_buffer.copy()
        if len( success_buffer) > 0 and success_buffer.index(0) > self.threshold:
            self.level += 1
        self.model.env.env_method("change_level", self.level)

    def _on_step(self) -> bool:
        return True