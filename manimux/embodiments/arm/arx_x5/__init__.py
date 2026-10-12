"""Physical X5 controller and separately selected RoboDojo offline geometry."""

from manimux.embodiments.arm.base import ArmBase, ArmModel

from .kinematics import ArxX5Kinematics
from .arm import ArxX5Arm
from .controller import ArxX5Controller


class RoboDojoX5Arm(ArmBase):
    """Model provider only; physical controller methods remain abstract."""

    @classmethod
    def load_model(cls, **options):
        model = ArxX5Kinematics(**options)
        return ArmModel(
            model, model.coordinates, model.urdf_path, None,
            base_frame=model.base_frame, visual_mapping=model.visual_configuration,
        )


__all__ = ["ArxX5Arm", "ArxX5Controller", "ArxX5Kinematics", "RoboDojoX5Arm"]
