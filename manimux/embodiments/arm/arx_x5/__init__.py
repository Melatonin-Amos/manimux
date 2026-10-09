"""Offline ARX X5 geometry for simulation policy adapters."""

from manimux.embodiments.arm.base import ArmBase, ArmModel

from .kinematics import ArxX5Kinematics


class ArxX5Arm(ArmBase):
    """Model provider only; physical controller methods remain abstract."""

    @classmethod
    def load_model(cls, **options):
        model = ArxX5Kinematics(**options)
        return ArmModel(
            model, model.coordinates, model.urdf_path, None,
            base_frame=model.base_frame, visual_mapping=model.visual_configuration,
        )


__all__ = ["ArxX5Arm", "ArxX5Kinematics"]
