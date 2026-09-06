"""Postcondition checkpoint for `grasp_sg` (authored by build_graph.py).

Loaded by the executor's checkpoint hook (`--checkpoints warn|raise`) and
evaluated against the sim's ground-truth world snapshot every time
`grasp_sg` exits on its success path.
"""

from gap.runtime.verify import Checkpoint


def _target_held(world) -> bool:
    """The target is in contact with a robot finger link after `close`."""
    return world.body('alphabet soup').is_grasped()


CHECKPOINTS = [
    Checkpoint(
        name="target_held",
        subgraph="grasp_sg",
        predicate=_target_held,
        rationale=(
            "After close, the grasp target must actually be held — "
            "ground-truth contacts prove the gripper closed ON the object, "
            "not on air."
        ),
        validate=True,
    ),
]
