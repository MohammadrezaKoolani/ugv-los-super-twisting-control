from __future__ import annotations

from ugv_control.types import Path, Segment


def get_segment(path: Path, segment_index: int) -> Segment:
    """
    Return the active path segment from waypoint k to waypoint k+1.

    Parameters
    ----------
    path : Path
        Piecewise-linear path.
    segment_index : int
        Active segment index k.

    Returns
    -------
    Segment
        Segment from p_k to p_{k+1}.

    Raises
    ------
    IndexError
        If segment_index is outside the valid range [0, num_segments - 1].
    """
    if segment_index < 0 or segment_index >= path.num_segments:
        raise IndexError(
            f"segment_index={segment_index} is out of range for path with "
            f"{path.num_segments} segments."
        )

    start = path.waypoint(segment_index)
    end = path.waypoint(segment_index + 1)
    return Segment(start=start, end=end, index=segment_index)