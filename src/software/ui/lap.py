"""Small LAP compatibility layer for the installed Ultralytics ByteTrack.

The Jetson image has SciPy but no aarch64 ``lap`` wheel.  ByteTrack only
calls ``lapjv(..., extend_cost=True, cost_limit=...)``.  Assign each track a
private dummy column at the limit so unmatched tracks remain possible.
"""
import numpy as np
from scipy.optimize import linear_sum_assignment

__version__ = 'scipy-fallback-1'


def lapjv(cost, extend_cost=True, cost_limit=np.inf):
    if not extend_cost:
        raise ValueError('ByteTrack requires extend_cost=True')
    matrix = np.asarray(cost, dtype=np.float64)
    if matrix.ndim != 2:
        raise ValueError('cost matrix must be two-dimensional')
    rows, cols = matrix.shape
    x = np.full(rows, -1, dtype=np.int32)
    y = np.full(cols, -1, dtype=np.int32)
    if not rows or not cols:
        return 0.0, x, y

    finite = matrix[np.isfinite(matrix)]
    limit = float(cost_limit)
    if not np.isfinite(limit):
        limit = (float(np.max(finite)) + 1.0) if finite.size else 1.0
    # A separate dummy per row allows every over-limit match to be rejected.
    padded = np.full((rows, cols + rows), limit, dtype=np.float64)
    padded[:, :cols] = np.where(np.isfinite(matrix) & (matrix <= limit),
                                matrix, limit + max(1.0, abs(limit)))
    chosen_rows, chosen_cols = linear_sum_assignment(padded)
    total = 0.0
    for row, col in zip(chosen_rows, chosen_cols):
        if col < cols and matrix[row, col] <= limit:
            x[row] = col
            y[col] = row
            total += float(matrix[row, col])
    return total, x, y
