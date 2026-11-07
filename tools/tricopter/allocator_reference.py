"""Independent float64 test oracles and convex certificates, never a runtime allocator.

The C core alone allocates. This module checks its output using direct cross
products, an independent feasible-set construction and linear support oracle.
"""

import numpy as np


def forward_geometry(positions, thrust, angle, reaction=0.0):
    positions = np.asarray(positions, np.float64)
    thrust, angle = np.asarray(thrust, np.float64), np.asarray(angle, np.float64)
    forces = np.zeros((len(thrust), 3, 3))
    forces[:, 0, 1] = -thrust[:, 0]*np.sin(angle)
    forces[:, 0, 2] = thrust[:, 0]*np.cos(angle)
    forces[:, 1:, 2] = thrust[:, 1:]
    force = forces.sum(axis=1)
    moment = np.cross(positions[None, :, :], forces).sum(axis=1)
    wrench = np.column_stack((force[:, 2], moment))
    wrench[:, 3] += reaction
    return force, wrench


def geometric_matrix(positions):
    """Build columns through independent unit-force cross products."""
    positions = np.asarray(positions, np.float64)
    columns = []
    for motor, vector in [(1, [0, 0, 1]), (2, [0, 0, 1]), (0, [0, 0, 1]), (0, [0, -1, 0])]:
        vector = np.asarray(vector, np.float64)
        columns.append(np.r_[vector[2], np.cross(positions[motor], vector)])
    return np.column_stack(columns)


def sector_projection(point, radius, minimum_angle, maximum_angle):
    """Independent closest feasible tail vector, in [vertical,rightward] order."""
    point = np.asarray(point, np.float64)
    angles = [minimum_angle, maximum_angle]
    direction = np.arctan2(point[1], point[0])
    if minimum_angle <= direction <= maximum_angle:
        angles.append(direction)
    candidates = [np.zeros(2)]
    for angle in angles:
        ray = np.asarray([np.cos(angle), np.sin(angle)])
        candidates.append(np.clip(point@ray, 0, radius)*ray)
    return min(candidates, key=lambda candidate: np.sum((candidate-point)**2))


def project_controls(controls, maximum, minimum_angle, maximum_angle):
    controls = np.asarray(controls, np.float64)
    result = np.asarray([np.clip(controls[0], 0, maximum[1]), np.clip(controls[1], 0, maximum[2]), 0., 0.])
    result[2:] = sector_projection(controls[2:], maximum[0], minimum_angle, maximum_angle)
    return result


def linear_support_minimum(gradient, maximum, minimum_angle, maximum_angle):
    """Exact linear minimization over independent fronts and a circular sector."""
    gradient = np.asarray(gradient, np.float64)
    point = np.asarray([maximum[1] if gradient[0] < 0 else 0., maximum[2] if gradient[1] < 0 else 0., 0., 0.])
    angles = [minimum_angle, maximum_angle]
    negative_direction = np.arctan2(-gradient[3], -gradient[2])
    if minimum_angle <= negative_direction <= maximum_angle:
        angles.append(negative_direction)
    candidates = [np.zeros(2)]+[maximum[0]*np.asarray([np.cos(angle), np.sin(angle)]) for angle in angles]
    point[2:] = min(candidates, key=lambda candidate: gradient[2:]@candidate)
    return point


def certificate(matrix, request, scales, controls, maximum, minimum_angle, maximum_angle, offset=None):
    """Primal feasibility plus convex f(x)-f* <= gradient·(x-argmin_C gradient·z).

    Tiny float32 publication rounding is projected independently to a feasible
    float64 point before certification. The correction is reported explicitly;
    larger violations reject the certificate. This is a numerical certificate,
    not an interval-arithmetic proof of floating-point rounding bounds.
    """
    matrix = np.asarray(matrix, np.float64)
    request, scales = np.asarray(request, np.float64), np.asarray(scales, np.float64)
    controls = np.asarray(controls, np.float64)
    projected = project_controls(controls, maximum, minimum_angle, maximum_angle)
    correction = float(np.linalg.norm(projected-controls))
    primal_tolerance = max(1e-7, float(np.max(maximum))*2e-6)
    if correction > primal_tolerance:
        raise ValueError(f'Independent primal feasibility check failed: correction={correction}')
    offset = np.zeros(4) if offset is None else np.asarray(offset, np.float64)
    residual = matrix@projected+offset-request
    gradient = matrix.T@(residual/scales**2)
    support = linear_support_minimum(gradient, maximum, minimum_angle, maximum_angle)
    gap = float(gradient@(projected-support))
    cost = .5*float(np.sum((residual/scales)**2))
    if gap < -1e-9:
        raise ValueError('Negative independent convex gap indicates an invalid support/primal calculation')
    gap = max(0., gap)
    baseline = .5*float(np.sum(((offset-request)/scales)**2))
    if cost > baseline+max(1e-6, baseline*1e-6):
        raise ValueError('Allocator objective exceeds the independently feasible zero-control baseline')
    return {'cost_at_certified_point': cost, 'global_objective_gap_upper_bound': gap,
            'global_objective_lower_bound': max(0., cost-gap), 'zero_control_baseline_cost': baseline,
            'published_control_projection_correction': correction, 'primal_tolerance': primal_tolerance,
            'certified_controls': projected.tolist()}
