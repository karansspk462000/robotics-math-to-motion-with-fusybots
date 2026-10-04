#pragma once

namespace kinematics {

// Named DH-derived link lengths for the specific 6R spherical-wrist arm
// this closed-form solution was derived for (see fusybots_six_dof.urdf.xacro).
// d2 is l2-l4 in the original derivation (the small offset between the
// joint-2 and joint-3 link segments); it is not used by the closed-form
// scalar IK equations, only by the full DH chain in FK.
struct SixAxisLinkLengths {
    double l1 = 0.0;
    double d2 = 0.0;
    double l3 = 0.0;
    double l5 = 0.0;
    double l6 = 0.0;
    double l7 = 0.0;
    double le = 0.0;
};

}  // namespace kinematics
