/* SPDX-License-Identifier: BSD-2-Clause */
/* See sv_triangulate.h (BSD-2, AIST 2019 + stella-cv 2022). */
#include "sv_triangulate.h"
#include "sv_linalg.h"
#include <math.h>

void sv_camera_convert_point_to_bearing(const sv_camera_perspective* cam,
                                         float undist_x, float undist_y,
                                         double bearing[3]) {
    double x_normalized = ((double)undist_x - cam->cx) / cam->fx;
    double y_normalized = ((double)undist_y - cam->cy) / cam->fy;
    double l2_norm = sqrt(x_normalized * x_normalized + y_normalized * y_normalized + 1.0);
    bearing[0] = x_normalized / l2_norm;
    bearing[1] = y_normalized / l2_norm;
    bearing[2] = 1.0 / l2_norm;
}

void sv_triangulate_bearings(const double bearing1[3], const double bearing2[3],
                              const double rot21[9], const double trans21[3],
                              double pos_c_in_ref[3]) {
    double trans12[3], neg_trans21[3];
    double bearing2_in_1[3];
    double A00, A10, A01, A11;
    double b0, b1;
    double det, invdet;
    double lambda0, lambda1;
    double pt1[3], pt2[3];

    /* rot_21.transpose() used inline in the real source
     * (`-rot_21.transpose() * trans_21`, `rot_21.transpose() * bearing_2`):
     * measured as ALL rows left-associative (see sv_mat3_mulv_lhs_transposed). */
    neg_trans21[0] = -trans21[0];
    neg_trans21[1] = -trans21[1];
    neg_trans21[2] = -trans21[2];
    sv_mat3_mulv_lhs_transposed(rot21, neg_trans21, trans12);
    sv_mat3_mulv_lhs_transposed(rot21, bearing2, bearing2_in_1);

    A00 = sv_vec3_dot(bearing1, bearing1);
    A10 = sv_vec3_dot(bearing1, bearing2_in_1);
    A01 = -A10;
    A11 = -sv_vec3_dot(bearing2_in_1, bearing2_in_1);

    b0 = sv_vec3_dot(bearing1, trans12);
    b1 = sv_vec3_dot(bearing2_in_1, trans12);

    /* Mat22_t::inverse(): Eigen's 2x2 compute_inverse (general_det3-style):
     * det = a00*a11-a10*a01 (determinant_impl<2>); inverse =
     * [[a11,-a01],[-a10,a00]] / det (adjugate/det, InverseImpl.h size-2). */
    det = A00 * A11 - A10 * A01;
    invdet = 1.0 / det;
    {
        double i00 = A11 * invdet;
        double i01 = -A01 * invdet;
        double i10 = -A10 * invdet;
        double i11 = A00 * invdet;
        lambda0 = i00 * b0 + i01 * b1;
        lambda1 = i10 * b0 + i11 * b1;
    }

    pt1[0] = lambda0 * bearing1[0];
    pt1[1] = lambda0 * bearing1[1];
    pt1[2] = lambda0 * bearing1[2];

    pt2[0] = lambda1 * bearing2_in_1[0] + trans12[0];
    pt2[1] = lambda1 * bearing2_in_1[1] + trans12[1];
    pt2[2] = lambda1 * bearing2_in_1[2] + trans12[2];

    pos_c_in_ref[0] = (pt1[0] + pt2[0]) / 2.0;
    pos_c_in_ref[1] = (pt1[1] + pt2[1]) / 2.0;
    pos_c_in_ref[2] = (pt1[2] + pt2[2]) / 2.0;
}

int sv_camera_reproject_to_image(const sv_camera_perspective* cam,
                                  const double rot_cw[9], const double trans_cw[3],
                                  const double pos_w[3], double reproj[2]) {
    double pos_c[3];
    double z_inv;
    sv_mat3_mulv(rot_cw, pos_w, pos_c);
    pos_c[0] += trans_cw[0];
    pos_c[1] += trans_cw[1];
    pos_c[2] += trans_cw[2];

    if (pos_c[2] <= 0.0) {
        return 0;
    }

    z_inv = 1.0 / pos_c[2];
    reproj[0] = cam->fx * pos_c[0] * z_inv + cam->cx;
    reproj[1] = cam->fy * pos_c[1] * z_inv + cam->cy;

    return (cam->min_x < reproj[0] && reproj[0] < cam->max_x
            && cam->min_y < reproj[1] && reproj[1] < cam->max_y);
}
