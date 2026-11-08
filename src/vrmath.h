#pragma once
// Row-major 4x4, D3D row-vector convention (v' = v * M) unless noted.
#include <cmath>
#include <cstring>

struct Mat4 { float m[4][4]; };

inline Mat4 m4_identity() { Mat4 r{}; for (int i = 0; i < 4; i++) r.m[i][i] = 1; return r; }

inline Mat4 m4_mul(const Mat4& a, const Mat4& b) {
    Mat4 r{};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j] + a.m[i][3] * b.m[3][j];
    return r;
}

inline Mat4 m4_transpose(const Mat4& a) {
    Mat4 r; for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) r.m[i][j] = a.m[j][i]; return r;
}

// Gauss-Jordan with partial pivoting. Returns false if singular.
inline bool m4_inverse(const Mat4& in, Mat4& out) {
    double a[4][8];
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) { a[i][j] = in.m[i][j]; a[i][j + 4] = (i == j); }
    for (int c = 0; c < 4; c++) {
        int p = c;
        for (int r = c + 1; r < 4; r++) if (std::fabs(a[r][c]) > std::fabs(a[p][c])) p = r;
        if (std::fabs(a[p][c]) < 1e-12) return false;
        if (p != c) for (int j = 0; j < 8; j++) { double t = a[c][j]; a[c][j] = a[p][j]; a[p][j] = t; }
        double inv = 1.0 / a[c][c];
        for (int j = 0; j < 8; j++) a[c][j] *= inv;
        for (int r = 0; r < 4; r++) if (r != c) {
            double f = a[r][c];
            if (f != 0) for (int j = 0; j < 8; j++) a[r][j] -= f * a[c][j];
        }
    }
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) out.m[i][j] = (float)a[i][j + 4];
    return true;
}

// Off-center perspective from tangents (l<0<r, up>0, down>0), keeping the game's own depth
// mapping (m22, m32) so z-buffer/fog behave as before. sgn = game's m23: +1 left-handed
// (w = z, looks +Z), -1 right-handed (w = -z, looks -Z; TDU uses this).
inline Mat4 m4_persp_tan(float l, float r, float up, float down, float m22, float m32, float sgn) {
    Mat4 p{};
    p.m[0][0] = 2.0f / (r - l);
    p.m[1][1] = 2.0f / (up + down);
    p.m[2][0] = -sgn * (l + r) / (r - l);
    p.m[2][1] = -sgn * (up - down) / (up + down);
    p.m[2][2] = m22;
    p.m[2][3] = sgn;
    p.m[3][2] = m32;
    return p;
}

// True if p looks like a perspective projection, either handedness (w = +-z).
inline bool m4_is_perspective(const Mat4& p) {
    return std::fabs(p.m[0][3]) < 1e-3f && std::fabs(p.m[1][3]) < 1e-3f &&
           std::fabs(std::fabs(p.m[2][3]) - 1.0f) < 1e-2f && std::fabs(p.m[3][3]) < 1e-3f &&
           p.m[0][0] > 0 && p.m[1][1] > 0;
}

// For X = A * P with A affine (row-vector: column 3 = (0,0,0,1)) and P a perspective projection
// with w = sgn*z, recover P's depth terms (m22, m32) from X alone. Games often draw the far world
// and the near cockpit with different near/far planes, so each draw needs its own.
// Returns false if X has no perspective divide or is not of that form (2D / orthographic).
// *flat (optional) is set when X has no perspective divide at all (2D / orthographic draw).
inline bool m4_recover_depth(const Mat4& X, float sgn, float* m22, float* m32, bool* flat = nullptr) {
    // X[i][3] = sgn*A[i][2] and, for i < 3, X[i][2] = m22*A[i][2]  =>  X[i][2] = m22*sgn*X[i][3].
    double num = 0, den = 0, mag = 0;
    for (int i = 0; i < 3; i++) {
        num += (double)X.m[i][2] * X.m[i][3];
        den += (double)X.m[i][3] * X.m[i][3];
        mag += (double)X.m[i][2] * X.m[i][2];
    }
    if (flat) *flat = den <= 1e-10 * (mag + den + 1e-30) || den < 1e-24;
    if (den <= 1e-10 * (mag + den) || den < 1e-24) return false;
    double ks = num / den, res = 0;  // ks = m22*sgn
    for (int i = 0; i < 3; i++) { double e = X.m[i][2] - ks * X.m[i][3]; res += e * e; }
    if (res > 1e-6 * (mag + den)) return false;
    *m22 = (float)(ks * sgn);
    *m32 = (float)(X.m[3][2] - ks * X.m[3][3]);
    return true;
}

// For a pair X = Y * P (Y affine like WorldView, P a perspective projection with w = sgn*z),
// get everything needed to swap P for the eye frustum without any matrix inverse:
// handedness, P's depth terms, and its x/y scales (for a symmetric P; used to pick the main FOV).
// Returns false for 2D/ortho (no perspective divide) or when X is not Y*P (unrelated matrices).
struct PairInfo { float sgn, m22, m32, a, b; };
// *flat (optional) is set when X has no perspective divide at all (2D / orthographic draw).
inline bool m4_pair_info(const Mat4& X, const Mat4& Y, PairInfo* pi, bool* flat = nullptr) {
    // Column 3: X[:,3] = sgn*Y[:,2]. Column 2: X[i][2] = m22*Y[i][2] (+ m32 in row 3).
    double xy = 0, yy = 0, xx = 0;
    for (int i = 0; i < 4; i++) {
        xy += (double)X.m[i][3] * Y.m[i][2];
        yy += (double)Y.m[i][2] * Y.m[i][2];
        xx += (double)X.m[i][3] * X.m[i][3];
    }
    if (flat) *flat = (X.m[0][3] * X.m[0][3] + X.m[1][3] * X.m[1][3] + X.m[2][3] * X.m[2][3]) < 1e-12 * (xx + 1e-30) + 1e-20;
    if (yy < 1e-20 || xx < 1e-6 * yy) return false;
    float sgn = xy > 0 ? 1.0f : -1.0f;
    double e = 0;
    for (int i = 0; i < 4; i++) { double d = X.m[i][3] - sgn * Y.m[i][2]; e += d * d; }
    if (e > 1e-6 * yy) return false;
    double xz = 0, zz = 0, x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    for (int i = 0; i < 3; i++) { xz += (double)X.m[i][2] * Y.m[i][2]; zz += (double)Y.m[i][2] * Y.m[i][2]; }
    for (int i = 0; i < 4; i++) {
        x0 += (double)X.m[i][0] * Y.m[i][0]; y0 += (double)Y.m[i][0] * Y.m[i][0];
        x1 += (double)X.m[i][1] * Y.m[i][1]; y1 += (double)Y.m[i][1] * Y.m[i][1];
    }
    if (zz < 1e-20 || y0 < 1e-20 || y1 < 1e-20) return false;
    pi->sgn = sgn;
    pi->m22 = (float)(xz / zz);
    pi->m32 = (float)(X.m[3][2] - (xz / zz) * Y.m[3][2]);
    pi->a = (float)(x0 / y0);
    pi->b = (float)(x1 / y1);
    return pi->a > 0 && pi->b > 0;
}

// Rebuild A from X = A * P (A affine, P a symmetric perspective with x/y scales a, b and w = sgn*z)
// without P's depth terms: A's third column comes straight from X's w column. So positions stay
// exact even though the depth terms are only known roughly (recovered per draw, with float noise:
// any error there would shift x/y through the eye offset and crack the seams between draws).
inline Mat4 m4_unproject(const Mat4& X, float a, float b, float sgn) {
    Mat4 A;
    for (int i = 0; i < 4; i++) {
        A.m[i][0] = X.m[i][0] / a;
        A.m[i][1] = X.m[i][1] / b;
        A.m[i][2] = X.m[i][3] * sgn;
        A.m[i][3] = i == 3 ? 1.0f : 0.0f;
    }
    return A;
}

// Camera -> world rotation (row-vector) from the WorldView of an object whose world matrix doesn't
// rotate (only axis scales and a translation): WorldView's 3x3 rows are then the view's, scaled.
inline bool m4_cam_from_unrotated(const Mat4& WV, Mat4* C) {
    *C = m4_identity();
    for (int i = 0; i < 3; i++) {
        float n = sqrtf(WV.m[i][0] * WV.m[i][0] + WV.m[i][1] * WV.m[i][1] + WV.m[i][2] * WV.m[i][2]);
        if (n < 1e-6f) return false;
        for (int j = 0; j < 3; j++) C->m[j][i] = WV.m[i][j] / n;  // the view rotation, transposed
    }
    return true;
}

// Symmetric perspective projection from PairInfo-style terms.
inline Mat4 m4_proj(float a, float b, float m22, float m32, float sgn) {
    Mat4 p{};
    p.m[0][0] = a; p.m[1][1] = b; p.m[2][2] = m22; p.m[2][3] = sgn; p.m[3][2] = m32;
    return p;
}

// Matrix that moves a full-screen 2D draw (clip x,y in [-1,1], w = 1) onto a flat virtual screen
// fixed in the recentered head frame: half-width tangent th, half-height tangent tv, distance D,
// seen by one eye (invEye: head frame -> eye view, row-vector) through its frustum Pe.
// Without it the same 2D picture lands on different directions in each (off-center) eye image
// and is seen double. Depth keeps the draw's own (tiny) z, times w / D: the game layers its menus
// with the depth test (buttons in front of the bars behind them), and at about D in front w ~ D.
inline Mat4 m4_ui_screen(float th, float tv, float D, float sgn, const Mat4& invEye, const Mat4& Pe) {
    Mat4 T{};
    T.m[0][0] = th * D;   // x -> x*th*D
    T.m[1][1] = tv * D;   // y -> y*tv*D
    T.m[3][2] = sgn * D;  // z =  -D in front for right-handed (+D left-handed)
    T.m[3][3] = 1;
    Mat4 U = m4_mul(m4_mul(T, invEye), Pe);
    for (int i = 0; i < 4; i++) U.m[i][2] = i == 2 ? D : 0;  // overlapping 2D shares one w per pixel, so its order holds exactly
    return U;
}

// Column-vector, right-handed (OpenVR-style) rotations: +yaw turns left (about +Y), +pitch looks up (about +X).
inline Mat4 m4_rot_y(float a) {
    Mat4 r = m4_identity();
    float c = cosf(a), s = sinf(a);
    r.m[0][0] = c; r.m[0][2] = s; r.m[2][0] = -s; r.m[2][2] = c;
    return r;
}
inline Mat4 m4_rot_x(float a) {
    Mat4 r = m4_identity();
    float c = cosf(a), s = sinf(a);
    r.m[1][1] = c; r.m[1][2] = -s; r.m[2][1] = s; r.m[2][2] = c;
    return r;
}

// Split a column-vector rotation into R = Ry(yaw) * Rx(pitch) * Rz(roll): look direction first,
// then the tilt around it - the order a game's head-look applies yaw and pitch.
inline void m4_yaw_pitch_roll(const Mat4& R, float* yaw, float* pitch, float* roll) {
    float s = -R.m[1][2];
    *pitch = asinf(s > 1 ? 1 : (s < -1 ? -1 : s));
    *yaw = atan2f(R.m[0][2], R.m[2][2]);
    *roll = atan2f(R.m[1][0], R.m[1][1]);
}

// Head look the way TDU's camera applies it (measured): relative to its default camera the game
// turns by G = Rx(-tilt) * Ry(yaw) * Rx(tilt + pitch), i.e. it yaws around the car's up axis while
// the camera itself looks `tilt` down. Picks yaw/pitch (clamped) so G follows the head rotation H
// and returns G; G^T * H is what the matrix rewrite still adds (roll, rotation beyond the clamps).
inline Mat4 m4_head_look(const Mat4& H, float tilt, float maxYaw, float maxPitch, float* yaw, float* pitch) {
    float y, p, r;
    m4_yaw_pitch_roll(m4_mul(m4_rot_x(tilt), H), &y, &p, &r);
    p -= tilt;
    *yaw = y > maxYaw ? maxYaw : (y < -maxYaw ? -maxYaw : y);
    *pitch = p > maxPitch ? maxPitch : (p < -maxPitch ? -maxPitch : p);
    return m4_mul(m4_mul(m4_rot_x(-tilt), m4_rot_y(*yaw)), m4_rot_x(tilt + *pitch));
}

// Flip Z on both sides (S*M*S, S = diag(1,1,-1,1)): converts a transform between
// right- and left-handed spaces. Same result in row- or column-vector convention.
inline Mat4 m4_flip_z(Mat4 m) {
    for (int i = 0; i < 4; i++) { m.m[i][2] = -m.m[i][2]; m.m[2][i] = -m.m[2][i]; }
    return m;
}
