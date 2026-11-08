// Self-check: matrix math identities + constant-table parser run over the real shaders in the exe.
#include <cassert>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <vector>
#include "ctab.h"
#include "vrmath.h"

static bool near_eq(const Mat4& a, const Mat4& b, float eps = 1e-4f) {
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) if (std::fabs(a.m[i][j] - b.m[i][j]) > eps) return false;
    return true;
}

int main(int argc, char** argv) {
    // Affine WorldView and a game projection (60 deg vfov, 16:9, near 0.5, far 5000).
    Mat4 WV = m4_identity();
    float c = cosf(0.7f), s = sinf(0.7f);
    WV.m[0][0] = c; WV.m[0][2] = -s; WV.m[2][0] = s; WV.m[2][2] = c;
    WV.m[3][0] = 3; WV.m[3][1] = -2; WV.m[3][2] = 40;
    float ty = tanf(0.5236f), tx = ty * 16 / 9.0f, zn = 0.5f, zf = 5000;
    Mat4 P = m4_persp_tan(-tx, tx, ty, ty, zf / (zf - zn), -zn * zf / (zf - zn), 1.0f);
    assert(m4_is_perspective(P));
    assert(std::fabs(P.m[1][1] - 1 / ty) < 1e-4f);

    Mat4 inv;
    assert(m4_inverse(WV, inv) && near_eq(m4_mul(WV, inv), m4_identity()));

    // Recovering P from WVP and WV, as the proxy does per draw.
    Mat4 WVP = m4_mul(WV, P), Prec = m4_mul(inv, WVP);
    assert(near_eq(Prec, P, 1e-3f));

    // Head at origin and HMD frustum == game frustum -> K is identity (no change to the image).
    Mat4 Pinv;
    assert(m4_inverse(P, Pinv));
    Mat4 K = m4_mul(m4_mul(Pinv, m4_identity()), P);
    assert(near_eq(K, m4_identity(), 1e-3f));

    // Eye moved 0.032 m right: WVP*K must equal WV*invEye*Peye.
    Mat4 invEye = m4_identity();
    invEye.m[3][0] = -0.032f;
    Mat4 Pe = m4_persp_tan(-1.3f, 1.1f, 1.2f, 1.4f, P.m[2][2], P.m[3][2], 1.0f);
    K = m4_mul(m4_mul(Pinv, invEye), Pe);
    assert(near_eq(m4_mul(WVP, K), m4_mul(m4_mul(WV, invEye), Pe), 1e-2f));

    // Off-center frustum edges land on NDC +-1.
    auto ndc = [&](float x, float y) { float v[4] = { x, y, 1, 1 }, o[4] = {};
        for (int j = 0; j < 4; j++) for (int k = 0; k < 4; k++) o[j] += v[k] * Pe.m[k][j];
        return std::make_pair(o[0] / o[3], o[1] / o[3]); };
    assert(std::fabs(ndc(-1.3f, 1.2f).first + 1) < 1e-4f && std::fabs(ndc(-1.3f, 1.2f).second - 1) < 1e-4f);
    assert(std::fabs(ndc(1.1f, -1.4f).first - 1) < 1e-4f && std::fabs(ndc(1.1f, -1.4f).second + 1) < 1e-4f);

    // Right-handed (TDU): the real game projection from the log is accepted, and the RH eye
    // frustum maps view-space edges (camera looks down -Z) onto NDC +-1.
    Mat4 G{};
    G.m[0][0] = 1.21084f; G.m[1][1] = 2.14451f; G.m[2][2] = -1.00002f; G.m[2][3] = -1; G.m[3][2] = -0.800016f;
    assert(m4_is_perspective(G));
    Mat4 Pr = m4_persp_tan(-1.3f, 1.1f, 1.2f, 1.4f, G.m[2][2], G.m[3][2], -1.0f);
    auto ndcr = [&](float x, float y) { float v[4] = { x, y, -1, 1 }, o[4] = {};
        for (int j = 0; j < 4; j++) for (int k = 0; k < 4; k++) o[j] += v[k] * Pr.m[k][j];
        return std::make_pair(o[0] / o[3], o[1] / o[3]); };
    assert(std::fabs(ndcr(-1.3f, 1.2f).first + 1) < 1e-4f && std::fabs(ndcr(-1.3f, 1.2f).second - 1) < 1e-4f);
    assert(std::fabs(ndcr(1.1f, -1.4f).first - 1) < 1e-4f && std::fabs(ndcr(1.1f, -1.4f).second + 1) < 1e-4f);
    // Symmetric frustum equal to the game's -> no change to the image.
    Mat4 Gi, Gs = m4_persp_tan(-1 / G.m[0][0], 1 / G.m[0][0], 1 / G.m[1][1], 1 / G.m[1][1], G.m[2][2], G.m[3][2], -1.0f);
    assert(m4_inverse(G, Gi) && near_eq(m4_mul(Gi, Gs), m4_identity(), 1e-3f));
    assert(near_eq(m4_flip_z(m4_flip_z(WV)), WV));

    // Depth terms recovered from a combined matrix alone (RH and LH); 2D/ortho rejected.
    {
        float m22, m32;
        assert(m4_recover_depth(m4_mul(WV, G), -1.0f, &m22, &m32));
        assert(std::fabs(m22 - G.m[2][2]) < 1e-4f && std::fabs(m32 - G.m[3][2]) < 1e-3f);
        assert(m4_recover_depth(m4_mul(WV, P), 1.0f, &m22, &m32));
        assert(std::fabs(m22 - P.m[2][2]) < 1e-4f && std::fabs(m32 - P.m[3][2]) < 1e-3f);
        Mat4 O = m4_identity();
        O.m[0][0] = 0.01f; O.m[1][1] = 0.02f; O.m[2][2] = 0.001f; O.m[3][2] = 0.5f;
        assert(!m4_recover_depth(m4_mul(WV, O), -1.0f, &m22, &m32));
        assert(!m4_recover_depth(O, -1.0f, &m22, &m32));
    }

    // Inverse-free pair path: recovers handedness, depth and FOV scales, and gives the same result
    // as the inverse-based rewrite; 2D/ortho and unrelated matrices are rejected.
    {
        PairInfo pi;
        Mat4 Ys = WV;  // add a non-uniform object scale to make it harder
        for (int j = 0; j < 4; j++) { Ys.m[0][j] *= 2.0f; Ys.m[1][j] *= 0.5f; }
        assert(m4_pair_info(m4_mul(Ys, G), Ys, &pi));
        assert(pi.sgn == -1.0f && std::fabs(pi.m22 - G.m[2][2]) < 1e-4f && std::fabs(pi.m32 - G.m[3][2]) < 1e-3f);
        assert(std::fabs(pi.a - G.m[0][0]) < 1e-3f && std::fabs(pi.b - G.m[1][1]) < 1e-3f);
        assert(m4_pair_info(m4_mul(WV, P), WV, &pi) && pi.sgn == 1.0f && std::fabs(pi.m22 - P.m[2][2]) < 1e-4f);
        Mat4 O = m4_identity();
        O.m[0][0] = 0.01f; O.m[1][1] = 0.02f; O.m[2][2] = 0.001f; O.m[3][2] = 0.5f;
        assert(!m4_pair_info(m4_mul(WV, O), WV, &pi));          // orthographic
        Mat4 other = WV;
        other.m[3][2] = 12;                                     // WVP from another object
        assert(!m4_pair_info(m4_mul(other, G), WV, &pi));

        Mat4 eye = m4_identity();
        float ce = cosf(-0.4f), se = sinf(-0.4f);
        eye.m[1][1] = ce; eye.m[1][2] = se; eye.m[2][1] = -se; eye.m[2][2] = ce; eye.m[3][0] = -0.032f; eye.m[3][1] = 0.05f;
        assert(m4_pair_info(m4_mul(Ys, G), Ys, &pi));
        Mat4 Pe3 = m4_persp_tan(-1.376f, 0.839f, 0.966f, 1.428f, pi.m22, pi.m32, pi.sgn), Gi3;
        assert(m4_inverse(G, Gi3));
        Mat4 slow = m4_mul(m4_mul(Ys, G), m4_mul(m4_mul(Gi3, eye), Pe3));
        Mat4 fast = m4_mul(m4_mul(Ys, eye), Pe3);
        assert(near_eq(slow, fast, 1e-2f));
    }

    // 2D virtual screen: the screen centre lands on each eye's true forward direction (not the
    // middle of its off-centre image), edges at the configured tangents, eye offset gives disparity,
    // the draw's own depth kept (the game layers its menus with it).
    {
        auto map = [](const Mat4& U, float x, float y, float out[3], float z = 0.5f) {
            float v[4] = { x, y, z, 1 }, o[4] = {};
            for (int j = 0; j < 4; j++) for (int k = 0; k < 4; k++) o[j] += v[k] * U.m[k][j];
            out[0] = o[0] / o[3]; out[1] = o[1] / o[3]; out[2] = o[2] / o[3];
        };
        float o[3], o2[3];
        Mat4 Psym = m4_persp_tan(-1, 1, 1, 1, -1, -0.1f, -1);
        Mat4 U = m4_ui_screen(0.5f, 0.25f, 2.0f, -1, m4_identity(), Psym);
        map(U, 1, 0, o);  assert(std::fabs(o[0] - 0.5f) < 1e-5f && std::fabs(o[1]) < 1e-5f);
        map(U, 0, -1, o); assert(std::fabs(o[0]) < 1e-5f && std::fabs(o[1] + 0.25f) < 1e-5f);
        map(U, 0, 0, o, 0.0006f); assert(std::fabs(o[2] - 0.0006f) < 1e-7f);   // depth = the draw's own at the centre
        map(U, 0.7f, 0.3f, o2, 0.0008f); map(U, 0.7f, 0.3f, o, 0.0006f);        // and its order anywhere on the screen
        assert(o[2] < o2[2] && o2[2] < 0.002f);

        Mat4 leftEye = m4_identity();
        leftEye.m[3][0] = 0.032f;  // head -> left-eye view: eye sits 3.2 cm left
        U = m4_ui_screen(0.5f, 0.25f, 2.0f, -1, leftEye, Psym);
        map(U, 0, 0, o);  assert(std::fabs(o[0] - 0.016f) < 1e-5f);

        Mat4 Pq = m4_persp_tan(-1.376f, 0.839f, 0.966f, 1.428f, -1, -0.1f, -1);  // Quest 3 left eye
        U = m4_ui_screen(0.5f, 0.25f, 2.0f, -1, m4_identity(), Pq);
        map(U, 0, 0, o);
        assert(std::fabs(o[0] - 0.537f / 2.215f) < 1e-3f && std::fabs(o[1] - 0.462f / 2.394f) < 1e-3f);
    }

    // Head-look split: yaw/pitch/roll round-trip, and taking the fed look rotation back out of the
    // head rotation leaves only the roll (what the matrix rewrite still has to add).
    {
        float a = 0.9f, b = -0.4f, c = 0.25f, y, p, r;
        Mat4 Rz = m4_identity();
        Rz.m[0][0] = cosf(c); Rz.m[0][1] = -sinf(c); Rz.m[1][0] = sinf(c); Rz.m[1][1] = cosf(c);
        Mat4 R = m4_mul(m4_mul(m4_rot_y(a), m4_rot_x(b)), Rz);
        m4_yaw_pitch_roll(R, &y, &p, &r);
        assert(std::fabs(y - a) < 1e-5f && std::fabs(p - b) < 1e-5f && std::fabs(r - c) < 1e-5f);
        Mat4 look = m4_mul(m4_rot_y(y), m4_rot_x(p)), lookInv;
        assert(m4_inverse(look, lookInv) && near_eq(m4_mul(lookInv, R), Rz));
        // +yaw turns the forward (-Z) view to the left (-X), +pitch looks up (+Y).
        assert(m4_rot_y(0.5f).m[0][2] > 0 && m4_rot_x(0.5f).m[1][2] < 0);

        // Camera rotation read off an unrotated (scaled, moved) object; a rotated one gives another.
        Mat4 cam = m4_mul(m4_rot_x(-0.2f), m4_rot_y(0.7f)), V, Cr;  // camera -> world
        cam.m[3][0] = 120; cam.m[3][1] = 4; cam.m[3][2] = -35;
        assert(m4_inverse(cam, V));
        Mat4 W = m4_identity();
        W.m[0][0] = 2; W.m[1][1] = 3; W.m[2][2] = 0.5f; W.m[3][0] = 100; W.m[3][1] = -5; W.m[3][2] = 30;
        assert(m4_cam_from_unrotated(m4_mul(W, V), &Cr));
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) assert(std::fabs(Cr.m[i][j] - cam.m[i][j]) < 1e-5f);
        assert(m4_cam_from_unrotated(m4_mul(m4_mul(m4_rot_y(0.01f), W), V), &Cr) && std::fabs(Cr.m[0][0] - cam.m[0][0]) + std::fabs(Cr.m[2][0] - cam.m[2][0]) > 2e-3f);

        // With the game camera's 4.7 deg tilt: whatever the head does within the clamps, taking the
        // game's rotation back out leaves only roll; beyond a clamp the rest stays for the rewrite.
        float tilt = -4.7f * 3.14159265f / 180, gy, gp;
        Mat4 Hhead = m4_mul(m4_mul(m4_mul(m4_rot_x(-tilt), m4_rot_y(1.2f)), m4_rot_x(0.3f)), Rz);
        Mat4 G = m4_head_look(Hhead, tilt, 1.66f, 0.51f, &gy, &gp);
        assert(std::fabs(gy - 1.2f) < 1e-4f && std::fabs(gp - (0.3f - tilt)) < 1e-4f);
        assert(near_eq(m4_mul(m4_transpose(G), Hhead), Rz));
        m4_head_look(Hhead, tilt, 0.5f, 0.51f, &gy, &gp);
        assert(gy == 0.5f);  // clamped
    }

    // Rebuilding WorldView from WorldViewProj needs only the FOV scales, not the depth terms.
    {
        Mat4 Ys = WV;
        for (int j = 0; j < 4; j++) { Ys.m[0][j] *= 2.0f; Ys.m[2][j] *= 0.7f; }
        assert(near_eq(m4_unproject(m4_mul(Ys, G), G.m[0][0], G.m[1][1], -1.0f), Ys, 1e-4f));
        assert(near_eq(m4_unproject(m4_mul(Ys, P), P.m[0][0], P.m[1][1], 1.0f), Ys, 1e-4f));
    }

    // A draw with a different near plane than the cockpit must still be rewritten exactly when
    // its own projection is used (per-draw K), for an eye that is moved and turned.
    {
        Mat4 Gfar = G;
        Gfar.m[3][2] = -5.0f;  // near 5 m instead of 0.8 m
        Mat4 eye = m4_identity();
        float ce = cosf(0.3f), se = sinf(0.3f);
        eye.m[0][0] = ce; eye.m[0][2] = se; eye.m[2][0] = -se; eye.m[2][2] = ce; eye.m[3][0] = 0.03f; eye.m[3][2] = -0.1f;
        Mat4 Pe2 = m4_persp_tan(-1.376f, 0.839f, 0.966f, 1.428f, Gfar.m[2][2], Gfar.m[3][2], -1.0f);
        Mat4 Gi2;
        assert(m4_inverse(Gfar, Gi2));
        Mat4 K2 = m4_mul(m4_mul(Gi2, eye), Pe2);
        assert(near_eq(m4_mul(m4_mul(WV, Gfar), K2), m4_mul(m4_mul(WV, eye), Pe2), 1e-2f));
    }

    assert(ctab_classify("gVSWorldViewProject") == MK_POST_K && ctab_classify("gWorldViewProj") == MK_POST_K);
    assert(ctab_classify("gVSWorldView") == MK_POST_EYE && ctab_classify("gView") == MK_POST_EYE);
    assert(ctab_classify("gVSM4InvView") == MK_PRE_E && ctab_classify("gProj") == MK_PROJ);
    assert(ctab_classify("gVSReflWorldViewProj") == -1 && ctab_classify("gPSWorld") == -1);
    printf("math ok\n");

    if (argc > 1) {  // parse every vertex shader embedded in the game exe
        std::ifstream f(argv[1], std::ios::binary);
        std::vector<char> d((std::istreambuf_iterator<char>(f)), {});
        int vs = 0, withMats = 0, pairs = 0;
        for (size_t i = 8; i + 4 < d.size(); i += 4) {
            const uint32_t* p = (const uint32_t*)(d.data() + i);
            if (p[0] == 0x42415443u && (p[-1] & 0xFFFF) == 0xFFFE && (p[-2] >> 16) == 0xFFFE) {
                vs++;
                ShaderMats m = ctab_parse(p - 2, (d.size() - i) / 4 + 2, [](const std::string&, int, int) {});
                if (!m.mats.empty()) withMats++;
                if (m.pairK >= 0 && m.pairV >= 0) pairs++;
            }
        }
        printf("exe: %d vertex shaders, %d with camera matrices, %d can recover projection\n", vs, withMats, pairs);
        assert(vs > 1000 && withMats > 900 && pairs > 400);
    }
    printf("all tests passed\n");
    return 0;
}
