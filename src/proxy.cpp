// d3d9.dll proxy for Test Drive Unlimited: hooks the real device's vtable, rewrites the
// camera matrices in vertex shader constants per draw (6DOF head pose + per-eye frustum),
// alternates eyes each frame and hands the back buffer to SteamVR.
#define CINTERFACE
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <d3d9.h>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <vector>
#include "ctab.h"
#include "vr.h"

static const int MAXC = 256;

static bool g_vr;
static int g_eye;
static unsigned g_frame;
static EyeView g_ev;
static FramePose g_framePose;    // pose the current frame renders with
static Mat4 g_invEyeLH, g_ELH;   // g_ev.invEye converted for a left-handed game, and its inverse
static Mat4 g_ERH;               // inverse(g_ev.invEye): pre-multiply for InvView matrices
static Mat4 g_mainP;             // projection most full-screen draws used last frame
static bool g_haveMain;
static UINT g_bbW, g_bbH, g_rtW, g_rtH;
static D3DVIEWPORT9 g_vp;

// 2D (menus, HUD, cursor) goes onto a virtual screen in front of you: width in degrees and
// distance in meters from tdu_vr.ini. g_uiU maps the 2D draw onto it for the current eye.
static float g_uiFov = 70.0f, g_uiDist = 1.5f;
static Mat4 g_uiU;
static std::unordered_set<const void*> g_rtTex;  // the game's render-target textures (post-processing inputs)
static const void* g_tex0;                       // texture bound to stage 0

static float g_vsc[MAXC][4];     // what the game last set (shadow)
static bool g_isMod[MAXC];       // hardware register currently holds our rewritten value
static std::vector<int> g_mod;
static std::unordered_map<IDirect3DVertexShader9*, ShaderMats> g_shaders;
static const ShaderMats* g_cur;
static std::set<std::string> g_logged;

// Back buffer readback ring: the copy queued at frame N is submitted at frame N+1, so the CPU
// never waits for the GPU to finish the frame it just issued.
struct Slot {
    IDirect3DSurface9* rt;
    IDirect3DSurface9* sys;
    IDirect3DQuery9* q;
    bool pending;
    int eye;
    unsigned pair;  // frames 2k (left) and 2k+1 (right) form pair k and share one head pose
    FramePose pose;
};

// Head look through the game's own TrackIR head-look (addresses of the 2007 exe, checked against
// the camera code before use): the game turns its camera by these angles, so it draws and culls
// toward where you look.
static float* const kLookYaw = (float*)0xfa403c;                  // radians, read at 0x8d4e43
static float* const kLookPitch = (float*)0xfa4040;                // radians, read at 0x8d4e99
static volatile unsigned char* const kLookOn = (volatile unsigned char*)0x13a70ac;  // the game's TrackIR option
static bool g_headLook;
// Measured on the in-car camera: the game clamps yaw at 97 deg and pitch at 30 deg, applies the
// angles on the next frame without smoothing, yaws like we do (+ = left) but pitches the other
// way, and its camera looks 4.7 deg down from the axis it yaws around.
static float g_maxYaw = 95 * 3.14159265f / 180, g_maxPitch = 29 * 3.14159265f / 180, g_lookTilt = -4.7f * 3.14159265f / 180;
static const float g_lookSignYaw = 1, g_lookSignPitch = -1;
static bool g_lookTest;                                 // TDUVR_LOOKTEST: feed fixed angles, log what the camera did
static Mat4 g_cam;                                      // game camera seen this frame (view -> world, row-vector)
static bool g_camOk;

// The game skips its camera update on some frames (about 1 in 9 measured), and then renders with
// the previous look. So the look actually applied is read back from its camera object, where
// +0x17c / +0x180 hold the yaw / pitch it used (after its clamps). The object is found once by a
// memory scan while a distinctive look is held for a moment.
static volatile LONG_PTR g_camObj;
static volatile bool g_scanHold;
static volatile float g_holdYaw, g_holdPitch;
static float g_fedYaw[4], g_fedPitch[4];  // recent fed looks, in the form the camera stores them
static unsigned g_fedN;
static float g_appliedYaw, g_appliedPitch;  // look the current eye matrices assume
static bool g_lookChecked;                  // applied look read back for this frame
static int g_camMisses;
static unsigned g_lookFixes;                // frames where the game used another look than fed
// Scan results waiting for proof: the real camera follows the fed look once the hold ends, stale
// copies of the held values (our own variables, stacks) don't.
static LONG_PTR g_cand[16];
static int g_candOk[16];
static volatile LONG g_nCand;
static int g_candFrames;
static volatile bool g_inWorld;             // a 3D camera was seen recently (scan only then)
static unsigned g_rcN[2], g_rcFix[2], g_rcBad[2];  // frames per eye: checked, rendered with another look than read, unmatched
static unsigned g_n3d;                      // main-view 3D draws so far this frame
static double g_capSum[2];                  // per eye: sum of the draw index where the frame's camera became known
static unsigned g_capN[2];
static bool g_camEarly;                     // this frame's camera came from an unrotated object, not yet checked
static unsigned g_earlyBad[2];              // per eye: frames where the first InvView/View chose another look

static bool headlook_verify() {
    static const unsigned char a[] = { 0xA0, 0xAC, 0x70, 0x3A, 0x01 };                    // mov al, [0x13a70ac]
    static const unsigned char b[] = { 0xF3, 0x0F, 0x10, 0x1D, 0x3C, 0x40, 0xFA, 0x00 };  // movss xmm3, [0xfa403c]
    static const unsigned char c[] = { 0xF3, 0x0F, 0x5C, 0x0D, 0x40, 0x40, 0xFA, 0x00 };  // subss xmm1, [0xfa4040]
    __try {
        return !memcmp((const void*)0x8d4e27, a, sizeof a) && !memcmp((const void*)0x8d4e43, b, sizeof b) &&
               !memcmp((const void*)0x8d4e99, c, sizeof c);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}
static Slot g_slot[2];
static UINT g_copyW, g_copyH;
static D3DFORMAT g_copyFmt;

// ---- originals ----
// On current Windows, d3d9 gives every IDirect3D9 / device object its own heap copy of the
// vtable, and TDU creates several of each, so every instance is hooked and originals are
// kept per vtable. Tables can be freed and their address reused, so "hooked" is checked by
// slot content, not address.
struct D3DOrig { const IDirect3D9Vtbl* vt; decltype(IDirect3D9Vtbl::CreateDevice) CreateDevice; };
static D3DOrig g_d3dOrig[16];
static int g_nD3D;

struct DevOrig {
    const IDirect3DDevice9Vtbl* vt;
    decltype(IDirect3DDevice9Vtbl::Present) Present;
    decltype(IDirect3DDevice9Vtbl::Reset) Reset;
    decltype(IDirect3DDevice9Vtbl::CreateVertexShader) CreateVertexShader;
    decltype(IDirect3DDevice9Vtbl::SetVertexShader) SetVertexShader;
    decltype(IDirect3DDevice9Vtbl::SetVertexShaderConstantF) SetVertexShaderConstantF;
    decltype(IDirect3DDevice9Vtbl::SetRenderTarget) SetRenderTarget;
    decltype(IDirect3DDevice9Vtbl::SetViewport) SetViewport;
    decltype(IDirect3DDevice9Vtbl::CreateTexture) CreateTexture;
    decltype(IDirect3DDevice9Vtbl::SetTexture) SetTexture;
    decltype(IDirect3DDevice9Vtbl::DrawPrimitive) DrawPrimitive;
    decltype(IDirect3DDevice9Vtbl::DrawIndexedPrimitive) DrawIndexedPrimitive;
    decltype(IDirect3DDevice9Vtbl::DrawPrimitiveUP) DrawPrimitiveUP;
    decltype(IDirect3DDevice9Vtbl::DrawIndexedPrimitiveUP) DrawIndexedPrimitiveUP;
};
static const int MAXORIG = 32;
static DevOrig g_orig[MAXORIG];
static int g_nOrig;  // total ever added; entries reused as a ring

static DevOrig& H(IDirect3DDevice9* d) {
    int n = g_nOrig < MAXORIG ? g_nOrig : MAXORIG;
    for (int i = 0; i < n; i++) if (g_orig[i].vt == d->lpVtbl) return g_orig[i];
    return g_orig[0];  // all instances point at the same d3d9.dll functions in practice
}

template <class T> static void hook(const T* slot, T fn, T* orig) {
    T* s = const_cast<T*>(slot);
    DWORD old;
    VirtualProtect(s, sizeof(T), PAGE_EXECUTE_READWRITE, &old);
    *orig = *s;
    *s = fn;
    VirtualProtect(s, sizeof(T), old, &old);
}

// ---- diagnostics: which hooks actually fire ----
enum { C_PRESENT, C_SCPRESENT, C_CVS, C_CVS_MATS, C_SVS, C_SVSC, C_SRT, C_DRAW, C_N };
static const char* kCntName[C_N] = { "Present", "SwapChainPresent", "CreateVertexShader", "shadersWithCamera",
                                     "SetVertexShader", "SetVSConstF", "SetRenderTarget", "Draw" };
static unsigned g_cnt[C_N];
static void count(int i) {
    if (g_cnt[i]++ == 0) log_printf("first call: %s\n", kCntName[i]);
}

// What happened to draws that carry camera matrices.
enum { R_PAIR, R_OTHER, R_UI, R_POSTFX, R_NOTPERSP, R_SHAPE, R_VIEWPORT, R_FORM, R_NOMAIN, R_PLAIN, R_N };
static const char* kRName[R_N] = { "rewritten(exact)", "rewritten(recovered)", "2D->screen", "skip:postfx",
                                   "skip:2D-other", "skip:offscreen-shape", "skip:sub-viewport", "skip:not-projection",
                                   "skip:no-camera-yet", "untouched(no camera matrix)" };
static unsigned g_st[R_N];
static double g_copyWaitMs, g_submitMs, g_applyMs;  // cumulative ms, for the status line

static double now_s() {
    static LARGE_INTEGER f = [] { LARGE_INTEGER x; QueryPerformanceFrequency(&x); return x; }();
    LARGE_INTEGER c;
    QueryPerformanceCounter(&c);
    return (double)c.QuadPart / (double)f.QuadPart;
}

static void log_status() {
    static double t0 = now_s(), last = t0;
    static unsigned lastFrame, lastSt[R_N], lastDraw;
    static double lastWait, lastSubmit, lastApply;
    double t = now_s();
    if (t - last < 5.0) return;
    unsigned frames = g_frame - lastFrame;
    double perF = frames ? 1.0 / frames : 0;
    std::string s;
    for (int i = 0; i < R_N; i++) {
        s += " " + std::string(kRName[i]) + "=" + std::to_string((unsigned)((g_st[i] - lastSt[i]) * perF + 0.5));
        lastSt[i] = g_st[i];
    }
    log_printf("t=%.0fs %.1f fps | per frame: mod draw hooks %.2f ms, copy wait %.2f ms, publish %.2f ms | draws/frame %u:%s | main vfov %.1f\n",
               t - t0, frames / (t - last), (g_applyMs - lastApply) * perF, (g_copyWaitMs - lastWait) * perF,
               (g_submitMs - lastSubmit) * perF, (unsigned)((g_cnt[C_DRAW] - lastDraw) * perF + 0.5), s.c_str(),
               g_haveMain ? 2 * atan(1 / g_mainP.m[1][1]) * 57.2958 : 0.0);
    if (g_vr) log_printf("  %s\n", vr_status());
    if (g_headLook) {
        static unsigned lastFixes;
        log_printf("  head look: yaw %+.1f pitch %+.1f deg, camera object %s, corrected %u frames | rendered with another look than read:"
                   " left %u/%u, right %u/%u frames (unmatched %u/%u), known by 3D draw %.1f/%.1f, early pick corrected %u/%u\n",
                   g_appliedYaw * 57.2958f, g_appliedPitch * 57.2958f, g_camObj ? "found" : "not found", g_lookFixes - lastFixes,
                   g_rcFix[0], g_rcN[0], g_rcFix[1], g_rcN[1], g_rcBad[0], g_rcBad[1],
                   g_capN[0] ? g_capSum[0] / g_capN[0] : -1.0, g_capN[1] ? g_capSum[1] / g_capN[1] : -1.0, g_earlyBad[0], g_earlyBad[1]);
        lastFixes = g_lookFixes;
        g_rcN[0] = g_rcN[1] = g_rcFix[0] = g_rcFix[1] = g_rcBad[0] = g_rcBad[1] = 0;
        g_capSum[0] = g_capSum[1] = 0;
        g_capN[0] = g_capN[1] = 0;
        g_earlyBad[0] = g_earlyBad[1] = 0;
    }
    last = t; lastFrame = g_frame; lastDraw = g_cnt[C_DRAW]; lastWait = g_copyWaitMs; lastSubmit = g_submitMs; lastApply = g_applyMs;
}

static std::string module_of(const void* p) {
    HMODULE m = nullptr;
    char name[MAX_PATH] = "?";
    if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCSTR)p, &m))
        GetModuleFileNameA(m, name, MAX_PATH);
    return name;
}

// ---- matrix helpers (registers hold columns: M[i][j] = c[reg+j][i]) ----
static Mat4 read_mat(const MatSlot& s) {
    Mat4 M = m4_identity();
    int cols = (s.kind == MK_POST_K || s.kind == MK_PROJ) ? 4 : 3;  // affine ones: column 3 = (0,0,0,1)
    for (int j = 0; j < cols && s.reg + j < MAXC; j++)
        for (int i = 0; i < 4; i++) M.m[i][j] = g_vsc[s.reg + j][i];
    return M;
}

static void write_mat(IDirect3DDevice9* d, const MatSlot& s, const Mat4& M) {
    float buf[4][4];
    int n = s.count > 4 ? 4 : s.count;
    if (s.reg + n > MAXC) return;
    for (int j = 0; j < n; j++) for (int i = 0; i < 4; i++) buf[j][i] = M.m[i][j];
    H(d).SetVertexShaderConstantF(d, s.reg, &buf[0][0], n);
    for (int j = 0; j < n; j++) if (!g_isMod[s.reg + j]) { g_isMod[s.reg + j] = true; g_mod.push_back(s.reg + j); }
}

// Eye frustum per game depth range, snapped: depth terms recovered per draw carry float noise
// (the near plane wanders by about 1.5%), while TDU only uses a few real ranges per frame (cockpit
// 0.05-3 m, world 0.8 m-40 km, sky 2 m-inf). Snapping gives each range one bit-identical frustum.
// m22 (set by the far plane) is precise and must match closely, so different ranges never merge;
// m32 (the near plane) is the noisy one.
struct PeEntry { float m22, m32, sgn; Mat4 Pe; bool valid; };
static PeEntry g_pe[8];
static int g_peNext;

static const Mat4& pe_for(float m22, float m32, float sgn) {
    for (auto& e : g_pe)
        if (e.valid && e.sgn == sgn && std::fabs(e.m22 - m22) <= 1e-4f * std::fabs(m22) + 1e-7f && std::fabs(e.m32 - m32) <= 0.03f * std::fabs(m32) + 1e-6f)
            return e.Pe;
    PeEntry& e = g_pe[g_peNext++ & 7];
    e = PeEntry{ m22, m32, sgn, m4_persp_tan(g_ev.l, g_ev.r, g_ev.up, g_ev.down, m22, m32, sgn), true };
    return e.Pe;
}

// New eye/pose: refresh the eye matrices and drop cached frustums.
static void new_frame_matrices() {
    g_invEyeLH = m4_flip_z(g_ev.invEye);
    m4_inverse(g_ev.invEye, g_ERH);
    m4_inverse(g_invEyeLH, g_ELH);
    for (auto& e : g_pe) e.valid = false;

    float sgn = g_haveMain && g_mainP.m[2][3] > 0 ? 1.0f : -1.0f;  // TDU: right-handed
    float th = tanf(g_uiFov * 0.5f * 3.14159265f / 180.0f), tv = g_bbW ? th * g_bbH / g_bbW : th;
    Mat4 Pe = m4_persp_tan(g_ev.l, g_ev.r, g_ev.up, g_ev.down, sgn, -0.1f, sgn);  // depth unused: pinned to near
    EyeView ui = g_ev;
    vr_eye_for_look(g_eye, 0, 0, g_lookTilt, &ui);  // the screen stays put when the game camera turns with your head
    g_uiU = m4_ui_screen(th, tv, g_uiDist, sgn, sgn > 0 ? m4_flip_z(ui.invEye) : ui.invEye, Pe);
}

// Main camera = the FOV most full-screen draws used last frame; it supplies the FOV for draws
// whose shader has no matrix pair.
struct Vote { PairInfo pi; unsigned n; };
static Vote g_votes[4];
static int g_nVotes;

// FOV of the last exactly-known draw this frame: draws whose shader only has a combined matrix take
// their FOV from it. (A FOV that is even slightly off moves their depth against the exact draws,
// and where the game draws a surface twice that shows as flickering seams.)
static PairInfo g_lastPair;
static unsigned g_lastPairFrame = ~0u;

static void vote(const PairInfo& pi) {
    g_lastPair = pi;
    g_lastPairFrame = g_frame;
    for (int i = 0; i < g_nVotes; i++) {
        PairInfo& q = g_votes[i].pi;
        if (std::fabs(pi.a / q.a - 1) < 0.02f && std::fabs(pi.b / q.b - 1) < 0.02f && pi.sgn == q.sgn) {
            g_votes[i].n++;
            q = pi;  // keep the latest value of the bucket (the game drifts its FOV with speed)
            return;
        }
    }
    if (g_nVotes < 4) g_votes[g_nVotes++] = Vote{ pi, 1 };
}

static void describe(const char* what, const PairInfo& p) {
    float d = 1 - p.sgn * p.m22;
    char farBuf[24];
    if (std::fabs(d) < 1e-5f) strcpy_s(farBuf, "inf");
    else snprintf(farBuf, sizeof farBuf, "%.0f", p.m32 / d);
    log_printf("%s: vfov %.1f deg, aspect %.3f, %s-handed, near %.3f, far %s (render target %ux%u, viewport %lux%lu)\n",
               what, 2 * atan(1 / p.b) * 57.2958, p.b / p.a, p.sgn > 0 ? "left" : "right", -p.sgn * p.m32 / p.m22, farBuf,
               g_rtW, g_rtH, g_vp.Width, g_vp.Height);
}

static void elect_main() {
    int best = -1;
    for (int i = 0; i < g_nVotes; i++) if (best < 0 || g_votes[i].n > g_votes[best].n) best = i;
    if (best >= 0) {
        const PairInfo& p = g_votes[best].pi;
        bool changed = !g_haveMain || std::fabs(p.a / g_mainP.m[0][0] - 1) > 0.05f || std::fabs(p.b / g_mainP.m[1][1] - 1) > 0.05f;
        g_mainP = m4_proj(p.a, p.b, p.m22, p.m32, p.sgn);
        g_haveMain = true;
        if (changed) describe("main camera", p);
    }
    g_nVotes = 0;
}

// Log each distinct camera (FOV / depth range) once, up to 24, to see how the game splits its scene.
static void note_camera(const PairInfo& p) {
    static int n;
    static unsigned sample;
    if (n >= 24 || (sample++ & 63) != 0) return;  // sampled: formatting text per draw is too slow
    float nearP = -p.sgn * p.m32 / p.m22, d = 1 - p.sgn * p.m22;
    char key[64];
    snprintf(key, sizeof key, "cam %.0f %.2g %.2g %d", 2 * atan(1 / p.b) * 57.2958, nearP, std::fabs(d) < 1e-5f ? 0.0f : p.m32 / d, (int)(p.b / p.a * 10));
    if (g_logged.insert(key).second) { n++; describe("camera seen", p); }
}

static bool owns(const ShaderMats* s, int r) {
    if (!s) return false;
    for (auto& m : s->mats) if (r >= m.reg && r < m.reg + m.count) return true;
    return false;
}

static void restore_stale(IDirect3DDevice9* d, const ShaderMats* keep) {
    size_t w = 0;
    for (int r : g_mod) {
        if (owns(keep, r)) { g_mod[w++] = r; continue; }
        H(d).SetVertexShaderConstantF(d, r, g_vsc[r], 1);
        g_isMod[r] = false;
    }
    g_mod.resize(w);
}

// The projection a draw without a matrix pair was made with, or false to leave the draw alone.
// *flat is set when the draw is 2D (its position matrix has no perspective divide).
static bool draw_projection(const ShaderMats* s, Mat4* Pd, bool* flat) {
    *flat = false;
    if (s->proj >= 0) {
        Mat4 Pc = read_mat(s->mats[s->proj]);
        if (m4_is_perspective(Pc)) { *Pd = Pc; return true; }
        *flat = std::fabs(Pc.m[0][3]) + std::fabs(Pc.m[1][3]) + std::fabs(Pc.m[2][3]) < 1e-6f;
        if (!*flat) g_st[R_FORM]++;
        return false;
    }
    if (s->anyK >= 0) {
        // Only a combined matrix: take the FOV from the main camera, the depth range from the
        // matrix itself; 2D/ortho matrices fail here.
        float m22, m32;
        if (!m4_recover_depth(read_mat(s->mats[s->anyK]), g_haveMain && g_mainP.m[2][3] > 0 ? 1.0f : -1.0f, &m22, &m32, flat)) {
            if (!*flat) g_st[R_FORM]++;
            return false;
        }
        if (g_lastPairFrame == g_frame) *Pd = m4_proj(g_lastPair.a, g_lastPair.b, m22, m32, g_lastPair.sgn);
        else if (g_haveMain) *Pd = g_mainP;
        else { g_st[R_NOMAIN]++; return false; }
        Pd->m[2][2] = m22;
        Pd->m[3][2] = m32;
        return true;
    }
    if (!g_haveMain) { g_st[R_NOMAIN]++; return false; }
    *Pd = g_mainP;  // only view-type matrices: handedness from the main camera
    return true;
}

// ---- head look: read back what the game applied ----
static bool read_look(LONG_PTR cam, float* y, float* p) {
    __try {
        *y = *(const float*)(cam + 0x17c);
        *p = *(const float*)(cam + 0x180);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

// The camera object's orientation (+0x70, camera -> world rows) together with its look.
struct CamSnap { float r[9], yaw, pitch; };
static bool read_snap(LONG_PTR cam, CamSnap* s) {
    __try {
        const float* o = (const float*)(cam + 0x70);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) s->r[i * 3 + j] = o[i * 4 + j];
        s->yaw = *(const float*)(cam + 0x17c);
        s->pitch = *(const float*)(cam + 0x180);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}
static CamSnap g_snap[4];  // its state at the last few first draws (in the car)
static unsigned g_snapN;

static bool fed_recently(float y, float p) {
    for (int i = 0; i < 4; i++) if (std::fabs(g_fedYaw[i] - y) < 1e-6f && std::fabs(g_fedPitch[i] - p) < 1e-6f) return true;
    return false;
}

// First draw of a frame: the game has updated (or skipped updating) its camera by now. If it used
// a different look than the eye matrices assume, redo them before anything 3D is drawn.
static void check_applied_look() {
    g_lookChecked = true;
    LONG_PTR cam = g_camObj;
    float y, p;
    if (!cam && g_nCand > 0 && !g_scanHold) {
        // After the hold, the real camera moves on to the freshly fed looks; stale copies keep the
        // held values. Drop anything showing other values; take the first to follow 8 times.
        int keep = 0;
        for (int i = 0; i < g_nCand; i++) {
            if (!read_look(g_cand[i], &y, &p)) continue;
            bool held = std::fabs(y - g_holdYaw) + std::fabs(p - g_holdPitch) < 1e-6f;
            if (!held && !fed_recently(y, p)) continue;
            g_cand[keep] = g_cand[i];
            g_candOk[keep] = (i == keep ? g_candOk[i] : g_candOk[i]) + (held ? 0 : 1);
            if (g_candOk[keep] >= 8 && !cam) cam = g_cand[keep];
            keep++;
        }
        g_nCand = keep;
        if (cam) {
            g_camMisses = 0;
            g_camObj = cam;
            g_nCand = 0;
            log_printf("head look: camera object at %p\n", (void*)cam);
        } else if (!keep || ++g_candFrames > 300) {
            g_nCand = 0;  // nothing followed: search again later
        }
    }
    if (!cam) return;
    bool ok = read_look(cam, &y, &p);
    if (ok && fed_recently(y, p)) g_camMisses = 0;
    else if (++g_camMisses > 60) { g_camObj = 0; log_printf("head look: camera object lost, searching again\n"); }
    float ay = 0, ap = 0;                       // menus and other views: no look was applied
    if (ok && g_camMisses < 10) {               // in the car; it renders with its last look when it skips an update
        ay = y;
        ap = p;
        read_snap(cam, &g_snap[g_snapN++ & 3]);
    }
    if (ay != g_appliedYaw || ap != g_appliedPitch) {
        g_lookFixes++;
        g_appliedYaw = ay;
        g_appliedPitch = ap;
        vr_eye_for_look(g_eye, ay, ap, g_lookTilt, &g_ev);
        new_frame_matrices();
    }
}

static void scan_range(const char* base, SIZE_T size, float y, float p, LONG_PTR* out, int* n, int cap) {
    __try {
        const float* f = (const float*)base;
        for (SIZE_T i = 0x17c / 4, m = size / 4; i + 1 < m; i++)
            if (std::fabs(f[i] - y) < 1e-6f && std::fabs(f[i + 1] - p) < 1e-6f && *n < cap) out[(*n)++] = (LONG_PTR)(f + i) - 0x17c;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
    }
}

extern "C" IMAGE_DOS_HEADER __ImageBase;  // this DLL: its own copies of the look must not count

static DWORD WINAPI cam_finder(void*) {
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    for (;;) {
        Sleep(4000);
        if (g_camObj || g_nCand > 0 || !g_inWorld) continue;
        // Hold an unusual look (the current one, nudged a hair) so the camera holds exactly it.
        float y = g_fedYaw[g_fedN & 3] + 0.000123f, p = g_fedPitch[g_fedN & 3] + 0.000077f;
        g_holdYaw = y;
        g_holdPitch = p;
        g_scanHold = true;
        Sleep(150);
        LONG_PTR found[16];
        int n = 0;
        MEMORY_BASIC_INFORMATION mbi;
        for (const char* a = (const char*)si.lpMinimumApplicationAddress; a < (const char*)si.lpMaximumApplicationAddress;
             a = (const char*)mbi.BaseAddress + mbi.RegionSize) {
            if (!VirtualQuery(a, &mbi, sizeof mbi)) break;
            if (mbi.State == MEM_COMMIT && (mbi.Protect & (PAGE_READWRITE | PAGE_EXECUTE_READWRITE)) && !(mbi.Protect & PAGE_GUARD) &&
                mbi.AllocationBase != (void*)&__ImageBase)
                scan_range((const char*)mbi.BaseAddress, mbi.RegionSize, y, p, found, &n, 16);
        }
        memcpy(g_cand, found, sizeof(LONG_PTR) * n);
        memset(g_candOk, 0, sizeof g_candOk);
        g_candFrames = 0;
        g_nCand = n;  // verified on the game thread once the hold ends
        g_scanHold = false;
    }
}

// The look read back at the first draw is not always the one the game renders with: while the head
// turns, about 15% of the frames that start a new pair (left eye) are drawn with the previous frame's
// camera although the object already holds the new one (measured; never on the other frames). So the
// rendered camera is matched against the object's state now and at the last few first draws, by
// orientation (+0x70: camera -> world rows), and the look of the match is used.
static float snap_diff(const CamSnap& s, const Mat4& cam) {
    float d = 0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            float e = std::fabs(s.r[i * 3 + j] - cam.m[i][j]);
            if (e > d) d = e;
        }
    return d;
}

static const CamSnap* best_state(const Mat4& cam, CamSnap* now, float* bestD) {
    const CamSnap* best = read_snap(g_camObj, now) ? now : nullptr;
    *bestD = best ? snap_diff(*now, cam) : 1e9f;
    for (const auto& s : g_snap) {
        float d = snap_diff(s, cam);
        if (d < *bestD) { *bestD = d; best = &s; }
    }
    return best;
}

static void use_look(const CamSnap* best) {
    if (best->yaw == g_appliedYaw && best->pitch == g_appliedPitch) return;
    g_rcFix[g_eye]++;
    g_appliedYaw = best->yaw;
    g_appliedPitch = best->pitch;
    vr_eye_for_look(g_eye, g_appliedYaw, g_appliedPitch, g_lookTilt, &g_ev);
    new_frame_matrices();
}

// The game's camera for this frame, matched to a camera object state (in the car) to learn the look
// it was drawn with. An InvView or a bare View matrix shows it, but the first comes about 1,100 draws
// into the frame, after most of the world. A draw whose world matrix doesn't rotate (ground pieces;
// in the car the very first 3D draw) shows it right away: its WorldView's 3x3 rows are the view's,
// scaled. That early pick is checked once more at the first InvView/View.
// The match allows about 1 deg: while driving the game sometimes draws a state from between two first
// draws (the car moved a bit), and while the head turns the looks to choose from differ by more.
static void capture_camera(const ShaderMats* s) {
    if (g_camOk && !g_camEarly) return;
    Mat4 C;
    bool exact = false;  // from an InvView / View matrix
    for (auto& m : s->mats) {
        if (m.kind == MK_PRE_E) { C = read_mat(m); exact = true; break; }
        if (m.kind == MK_POST_EYE && m.name.find("World") == std::string::npos && m4_inverse(read_mat(m), C)) { exact = true; break; }
    }
    bool inCar = g_camObj && g_camMisses < 10;
    if (!exact && (g_camOk || !inCar || s->pairV < 0 || s->mats[s->pairV].name.find("World") == std::string::npos ||
                   !m4_cam_from_unrotated(read_mat(s->mats[s->pairV]), &C)))
        return;
    if (!inCar) { g_cam = C; g_camOk = true; return; }  // not in the car: no look to take out
    CamSnap now;
    float d;
    const CamSnap* best = best_state(C, &now, &d);
    if (best && d > 0.02f) best = nullptr;  // over 1 deg from every state: keep the look read at the first draw
    if (g_camEarly) {
        g_camEarly = false;
        if (best && (best->yaw != g_appliedYaw || best->pitch != g_appliedPitch)) { g_earlyBad[g_eye]++; use_look(best); }
        return;
    }
    if (!exact && !best) return;  // a rotated object: try the next draw
    g_cam = C;
    g_camOk = true;
    g_camEarly = !exact;
    g_rcN[g_eye]++;
    g_capSum[g_eye] += g_n3d;
    g_capN[g_eye]++;
    if (d > 2e-3f) g_rcBad[g_eye]++;  // no state matches exactly
    if (best) use_look(best);
}

// Rewrite a 3D draw's camera matrices for this eye. The draw's projection: x/y scales a, b, depth
// terms m22, m32, handedness sgn. Positions come from WorldView - exact (Yexact, for the matrix at
// index exactK) or rebuilt without the depth terms - so the noisy depth terms only touch depth.
static void rewrite(IDirect3DDevice9* d, const ShaderMats* s, float a, float b, float m22, float m32, float sgn,
                    const Mat4* Yexact, int exactK) {
    const Mat4& invEye = sgn > 0 ? g_invEyeLH : g_ev.invEye;
    const Mat4& E = sgn > 0 ? g_ELH : g_ERH;
    const Mat4& Pe = pe_for(m22, m32, sgn);
    for (int i = 0; i < (int)s->mats.size(); i++) {
        const MatSlot& m = s->mats[i];
        Mat4 R;
        switch (m.kind) {
            case MK_POST_K: {  // ...*P -> (...*invEye)*Peye
                Mat4 A = i == exactK ? *Yexact : m4_unproject(read_mat(m), a, b, sgn);
                R = m4_mul(m4_mul(A, invEye), Pe);
                break;
            }
            case MK_POST_EYE: R = m4_mul(read_mat(m), invEye); break;  // ...*View -> ...*View*invEye
            case MK_PROJ: {                                            // P -> Peye
                Mat4 M = read_mat(m);
                R = pe_for(M.m[2][2], M.m[3][2], sgn);
                break;
            }
            default: R = m4_mul(E, read_mat(m)); break;                // InvView -> E*InvView
        }
        write_mat(d, m, R);
    }
}

// A full-screen 2D draw (menu, HUD, cursor): move it onto the virtual screen for this eye.
// Post-processing quads read the game's own render targets and must keep covering the frame.
static void ui_screen(IDirect3DDevice9* d, const ShaderMats* s) {
    restore_stale(d, nullptr);
    if (g_tex0 && g_rtTex.count(g_tex0)) { g_st[R_POSTFX]++; return; }
    g_st[R_UI]++;
    for (auto& m : s->mats)
        if (m.kind == MK_POST_K || m.kind == MK_PROJ) write_mat(d, m, m4_mul(read_mat(m), g_uiU));
}

// Draws the rewrite can't touch (no vertex shader, or one without camera matrices): log each kind once.
static void note_plain(IDirect3DDevice9* d, const ShaderMats* s) {
    g_st[R_PLAIN]++;
    static int n;
    if (n >= 40) return;
    IDirect3DVertexShader9* vs = nullptr;
    DWORD fvf = 0;
    d->lpVtbl->GetVertexShader(d, &vs);
    d->lpVtbl->GetFVF(d, &fvf);
    char key[128];
    snprintf(key, sizeof key, "untouched draw: vertex shader %p (%s), fvf 0x%lx, target %ux%u, viewport %lux%lu", (void*)vs,
             !vs ? "none" : s ? "no camera matrix" : "not seen created", fvf, g_rtW, g_rtH, g_vp.Width, g_vp.Height);
    if (vs) vs->lpVtbl->Release(vs);
    if (g_logged.insert(key).second) { n++; log_printf("%s\n", key); }
}

static void apply_body(IDirect3DDevice9* d) {
    count(C_DRAW);
    if (g_headLook && !g_lookChecked) check_applied_look();
    const ShaderMats* s = g_cur;
    if (!g_vr || !s || s->mats.empty()) {
        if (g_vr) note_plain(d, s);
        restore_stale(d, nullptr);
        return;
    }

    // Main-view passes only: render targets shaped like the back buffer (TDU also renders the
    // scene at half size) and a (nearly) full viewport. Cube/shadow maps have other shapes;
    // mirrors and menu car previews use a sub-viewport.
    bool shapeOK = std::fabs((double)g_rtW * g_bbH - (double)g_rtH * g_bbW) <= 0.02 * g_rtW * g_bbH;
    bool fullVP = g_vp.Width * 10 >= g_rtW * 9 && g_vp.Height * 10 >= g_rtH * 9;
    if (!shapeOK) { g_st[R_SHAPE]++; restore_stale(d, nullptr); return; }

    if (s->pairK >= 0 && s->pairV >= 0) {
        // Exact and cheap: X = Y*P  ->  X' = (Y*invEye) * Peye, with P's depth terms kept.
        Mat4 X = read_mat(s->mats[s->pairK]), Y = read_mat(s->mats[s->pairV]);
        PairInfo pi;
        bool flat = false;
        if (!m4_pair_info(X, Y, &pi, &flat)) {
            if (flat && fullVP) { ui_screen(d, s); return; }
            g_st[flat ? R_NOTPERSP : R_FORM]++;
            restore_stale(d, nullptr);
            return;
        }
        if (!fullVP) { g_st[R_VIEWPORT]++; restore_stale(d, nullptr); return; }
        capture_camera(s);
        g_n3d++;
        vote(pi);
        note_camera(pi);
        restore_stale(d, s);
        g_st[R_PAIR]++;
        rewrite(d, s, pi.a, pi.b, pi.m22, pi.m32, pi.sgn, &Y, s->pairK);
        return;
    }

    Mat4 Pd;
    bool flat;
    if (!draw_projection(s, &Pd, &flat)) {
        if (flat && fullVP) { ui_screen(d, s); return; }
        if (flat) g_st[R_NOTPERSP]++;
        restore_stale(d, nullptr);
        return;
    }
    if (!fullVP) { g_st[R_VIEWPORT]++; restore_stale(d, nullptr); return; }
    capture_camera(s);
    g_n3d++;
    restore_stale(d, s);
    g_st[R_OTHER]++;
    rewrite(d, s, Pd.m[0][0], Pd.m[1][1], Pd.m[2][2], Pd.m[3][2], Pd.m[2][3] > 0 ? 1.0f : -1.0f, nullptr, -1);
}

// Called before every draw.
static void apply(IDirect3DDevice9* d) {
    double t0 = now_s();
    apply_body(d);
    g_applyMs += (now_s() - t0) * 1000;
}

// ---- back buffer -> headset ----
static void release_copies() {
    for (auto& s : g_slot) {
        if (s.rt) s.rt->lpVtbl->Release(s.rt);
        if (s.sys) s.sys->lpVtbl->Release(s.sys);
        if (s.q) s.q->lpVtbl->Release(s.q);
        s = Slot{};
    }
}

static void read_bb_size(IDirect3DDevice9* d) {
    IDirect3DSurface9* bb = nullptr;
    if (FAILED(d->lpVtbl->GetBackBuffer(d, 0, 0, D3DBACKBUFFER_TYPE_MONO, &bb))) return;
    D3DSURFACE_DESC dd;
    bb->lpVtbl->GetDesc(bb, &dd);
    g_bbW = g_rtW = dd.Width;
    g_bbH = g_rtH = dd.Height;
    g_vp = D3DVIEWPORT9{ 0, 0, dd.Width, dd.Height, 0.0f, 1.0f };
    bb->lpVtbl->Release(bb);
    log_printf("back buffer %ux%u fmt %d ms %d\n", dd.Width, dd.Height, dd.Format, dd.MultiSampleType);
}

// TDUVR_DUMP=<file.bmp>[,<seconds>]: write one eye image, exactly as sent to the headset, for checking.
// ",+<seconds>" counts from when head look found the in-car camera (the attract demo starts at random times).
static void maybe_dump(const void* bits, int pitch, int w, int h) {
    static bool done;
    static double t0 = now_s(), tCam;
    char env[MAX_PATH + 16];
    if (done || GetEnvironmentVariableA("TDUVR_DUMP", env, sizeof env) == 0) return;
    char* comma = strrchr(env, ',');
    double after = comma ? atof(comma + 1) : 60;
    if (comma && comma[1] == '+') {
        if (!g_camObj) return;
        if (!tCam) tCam = now_s();
        if (now_s() - tCam < after) return;
    } else if (now_s() - t0 < after) {
        return;
    }
    if (comma) *comma = 0;
    done = true;
    FILE* f = nullptr;
    if (fopen_s(&f, env, "wb") || !f) return;
    BITMAPFILEHEADER fh{};
    BITMAPINFOHEADER ih{};
    ih.biSize = sizeof ih; ih.biWidth = w; ih.biHeight = -h; ih.biPlanes = 1; ih.biBitCount = 32;
    fh.bfType = 0x4D42; fh.bfOffBits = sizeof fh + sizeof ih; fh.bfSize = fh.bfOffBits + (DWORD)w * h * 4;
    fwrite(&fh, sizeof fh, 1, f);
    fwrite(&ih, sizeof ih, 1, f);
    for (int y = 0; y < h; y++) fwrite((const char*)bits + (size_t)pitch * y, 4, w, f);
    fclose(f);
    log_printf("dumped eye image %dx%d to %s\n", w, h, env);
}

// Wait for a queued copy (normally already done a frame later) and send it to the headset.
static void finish_slot(Slot& s) {
    double t = now_s();
    while (s.q->lpVtbl->GetData(s.q, nullptr, 0, D3DGETDATA_FLUSH) == S_FALSE) SwitchToThread();
    double t1 = now_s();
    g_copyWaitMs += (t1 - t) * 1000;
    D3DLOCKED_RECT lr;
    if (SUCCEEDED(s.sys->lpVtbl->LockRect(s.sys, &lr, nullptr, D3DLOCK_READONLY))) {
        vr_publish(s.eye, s.pair, s.pose, lr.pBits, lr.Pitch, (int)g_copyW, (int)g_copyH);
        maybe_dump(lr.pBits, lr.Pitch, (int)g_copyW, (int)g_copyH);
        s.sys->lpVtbl->UnlockRect(s.sys);
    }
    g_submitMs += (now_s() - t1) * 1000;
    s.pending = false;
}

// shortcut: still a CPU copy (GPU -> system memory -> D3D11), just not stalling any more; move to
// D3D9On12 (GPU-only copy) if high resolutions stay slow.
static void submit_backbuffer(IDirect3DDevice9* d) {
    IDirect3DSurface9* bb = nullptr;
    if (FAILED(d->lpVtbl->GetBackBuffer(d, 0, 0, D3DBACKBUFFER_TYPE_MONO, &bb))) return;
    D3DSURFACE_DESC dd;
    bb->lpVtbl->GetDesc(bb, &dd);
    if (dd.Format != D3DFMT_X8R8G8B8 && dd.Format != D3DFMT_A8R8G8B8) {
        static bool once = (log_printf("unsupported back buffer format %d\n", dd.Format), true);
        bb->lpVtbl->Release(bb);
        return;
    }
    if (!g_slot[0].rt || g_copyW != dd.Width || g_copyH != dd.Height || g_copyFmt != dd.Format) {
        release_copies();
        for (auto& s : g_slot) {
            d->lpVtbl->CreateRenderTarget(d, dd.Width, dd.Height, dd.Format, D3DMULTISAMPLE_NONE, 0, FALSE, &s.rt, nullptr);
            d->lpVtbl->CreateOffscreenPlainSurface(d, dd.Width, dd.Height, dd.Format, D3DPOOL_SYSTEMMEM, &s.sys, nullptr);
            d->lpVtbl->CreateQuery(d, D3DQUERYTYPE_EVENT, &s.q);
        }
        g_copyW = dd.Width;
        g_copyH = dd.Height;
        g_copyFmt = dd.Format;
    }

    Slot& cur = g_slot[g_frame & 1];
    Slot& prev = g_slot[(g_frame & 1) ^ 1];
    if (cur.rt && cur.sys && cur.q) {
        if (cur.pending) finish_slot(cur);
        HRESULT h1 = d->lpVtbl->StretchRect(d, bb, nullptr, cur.rt, nullptr, D3DTEXF_NONE);
        HRESULT h2 = SUCCEEDED(h1) ? d->lpVtbl->GetRenderTargetData(d, cur.rt, cur.sys) : E_FAIL;
        if (SUCCEEDED(h2)) {
            cur.q->lpVtbl->Issue(cur.q, D3DISSUE_END);
            cur.pending = true;
            cur.eye = g_eye;
            cur.pair = g_frame >> 1;
            cur.pose = g_framePose;
        } else {
            static int fails;
            if (fails++ < 5) log_printf("readback failed: stretch 0x%08lx getrtdata 0x%08lx\n", h1, h2);
        }
    } else {
        static bool once = (log_printf("could not create readback surfaces %ux%u\n", dd.Width, dd.Height), true);
    }
    bb->lpVtbl->Release(bb);
    if (prev.pending && prev.q) finish_slot(prev);  // last frame's image: had a whole frame to copy
}

// ---- device hooks ----
static bool g_inPresent;  // device Present may call swap chain Present internally: submit once
static IDirect3DDevice9* volatile g_watchDev;  // the game's device (the one the mod works on)

// TDUVR_LOOKTEST: cycle fixed head-look angles (2 s each) and log the camera the game produced,
// to check sign, limits and smoothing of the game's head-look.
static void look_test(float* yaw, float* pitch) {
    char alt[16];
    if (GetEnvironmentVariableA("TDUVR_LOOKTEST_ALT", alt, sizeof alt) > 0) {
        // Alternate the yaw every frame: does the camera follow on every frame? (Starts after the
        // given number of seconds, once the game is showing the in-car camera.)
        static float prev;
        static int n;
        static double t0 = now_s(), tPrev;
        double tn = now_s();
        if (g_camOk && n < 400 && tn - t0 > atof(alt))
            log_printf("alt test %d: fed %.2f dt %.2f ms -> cam fwd %+.4f %+.4f %+.4f\n", n++, prev, (tn - tPrev) * 1000,
                       -g_cam.m[2][0], -g_cam.m[2][1], -g_cam.m[2][2]);
        tPrev = tn;
        prev = (g_frame & 1) ? 0.2f : 0.0f;
        *yaw = prev;
        *pitch = 0;
        return;
    }
    static const float phases[][2] = { { 0, 0 }, { 0.5f, 0 }, { 0, 0 }, { 0, 0.3f }, { 0, 0 }, { 3.0f, 0 }, { 0, 1.4f }, { 0, -1.4f }, { 0.5f, 0.3f } };
    static const int n = sizeof phases / sizeof phases[0];
    static double t0 = now_s();
    static int last = -1, framesIn;
    int ph = (int)((now_s() - t0) / 2.0) % n;
    if (ph != last) framesIn = 0;
    // The camera seen this frame was made from the angles written at the previous Present.
    if (last >= 0 && g_camOk && (framesIn < 5 || ph != last)) {
        const Mat4& c = g_cam;
        log_printf("look test: fed yaw %+.3f pitch %+.3f (frame %d of phase) -> cam fwd %+.3f %+.3f %+.3f up %+.3f %+.3f %+.3f pos %.2f %.2f %.2f\n",
                   phases[last][0], phases[last][1], framesIn, -c.m[2][0], -c.m[2][1], -c.m[2][2], c.m[1][0], c.m[1][1], c.m[1][2],
                   c.m[3][0], c.m[3][1], c.m[3][2]);
    }
    framesIn++;
    last = ph;
    *yaw = phases[ph][0];
    *pitch = phases[ph][1];
}

// AutoDrive (tdu_vr.ini): at startup, continue the saved game and leave the house for the car, so no
// menu has to be used in the headset. Presses the keys a player would (the game reads the keyboard
// through DirectInput, so window messages don't work), held 0.4 s, only while the game window is in
// front, so nothing is ever typed elsewhere:
// - Enter on the main menu ("Kontynuuj grę" is preselected) once it has been up for 8 s, again every
//   8 s while it stays (a press while it still animates in is ignored);
// - Backspace ("Wyjście") once, 10 s after the house shows up. Not repeated: the car looks the same.
static int g_autoStage;  // 1 at the main menu, 2 in the house, 0 done or off
static HWND g_gameWnd;

static void auto_drive(unsigned draws, unsigned draws3d) {
    static double tStart = now_s(), tSince = tStart, tUp;
    static BYTE held;
    double t = now_s();
    if (held && t >= tUp) {
        keybd_event(held, (BYTE)MapVirtualKeyA(held, MAPVK_VK_TO_VSC), KEYEVENTF_KEYUP, 0);
        held = 0;
    }
    if (!g_autoStage || held) return;
    if (t - tStart > 300) { g_autoStage = 0; log_printf("auto drive: gave up\n"); return; }
    // The main menu is a small 3D scene (about 120 draws); the house a big one (thousands).
    bool menu = draws3d > 0 && draws < 1000, house = draws > 1000;
    if (g_autoStage == 1 && house) { g_autoStage = 2; tSince = t; }  // the saved game has loaded
    if (!(g_autoStage == 1 ? menu : house)) tSince = t;
    if (t - tSince < (g_autoStage == 1 ? 8 : 10) || GetForegroundWindow() != g_gameWnd) return;
    held = g_autoStage == 1 ? VK_RETURN : VK_BACK;
    keybd_event(held, (BYTE)MapVirtualKeyA(held, MAPVK_VK_TO_VSC), 0, 0);
    tUp = t + 0.4;
    log_printf("auto drive: %s\n", g_autoStage == 1 ? "main menu, pressing Enter (continue)" : "house, pressing Backspace (exit to the car)");
    if (g_autoStage == 2) g_autoStage = 0;
    tSince = t;
}

static void on_present(IDirect3DDevice9* d) {
    log_status();
    if (g_vr) {
        static unsigned lastDraws, last3d;
        unsigned n3d = g_st[R_PAIR] + g_st[R_OTHER];
        auto_drive(g_cnt[C_DRAW] - lastDraws, n3d - last3d);
        lastDraws = g_cnt[C_DRAW];
        last3d = n3d;
        submit_backbuffer(d);
        static bool prevKey;
        bool key = (GetAsyncKeyState(VK_F8) & 0x8000) != 0;
        if (key && !prevKey) vr_recenter();
        prevKey = key;
        elect_main();
        g_eye ^= 1;
        g_frame++;
        vr_begin_frame(g_eye, g_eye == 0, g_maxYaw, g_maxPitch, g_lookTilt, &g_ev, &g_framePose);
        float yaw = 0, pitch = 0;
        bool inCar = g_camObj && g_camMisses < 10;  // the in-car camera follows the fed look (it is on screen)
        if (g_headLook) {
            // Until the in-car camera is found it is kept facing forward, so no look needs taking out
            // whichever camera the game shows. Menus and other views ignore what is fed.
            if (g_camObj) {
                yaw = g_ev.lookYaw;
                pitch = g_ev.lookPitch;
            } else {
                yaw = 1e-4f * (1 + (g_fedN & 7));  // under 0.05 deg, but changing, so the camera scan can tell it apart
            }
            if (g_lookTest) look_test(&yaw, &pitch);
            else if (g_scanHold) { yaw = g_holdYaw; pitch = g_holdPitch; }
            *kLookYaw = g_lookSignYaw * yaw;  // used by the game's camera update for the next frame
            *kLookPitch = g_lookSignPitch * pitch;
            *kLookOn = 1;
            g_fedN++;
            g_fedYaw[g_fedN & 3] = yaw;
            g_fedPitch[g_fedN & 3] = pitch;
        }
        // Assume the in-car camera applies what we just fed until it says otherwise (first draw).
        g_appliedYaw = inCar ? yaw : 0;
        g_appliedPitch = inCar ? pitch : 0;
        vr_eye_for_look(g_eye, g_appliedYaw, g_appliedPitch, g_lookTilt, &g_ev);
        new_frame_matrices();
        static unsigned lastCam;
        if (g_camOk) lastCam = g_frame;
        g_inWorld = g_frame - lastCam < 60;
        g_lookChecked = false;
        g_camOk = false;
        g_camEarly = false;
        g_n3d = 0;
    } else {
        g_frame++;
    }
}

// In menus the game never clears the screen: its full-screen 2D background normally covers it. On the
// virtual screen that background covers only part of the frame, so the rest kept older frames (the
// other eye's image, seen as a second menu). Cleared after each Present (the copy for the headset is
// queued before it).
static void clear_frame(IDirect3DDevice9* d) {
    IDirect3DSurface9 *bb = nullptr, *rt = nullptr;
    if (FAILED(d->lpVtbl->GetBackBuffer(d, 0, 0, D3DBACKBUFFER_TYPE_MONO, &bb))) return;
    if (SUCCEEDED(d->lpVtbl->GetRenderTarget(d, 0, &rt)) && rt == bb) {  // otherwise the game is mid-way through something
        D3DVIEWPORT9 vp, full = { 0, 0, g_bbW, g_bbH, 0.0f, 1.0f };
        d->lpVtbl->GetViewport(d, &vp);
        H(d).SetViewport(d, &full);
        d->lpVtbl->Clear(d, 0, nullptr, D3DCLEAR_TARGET, 0, 1.0f, 0);
        H(d).SetViewport(d, &vp);
    }
    if (rt) rt->lpVtbl->Release(rt);
    bb->lpVtbl->Release(bb);
}

static HRESULT STDMETHODCALLTYPE hPresent(IDirect3DDevice9* d, const RECT* a, const RECT* b, HWND w, const RGNDATA* r) {
    count(C_PRESENT);
    on_present(d);
    g_inPresent = true;
    HRESULT hr = H(d).Present(d, a, b, w, r);
    g_inPresent = false;
    if (g_vr && SUCCEEDED(hr)) clear_frame(d);
    return hr;
}

static decltype(IDirect3DSwapChain9Vtbl::Present) oSCPresent;
static HRESULT STDMETHODCALLTYPE hSCPresent(IDirect3DSwapChain9* sc, const RECT* a, const RECT* b, HWND w, const RGNDATA* r, DWORD f) {
    if (g_inPresent) return oSCPresent(sc, a, b, w, r, f);
    count(C_SCPRESENT);
    IDirect3DDevice9* d = nullptr;
    if (FAILED(sc->lpVtbl->GetDevice(sc, &d)) || !d) return oSCPresent(sc, a, b, w, r, f);
    if (d != g_watchDev) { d->lpVtbl->Release(d); return oSCPresent(sc, a, b, w, r, f); }  // another device's swap chain (shared vtable)
    on_present(d);
    HRESULT hr = oSCPresent(sc, a, b, w, r, f);
    if (g_vr && SUCCEEDED(hr)) clear_frame(d);
    d->lpVtbl->Release(d);
    return hr;
}

// The game has no windowed/vsync options, so force them: SteamVR paces frames, and windowed
// mode stops the game losing its device when the headset/SteamVR takes focus.
static void force_pp(D3DPRESENT_PARAMETERS* pp) {
    if (!g_vr || !pp) return;
    pp->PresentationInterval = D3DPRESENT_INTERVAL_IMMEDIATE;
    pp->Windowed = TRUE;
    pp->FullScreen_RefreshRateInHz = 0;  // must be 0 when windowed
}

static void hook_device(IDirect3DDevice9* d);

static HRESULT STDMETHODCALLTYPE hReset(IDirect3DDevice9* d, D3DPRESENT_PARAMETERS* pp) {
    release_copies();
    g_rtTex.clear();  // render targets must be released before Reset; the game recreates them after
    g_tex0 = nullptr;
    force_pp(pp);
    HRESULT hr = H(d).Reset(d, pp);
    log_printf("Reset hr=0x%08lx %ux%u fmt %d windowed %d\n", hr, pp ? pp->BackBufferWidth : 0, pp ? pp->BackBufferHeight : 0,
               pp ? pp->BackBufferFormat : 0, pp ? pp->Windowed : -1);
    for (int r : g_mod) g_isMod[r] = false;
    g_mod.clear();
    if (SUCCEEDED(hr)) {
        hook_device(d);  // in case the runtime rebuilt the table
        read_bb_size(d);
    }
    return hr;
}

static HRESULT STDMETHODCALLTYPE hCreateVertexShader(IDirect3DDevice9* d, const DWORD* fn, IDirect3DVertexShader9** out) {
    count(C_CVS);
    HRESULT hr = H(d).CreateVertexShader(d, fn, out);
    if (SUCCEEDED(hr) && fn && out && *out) {
        auto ignored = [](const std::string& n, int reg, int cnt) {
            if ((n.find("View") != std::string::npos || n.find("Proj") != std::string::npos) && g_logged.insert("-" + n + std::to_string(cnt)).second)
                log_printf("  ignored matrix %s c%d x%d\n", n.c_str(), reg, cnt);
        };
        ShaderMats sm = ctab_parse((const uint32_t*)fn, 1u << 16, ignored);
        for (auto& m : sm.mats)
            if (g_logged.insert(m.name + std::to_string(m.count)).second)
                log_printf("  camera matrix %s c%d x%d kind %d\n", m.name.c_str(), m.reg, m.count, m.kind);
        if (!sm.mats.empty()) count(C_CVS_MATS);
        else if (g_cnt[C_CVS] <= 3) log_printf("  shader without camera matrices, tokens %08lx %08lx\n", fn[0], fn[1]);
        g_shaders[*out] = std::move(sm);
    }
    return hr;
}

static HRESULT STDMETHODCALLTYPE hSetVertexShader(IDirect3DDevice9* d, IDirect3DVertexShader9* s) {
    count(C_SVS);
    auto it = s ? g_shaders.find(s) : g_shaders.end();
    g_cur = it == g_shaders.end() ? nullptr : &it->second;
    return H(d).SetVertexShader(d, s);
}

static HRESULT STDMETHODCALLTYPE hSetVSConstF(IDirect3DDevice9* d, UINT start, const float* data, UINT count) {
    ::count(C_SVSC);
    for (UINT i = 0; i < count && start + i < MAXC; i++) {
        memcpy(g_vsc[start + i], data + i * 4, 16);
        g_isMod[start + i] = false;  // hardware now holds the game's value again
    }
    if (!g_mod.empty()) {
        size_t w = 0;
        for (int r : g_mod) if (g_isMod[r]) g_mod[w++] = r;
        g_mod.resize(w);
    }
    return H(d).SetVertexShaderConstantF(d, start, data, count);
}

static HRESULT STDMETHODCALLTYPE hSetRenderTarget(IDirect3DDevice9* d, DWORD idx, IDirect3DSurface9* s) {
    count(C_SRT);
    if (idx == 0 && s) {
        D3DSURFACE_DESC dd;
        if (SUCCEEDED(s->lpVtbl->GetDesc(s, &dd))) {
            g_rtW = dd.Width; g_rtH = dd.Height;
            g_vp = D3DVIEWPORT9{ 0, 0, dd.Width, dd.Height, 0.0f, 1.0f };  // D3D9 resets the viewport to the new target
            if (g_logged.size() < 400 && g_logged.insert("rt" + std::to_string(dd.Width) + "x" + std::to_string(dd.Height)).second)
                log_printf("render target %ux%u fmt %d\n", dd.Width, dd.Height, dd.Format);
        }
    }
    return H(d).SetRenderTarget(d, idx, s);
}

static HRESULT STDMETHODCALLTYPE hSetViewport(IDirect3DDevice9* d, const D3DVIEWPORT9* vp) {
    if (vp) g_vp = *vp;
    return H(d).SetViewport(d, vp);
}

static HRESULT STDMETHODCALLTYPE hCreateTexture(IDirect3DDevice9* d, UINT w, UINT h, UINT levels, DWORD usage, D3DFORMAT fmt,
                                                D3DPOOL pool, IDirect3DTexture9** out, HANDLE* shared) {
    HRESULT hr = H(d).CreateTexture(d, w, h, levels, usage, fmt, pool, out, shared);
    if (SUCCEEDED(hr) && out && *out && (usage & D3DUSAGE_RENDERTARGET)) g_rtTex.insert(*out);
    return hr;
}

static HRESULT STDMETHODCALLTYPE hSetTexture(IDirect3DDevice9* d, DWORD stage, IDirect3DBaseTexture9* t) {
    if (stage == 0) g_tex0 = t;
    return H(d).SetTexture(d, stage, t);
}

static HRESULT STDMETHODCALLTYPE hDrawPrimitive(IDirect3DDevice9* d, D3DPRIMITIVETYPE t, UINT a, UINT b) {
    apply(d); return H(d).DrawPrimitive(d, t, a, b);
}
static HRESULT STDMETHODCALLTYPE hDrawIndexedPrimitive(IDirect3DDevice9* d, D3DPRIMITIVETYPE t, INT a, UINT b, UINT c, UINT e, UINT f) {
    apply(d); return H(d).DrawIndexedPrimitive(d, t, a, b, c, e, f);
}
static HRESULT STDMETHODCALLTYPE hDrawPrimitiveUP(IDirect3DDevice9* d, D3DPRIMITIVETYPE t, UINT a, const void* b, UINT c) {
    apply(d); return H(d).DrawPrimitiveUP(d, t, a, b, c);
}
static HRESULT STDMETHODCALLTYPE hDrawIndexedPrimitiveUP(IDirect3DDevice9* d, D3DPRIMITIVETYPE t, UINT a, UINT b, UINT c, const void* e, D3DFORMAT f, const void* g, UINT h) {
    apply(d); return H(d).DrawIndexedPrimitiveUP(d, t, a, b, c, e, f, g, h);
}

static void hook_device(IDirect3DDevice9* d) {
    auto* vt = d->lpVtbl;
    log_printf("device %p vtable %p in %s, Present in %s, Draw in %s\n", (void*)d, (const void*)vt,
               module_of(vt).c_str(), module_of((const void*)vt->Present).c_str(), module_of((const void*)vt->DrawIndexedPrimitive).c_str());
    IDirect3DSwapChain9* sc = nullptr;
    if (SUCCEEDED(vt->GetSwapChain(d, 0, &sc)) && sc) {
        if (sc->lpVtbl->Present != hSCPresent) {
            // shortcut: all swap chain instances assumed to share one Present in d3d9.dll; keep per-vtable originals if logs show otherwise
            decltype(oSCPresent) orig;
            hook(&sc->lpVtbl->Present, hSCPresent, &orig);
            if (!oSCPresent) oSCPresent = orig;
        }
        sc->lpVtbl->Release(sc);
    }
    if (vt->Present == hPresent) return;  // already hooked
    int n = g_nOrig < MAXORIG ? g_nOrig : MAXORIG, slot = -1;
    for (int i = 0; i < n; i++) if (g_orig[i].vt == vt) slot = i;  // reused address: refill
    bool fresh = slot < 0;
    if (fresh) slot = g_nOrig % MAXORIG;
    DevOrig& o = g_orig[slot];
    o.vt = vt;
    hook(&vt->Present, hPresent, &o.Present);
    hook(&vt->Reset, hReset, &o.Reset);
    hook(&vt->CreateVertexShader, hCreateVertexShader, &o.CreateVertexShader);
    hook(&vt->SetVertexShader, hSetVertexShader, &o.SetVertexShader);
    hook(&vt->SetVertexShaderConstantF, hSetVSConstF, &o.SetVertexShaderConstantF);
    hook(&vt->SetRenderTarget, hSetRenderTarget, &o.SetRenderTarget);
    hook(&vt->SetViewport, hSetViewport, &o.SetViewport);
    hook(&vt->CreateTexture, hCreateTexture, &o.CreateTexture);
    hook(&vt->SetTexture, hSetTexture, &o.SetTexture);
    hook(&vt->DrawPrimitive, hDrawPrimitive, &o.DrawPrimitive);
    hook(&vt->DrawIndexedPrimitive, hDrawIndexedPrimitive, &o.DrawIndexedPrimitive);
    hook(&vt->DrawPrimitiveUP, hDrawPrimitiveUP, &o.DrawPrimitiveUP);
    hook(&vt->DrawIndexedPrimitiveUP, hDrawIndexedPrimitiveUP, &o.DrawIndexedPrimitiveUP);
    if (fresh) g_nOrig++;  // publish only after the table is filled
    log_printf("hooked device vtable %p (slot %d)\n", (const void*)vt, slot);
}

// The runtime can repoint a device's lpVtbl at a fresh table after setup, dropping our hooks.
// A watcher re-hooks the newest device (g_watchDev) whenever its Present slot is not ours.

static const void* peek_vtbl(IDirect3DDevice9* d, bool* hooked) {
    __try {
        *hooked = d->lpVtbl->Present == hPresent;
        return d->lpVtbl;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return nullptr;  // device freed
    }
}

static DWORD WINAPI watch_thread(void*) {
    for (;;) {
        Sleep(100);
        IDirect3DDevice9* d = g_watchDev;
        if (!d) continue;
        bool hooked = true;
        const void* vt = peek_vtbl(d, &hooked);
        if (!vt) { g_watchDev = nullptr; continue; }
        if (!hooked) {
            log_printf("device %p vtable now %p, re-hooking\n", (void*)d, vt);
            hook_device(d);
        }
    }
}

static HRESULT STDMETHODCALLTYPE hCreateDevice(IDirect3D9* This, UINT ad, D3DDEVTYPE t, HWND w, DWORD f, D3DPRESENT_PARAMETERS* pp, IDirect3DDevice9** out) {
    static bool tried;
    if (!tried) {
        tried = true;
        g_vr = vr_init();
        if (g_vr) {
            g_uiFov = vr_ini_float("UiFov", 70.0f);
            g_uiDist = vr_ini_float("UiDistance", 1.5f);
            if (!(g_uiFov >= 20 && g_uiFov <= 120)) g_uiFov = 70.0f;
            if (!(g_uiDist >= 0.3f && g_uiDist <= 20)) g_uiDist = 1.5f;
            log_printf("UI screen: %.0f deg wide at %.2f m\n", g_uiFov, g_uiDist);
            g_headLook = vr_ini_float("HeadLook", 1) != 0 && headlook_verify();
            g_lookTest = GetEnvironmentVariableA("TDUVR_LOOKTEST", nullptr, 0) > 0;
            log_printf("head look: %s%s\n", g_headLook ? "on" : "off (setting or unknown exe)", g_lookTest ? ", TEST MODE" : "");
            if (g_headLook && !g_lookTest) CreateThread(nullptr, 0, cam_finder, nullptr, 0, nullptr);
            g_autoStage = vr_ini_float("AutoDrive", 1) != 0 ? 1 : 0;
            log_printf("auto drive: %s\n", g_autoStage ? "on" : "off");
            vr_begin_frame(0, true, g_maxYaw, g_maxPitch, g_lookTilt, &g_ev, &g_framePose);
            vr_eye_for_look(0, 0, 0, g_lookTilt, &g_ev);
            new_frame_matrices();
        }
    }
    auto real = g_d3dOrig[0].CreateDevice;
    for (int i = 0; i < g_nD3D && i < 16; i++) if (g_d3dOrig[i].vt == This->lpVtbl) real = g_d3dOrig[i].CreateDevice;
    // In the house the game opens a tiny second device (120x61) while the main one keeps drawing.
    // Taking it over reset the mod's state (back buffer size, copies) and crashed in d3d9's shared
    // vtable, so a device too small to be the game's view is left alone.
    UINT bw = pp ? pp->BackBufferWidth : 0;
    HWND cw = pp && pp->hDeviceWindow ? pp->hDeviceWindow : w;
    RECT cr;
    if (!bw && cw && GetClientRect(cw, &cr)) bw = (UINT)(cr.right - cr.left);
    if (g_vr && bw && bw < 320) {
        HRESULT hr = real(This, ad, t, w, f, pp, out);
        log_printf("small extra device %ux%u (hr=0x%08lx): left alone\n", bw, pp ? pp->BackBufferHeight : 0, hr);
        return hr;
    }
    force_pp(pp);
    if (w) g_gameWnd = w;
    else if (pp && pp->hDeviceWindow) g_gameWnd = pp->hDeviceWindow;
    release_copies();  // they belong to the previous device
    for (int r : g_mod) g_isMod[r] = false;
    g_mod.clear();
    g_cur = nullptr;
    HRESULT hr = real(This, ad, t, w, f, pp, out);
    log_printf("CreateDevice hr=0x%08lx type %d flags 0x%lx %ux%u windowed %d\n", hr, t, f,
               pp ? pp->BackBufferWidth : 0, pp ? pp->BackBufferHeight : 0, pp ? pp->Windowed : -1);
    // No headset: stay out of the game's way. (Fullscreen d3d9 swaps the device's vtable to its shared
    // one and frees devices on mode switches; the watcher re-hooking those crashed the game.)
    if (SUCCEEDED(hr) && out && *out && g_vr) {
        hook_device(*out);
        g_watchDev = *out;
        static HANDLE th = CreateThread(nullptr, 0, watch_thread, nullptr, 0, nullptr);
        read_bb_size(*out);
    }
    return hr;
}

// ---- crash/exit diagnostics (TDU's own crash handler exits silently) ----
static LONG CALLBACK on_exception(EXCEPTION_POINTERS* e) {
    DWORD code = e->ExceptionRecord->ExceptionCode;
    // Skip benign ones: C++ throw, thread naming, debug output, and SecuROM's anti-debug tricks
    // (illegal instruction, single step, breakpoint).
    if (code == 0xE06D7363 || code == 0x406D1388 || code == 0x40010006 || code == 0x4001000A ||
        code == 0xC000001D || code == 0x80000004 || code == 0x80000003)
        return EXCEPTION_CONTINUE_SEARCH;
    static int n;
    if (n++ < 40) {
        void* at = e->ExceptionRecord->ExceptionAddress;
        HMODULE m = nullptr;
        GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCSTR)at, &m);
        log_printf("exception 0x%08lx at %p (%s+0x%lx)%s\n", code, at, module_of(at).c_str(),
                   (unsigned long)((char*)at - (char*)m), code == 0xC0000005 ? " ACCESS VIOLATION" : "");
    }
    return EXCEPTION_CONTINUE_SEARCH;
}

BOOL WINAPI DllMain(HINSTANCE, DWORD reason, LPVOID) {
    if (reason == DLL_PROCESS_ATTACH) {
        log_printf("loaded, cmdline: %s\n", GetCommandLineA());
        AddVectoredExceptionHandler(1, on_exception);
    } else if (reason == DLL_PROCESS_DETACH) {
        log_printf("process exiting (frame %u)\n", g_frame);
    }
    return TRUE;
}

// ---- exports ----
static HMODULE real_d3d9() {
    static HMODULE h;
    if (!h) {
        char p[MAX_PATH];
        GetSystemDirectoryA(p, MAX_PATH);  // SysWOW64 for this 32-bit process
        strcat_s(p, "\\d3d9.dll");
        h = LoadLibraryA(p);
    }
    return h;
}

extern "C" IDirect3D9* WINAPI Direct3DCreate9(UINT sdk) {
    auto fn = (IDirect3D9*(WINAPI*)(UINT))GetProcAddress(real_d3d9(), "Direct3DCreate9");
    IDirect3D9* d3d = fn ? fn(sdk) : nullptr;
    log_printf("Direct3DCreate9 -> %p vtable %p\n", (void*)d3d, d3d ? (const void*)d3d->lpVtbl : nullptr);
    if (d3d && d3d->lpVtbl->CreateDevice != hCreateDevice) {
        int slot = -1;
        for (int i = 0; i < g_nD3D && i < 16; i++) if (g_d3dOrig[i].vt == d3d->lpVtbl) slot = i;
        bool fresh = slot < 0;
        if (fresh) slot = g_nD3D % 16;
        g_d3dOrig[slot].vt = d3d->lpVtbl;
        hook(&d3d->lpVtbl->CreateDevice, hCreateDevice, &g_d3dOrig[slot].CreateDevice);
        if (fresh) g_nD3D++;
    }
    return d3d;
}

extern "C" void WINAPI D3DPERF_SetOptions(DWORD opt) {
    auto fn = (void(WINAPI*)(DWORD))GetProcAddress(real_d3d9(), "D3DPERF_SetOptions");
    if (fn) fn(opt);
}
