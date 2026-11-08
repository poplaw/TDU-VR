// d3d9.dll proxy for Test Drive Unlimited: hooks the real device's vtable, rewrites the
// camera matrices in vertex shader constants per draw (6DOF head pose + per-eye frustum),
// alternates eyes each frame and hands the back buffer to SteamVR.
#define CINTERFACE
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <d3d9.h>
#include <algorithm>
#include <unordered_map>
#include <set>
#include <vector>
#include "ctab.h"
#include "gpu.h"
#include "vk.h"
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

// ---- the game's simulation (2007 exe) ----
// The game steps its simulation (cars, physics, cameras) apart from drawing: a multimedia timer calls
// 0x81ee50 every 10 ms (100 Hz from its config, [0xf87ffc]), which runs the step 0x81dfd0(game, dt, 0)
// with dt = the real time since the last step. Frames show whichever step finished last.
static volatile unsigned __int64* const kSimSteps = (volatile unsigned __int64*)0x10e7a50;  // steps the timer has run
static const volatile float* const kSimAvgS = (const volatile float*)0x10e7a58;             // their average time, s (1 ms clock, +1 ms)
static bool g_knownExe;            // the 2007 exe these addresses were checked against
static unsigned g_stepHist[4];     // frames by how many simulation steps ran since the previous one: 0, 1, 2, 3+
static std::vector<float> g_frameMs;  // frame times in this status window

// SimLock (tdu_vr.ini): that timer steps at moments unrelated to frames (91/s measured), so a frame
// shows a state 0-11 ms old, the two eyes of a pair show different states, and a step can land in the
// middle of drawing a frame (half the scene from before it, half from after). With SimLock the mod
// stops the game's timer and runs the steps itself, on its own thread, right before each new pair:
// both eyes see one state, it advances by the real time since the previous pair, and nothing moves
// while a pair is drawn. The steps run as the timer callback runs them (its pause flags, dt split
// into steps of at most ~10 ms like its 100 Hz), off the game thread like before: D3D9 lowers the game
// thread's x87 precision, the physics never ran with that.
// Outside the car (menus, loading, the house) the mod's thread instead calls the game's own timer
// callback on an exact 10 ms grid: the game's behaviour, without the old timer's jitter. Locking
// there stopped the main menu from reacting to Enter.
typedef void(__stdcall* SimStepFn)(void* game, float dt, float t);
typedef void(__stdcall* TimerProcFn)(UINT id, UINT msg, DWORD_PTR user, DWORD_PTR dw1, DWORD_PTR dw2);
static const SimStepFn kSimStep = (SimStepFn)0x81dfd0;
static const TimerProcFn kTimerProc = (TimerProcFn)0x81ee50;
static void* const kSimGame = (void*)0x1293880;
static volatile LONG* const kTimerCtl = (volatile LONG*)0x129bef0;   // the game's timer thread: 1 = stop the timer, 2 = stopped
static volatile BYTE* const kTimerBusy = (volatile BYTE*)0x10e7c08;  // the timer callback is running
static volatile LONGLONG* const kTimerLast = (volatile LONGLONG*)0x10e7c00;  // callback: QPC of its last step
static volatile BYTE* const kSimSkip = (volatile BYTE*)0x129beed;    // callback: no step at all
static volatile BYTE* const kSimHold1 = (volatile BYTE*)0x129beeb;   // callback: step with dt 0
static volatile BYTE* const kSimHold2 = (volatile BYTE*)0x129beec;
static int g_simLock;                // 0 off, 1 waiting for the game's timer to stop, 2 the mod's thread steps
static volatile bool g_simPairs;     // in the car: steps at each new pair (else every 10 ms through the callback)
static HANDLE g_simGo, g_simDone;
static volatile double g_simTarget;  // the steps catch the simulation up to this time
static volatile LONG g_simSteps;     // steps run at pairs
static double g_simMs, g_simMaxMs;   // time the game thread waited for them (status window)

static bool sim_code_ok() {
    // timeSetEvent(period, resolution, 0x81ee50, 0, TIME_PERIODIC) in the timer thread, and the
    // callback's call of the step with the game object.
    static const unsigned char a[] = { 0x68, 0x50, 0xEE, 0x81, 0x00 };              // push 0x81ee50
    static const unsigned char b[] = { 0x68, 0x80, 0x38, 0x29, 0x01, 0xE8 };        // push 0x1293880; call
    static const unsigned char c[] = { 0x83, 0x3D, 0xF0, 0xBE, 0x29, 0x01, 0x01 };  // cmp dword [0x129bef0], 1
    __try {
        return !memcmp((const void*)0x81f14e, a, sizeof a) && !memcmp((const void*)0x81efa4, b, sizeof b) &&
               !memcmp((const void*)0x81f177, c, sizeof c);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

static void sim_step(float dt) {
    if (*kSimSkip) return;
    kSimStep(kSimGame, *kSimHold1 || *kSimHold2 ? 0.0f : dt, 0.0f);
    InterlockedIncrement(&g_simSteps);
}

static double now_s();

static DWORD WINAPI sim_thread(void*) {
    HANDLE timer = CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS);
    double last = 0, lastPair = 0, nextTick = now_s();
    bool wasPairs = false;
    for (;;) {
        // Woken at each new pair, or by the 10 ms grid. Nothing until the game's timer has stopped.
        double wait = nextTick - now_s();
        if (wait > 0 && timer) {
            LARGE_INTEGER due;
            due.QuadPart = -(LONGLONG)(wait * 1e7);
            SetWaitableTimer(timer, &due, 0, nullptr, nullptr, FALSE);
        }
        HANDLE hs[2] = { g_simGo, timer };
        DWORD r = wait > 0 ? WaitForMultipleObjects(timer ? 2 : 1, hs, FALSE, timer ? 100 : (DWORD)(wait * 1000) + 1) : WAIT_TIMEOUT;
        bool pair = r == WAIT_OBJECT_0;
        double tn = now_s();
        if (!pair && tn >= nextTick) nextTick = tn - nextTick > 0.05 ? tn + 0.01 : nextTick + 0.01;
        if (g_simLock != 2) {
            if (pair) SetEvent(g_simDone);
            continue;
        }
        bool pairs = g_simPairs;
        if (pair) lastPair = tn;
        // Pairs mode, unless frames stopped coming (a load): then the 10 ms grid takes over.
        if (pairs && tn - lastPair < 0.05) {
            if (!wasPairs) last = tn - 0.01;  // the callback stepped until just now
            wasPairs = true;
            if (pair) {
                double target = g_simTarget, dt = target - last;
                if (dt >= 0.004) {  // like the callback: no steps closer than half its period
                    if (dt > 0.1) dt = 0.1;  // a long stall: let the world slow down rather than jump
                    int n = (int)ceil(dt / 0.0105);
                    for (int i = 0; i < n; i++) sim_step((float)(dt / n));
                    last = target;
                }
            }
        } else if (!pair) {
            if (wasPairs) {  // back to the callback: it measures its dt from its last step, make that now
                LARGE_INTEGER c;
                QueryPerformanceCounter(&c);
                *kTimerLast = c.QuadPart;
                wasPairs = false;
            }
            kTimerProc(0, 0, 0, 0, 0);  // the game's own step, as its timer ran it
        }
        if (pair) SetEvent(g_simDone);
    }
}

// Called at the start of every frame from Present.
static void sim_frame(bool newPair, bool inCar) {
    if (g_simLock == 1) {
        if (*kTimerCtl == 0) InterlockedExchange(kTimerCtl, 1);  // the game (re)initialised it before its timer thread saw the request
        if (*kTimerCtl == 2 && !*kTimerBusy) {
            g_simLock = 2;
            log_printf("SimLock: the game's timer is stopped; the mod steps the simulation (at each eye pair in the car)\n");
        }
    }
    if (g_simPairs != inCar) {
        g_simPairs = inCar;
        if (g_simLock == 2) log_printf("SimLock: %s\n", inCar ? "in the car, steps at each eye pair" : "out of the car, steps every 10 ms");
    }
    if (g_simLock != 2 || !newPair || !inCar) return;
    double t0 = now_s();
    g_simTarget = t0;
    SetEvent(g_simGo);
    WaitForSingleObject(g_simDone, 200);
    double ms = (now_s() - t0) * 1000;
    g_simMs += ms;
    if (ms > g_simMaxMs) g_simMaxMs = ms;
}

static void sim_lock_start() {
    // shortcut: off by default until tested in the headset while driving; make it the default once it holds up there.
    if (!g_knownExe || vr_ini_float("SimLock", 0) == 0) { log_printf("SimLock: off\n"); return; }
    if (!sim_code_ok()) { log_printf("SimLock: off (unknown exe)\n"); return; }
    g_simGo = CreateEventA(nullptr, FALSE, FALSE, nullptr);
    g_simDone = CreateEventA(nullptr, FALSE, FALSE, nullptr);
    HANDLE th = CreateThread(nullptr, 0, sim_thread, nullptr, CREATE_SUSPENDED, nullptr);
    if (!th) return;
    SetThreadPriority(th, THREAD_PRIORITY_HIGHEST);
    // The game's timer thread checks this every 500 ms, then kills the timer and sets 2.
    InterlockedExchange(kTimerCtl, 1);
    g_simLock = 1;
    ResumeThread(th);
    log_printf("SimLock: on, waiting for the game's timer to stop\n");
}

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

// ---- culling for the headset's view (2007 exe) ----
// Jumps from game code at f (16-aligned) to fn; returns the trampoline (the first len bytes of f, whole
// position-independent instructions, then a jump back), or null if f doesn't start with head: another exe,
// or SecuROM (which encrypts this code in the file) hasn't decrypted it yet.
static void* hook_code(unsigned char* f, const unsigned char* head, size_t headLen, size_t len, void* fn) {
    __try {
        if (memcmp(f, head, headLen)) return nullptr;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return nullptr;
    }
    unsigned char* t = (unsigned char*)VirtualAlloc(nullptr, 32, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE);
    if (!t) return nullptr;
    memcpy(t, f, len);
    t[len] = 0xE9;
    *(LONG*)(t + len + 1) = (LONG)((f + len) - (t + len + 5));
    unsigned char patch[8];
    memcpy(patch, f, 8);
    patch[0] = 0xE9;
    *(LONG*)(patch + 1) = (LONG)((unsigned char*)fn - (f + 5));
    for (size_t i = 5; i < 8 && i < len; i++) patch[i] = 0x90;
    DWORD old;
    VirtualProtect(f, 8, PAGE_EXECUTE_READWRITE, &old);
    InterlockedExchange64((volatile LONG64*)f, *(LONG64*)patch);  // one aligned write: no thread sees half of it
    VirtualProtect(f, 8, old, &old);
    FlushInstructionCache(GetCurrentProcess(), f, 8);
    return t;
}

static const DWORD kInCarCamVt = 0xf14638;  // the in-car camera's class

// The game's sphere cull (0x50b3b0), a camera method (thiscall: center, radius -> 0 outside, 1 inside,
// 2 crossing): the center goes through the camera's view (+0x30), then |x| and |y| (both scaled by
// 1/aspect at +0xbc) are tested against tan/cos/sin of half the vertical FOV (+0x20/+0x18/+0x1c): about
// 44 deg around where the camera looks. The game culls cars with it (spheres of 2.2-2.8 m, from 0x8bb52d
// and 0x96ac2c), and buildings and terrain sectors. The headset shows up to 55 deg (Quest 3), and past
// the head-look limits the camera stops following the head: cars beside you vanished, so you drove into
// them. For the main view, what the game culls also gets tested against a cone around this frame's eye
// (the camera's view times invEye) as wide as the eye's frustum plus a margin; what the game keeps is
// left alone. Other cameras (reflection cube faces, shadows) keep the game's test.
typedef int(__fastcall* CullFn)(const char* cam, void* edx, const float* center, float radius);
static CullFn g_cullOrig;
static const char* const kMainCam = (const char*)0x13a67e0;  // the game's copy of the camera it draws the main view with
static unsigned g_cullN, g_cullBack;                         // main-view objects the game culled / of those, kept (status line)

// The in-car camera, or one of the copies the game keeps of the main camera (world and render-relative
// coordinates): same rotation. (The in-car camera itself can be a frame ahead of the copies while turning.)
static bool main_view(const char* cam) {
    if (*(const DWORD*)cam == kInCarCamVt) return true;
    const float* a = (const float*)(cam + 0x30);
    const float* b = (const float*)(kMainCam + 0x30);
    for (int i = 0; i < 12; i++)
        if ((i & 3) != 3 && std::fabs(a[i] - b[i]) > 1e-3f) return false;
    return true;
}

static int __fastcall hCull(const char* cam, void* edx, const float* c, float r) {
    int game = g_cullOrig(cam, edx, c, r);
    if (game || !main_view(cam)) return game;
    static const float tanW = [] {  // widest eye tangent + 5 deg for head motion until the frame is shown
        float t = -g_ev.l > g_ev.r ? -g_ev.l : g_ev.r, v = g_ev.up > g_ev.down ? g_ev.up : g_ev.down;
        return tanf(atanf(t > v ? t : v) + 5 * 3.14159265f / 180);
    }();
    alignas(16) char eye[0xc0];  // the test reads +0x18..+0x20, the view at +0x30 and +0xbc
    memcpy(eye, cam, sizeof eye);
    Mat4 V;
    memcpy(V.m, cam + 0x30, sizeof V.m);
    Mat4 E = m4_mul(V, g_ev.invEye);
    memcpy(eye + 0x30, E.m, sizeof E.m);
    float cs = 1 / sqrtf(1 + tanW * tanW);
    *(float*)(eye + 0x18) = cs;
    *(float*)(eye + 0x1c) = tanW * cs;
    *(float*)(eye + 0x20) = tanW;
    *(float*)(eye + 0xbc) = 1;  // the same angle on both axes
    int res = g_cullOrig(eye, edx, c, r);
    g_cullN++;
    if (res) g_cullBack++;
    return res;
}

// The in-car camera's FOV (Camera::SetFov 0x404840: +0x10 the FOV, +0x18/+0x1c/+0x20 cos/sin/tan of half of
// it). The game uses the two differently (parked at the house, splitting the fields): the FOV itself builds
// the projection and a frustum it culls part of the world with (2922 draws at 57 deg, 3624 with only +0x10 at
// 110: vegetation and props), cos/sin/tan set level of detail and draw distance (2390 with only those at 110:
// distant trees and signs gone). In the car the FOV is 57 deg tall, 65 at speed; the headset shows 99 deg
// tall (Quest 3), so things at the edges of your view were culled and popped in as you looked around. The
// mod's rewrite replaces the projection anyway, so +0x10 is set as wide as this frame's eye sees, wherever
// it looks (rolled, or past the head-look limits), plus 5 deg: 120 deg on a Quest 3 looking straight ahead.
// Level of detail keeps the car's FOV at rest: the detail you'd get on a monitor, no longer less at speed
// (the widening there is a flat-screen speed effect you don't see in the headset).
typedef void(__fastcall* SetFovFn)(char* cam, void* edx, float fov);
static SetFovFn g_setFovOrig;
static float g_cullFov, g_lodFov;  // last set, radians (status line)

static void __fastcall hSetFov(char* cam, void* edx, float fov) {
    if (*(const DWORD*)cam != kInCarCamVt) { g_setFovOrig(cam, edx, fov); return; }
    static const char* car;
    static float atRest;
    if (cam != car || fov < atRest) { car = cam; atRest = fov; }  // the smallest FOV seen: no speed effect
    g_setFovOrig(cam, edx, atRest);
    // The corners of the eye's frustum (either eye: x as wide as the wider side) in camera space: p_cam =
    // p_eye * transpose(R), R the rotation in invEye (camera -> eye). The culling frustum must reach them.
    const Mat4& M = g_ev.invEye;
    float x = -g_ev.l > g_ev.r ? -g_ev.l : g_ev.r, aspect = *(const float*)(cam + 0x14), needV = 0, needH = 0;
    bool behind = !(aspect > 0.1f);
    for (int k = 0; k < 4; k++) {
        float e[3] = { k & 1 ? x : -x, k & 2 ? g_ev.up : -g_ev.down, -1 }, c[3];
        for (int i = 0; i < 3; i++) c[i] = e[0] * M.m[i][0] + e[1] * M.m[i][1] + e[2] * M.m[i][2];
        if (!(c[2] < -1e-3f)) { behind = true; break; }  // a corner at or past 90 deg from the camera's axis
        float v = atanf(std::fabs(c[1] / c[2])), h = atanf(std::fabs(c[0] / c[2]));
        needV = v > needV ? v : needV;
        needH = h > needH ? h : needH;
    }
    float hv = behind ? 0 : atanf(tanf(needH) / aspect);  // vertical half-angle that gives that much horizontally
    float half = (needV > hv ? needV : hv) + 5 * 3.14159265f / 180;  // + head motion until the frame is shown
    if (behind || !(half < 1.396f)) half = 1.396f;  // 80 deg at most: the projection needs under 90 (and not NaN)
    *(float*)(cam + 0x10) = 2 * half;
    g_cullFov = 2 * half;
    g_lodFov = atRest;
}

static void cull_hooks() {
    static const unsigned char cullHead[] = { 0x55, 0x8B, 0xEC, 0x83, 0xE4, 0xF0, 0x83, 0xEC, 0x1C };  // push ebp; mov ebp,esp; and esp,-16; sub esp,0x1c
    static const unsigned char fovHead[] = { 0xD9, 0x44, 0x24, 0x04, 0xF3, 0x0F, 0x10, 0x44, 0x24, 0x04 };  // fld [esp+4]; movss xmm0,[esp+4]
    if (!g_cullOrig) g_cullOrig = (CullFn)hook_code((unsigned char*)0x50b3b0, cullHead, sizeof cullHead, 6, (void*)hCull);
    if (!g_setFovOrig) g_setFovOrig = (SetFovFn)hook_code((unsigned char*)0x404840, fovHead, sizeof fovHead, 10, (void*)hSetFov);
    static bool logged;
    if (g_cullOrig && g_setFovOrig && !logged) {
        logged = true;
        log_printf("culling: the main view's covers the eye's frustum (sphere cull 0x50b3b0, in-car FOV 0x404840)\n");
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
    decltype(IDirect3DDevice9Vtbl::DrawPrimitive) DrawPrimitive;
    decltype(IDirect3DDevice9Vtbl::DrawIndexedPrimitive) DrawIndexedPrimitive;
    decltype(IDirect3DDevice9Vtbl::DrawPrimitiveUP) DrawPrimitiveUP;
    decltype(IDirect3DDevice9Vtbl::DrawIndexedPrimitiveUP) DrawIndexedPrimitiveUP;
    decltype(IDirect3DDevice9Vtbl::GetVertexShaderConstantF) GetVertexShaderConstantF;
    decltype(IDirect3DDevice9Vtbl::ProcessVertices) ProcessVertices;
    decltype(IDirect3DDevice9Vtbl::CreateStateBlock) CreateStateBlock;
    decltype(IDirect3DDevice9Vtbl::BeginStateBlock) BeginStateBlock;
    decltype(IDirect3DDevice9Vtbl::EndStateBlock) EndStateBlock;
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
enum { R_PAIR, R_OTHER, R_SAME, R_UI, R_POSTFX, R_NOTPERSP, R_SHAPE, R_VIEWPORT, R_FORM, R_NOMAIN, R_PLAIN, R_N };
static const char* kRName[R_N] = { "rewritten(exact)", "rewritten(recovered)", "kept(same object)", "2D->screen", "skip:postfx",
                                   "skip:2D-other", "skip:offscreen-shape", "skip:sub-viewport", "skip:not-projection",
                                   "skip:no-camera-yet", "untouched(no camera matrix)" };
static unsigned g_st[R_N];
static bool g_batch;                            // constant batching (TDUVR_CONSTBATCH), see flush_consts
static bool g_recording;                        // the game is recording a state block (see hBeginStateBlock)
static unsigned g_stSbCreate, g_stSbApply, g_stSbCapture;  // state block use (status line)
static unsigned g_stConstCalls, g_stConstRegs;  // device calls / registers sent (status line)
static double g_copyWaitMs, g_submitMs, g_applyMs, g_readbackMs;  // cumulative ms, for the status line
static IDirect3DDevice9* volatile g_watchDev;  // the game's device (the one the mod works on)
// Another device sharing the hooked table: DXVK gives all its devices one vtable (Windows' d3d9 a copy
// each), so the tiny second device the game opens in the house comes through the hooks too.
static bool other_dev(IDirect3DDevice9* d) { return g_watchDev && d != g_watchDev; }
static double g_presentMs, g_presentMax;                           // time inside the real Present

static double now_s() {
    static LARGE_INTEGER f = [] { LARGE_INTEGER x; QueryPerformanceFrequency(&x); return x; }();
    LARGE_INTEGER c;
    QueryPerformanceCounter(&c);
    return (double)c.QuadPart / (double)f.QuadPart;
}

static void log_status() {
    static double t0 = now_s(), last = t0;
    static unsigned lastFrame, lastSt[R_N], lastDraw;
    static double lastWait, lastSubmit, lastApply, lastReadback;
    double t = now_s();
    if (t - last < 5.0) return;
    unsigned frames = g_frame - lastFrame;
    double perF = frames ? 1.0 / frames : 0;
    std::string s;
    for (int i = 0; i < R_N; i++) {
        s += " " + std::string(kRName[i]) + "=" + std::to_string((unsigned)((g_st[i] - lastSt[i]) * perF + 0.5));
        lastSt[i] = g_st[i];
    }
    log_printf("t=%.0fs %.1f fps | per frame: mod draw hooks %.2f ms, %s %.2f ms, copy wait %.2f ms, publish %.2f ms | draws/frame %u:%s | main vfov %.1f\n",
               t - t0, frames / (t - last), (g_applyMs - lastApply) * perF, gpu_on() ? "GPU copy" : "readback calls",
               (g_readbackMs - lastReadback) * perF, (g_copyWaitMs - lastWait) * perF,
               (g_submitMs - lastSubmit) * perF, (unsigned)((g_cnt[C_DRAW] - lastDraw) * perF + 0.5), s.c_str(),
               g_haveMain ? 2 * atan(1 / g_mainP.m[1][1]) * 57.2958 : 0.0);
    lastReadback = g_readbackMs;
    if (!g_frameMs.empty()) {
        std::sort(g_frameMs.begin(), g_frameMs.end());
        auto pct = [](double p) { return g_frameMs[(size_t)(p * (g_frameMs.size() - 1))]; };
        static unsigned __int64 lastSteps;
        unsigned __int64 steps = 0;
        float avg = 0;
        if (g_knownExe) { steps = (unsigned __int64)g_simSteps + *kSimSteps; avg = *kSimAvgS; }
        char simBuf[96];
        if (g_simLock == 2 && g_simPairs) snprintf(simBuf, sizeof simBuf, "at eye pairs, game thread waited %.2f ms/frame (max %.1f)", g_simMs * perF, g_simMaxMs);
        else snprintf(simBuf, sizeof simBuf, "%s timer (avg %.1f ms incl. +1)", g_simLock == 2 ? "10 ms" : "game", avg * 1000);
        log_printf("  frame ms p50 %.1f p90 %.1f p99 %.1f max %.1f, in Present %.2f avg %.1f max | game steps %.0f/s, %s, frames after 0/1/2/3+ steps: %u/%u/%u/%u\n",
                   pct(0.5), pct(0.9), pct(0.99), g_frameMs.back(), g_presentMs * perF, g_presentMax,
                   lastSteps ? (steps - lastSteps) / (t - last) : 0.0, simBuf,
                   g_stepHist[0], g_stepHist[1], g_stepHist[2], g_stepHist[3]);
        lastSteps = steps;
        g_simMs = g_simMaxMs = 0;
        g_presentMs = g_presentMax = 0;
        g_frameMs.clear();
        memset(g_stepHist, 0, sizeof g_stepHist);
    }
    if (g_vr) {
        // Address space in use (reserved or committed) and the largest free block: the 2 GB limit of this
        // 32-bit game is what the old CPU copy ran into at 2560x1440.
        MEMORY_BASIC_INFORMATION mbi;
        size_t used = 0, maxFree = 0;
        for (const char* a = nullptr; VirtualQuery(a, &mbi, sizeof mbi); a = (const char*)mbi.BaseAddress + mbi.RegionSize) {
            if (mbi.State == MEM_FREE && mbi.RegionSize > maxFree) maxFree = mbi.RegionSize;
            else used += mbi.RegionSize;
            if ((UINT_PTR)a + mbi.RegionSize < (UINT_PTR)a) break;
        }
        log_printf("  %s | address space used %u MB, largest free block %u MB\n", vr_status(), (unsigned)(used >> 20), (unsigned)(maxFree >> 20));
    }
    if (g_vr && gpu_on()) log_printf("  %s\n", gpu_status());
    {
        static unsigned lastCalls, lastRegs, lastGame;
        static unsigned lastSb[3];
        if (g_batch)
            log_printf("  constants: game sets %.0f times per frame, batched into %.0f device calls (%.0f registers); state blocks made %u, applied %u, captured %u\n",
                       (g_cnt[C_SVSC] - lastGame) * perF, (g_stConstCalls - lastCalls) * perF, (g_stConstRegs - lastRegs) * perF,
                       g_stSbCreate - lastSb[0], g_stSbApply - lastSb[1], g_stSbCapture - lastSb[2]);
        lastSb[0] = g_stSbCreate; lastSb[1] = g_stSbApply; lastSb[2] = g_stSbCapture;
        lastCalls = g_stConstCalls;
        lastRegs = g_stConstRegs;
        lastGame = g_cnt[C_SVSC];
    }
    if (g_headLook) {
        static unsigned lastFixes;
        log_printf("  head look: yaw %+.1f pitch %+.1f deg, camera object %s, corrected %u frames | rendered with another look than read:"
                   " left %u/%u, right %u/%u frames (unmatched %u/%u), known by 3D draw %.1f/%.1f, early pick corrected %u/%u | camera at %.1f %.1f %.1f\n",
                   g_appliedYaw * 57.2958f, g_appliedPitch * 57.2958f, g_camObj ? "found" : "not found", g_lookFixes - lastFixes,
                   g_rcFix[0], g_rcN[0], g_rcFix[1], g_rcN[1], g_rcBad[0], g_rcBad[1],
                   g_capN[0] ? g_capSum[0] / g_capN[0] : -1.0, g_capN[1] ? g_capSum[1] / g_capN[1] : -1.0, g_earlyBad[0], g_earlyBad[1],
                   g_cam.m[3][0], g_cam.m[3][1], g_cam.m[3][2]);
        lastFixes = g_lookFixes;
        g_rcN[0] = g_rcN[1] = g_rcFix[0] = g_rcFix[1] = g_rcBad[0] = g_rcBad[1] = 0;
        g_capSum[0] = g_capSum[1] = 0;
        g_capN[0] = g_capN[1] = 0;
        g_earlyBad[0] = g_earlyBad[1] = 0;
    }
    if (g_cullOrig && frames) {
        log_printf("  culling: in-car FOV for culling %.0f deg, level of detail %.0f deg | sphere cull: %.1f main-view objects per frame outside"
                   " the game's cone, %.1f of them kept by the eye's\n",
                   g_cullFov * 57.2958f, g_lodFov * 57.2958f, g_cullN / (double)frames, g_cullBack / (double)frames);
        g_cullN = g_cullBack = 0;
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

// ---- constant batching (TDUVR_CONSTBATCH) ----
// The game's SetVertexShaderConstantF calls and the mod's matrix writes only update g_want; right before
// each draw, the registers that really changed go to the device in as few calls as possible. Under
// D3D9On12 every call is a command for its worker thread, and the game thread spent ~23% of each frame
// waiting on that worker inside these calls (profiled). Off while the game records or creates state
// blocks: those capture and restore constants behind our back.
static float g_want[MAXC][4];        // what the device must hold at the next draw (the game's value or a rewrite)
static float g_have[MAXC][4];        // what it holds
static unsigned g_dirty[MAXC / 32];  // g_want may differ from g_have
static unsigned g_haveOk[MAXC / 32]; // g_have is known

static void mark(int r) { g_dirty[r >> 5] |= 1u << (r & 31); }
static void set_want(int r, const float* v) { memcpy(g_want[r], v, 16); mark(r); }

// Every register strictly between a and b has a known device value (so it can go along in a window).
static bool known_between(int a, int b) {
    for (int r = a + 1; r < b; r++)
        if (!((g_haveOk[r >> 5] >> (r & 31)) & 1)) return false;
    return true;
}

static void flush_consts(IDirect3DDevice9* d) {
    int lo = -1, hi = -1;
    auto send = [&] {
        H(d).SetVertexShaderConstantF(d, lo, g_have[lo], hi - lo + 1);
        g_stConstCalls++;
        g_stConstRegs += hi - lo + 1;
    };
    for (int w = 0; w < MAXC / 32; w++) {
        for (unsigned bits = g_dirty[w]; bits; bits &= bits - 1) {
            unsigned long b;
            _BitScanForward(&b, bits);
            int r = w * 32 + (int)b;
            bool known = (g_haveOk[w] >> b) & 1;
            if (known && !memcmp(g_want[r], g_have[r], 16)) continue;
            memcpy(g_have[r], g_want[r], 16);
            g_haveOk[w] |= 1u << b;
            // Windows of up to 16 registers: D3D9On12's command buffer (d3d9.dll lhbatchfilter) gives every
            // call of 1-16 registers a 268-byte slot and of 17-256 a 4108-byte one, and each time the 768 KB
            // buffer fills the game thread waits for the worker. Unchanged registers inside a window go
            // along (g_have holds the device's value for them): they cost nothing extra.
            if (lo >= 0 && r - lo < 16 && known_between(hi, r)) { hi = r; continue; }
            if (lo >= 0) send();
            lo = hi = r;
        }
        g_dirty[w] = 0;
    }
    if (lo >= 0) send();
}

static void batch_off(IDirect3DDevice9* d, const char* why) {
    if (!g_batch) return;
    flush_consts(d);
    g_batch = false;
    log_printf("constant batching: off (%s)\n", why);
}

// The device's constants are unknown (new device, Reset): send everything the game wants again.
static void consts_unknown() {
    memset(g_haveOk, 0, sizeof g_haveOk);
    memset(g_dirty, 0xff, sizeof g_dirty);
}

// Batching on/off while running (A/B test). On: start from the game's values, rewrites are redone at the
// next draws. Off: the device first gets everything pending.
static void set_batch(IDirect3DDevice9* d, bool on) {
    if (on == g_batch) return;
    if (!on) { flush_consts(d); g_batch = false; return; }
    for (int r : g_mod) g_isMod[r] = false;
    g_mod.clear();
    memcpy(g_want, g_vsc, sizeof g_want);
    consts_unknown();
    g_batch = true;
}

static void write_mat(IDirect3DDevice9* d, const MatSlot& s, const Mat4& M) {
    float buf[4][4];
    int n = s.count > 4 ? 4 : s.count;
    if (s.reg + n > MAXC) return;
    for (int j = 0; j < n; j++) for (int i = 0; i < 4; i++) buf[j][i] = M.m[i][j];
    if (g_batch) for (int j = 0; j < n; j++) set_want(s.reg + j, buf[j]);
    else H(d).SetVertexShaderConstantF(d, s.reg, &buf[0][0], n);
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

// The last rewrite: for which shader, with which eye matrices (g_eyeGen counts new_frame_matrices).
// When the next draw uses that shader and the game has not set any of its matrix registers since,
// the registers still hold exactly what a rewrite would write (the game draws an object in several
// pieces with the same transform): nothing to do.
static const ShaderMats* g_rwShader;
static unsigned g_rwGen, g_eyeGen;

// New eye/pose: refresh the eye matrices and drop cached frustums.
static void new_frame_matrices() {
    g_eyeGen++;
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
        if (g_batch) set_want(r, g_vsc[r]);
        else H(d).SetVertexShaderConstantF(d, r, g_vsc[r], 1);
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
    g_rwShader = s;
    g_rwGen = g_eyeGen;
}

static bool rewrite_still_valid(const ShaderMats* s) {
    if (s != g_rwShader || g_rwGen != g_eyeGen) return false;
    for (auto& m : s->mats)
        for (int j = 0; j < (m.count > 4 ? 4 : m.count); j++)
            if (m.reg + j < MAXC && !g_isMod[m.reg + j]) return false;
    return true;
}

// Post-processing quads read the game's own render targets and must keep covering the frame. They are
// drawn without the depth test, while all of the game's 2D tests depth (tiny z): that includes the
// minimap, which reads a render target of its own (the map), and menu parts drawn from render targets.
// (Asks the bound texture itself: on D3D9On12 some of the game's render targets never came through
// CreateTexture's hook, so a list of them missed the full-screen pass.)
static bool postfx_pass(IDirect3DDevice9* d) {
    IDirect3DBaseTexture9* t = nullptr;
    if (FAILED(d->lpVtbl->GetTexture(d, 0, &t)) || !t) return false;
    D3DSURFACE_DESC sd{};
    if (t->lpVtbl->GetType(t) == D3DRTYPE_TEXTURE) ((IDirect3DTexture9*)t)->lpVtbl->GetLevelDesc((IDirect3DTexture9*)t, 0, &sd);
    t->lpVtbl->Release(t);
    DWORD z;
    return (sd.Usage & D3DUSAGE_RENDERTARGET) && (FAILED(d->lpVtbl->GetRenderState(d, D3DRS_ZENABLE, &z)) || !z);
}

// The virtual screen's edges: what the game draws past the edge of its screen ("chat", parked menu parts)
// is out of sight on a monitor, but around the virtual screen it would show. Four clip planes cut 2D
// draws at the screen's edges: for the draw's own clip position c, -w <= x, y <= w; the rewritten draw
// outputs c' = c * g_uiU, so each plane p becomes inverse(g_uiU) * p.
static bool g_uiClip;       // clip planes enabled (only around 2D draws on the virtual screen)
static unsigned g_uiClipGen;  // eye matrices the planes were set for

static void ui_clip(IDirect3DDevice9* d, bool on) {
    if (on && g_uiClipGen != g_eyeGen) {
        g_uiClipGen = g_eyeGen;
        Mat4 inv;
        if (!m4_inverse(g_uiU, inv)) return;
        static const float edge[4][2] = { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } };  // x >= -w, x <= w, y >= -w, y <= w
        for (int k = 0; k < 4; k++) {
            float p[4];
            for (int i = 0; i < 4; i++) p[i] = inv.m[i][0] * edge[k][0] + inv.m[i][1] * edge[k][1] + inv.m[i][3];
            d->lpVtbl->SetClipPlane(d, k, p);
        }
    }
    if (on == g_uiClip) return;
    g_uiClip = on;
    d->lpVtbl->SetRenderState(d, D3DRS_CLIPPLANEENABLE, on ? 0xF : 0);
}

// A full-screen 2D draw (menu, HUD, cursor): move it onto the virtual screen for this eye.
static void ui_screen(IDirect3DDevice9* d, const ShaderMats* s) {
    restore_stale(d, nullptr);
    if (postfx_pass(d)) { g_st[R_POSTFX]++; return; }
    g_st[R_UI]++;
    for (auto& m : s->mats)
        if (m.kind == MK_POST_K || m.kind == MK_PROJ) write_mat(d, m, m4_mul(read_mat(m), g_uiU));
    ui_clip(d, true);
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

// ---- TDUVR_DRAWLOG (test): every draw of two frames, with what the mod did with it ----
// TDUVR_DRAWLOG=<s>[,<s>...]: at each time (seconds from start, or "+<s>" from when head look found the
// in-car camera), logs every draw of the next two frames (one per eye): the mod's classification, shaders,
// target, textures, where the vertices land on the game's own screen (clip x/w, y/w through the shader's
// combined matrix as the game set it), and the render states that can hide a draw. Reads vertex buffers
// back, so test use only.
struct DrawArgs { const char* call; D3DPRIMITIVETYPE pt; UINT prims; INT base; UINT first, verts; const void* up; UINT upStride; };
static DrawArgs g_da;
static int g_dlFrames;  // frames of the current log still to go
static unsigned g_dlDraw;

static UINT prim_verts(D3DPRIMITIVETYPE t, UINT n) {
    return t == D3DPT_POINTLIST ? n : t == D3DPT_LINELIST ? 2 * n : t == D3DPT_LINESTRIP ? n + 1 : t == D3DPT_TRIANGLELIST ? 3 * n : n + 2;
}

static void drawlog_present() {
    static char spec[128];
    static const char* next = GetEnvironmentVariableA("TDUVR_DRAWLOG", spec, sizeof spec) ? spec : nullptr;
    static double t0 = now_s(), tCam;
    if (g_dlFrames && --g_dlFrames == 0) log_printf("draw log: end\n");
    if (!next || g_dlFrames) return;
    bool fromCar = *next == '+';
    if (fromCar && !g_camObj) return;
    if (fromCar && !tCam) tCam = now_s();
    if (now_s() - (fromCar ? tCam : t0) < atof(next + fromCar)) return;
    next = strchr(next, ',');
    if (next) next++;
    g_dlFrames = 2;
    g_dlDraw = 0;
    log_printf("draw log: frames %u-%u, first eye %d\n", g_frame, g_frame + 1, g_eye);
}

static void tex_desc(IDirect3DDevice9* d, DWORD stage, char* out, size_t cap) {
    IDirect3DBaseTexture9* t = nullptr;
    if (FAILED(d->lpVtbl->GetTexture(d, stage, &t)) || !t) { snprintf(out, cap, "-"); return; }
    D3DSURFACE_DESC sd{};
    D3DRESOURCETYPE ty = t->lpVtbl->GetType(t);
    if (ty == D3DRTYPE_TEXTURE) ((IDirect3DTexture9*)t)->lpVtbl->GetLevelDesc((IDirect3DTexture9*)t, 0, &sd);
    snprintf(out, cap, "%p %ux%u f%d%s%s", (void*)t, sd.Width, sd.Height, (int)sd.Format, ty == D3DRTYPE_CUBETEXTURE ? " cube" : "",
             (sd.Usage & D3DUSAGE_RENDERTARGET) ? " RT" : "");
    t->lpVtbl->Release(t);
}

// Range of clip x/w, y/w, z/w of the draw's vertices through M (row vector), or of the raw positions without M.
static bool draw_extent(IDirect3DDevice9* d, const Mat4* M, float r[6], bool* pretransformed) {
    IDirect3DVertexDeclaration9* decl = nullptr;
    if (FAILED(d->lpVtbl->GetVertexDeclaration(d, &decl)) || !decl) return false;
    D3DVERTEXELEMENT9 el[MAXD3DDECLLENGTH + 1];
    UINT ne = 0;
    HRESULT hr = decl->lpVtbl->GetDeclaration(decl, el, &ne);
    decl->lpVtbl->Release(decl);
    if (FAILED(hr)) return false;
    const D3DVERTEXELEMENT9* pos = nullptr;
    for (UINT i = 0; i < ne && el[i].Stream != 0xFF; i++)
        if ((el[i].Usage == D3DDECLUSAGE_POSITION || el[i].Usage == D3DDECLUSAGE_POSITIONT) && el[i].UsageIndex == 0) { pos = &el[i]; break; }
    int comps = !pos ? 0 : pos->Type == D3DDECLTYPE_FLOAT2 ? 2 : pos->Type == D3DDECLTYPE_FLOAT3 ? 3 : pos->Type == D3DDECLTYPE_FLOAT4 ? 4 : 0;
    if (!comps) return false;
    *pretransformed = pos->Usage == D3DDECLUSAGE_POSITIONT;
    const BYTE* data = (const BYTE*)g_da.up;
    UINT stride = g_da.upStride;
    __int64 first = g_da.first, size = 1ll << 40;
    IDirect3DVertexBuffer9* vb = nullptr;
    void* p = nullptr;
    if (!data) {
        UINT off = 0;
        if (FAILED(d->lpVtbl->GetStreamSource(d, pos->Stream, &vb, &off, &stride)) || !vb) return false;
        D3DVERTEXBUFFER_DESC vd{};
        vb->lpVtbl->GetDesc(vb, &vd);
        if (FAILED(vb->lpVtbl->Lock(vb, 0, 0, &p, D3DLOCK_READONLY)) || !p) { vb->lpVtbl->Release(vb); return false; }
        data = (const BYTE*)p + off;
        size = (__int64)vd.Size - off;
        first += g_da.base;
    }
    r[0] = r[2] = r[4] = 1e30f;
    r[1] = r[3] = r[5] = -1e30f;
    bool any = false;
    for (UINT i = 0; i < g_da.verts && i < 4096; i++) {
        __int64 o = (first + i) * stride + pos->Offset;
        if (o < 0 || o + comps * 4 > size) break;
        float v[4] = { 0, 0, 0, 1 };
        memcpy(v, data + o, comps * 4);
        if (comps < 4) v[3] = 1;
        float x = v[0], y = v[1], z = v[2];
        if (M && !*pretransformed) {
            float c[4];
            for (int j = 0; j < 4; j++) c[j] = v[0] * M->m[0][j] + v[1] * M->m[1][j] + v[2] * M->m[2][j] + v[3] * M->m[3][j];
            if (c[3] <= 1e-6f) continue;  // behind the camera
            x = c[0] / c[3];
            y = c[1] / c[3];
            z = c[2] / c[3];
        }
        r[0] = x < r[0] ? x : r[0]; r[1] = x > r[1] ? x : r[1];
        r[2] = y < r[2] ? y : r[2]; r[3] = y > r[3] ? y : r[3];
        r[4] = z < r[4] ? z : r[4]; r[5] = z > r[5] ? z : r[5];
        any = true;
    }
    if (vb) { vb->lpVtbl->Unlock(vb); vb->lpVtbl->Release(vb); }
    return any;
}

static void drawlog_draw(IDirect3DDevice9* d, const ShaderMats* s, const unsigned* before) {
    const char* cls = "-";
    for (int i = 0; i < R_N; i++) if (g_st[i] != before[i]) cls = kRName[i];
    char where[160] = "where ?";
    Mat4 M;
    bool haveM = s && s->anyK >= 0, pre = false;
    if (haveM) M = read_mat(s->mats[s->anyK]);
    float r[6];
    if (draw_extent(d, haveM ? &M : nullptr, r, &pre))
        snprintf(where, sizeof where, "%s x %.3f..%.3f y %.3f..%.3f z %.4g..%.4g", pre ? "pixels" : haveM ? "clip" : "raw", r[0], r[1], r[2], r[3], r[4], r[5]);
    static const D3DRENDERSTATETYPE kRs[11] = { D3DRS_ZENABLE, D3DRS_ZWRITEENABLE, D3DRS_ZFUNC, D3DRS_ALPHABLENDENABLE, D3DRS_SRCBLEND, D3DRS_DESTBLEND,
                                                D3DRS_ALPHATESTENABLE, D3DRS_COLORWRITEENABLE, D3DRS_STENCILENABLE, D3DRS_SCISSORTESTENABLE, D3DRS_CLIPPLANEENABLE };
    DWORD rs[11] = {};
    for (int i = 0; i < 11; i++) d->lpVtbl->GetRenderState(d, kRs[i], &rs[i]);
    RECT sc{};
    if (rs[9]) d->lpVtbl->GetScissorRect(d, &sc);
    IDirect3DVertexShader9* vs = nullptr;
    IDirect3DPixelShader9* ps = nullptr;
    d->lpVtbl->GetVertexShader(d, &vs);
    d->lpVtbl->GetPixelShader(d, &ps);
    if (vs) vs->lpVtbl->Release(vs);
    if (ps) ps->lpVtbl->Release(ps);
    char t0[96], t1[96];
    tex_desc(d, 0, t0, sizeof t0);
    tex_desc(d, 1, t1, sizeof t1);
    log_printf("draw %u eye %d %s pt%d x%u | %s | vs %p (%d mats) ps %p | rt %ux%u vp %lu,%lu %lux%lu | t0 %s | t1 %s | %s | z %lu/%lu f%lu blend %lu %lu/%lu at %lu cw %lx st %lu sc %lu [%ld %ld %ld %ld] clip %lx\n",
               g_dlDraw++, g_eye, g_da.call, (int)g_da.pt, g_da.prims, cls, (void*)vs, s ? (int)s->mats.size() : -1, (void*)ps, g_rtW, g_rtH,
               g_vp.X, g_vp.Y, g_vp.Width, g_vp.Height, t0, t1, where, rs[0], rs[1], rs[2], rs[3], rs[4], rs[5], rs[6], rs[7], rs[8], rs[9],
               sc.left, sc.top, sc.right, sc.bottom, rs[10]);
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
    if (fullVP && rewrite_still_valid(s)) { g_st[R_SAME]++; g_n3d++; return; }

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

static bool g_gpuDue;  // a frame was captured for the GPU copy: finish it after Present
static bool use_dxvk();

// Called before every draw.
static void apply(IDirect3DDevice9* d) {
    if (g_recording) {
        g_recording = false;
        static int n;
        if (n++ < 5) log_printf("state block: draw while recording, recording assumed dropped (frame %u)\n", g_frame);
    }
    double t0 = now_s();
    unsigned before[R_N];
    if (g_dlFrames) memcpy(before, g_st, sizeof before);
    const ShaderMats* s = g_cur;
    unsigned ui = g_st[R_UI];
    apply_body(d);
    if (g_uiClip && g_st[R_UI] == ui) ui_clip(d, false);  // the clip planes are for 2D on the virtual screen only
    if (g_batch) flush_consts(d);
    g_applyMs += (now_s() - t0) * 1000;
    if (g_dlFrames) drawlog_draw(d, s, before);
}

// ---- back buffer -> headset ----
static IDirect3DSurface9* g_mirror;  // the desktop window's picture (see mirror_eye)
static bool g_mirrorOk;
static HWND g_gameWnd;

static void release_copies() {
    for (auto& s : g_slot) {
        if (s.rt) s.rt->lpVtbl->Release(s.rt);
        if (s.sys) s.sys->lpVtbl->Release(s.sys);
        if (s.q) s.q->lpVtbl->Release(s.q);
        s = Slot{};
    }
    if (g_mirror) g_mirror->lpVtbl->Release(g_mirror);
    g_mirror = nullptr;
    g_mirrorOk = false;
}

// The desktop window shows the left eye only: both eyes in turn looked like two pictures at once. The
// back buffer holds the eye's whole frustum (about square, stretched to the game's resolution), so the
// picture is cut to the window's shape around where the eye looks; the right eye's frames show it again.
// Called after the eye's copy for the headset is queued, before Present.
static void mirror_eye(IDirect3DDevice9* d) {
    if (g_eye && !g_mirrorOk) return;
    IDirect3DSurface9* bb = nullptr;
    if (FAILED(d->lpVtbl->GetBackBuffer(d, 0, 0, D3DBACKBUFFER_TYPE_MONO, &bb))) return;
    D3DSURFACE_DESC dd;
    bb->lpVtbl->GetDesc(bb, &dd);
    if (!g_mirror && FAILED(d->lpVtbl->CreateRenderTarget(d, dd.Width, dd.Height, dd.Format, D3DMULTISAMPLE_NONE, 0, FALSE, &g_mirror, nullptr)))
        g_mirror = nullptr;
    if (g_mirror && g_eye == 0) {
        RECT wr;  // Present stretches the back buffer onto the window, which needn't have its shape
        float aspect = g_gameWnd && GetClientRect(g_gameWnd, &wr) && wr.bottom > 0 ? (float)wr.right / wr.bottom : (float)dd.Width / dd.Height;
        float v = g_ev.up < g_ev.down ? g_ev.up : g_ev.down;
        float hx = -g_ev.l < g_ev.r ? -g_ev.l : g_ev.r, hy = hx / aspect;  // half-width/height as tangents
        if (hy > v) { hy = v; hx = v * aspect; }
        float sx = dd.Width / (g_ev.r - g_ev.l), sy = dd.Height / (g_ev.up + g_ev.down);
        RECT src = { (LONG)((-hx - g_ev.l) * sx), (LONG)((g_ev.up - hy) * sy), (LONG)((hx - g_ev.l) * sx), (LONG)((g_ev.up + hy) * sy) };
        g_mirrorOk = SUCCEEDED(d->lpVtbl->StretchRect(d, bb, &src, g_mirror, nullptr, D3DTEXF_LINEAR));
    }
    if (g_mirrorOk) d->lpVtbl->StretchRect(d, g_mirror, nullptr, bb, nullptr, D3DTEXF_NONE);
    bb->lpVtbl->Release(bb);
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
// A path with one %d (or %03d) writes an image every <seconds> instead, numbered.
static char g_dumpPath[MAX_PATH + 16];
static bool dump_due() {
    static bool done;
    static int n;
    static double t0 = now_s(), tCam, tLast;
    char spec[MAX_PATH + 16];
    if (done || GetEnvironmentVariableA("TDUVR_DUMP", spec, sizeof spec) == 0) return false;
    char* comma = strrchr(spec, ',');
    double after = comma ? atof(comma + 1 + (comma[1] == '+')) : 60;
    if (comma && comma[1] == '+') {
        if (!g_camObj) return false;
        if (!tCam) tCam = now_s();
        if (now_s() - tCam < after) return false;
    } else if (now_s() - t0 < after) {
        return false;
    }
    if (comma) *comma = 0;
    const char* pct = strchr(spec, '%');
    size_t digits = pct ? strspn(pct + 1, "0123456789") : 0;
    bool repeat = pct && pct[1 + digits] == 'd' && !strchr(pct + 1, '%') && after > 0;
    if (!repeat) {
        strcpy_s(g_dumpPath, spec);
        done = true;
        return true;
    }
    if (tLast && now_s() - tLast < after) return false;
    tLast = now_s();
    snprintf(g_dumpPath, sizeof g_dumpPath, spec, n++);  // checked above: exactly one %[digits]d
    return true;
}

static void write_bmp(const void* bits, int pitch, int w, int h) {
    const char* env = g_dumpPath;
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
        if (dump_due()) write_bmp(lr.pBits, lr.Pitch, (int)g_copyW, (int)g_copyH);
        s.sys->lpVtbl->UnlockRect(s.sys);
    }
    g_submitMs += (now_s() - t1) * 1000;
    s.pending = false;
}

// GPU path (D3D9On12, GpuCopy=1): the back buffer is copied into an eye image on the GPU. CPU path
// (plain d3d9): GPU -> system memory -> D3D11, 15 MB per frame at 2560x1440, and GetRenderTargetData
// can hold the game thread until the GPU has caught up.

static void submit_backbuffer(IDirect3DDevice9* d) {
    static bool noCopy = GetEnvironmentVariableA("TDUVR_NOCOPY", nullptr, 0) > 0;  // test: measure the renderer alone
    if (noCopy) return;
    if (use_dxvk()) {  // DXVK: Vulkan images straight to SteamVR (falls through if the interop is missing)
        double t = now_s();
        bool done = vk_capture(d, g_eye, g_frame >> 1, g_framePose);
        g_readbackMs += (now_s() - t) * 1000;
        if (done && g_eye == 0 && dump_due()) vk_read_eye(1, write_bmp);  // the pair just bridged
        if (done) return;
    }
    if (gpu_on()) {
        double t = now_s();
        g_gpuDue = gpu_capture(d, g_eye, g_frame >> 1, g_framePose);
        g_readbackMs += (now_s() - t) * 1000;
        return;
    }
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
            HRESULT h1 = d->lpVtbl->CreateRenderTarget(d, dd.Width, dd.Height, dd.Format, D3DMULTISAMPLE_NONE, 0, FALSE, &s.rt, nullptr);
            HRESULT h2 = d->lpVtbl->CreateOffscreenPlainSurface(d, dd.Width, dd.Height, dd.Format, D3DPOOL_SYSTEMMEM, &s.sys, nullptr);
            HRESULT h3 = d->lpVtbl->CreateQuery(d, D3DQUERYTYPE_EVENT, &s.q);
            static int fails;
            if ((FAILED(h1) || FAILED(h2) || FAILED(h3)) && fails++ < 3)  // E_OUTOFMEMORY: the 2 GB address space is full
                log_printf("readback surfaces %ux%u: render target 0x%08lx, system memory 0x%08lx, query 0x%08lx\n", dd.Width, dd.Height, h1, h2, h3);
        }
        g_copyW = dd.Width;
        g_copyH = dd.Height;
        g_copyFmt = dd.Format;
    }

    Slot& cur = g_slot[g_frame & 1];
    Slot& prev = g_slot[(g_frame & 1) ^ 1];
    if (cur.rt && cur.sys && cur.q) {
        if (cur.pending) finish_slot(cur);
        double tr = now_s();
        HRESULT h1 = d->lpVtbl->StretchRect(d, bb, nullptr, cur.rt, nullptr, D3DTEXF_NONE);
        HRESULT h2 = SUCCEEDED(h1) ? d->lpVtbl->GetRenderTargetData(d, cur.rt, cur.sys) : E_FAIL;
        g_readbackMs += (now_s() - tr) * 1000;
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

static void auto_drive(unsigned draws, unsigned draws3d) {
    static double tStart = now_s(), tSince = tStart, tUp, tCar;
    static BYTE held;
    double t = now_s();
    if (held && t >= tUp) {
        keybd_event(held, (BYTE)MapVirtualKeyA(held, MAPVK_VK_TO_VSC), KEYEVENTF_KEYUP, 0);
        held = 0;
    }
    // TDUVR_DRIVETEST=<seconds>: that long after the in-car camera is found, hold the accelerator
    // (Up) for 5 s, to see the car move in a test without a headset.
    static char dt[16];
    static bool driveTest = GetEnvironmentVariableA("TDUVR_DRIVETEST", dt, sizeof dt) > 0;
    if (driveTest && !held && g_camObj && GetForegroundWindow() == g_gameWnd) {
        if (!tCar) tCar = t;
        if (t - tCar > atof(dt)) {
            driveTest = false;
            held = VK_UP;
            keybd_event(held, (BYTE)MapVirtualKeyA(held, MAPVK_VK_TO_VSC), 0, 0);
            tUp = t + 5;
            log_printf("drive test: holding Up for 5 s\n");
        }
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

// TDUVR_ABTEST=<seconds>: once in the car, cycle constant batching and copy lag through all four
// combinations, <seconds> each, to compare them in one run (the status lines name the mode).
static void ab_test(IDirect3DDevice9* d) {
    static double every = [] {
        char v[16];
        return GetEnvironmentVariableA("TDUVR_ABTEST", v, sizeof v) ? atof(v) : 0.0;
    }();
    static double tCar;
    static int mode = -1;
    if (every <= 0 || !g_camObj) return;
    if (!tCar) tCar = now_s();
    int m = (int)((now_s() - tCar) / every) % 4;
    if (m == mode) return;
    mode = m;
    bool batch = (m & 1) != 0, pair = (m & 2) != 0;
    gpu_set_pair_copy(pair);
    set_batch(d, batch);
    log_printf("A/B test: constant batching %s, pair copy %s\n", batch ? "on" : "off", pair ? "on" : "off");
}

static void on_present(IDirect3DDevice9* d) {
    {
        static double tPrev;
        static unsigned __int64 stepsPrev;
        double tn = now_s();
        if (tPrev > 0) g_frameMs.push_back((float)((tn - tPrev) * 1000));
        tPrev = tn;
        if (g_knownExe) {
            unsigned __int64 s = (unsigned __int64)g_simSteps + *kSimSteps;  // pair steps + the callback's
            g_stepHist[s - stepsPrev < 3 ? (int)(s - stepsPrev) : 3]++;
            stepsPrev = s;
        }
    }
    log_status();
    if (g_vr) ab_test(d);
    if (g_vr) {
        static unsigned lastDraws, last3d;
        unsigned n3d = g_st[R_PAIR] + g_st[R_OTHER];
        auto_drive(g_cnt[C_DRAW] - lastDraws, n3d - last3d);
        lastDraws = g_cnt[C_DRAW];
        last3d = n3d;
        submit_backbuffer(d);
        if (!gpu_on()) mirror_eye(d);  // (D3D9On12 doesn't show the game on the desktop: skip_desktop)
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
        prof_frame(inCar);
        sim_frame(g_eye == 0, inCar);  // SimLock: the pair's steps, after its look is fed so its camera uses it
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
        drawlog_present();
        static bool cullOff = GetEnvironmentVariableA("TDUVR_NOCULLFIX", nullptr, 0) > 0;  // test: the game's own culling
        if ((!g_cullOrig || !g_setFovOrig) && g_knownExe && !cullOff) cull_hooks();  // retried until SecuROM has decrypted the code
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

// GPU path: the second half of the copy, once Present has submitted the frame's work.
static void finish_gpu_frame() {
    if (!g_gpuDue) return;
    g_gpuDue = false;
    double t = now_s();
    int eye = gpu_finish();
    g_readbackMs += (now_s() - t) * 1000;
    if (eye >= 0 && dump_due()) gpu_read_eye(eye, write_bmp);
}

// GPU copy: the game's frames are not shown on the desktop. D3D9On12's Present to the game window
// blocked ~300 ms on every other frame (measured, with or without V-Sync: 6 fps), and the headset
// gets its images from the GPU copy anyway. The window keeps the last frame shown before VR started.
static bool skip_desktop() { return g_vr && gpu_on(); }

static HRESULT STDMETHODCALLTYPE hPresent(IDirect3DDevice9* d, const RECT* a, const RECT* b, HWND w, const RGNDATA* r) {
    if (other_dev(d)) return H(d).Present(d, a, b, w, r);
    count(C_PRESENT);
    on_present(d);
    g_inPresent = true;
    double tp = now_s();
    HRESULT hr = skip_desktop() ? S_OK : H(d).Present(d, a, b, w, r);
    tp = (now_s() - tp) * 1000;
    g_presentMs += tp;
    if (tp > g_presentMax) g_presentMax = tp;
    g_inPresent = false;
    finish_gpu_frame();
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
    double tp = now_s();
    HRESULT hr = skip_desktop() ? S_OK : oSCPresent(sc, a, b, w, r, f);
    tp = (now_s() - tp) * 1000;
    g_presentMs += tp;
    if (tp > g_presentMax) g_presentMax = tp;
    finish_gpu_frame();
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
    if (other_dev(d)) return H(d).Reset(d, pp);
    release_copies();
    gpu_release();
    vk_release();
    force_pp(pp);
    HRESULT hr = H(d).Reset(d, pp);
    log_printf("Reset hr=0x%08lx %ux%u fmt %d windowed %d\n", hr, pp ? pp->BackBufferWidth : 0, pp ? pp->BackBufferHeight : 0,
               pp ? pp->BackBufferFormat : 0, pp ? pp->Windowed : -1);
    for (int r : g_mod) g_isMod[r] = false;
    g_mod.clear();
    consts_unknown();
    g_recording = false;
    g_uiClip = false;  // Reset turned the clip planes off
    g_uiClipGen = 0;
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
    if (other_dev(d)) return H(d).SetVertexShader(d, s);
    count(C_SVS);
    auto it = s ? g_shaders.find(s) : g_shaders.end();
    g_cur = it == g_shaders.end() ? nullptr : &it->second;
    return H(d).SetVertexShader(d, s);
}

static HRESULT STDMETHODCALLTYPE hSetVSConstF(IDirect3DDevice9* d, UINT start, const float* data, UINT count) {
    ::count(C_SVSC);
    if (g_recording || other_dev(d)) return H(d).SetVertexShaderConstantF(d, start, data, count);  // into the state block / not ours
    for (UINT i = 0; i < count && start + i < MAXC; i++) {
        memcpy(g_vsc[start + i], data + i * 4, 16);
        g_isMod[start + i] = false;  // hardware now holds the game's value again
    }
    if (!g_mod.empty()) {
        size_t w = 0;
        for (int r : g_mod) if (g_isMod[r]) g_mod[w++] = r;
        g_mod.resize(w);
    }
    if (g_batch && data && start + count <= MAXC) {
        for (UINT i = 0; i < count; i++) set_want(start + i, data + i * 4);
        return D3D_OK;
    }
    if (g_batch) {  // out of the shadowed range: keep the device's order, and forget what it held there
        flush_consts(d);
        for (UINT r = start; r < MAXC && r < start + count; r++) g_haveOk[r >> 5] &= ~(1u << (r & 31));
    }
    return H(d).SetVertexShaderConstantF(d, start, data, count);
}

// Calls that read or capture the vertex shader constants: the device must hold what the game set first.
static HRESULT STDMETHODCALLTYPE hGetVSConstF(IDirect3DDevice9* d, UINT start, float* data, UINT count) {
    if (g_batch && !other_dev(d)) flush_consts(d);
    return H(d).GetVertexShaderConstantF(d, start, data, count);
}
static HRESULT STDMETHODCALLTYPE hProcessVertices(IDirect3DDevice9* d, UINT a, UINT b, UINT c, IDirect3DVertexBuffer9* vb,
                                                  IDirect3DVertexDeclaration9* decl, DWORD f) {
    if (g_batch && !other_dev(d)) flush_consts(d);
    return H(d).ProcessVertices(d, a, b, c, vb, decl, f);
}
// State blocks (TDU records some). Their Capture and Apply read and write device state behind the
// batching shadows: both get the pending constants first, and after an Apply the device's constants are
// unknown (the block may hold some) until the game sets them again, and rewrites are redone. While the
// game records a block, its constant calls go straight to the device: they belong to the block, the
// device's live state doesn't change.
static decltype(IDirect3DStateBlock9Vtbl::Apply) oSbApply;
static decltype(IDirect3DStateBlock9Vtbl::Capture) oSbCapture;

static HRESULT STDMETHODCALLTYPE hSbApply(IDirect3DStateBlock9* sb) {
    IDirect3DDevice9* d = g_watchDev;
    if (g_batch && d) flush_consts(d);
    HRESULT hr = oSbApply(sb);
    g_stSbApply++;
    memset(g_haveOk, 0, sizeof g_haveOk);
    for (int r : g_mod) g_isMod[r] = false;
    g_mod.clear();
    return hr;
}
static HRESULT STDMETHODCALLTYPE hSbCapture(IDirect3DStateBlock9* sb) {
    IDirect3DDevice9* d = g_watchDev;
    if (g_batch && d) flush_consts(d);
    g_stSbCapture++;
    return oSbCapture(sb);
}
static void hook_state_block(IDirect3DStateBlock9* sb) {
    if (!sb) return;
    g_stSbCreate++;
    // shortcut: all state blocks assumed to share d3d9's Apply/Capture (like the swap chain Present above)
    if (sb->lpVtbl->Apply != hSbApply) {
        decltype(oSbApply) a;
        hook(&sb->lpVtbl->Apply, hSbApply, &a);
        if (!oSbApply) oSbApply = a;
    }
    if (sb->lpVtbl->Capture != hSbCapture) {
        decltype(oSbCapture) c;
        hook(&sb->lpVtbl->Capture, hSbCapture, &c);
        if (!oSbCapture) oSbCapture = c;
    }
}
static HRESULT STDMETHODCALLTYPE hCreateStateBlock(IDirect3DDevice9* d, D3DSTATEBLOCKTYPE t, IDirect3DStateBlock9** out) {
    if (other_dev(d)) return H(d).CreateStateBlock(d, t, out);
    if (g_batch) flush_consts(d);  // it captures the current state
    HRESULT hr = H(d).CreateStateBlock(d, t, out);
    if (SUCCEEDED(hr) && out) hook_state_block(*out);
    return hr;
}
static HRESULT STDMETHODCALLTYPE hBeginStateBlock(IDirect3DDevice9* d) {
    if (other_dev(d)) return H(d).BeginStateBlock(d);
    if (g_batch) flush_consts(d);
    HRESULT hr = H(d).BeginStateBlock(d);
    if (SUCCEEDED(hr)) g_recording = true;
    static int n;
    if (n++ < 5) log_printf("state block: recording starts (device %p, frame %u, hr 0x%08lx)\n", (void*)d, g_frame, hr);
    return hr;
}
static HRESULT STDMETHODCALLTYPE hEndStateBlock(IDirect3DDevice9* d, IDirect3DStateBlock9** out) {
    if (other_dev(d)) return H(d).EndStateBlock(d, out);
    HRESULT hr = H(d).EndStateBlock(d, out);
    g_recording = false;
    if (SUCCEEDED(hr) && out) hook_state_block(*out);
    static int n;
    if (n++ < 5) log_printf("state block: recording ends (device %p, frame %u, hr 0x%08lx)\n", (void*)d, g_frame, hr);
    return hr;
}

static HRESULT STDMETHODCALLTYPE hSetRenderTarget(IDirect3DDevice9* d, DWORD idx, IDirect3DSurface9* s) {
    if (other_dev(d)) return H(d).SetRenderTarget(d, idx, s);
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
    if (vp && !other_dev(d)) g_vp = *vp;
    return H(d).SetViewport(d, vp);
}

static HRESULT STDMETHODCALLTYPE hDrawPrimitive(IDirect3DDevice9* d, D3DPRIMITIVETYPE t, UINT a, UINT b) {
    if (g_dlFrames) g_da = DrawArgs{ "DP", t, b, 0, a, prim_verts(t, b), nullptr, 0 };
    if (!other_dev(d)) apply(d);
    return H(d).DrawPrimitive(d, t, a, b);
}
static HRESULT STDMETHODCALLTYPE hDrawIndexedPrimitive(IDirect3DDevice9* d, D3DPRIMITIVETYPE t, INT a, UINT b, UINT c, UINT e, UINT f) {
    if (g_dlFrames) g_da = DrawArgs{ "DIP", t, f, a, b, c, nullptr, 0 };
    if (!other_dev(d)) apply(d);
    return H(d).DrawIndexedPrimitive(d, t, a, b, c, e, f);
}
static HRESULT STDMETHODCALLTYPE hDrawPrimitiveUP(IDirect3DDevice9* d, D3DPRIMITIVETYPE t, UINT a, const void* b, UINT c) {
    if (g_dlFrames) g_da = DrawArgs{ "DPUP", t, a, 0, 0, prim_verts(t, a), b, c };
    if (!other_dev(d)) apply(d);
    return H(d).DrawPrimitiveUP(d, t, a, b, c);
}
static HRESULT STDMETHODCALLTYPE hDrawIndexedPrimitiveUP(IDirect3DDevice9* d, D3DPRIMITIVETYPE t, UINT a, UINT b, UINT c, const void* e, D3DFORMAT f, const void* g, UINT h) {
    if (g_dlFrames) g_da = DrawArgs{ "DIPUP", t, c, 0, a, b, g, h };
    if (!other_dev(d)) apply(d);
    return H(d).DrawIndexedPrimitiveUP(d, t, a, b, c, e, f, g, h);
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
    hook(&vt->DrawPrimitive, hDrawPrimitive, &o.DrawPrimitive);
    hook(&vt->DrawIndexedPrimitive, hDrawIndexedPrimitive, &o.DrawIndexedPrimitive);
    hook(&vt->DrawPrimitiveUP, hDrawPrimitiveUP, &o.DrawPrimitiveUP);
    hook(&vt->DrawIndexedPrimitiveUP, hDrawIndexedPrimitiveUP, &o.DrawIndexedPrimitiveUP);
    hook(&vt->GetVertexShaderConstantF, hGetVSConstF, &o.GetVertexShaderConstantF);
    hook(&vt->ProcessVertices, hProcessVertices, &o.ProcessVertices);
    hook(&vt->CreateStateBlock, hCreateStateBlock, &o.CreateStateBlock);
    hook(&vt->BeginStateBlock, hBeginStateBlock, &o.BeginStateBlock);
    hook(&vt->EndStateBlock, hEndStateBlock, &o.EndStateBlock);

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

// SteamVR and the settings, once, at the game's first Direct3DCreate9: whether VR is on decides
// whether the game's D3D9 runs on D3D12 (GPU copy).
static void init_once() {
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
            g_knownExe = headlook_verify();
            g_headLook = vr_ini_float("HeadLook", 1) != 0 && g_knownExe;
            g_lookTest = GetEnvironmentVariableA("TDUVR_LOOKTEST", nullptr, 0) > 0;
            log_printf("head look: %s%s\n", g_headLook ? "on" : "off (setting or unknown exe)", g_lookTest ? ", TEST MODE" : "");
            if (g_headLook && !g_lookTest) CreateThread(nullptr, 0, cam_finder, nullptr, 0, nullptr);
            g_autoStage = vr_ini_float("AutoDrive", 1) != 0 ? 1 : 0;
            log_printf("auto drive: %s\n", g_autoStage ? "on" : "off");
            sim_lock_start();
            prof_init();
            char cb[4] = "";
            g_batch = !(GetEnvironmentVariableA("TDUVR_CONSTBATCH", cb, sizeof cb) > 0 && cb[0] == '0');  // on unless =0
            log_printf("constant batching: %s\n", g_batch ? "on" : "off");
            // In the headset nobody touches mouse or keyboard (and a gamepad doesn't count as input to
            // Windows): keep the monitor from going to sleep, which loses a plain D3D9 device.
            SetThreadExecutionState(ES_CONTINUOUS | ES_DISPLAY_REQUIRED | ES_SYSTEM_REQUIRED);
            vr_begin_frame(0, true, g_maxYaw, g_maxPitch, g_lookTilt, &g_ev, &g_framePose);
            vr_eye_for_look(0, 0, 0, g_lookTilt, &g_ev);
            new_frame_matrices();
        }
    }
}

static HMODULE real_d3d9();

static HRESULT STDMETHODCALLTYPE hCreateDevice(IDirect3D9* This, UINT ad, D3DDEVTYPE t, HWND w, DWORD f, D3DPRESENT_PARAMETERS* pp, IDirect3DDevice9** out) {
    init_once();
    auto real = g_d3dOrig[0].CreateDevice;
    for (int i = 0; i < g_nD3D && i < 16; i++) if (g_d3dOrig[i].vt == This->lpVtbl) real = g_d3dOrig[i].CreateDevice;
    // In the house the game opens a tiny second device (120x61) while the main one keeps drawing.
    // Taking it over reset the mod's state (back buffer size, copies) and crashed in d3d9's shared
    // vtable, so a device too small to be the game's view is left alone. With the GPU copy it gets
    // plain d3d9: its own Present to the desktop would stall in D3D9On12 like the main one's.
    UINT bw = pp ? pp->BackBufferWidth : 0;
    HWND cw = pp && pp->hDeviceWindow ? pp->hDeviceWindow : w;
    RECT cr;
    if (!bw && cw && GetClientRect(cw, &cr)) bw = (UINT)(cr.right - cr.left);
    if (g_vr && bw && bw < 320) {
        IDirect3D9* by = This;
        if (gpu_on()) {
            static IDirect3D9* native = [] {
                auto fn = (IDirect3D9*(WINAPI*)(UINT))GetProcAddress(real_d3d9(), "Direct3DCreate9");
                return fn ? fn(D3D_SDK_VERSION) : nullptr;
            }();
            if (native) by = native;  // same d3d9.dll class: the original CreateDevice works on it
        }
        HRESULT hr = real(by, ad, t, w, f, pp, out);
        log_printf("small extra device %ux%u (hr=0x%08lx%s): left alone\n", bw, pp ? pp->BackBufferHeight : 0, hr, by != This ? ", plain d3d9" : "");
        return hr;
    }
    force_pp(pp);
    if (w) g_gameWnd = w;
    else if (pp && pp->hDeviceWindow) g_gameWnd = pp->hDeviceWindow;
    release_copies();  // they belong to the previous device
    gpu_release();
    vk_release();
    for (int r : g_mod) g_isMod[r] = false;
    g_mod.clear();
    consts_unknown();
    g_recording = false;
    g_cur = nullptr;
    g_uiClip = false;
    g_uiClipGen = 0;
    HRESULT hr = real(This, ad, t, w, f, pp, out);
    log_printf("CreateDevice hr=0x%08lx type %d flags 0x%lx %ux%u windowed %d\n", hr, t, f,
               pp ? pp->BackBufferWidth : 0, pp ? pp->BackBufferHeight : 0, pp ? pp->Windowed : -1);
    if (FAILED(hr) && pp)
        log_printf("  failed with: adapter %u of %u, format %d x%u, multisample %d/%lu, swap effect %d, window %p/%p, depth %d/%d, flags 0x%lx, refresh %u, interval 0x%x\n",
                   ad, This->lpVtbl->GetAdapterCount(This), pp->BackBufferFormat, pp->BackBufferCount, pp->MultiSampleType, pp->MultiSampleQuality, pp->SwapEffect,
                   (void*)pp->hDeviceWindow, (void*)w, pp->EnableAutoDepthStencil, pp->AutoDepthStencilFormat, pp->Flags,
                   pp->FullScreen_RefreshRateInHz, pp->PresentationInterval);
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
        SYSTEM_INFO si;
        GetSystemInfo(&si);
        log_printf("address space: %u MB (exe %s large address aware)\n", (unsigned)(((UINT_PTR)si.lpMaximumApplicationAddress >> 20) + 1),
                   (((IMAGE_NT_HEADERS*)((char*)GetModuleHandleA(nullptr) + ((IMAGE_DOS_HEADER*)GetModuleHandleA(nullptr))->e_lfanew))->FileHeader.Characteristics &
                    IMAGE_FILE_LARGE_ADDRESS_AWARE) ? "is" : "is not");
        AddVectoredExceptionHandler(1, on_exception);
    } else if (reason == DLL_PROCESS_DETACH) {
        log_printf("process exiting (frame %u)\n", g_frame);
    }
    return TRUE;
}

// ---- exports ----
// DXVK=1 (tdu_vr.ini) or TDUVR_DXVK=1: the game's D3D9 runs on DXVK (Vulkan) instead of Windows' d3d9:
// dxvk_d3d9.dll next to this dll, which is DXVK's 32-bit d3d9.dll renamed. Only with a headset: without
// one the mod stays out of the way.
static bool g_onDxvk;  // DXVK is loaded and is the game's D3D9
static bool use_dxvk() { return g_onDxvk; }

static HMODULE real_d3d9() {
    static HMODULE h;
    if (!h) {
        init_once();  // whether VR is on (and DXVK wanted) decides which d3d9 to load
        char p[MAX_PATH];
        if (g_vr && (GetEnvironmentVariableA("TDUVR_DXVK", nullptr, 0) > 0 || vr_ini_float("DXVK", 0) != 0)) {
            GetModuleFileNameA((HMODULE)&__ImageBase, p, MAX_PATH);
            char* slash = strrchr(p, '\\');
            if (slash) strcpy_s(slash + 1, MAX_PATH - (slash + 1 - p), "dxvk_d3d9.dll");
            h = LoadLibraryA(p);
            g_onDxvk = h && GetProcAddress(h, "Direct3DCreate9");
            log_printf("DXVK: %s %s\n", p, g_onDxvk ? "loaded" : "could not be loaded, using Windows' d3d9");
            if (g_onDxvk) return h;
        }
        GetSystemDirectoryA(p, MAX_PATH);  // SysWOW64 for this 32-bit process
        strcat_s(p, "\\d3d9.dll");
        h = LoadLibraryA(p);
    }
    return h;
}

// DXVK reports the GPU a little differently from Windows' d3d9 (a made-up driver version, no subsystem
// id or revision, modes in another order), and TDU resets its video settings when the GPU it sees
// changes: one run on DXVK set the saved resolution back to the default. So on DXVK the game gets
// Windows' answers for its adapter's identity and display modes.
static IDirect3D9* windows_d3d9() {
    static IDirect3D9* w = [] {
        char p[MAX_PATH];
        GetSystemDirectoryA(p, MAX_PATH);
        strcat_s(p, "\\d3d9.dll");
        HMODULE m = LoadLibraryA(p);
        auto fn = m ? (IDirect3D9*(WINAPI*)(UINT))GetProcAddress(m, "Direct3DCreate9") : nullptr;
        return fn ? fn(D3D_SDK_VERSION) : nullptr;
    }();
    return w;
}
static HRESULT STDMETHODCALLTYPE hGetAdapterIdentifier(IDirect3D9* This, UINT a, DWORD f, D3DADAPTER_IDENTIFIER9* id) {
    IDirect3D9* w = windows_d3d9();
    return w ? w->lpVtbl->GetAdapterIdentifier(w, a, f, id) : D3DERR_NOTAVAILABLE;
}
static UINT STDMETHODCALLTYPE hGetAdapterModeCount(IDirect3D9* This, UINT a, D3DFORMAT fmt) {
    IDirect3D9* w = windows_d3d9();
    return w ? w->lpVtbl->GetAdapterModeCount(w, a, fmt) : 0;
}
static HRESULT STDMETHODCALLTYPE hEnumAdapterModes(IDirect3D9* This, UINT a, D3DFORMAT fmt, UINT i, D3DDISPLAYMODE* m) {
    IDirect3D9* w = windows_d3d9();
    return w ? w->lpVtbl->EnumAdapterModes(w, a, fmt, i, m) : D3DERR_NOTAVAILABLE;
}
static void mirror_windows_adapter(IDirect3D9* d3d) {
    if (!windows_d3d9() || d3d->lpVtbl->GetAdapterIdentifier == hGetAdapterIdentifier) return;
    decltype(IDirect3D9Vtbl::GetAdapterIdentifier) a;
    decltype(IDirect3D9Vtbl::GetAdapterModeCount) b;
    decltype(IDirect3D9Vtbl::EnumAdapterModes) c;
    hook(&d3d->lpVtbl->GetAdapterIdentifier, hGetAdapterIdentifier, &a);
    hook(&d3d->lpVtbl->GetAdapterModeCount, hGetAdapterModeCount, &b);
    hook(&d3d->lpVtbl->EnumAdapterModes, hEnumAdapterModes, &c);
    log_printf("DXVK: adapter identity and modes come from Windows' d3d9\n");
}

extern "C" IDirect3D9* WINAPI Direct3DCreate9(UINT sdk) {
    auto fn = (IDirect3D9*(WINAPI*)(UINT))GetProcAddress(real_d3d9(), "Direct3DCreate9");
    // GpuCopy=1 (tdu_vr.ini) and SteamVR running: run the game's D3D9 on D3D12 (D3D9On12), so the eye
    // images never pass through system memory. Decided here, before the game has a device. Never
    // without VR: 9On12 then has to show the game on the desktop, and its Present stalls (see hPresent).
    init_once();
    static int gpuWanted = -1;
    if (gpuWanted < 0) {
        gpuWanted = g_vr && vr_ini_float("GpuCopy", 1) != 0 && !g_onDxvk;
        log_printf("GPU copy (D3D9On12): %s\n", gpuWanted ? "on" : "off");
    }
    IDirect3D9* d3d = gpuWanted ? (IDirect3D9*)gpu_create_d3d9(real_d3d9(), sdk) : nullptr;
    if (gpuWanted && !d3d) { gpuWanted = 0; log_printf("GPU copy: D3D9On12 unavailable, using the CPU copy\n"); }
    if (!d3d && fn) d3d = fn(sdk);
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
    if (d3d && g_onDxvk) mirror_windows_adapter(d3d);
    return d3d;
}

extern "C" void WINAPI D3DPERF_SetOptions(DWORD opt) {
    auto fn = (void(WINAPI*)(DWORD))GetProcAddress(real_d3d9(), "D3DPERF_SetOptions");
    if (fn) fn(opt);
}
