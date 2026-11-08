// SteamVR side: head/eye poses -> game view space, and a presenter thread that gives the
// compositor the latest image of each eye every headset frame.
//
// Why a thread: with alternate-eye rendering each game frame finishes only one eye. If the game
// thread paced itself on WaitGetPoses, SteamVR would see half-finished frames whenever the game
// runs below twice the refresh rate, throttle the app and (by default) turn on Motion Smoothing.
// The presenter re-submits each eye's newest image with the pose it was rendered with, so the
// compositor gets a complete, on-time frame every refresh and only has to reproject.
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <d3d11.h>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>
#include <openvr.h>
#include "vr.h"

static vr::IVRSystem* g_sys;
static ID3D11Device* g_dev;
static ID3D11DeviceContext* g_ctx;  // presenter thread only, after init
static Mat4 g_originInv = m4_identity();
static bool g_needRecenter = true;
static float g_scale = 1.0f, g_freq = 90.0f, g_v2p = 0.011f;
static vr::HmdMatrix34_t g_lastPose = { { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 } } };
static Mat4 g_eyeToHead[2];
static float g_raw[2][4];           // GetProjectionRaw l, r, t, b per eye
static double g_lastBegin, g_frameDt = 1.0 / 90;

// Game thread -> presenter thread, triple buffered per eye so neither side ever waits.
struct EyeBuf { std::vector<unsigned char> px; int w = 0, h = 0; unsigned pair = 0; FramePose pose{}; };
struct EyeChain { EyeBuf buf[3]; int write = 0, latest = 1, read = 2; bool fresh = false; };
static EyeChain g_chain[2];
static std::mutex g_mx;

// Presenter thread state: the last few uploaded images per eye, so a left and a right image
// from the same pair can always be found and shown together.
struct GpuImg { ID3D11Texture2D* tex = nullptr; int w = 0, h = 0; unsigned pair = 0; FramePose pose{}; bool valid = false; };
static GpuImg g_img[2][3];
static int g_shown[2] = { -1, -1 };

// Stats for the log (written by the presenter under g_mx).
static unsigned g_stFrames, g_stUploads, g_stPairs;
static double g_stUploadMs;
static vr::Compositor_FrameTiming g_stTiming;

static double now_s() {
    static LARGE_INTEGER f = [] { LARGE_INTEGER x; QueryPerformanceFrequency(&x); return x; }();
    LARGE_INTEGER c;
    QueryPerformanceCounter(&c);
    return (double)c.QuadPart / (double)f.QuadPart;
}

static std::string game_dir() {
    char p[MAX_PATH];
    GetModuleFileNameA(nullptr, p, MAX_PATH);
    std::string s(p);
    return s.substr(0, s.find_last_of("\\/") + 1);
}

// SecuROM's launcher process starts the real game as a child (/Sonydadc ...): the launcher
// starts a fresh log, the child appends, every line tagged with the process id.
void log_printf(const char* fmt, ...) {
    static FILE* f = _fsopen((game_dir() + "tdu_vr.log").c_str(),
                             strstr(GetCommandLineA(), "/Sonydadc") ? "a" : "w", _SH_DENYNO);
    if (!f) return;
    char buf[1024];
    va_list a;
    va_start(a, fmt);
    vsnprintf(buf, sizeof buf, fmt, a);
    va_end(a);
    fprintf(f, "[%lu] %s", GetCurrentProcessId(), buf);
    fflush(f);
}

float vr_ini_float(const char* key, float def) {
    char buf[32], d[32];
    snprintf(d, sizeof d, "%g", def);
    GetPrivateProfileStringA("VR", key, d, buf, sizeof buf, (game_dir() + "tdu_vr.ini").c_str());
    return (float)atof(buf);
}

// OpenVR 3x4 (column-vector, right-handed) -> 4x4 in the same convention.
static Mat4 from34(const vr::HmdMatrix34_t& p) {
    Mat4 r = m4_identity();
    for (int i = 0; i < 3; i++) for (int j = 0; j < 4; j++) r.m[i][j] = p.m[i][j];
    return r;
}

static DWORD WINAPI presenter(void*) {
    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
    double lastTiming = 0;
    for (;;) {
        if (vr::VRCompositor()->WaitGetPoses(poses, vr::k_unMaxTrackedDeviceCount, nullptr, 0) != vr::VRCompositorError_None) {
            Sleep(5);  // no focus (dashboard / other app): don't spin
            continue;
        }
        double t0 = now_s();
        unsigned uploads = 0, pairs = 0;
        for (int e = 0; e < 2; e++) {
            EyeChain& c = g_chain[e];
            bool fresh;
            {
                std::lock_guard<std::mutex> lk(g_mx);
                fresh = c.fresh;
                if (fresh) { std::swap(c.read, c.latest); c.fresh = false; }
            }
            if (!fresh) continue;
            const EyeBuf& b = c.buf[c.read];
            // Upload into the oldest image slot that is not on screen.
            int s = -1;
            for (int i = 0; i < 3; i++) {
                if (i == g_shown[e]) continue;
                if (s < 0 || !g_img[e][i].valid || (g_img[e][s].valid && (int)(g_img[e][i].pair - g_img[e][s].pair) < 0)) s = i;
            }
            GpuImg& g = g_img[e][s];
            if (!g.tex || g.w != b.w || g.h != b.h) {
                if (g.tex) g.tex->Release();
                g.tex = nullptr;
                D3D11_TEXTURE2D_DESC td{};
                td.Width = b.w; td.Height = b.h; td.MipLevels = 1; td.ArraySize = 1;
                td.Format = DXGI_FORMAT_B8G8R8A8_UNORM;
                td.SampleDesc.Count = 1;
                td.Usage = D3D11_USAGE_DEFAULT;
                td.BindFlags = D3D11_BIND_SHADER_RESOURCE;
                if (FAILED(g_dev->CreateTexture2D(&td, nullptr, &g.tex))) { log_printf("CreateTexture2D failed\n"); g.valid = false; continue; }
                g.w = b.w; g.h = b.h;
            }
            g_ctx->UpdateSubresource(g.tex, 0, nullptr, b.px.data(), b.w * 4, 0);
            g.pair = b.pair;
            g.pose = b.pose;
            g.valid = true;
            uploads++;
        }
        // Show the newest pair that both eyes have (both images share one head pose).
        int bl = -1, br = -1;
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
            const GpuImg &L = g_img[0][i], &R = g_img[1][j];
            if (L.valid && R.valid && L.pair == R.pair && (bl < 0 || (int)(L.pair - g_img[0][bl].pair) > 0)) { bl = i; br = j; }
        }
        if (bl >= 0 && (g_shown[0] < 0 || (int)(g_img[0][bl].pair - g_img[0][g_shown[0]].pair) > 0)) {
            g_shown[0] = bl;
            g_shown[1] = br;
            pairs++;
        }
        double t1 = now_s();

        if (g_shown[0] >= 0 && g_shown[1] >= 0) {
            for (int e = 0; e < 2; e++) {
                const GpuImg& g = g_img[e][g_shown[e]];
                vr::VRTextureWithPose_t tex{};
                tex.handle = g.tex;
                tex.eType = vr::TextureType_DirectX;
                tex.eColorSpace = vr::ColorSpace_Gamma;
                memcpy(tex.mDeviceToAbsoluteTracking.m, g.pose.m, sizeof g.pose.m);  // reproject from the render pose
                vr::VRTextureBounds_t bounds{ 0, 0, 1, 1 };
                vr::EVRCompositorError err = vr::VRCompositor()->Submit((vr::EVREye)e, &tex, &bounds, vr::Submit_TextureWithPose);
                static int errors;
                if (err != vr::VRCompositorError_None && errors++ < 10) log_printf("Submit eye %d error %d\n", e, err);
            }
            g_ctx->Flush();
        }

        std::lock_guard<std::mutex> lk(g_mx);
        g_stFrames++;
        g_stUploads += uploads;
        g_stPairs += pairs;
        g_stUploadMs += (t1 - t0) * 1000;
        if (t1 - lastTiming > 1.0) {
            lastTiming = t1;
            g_stTiming = {};
            g_stTiming.m_nSize = sizeof g_stTiming;
            vr::VRCompositor()->GetFrameTiming(&g_stTiming, 0);
        }
    }
}

bool vr_init() {
    g_scale = vr_ini_float("WorldScale", 1.0f);
    if (!(g_scale > 0.01f && g_scale < 100.0f)) g_scale = 1.0f;
    log_printf("TDU VR: WorldScale %.3f\n", g_scale);

    if (!vr::VR_IsHmdPresent()) { log_printf("no headset found, passthrough\n"); return false; }
    vr::EVRInitError e = vr::VRInitError_None;
    g_sys = vr::VR_Init(&e, vr::VRApplication_Scene);
    if (!g_sys) { log_printf("VR_Init failed: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(e)); return false; }
    if (!vr::VRCompositor()) { log_printf("no SteamVR compositor\n"); vr::VR_Shutdown(); return false; }
    vr::VRCompositor()->SetTrackingSpace(vr::TrackingUniverseSeated);

    if (FAILED(D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, nullptr, 0, D3D11_SDK_VERSION, &g_dev, nullptr, &g_ctx))) {
        log_printf("D3D11CreateDevice failed\n");
        vr::VR_Shutdown();
        return false;
    }
    float f = g_sys->GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_DisplayFrequency_Float);
    if (f > 30) g_freq = f;
    g_v2p = g_sys->GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SecondsFromVsyncToPhotons_Float);

    uint32_t w, h;
    g_sys->GetRecommendedRenderTargetSize(&w, &h);
    for (int eye = 0; eye < 2; eye++) {
        g_eyeToHead[eye] = from34(g_sys->GetEyeToHeadTransform((vr::EVREye)eye));
        float* t = g_raw[eye];
        g_sys->GetProjectionRaw((vr::EVREye)eye, &t[0], &t[1], &t[2], &t[3]);
        log_printf("eye %d tangents l %.3f r %.3f t %.3f b %.3f, offset x %.4f m\n", eye, t[0], t[1], t[2], t[3], g_eyeToHead[eye].m[0][3]);
    }
    log_printf("SteamVR ready: %.0f Hz, recommended per-eye %ux%u (aspect %.2f)\n", g_freq, w, h, (double)w / h);
    CreateThread(nullptr, 0, presenter, nullptr, 0, nullptr);
    return true;
}

void vr_recenter() { g_needRecenter = true; }

static Mat4 g_pairH = m4_identity();  // head pose of the current pair, relative to the recentered origin

void vr_eye_for_look(int eye, float yaw, float pitch, float tilt, EyeView* out) {
    // The game turned its camera by G = Rx(-tilt) * Ry(yaw) * Rx(tilt + pitch) (measured): take G
    // back out of the head pose, so the rewrite only adds roll, position and eye offset.
    Mat4 G = m4_mul(m4_mul(m4_rot_x(-tilt), m4_rot_y(yaw)), m4_rot_x(tilt + pitch));
    // Eye pose in game camera space (column-vector, right-handed like OpenVR; the proxy
    // flips Z if the game turns out to be left-handed).
    Mat4 M = m4_mul(m4_mul(m4_transpose(G), g_pairH), g_eyeToHead[eye]);  // inverse of a rotation = its transpose
    for (int i = 0; i < 3; i++) M.m[i][3] *= g_scale;
    Mat4 inv;
    m4_inverse(M, inv);
    out->invEye = m4_transpose(inv);  // to row-vector convention
}

void vr_begin_frame(int eye, bool newPair, float maxYaw, float maxPitch, float tilt, EyeView* out, FramePose* pose) {
    double tNow = now_s();
    if (g_lastBegin > 0) g_frameDt = 0.9 * g_frameDt + 0.1 * (tNow - g_lastBegin < 0.1 ? tNow - g_lastBegin : 0.1);
    g_lastBegin = tNow;

    static vr::HmdMatrix34_t pairPose = g_lastPose;
    if (newPair) {
        // Predict for when this pair reaches the eyes: both eyes rendered + one frame readback lag
        // + presenter pickup + photons. The compositor corrects the rest from the tagged pose.
        float predict = (float)(3 * g_frameDt) + 0.5f / g_freq + g_v2p;
        vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
        g_sys->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseSeated, predict < 0.1f ? predict : 0.1f, poses, vr::k_unMaxTrackedDeviceCount);
        const auto& hp = poses[vr::k_unTrackedDeviceIndex_Hmd];
        if (hp.bPoseIsValid) g_lastPose = hp.mDeviceToAbsoluteTracking;
        // TDUVR_FAKEHEAD=<degrees>,<seconds>: test without wearing the headset; the head looks straight
        // ahead and turns left/right by that many degrees, once per that many seconds.
        static char fake[32];
        static bool fakeHead = GetEnvironmentVariableA("TDUVR_FAKEHEAD", fake, sizeof fake) > 0;
        if (fakeHead) {
            const char* c = strchr(fake, ',');
            float amp = (float)atof(fake) * 3.14159265f / 180, per = c ? (float)atof(c + 1) : 4;
            Mat4 r = m4_rot_y(amp * sinf(2 * 3.14159265f * (float)fmod(tNow, per > 0.1f ? per : 4) / (per > 0.1f ? per : 4)));
            for (int i = 0; i < 3; i++) for (int j = 0; j < 4; j++) g_lastPose.m[i][j] = r.m[i][j];
        }
        pairPose = g_lastPose;

        if (g_needRecenter && hp.bPoseIsValid) {
            // New origin: current head position, yaw only (keep the horizon level).
            g_needRecenter = false;
            Mat4 head = from34(pairPose);
            float a = atan2f(head.m[0][2], head.m[2][2]), c = cosf(a), s = sinf(a);
            Mat4 o = m4_identity();
            o.m[0][0] = c;  o.m[0][2] = s;
            o.m[2][0] = -s; o.m[2][2] = c;
            o.m[0][3] = head.m[0][3]; o.m[1][3] = head.m[1][3]; o.m[2][3] = head.m[2][3];
            m4_inverse(o, g_originInv);
            log_printf("recentered\n");
        }
    }
    memcpy(pose->m, pairPose.m, sizeof pose->m);

    // Head pose relative to the recentered origin (= the game camera, column-vector, right-handed),
    // and the look to give the game: split like the game applies it (in the frame it yaws in).
    g_pairH = m4_mul(g_originInv, from34(pairPose));
    m4_head_look(g_pairH, tilt, maxYaw, maxPitch, &out->lookYaw, &out->lookPitch);
    vr_eye_for_look(eye, 0, 0, tilt, out);  // until the caller knows what the game applied

    // OpenVR's top/bottom are inverted: Valve's own projection maps the image top (NDC +y) to
    // "bottom", so bottom = tan(up) and top = -tan(down). (Quest 3 reports t -1.428 / b 0.966:
    // 55 deg down, 44 deg up.)
    const float* t = g_raw[eye];
    out->l = t[0]; out->r = t[1]; out->up = t[3]; out->down = -t[2];
}

void vr_publish(int eye, unsigned pair, const FramePose& pose, const void* bgra, int pitch, int w, int h) {
    EyeChain& c = g_chain[eye];
    EyeBuf& b = c.buf[c.write];  // only this (game) thread touches the write buffer
    size_t row = (size_t)w * 4;
    b.px.resize(row * h);
    for (int y = 0; y < h; y++) memcpy(&b.px[row * y], (const unsigned char*)bgra + (size_t)pitch * y, row);
    b.w = w; b.h = h; b.pair = pair; b.pose = pose;
    std::lock_guard<std::mutex> lk(g_mx);
    std::swap(c.write, c.latest);
    c.fresh = true;
}

const char* vr_status() {
    static char buf[600];
    static unsigned lastFrames, lastUploads, lastPairs;
    static double lastUploadMs, lastT;
    double t = now_s(), dt = lastT > 0 ? t - lastT : 0;  // first call: no interval yet
    std::lock_guard<std::mutex> lk(g_mx);
    unsigned nf = g_stFrames - lastFrames, nu = g_stUploads - lastUploads, np = g_stPairs - lastPairs;
    const auto& ft = g_stTiming;
    static const char* kActivity[] = { "idle", "in use", "in use (paused)", "STANDBY", "idle" };
    int act = g_sys->GetTrackedDeviceActivityLevel(vr::k_unTrackedDeviceIndex_Hmd);
    snprintf(buf, sizeof buf,
             "presenter %.0f Hz, new eye images %.0f/s (upload %.2f ms each), new pairs shown %.0f/s | SteamVR: headset %s, throttled %u, "
             "motion smoothing %s, dropped %u, app gpu %.1f ms (total %.1f), compositor gpu %.1f ms, submit %.2f ms",
             dt > 0 ? nf / dt : 0.0, dt > 0 ? nu / dt : 0.0, nu ? (g_stUploadMs - lastUploadMs) / nu : 0.0, dt > 0 ? np / dt : 0.0,
             act >= 0 && act <= 4 ? kActivity[act] : "?", VR_COMPOSITOR_NUMBER_OF_THROTTLED_FRAMES(ft),
             (ft.m_nReprojectionFlags & vr::VRCompositor_ReprojectionMotion) ? "ACTIVE" : "off", ft.m_nNumDroppedFrames,
             ft.m_flPreSubmitGpuMs, ft.m_flTotalRenderGpuMs, ft.m_flCompositorRenderGpuMs, ft.m_flSubmitFrameMs);
    lastFrames = g_stFrames; lastUploads = g_stUploads; lastPairs = g_stPairs; lastUploadMs = g_stUploadMs; lastT = t;
    return buf;
}
