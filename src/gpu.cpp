// D3D9On12 path. Each frame: StretchRect the back buffer into a render target of ours, take its D3D12
// resource out of 9On12 (UnwrapUnderlyingResource: it arrives in the COMMON state and our queue waits
// for 9On12's work on it), copy it into an eye image on our queue, and hand the render target back
// with a fence 9On12 waits on before writing it again. The presenter submits the eye images to
// SteamVR on that same queue, so the compositor's read is ordered after our copy.
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <d3d9.h>
#include <d3d12.h>
#include <d3d9on12.h>
#include <cstdio>
#include <utility>
#include "gpu.h"

static ID3D12Device* g_dev;
static ID3D12CommandQueue* g_q9;  // 9On12's queue: the game's rendering
static ID3D12CommandQueue* g_q;   // our copies, and the compositor's reads of the eye images
static ID3D12Fence* g_fence;
static UINT64 g_fv;
static HANDLE g_ev;

struct Cmd { ID3D12CommandAllocator* alloc; ID3D12GraphicsCommandList* list; UINT64 fv; };
static Cmd g_cmd[4];
static unsigned g_cmdN;

static IDirect3DDevice9* g_rtDev;  // device the render targets below belong to
static IDirect3DDevice9On12* g_on12;
static UINT g_w, g_h;
// The back buffer is copied into the render target of its eye, then copied on in D3D12 once the pair
// is complete (see gpu_finish).
struct RtSlot {
    IDirect3DSurface9* rt;
    unsigned uses;       // frames since the render target was made
    bool captured;       // holds a frame gpu_finish has not taken yet
    int eye;
    unsigned pair;
    FramePose pose;
};
static RtSlot g_slots[2];  // by eye
static bool g_pairCopy = true;  // A/B test: false copies every frame
// 9On12 keeps every fence handed back with a resource: ReturnUnderlyingResource got slower with each
// frame (+0.03 ms per 1000, measured: ~10 ms per frame after an hour). So a render target is replaced
// every kRtFrames frames; the old one is freed once our last copy from it is done.
static const unsigned kRtFrames = 1024;
static IDirect3DSurface9* g_oldRt[2];
static UINT64 g_oldRtFence[2];

// Time per phase of the copy, for the status line (ms summed since the last gpu_status call).
enum { P_CAPTURE, P_UNWRAP, P_RECORD, P_RETURN, P_THROTTLE, P_N };
static double g_ph[P_N];
static unsigned g_phN;
static double qpc_ms() {
    static LARGE_INTEGER f = [] { LARGE_INTEGER x; QueryPerformanceFrequency(&x); return x; }();
    LARGE_INTEGER c;
    QueryPerformanceCounter(&c);
    return c.QuadPart * 1000.0 / f.QuadPart;
}

const char* gpu_status() {
    static char buf[200];
    double n = g_phN ? g_phN : 1;
    snprintf(buf, sizeof buf, "GPU copy per frame: capture %.2f, unwrap %.2f, record+submit %.2f, return %.2f, wait for GPU %.2f ms",
             g_ph[P_CAPTURE] / n, g_ph[P_UNWRAP] / n, g_ph[P_RECORD] / n, g_ph[P_RETURN] / n, g_ph[P_THROTTLE] / n);
    for (auto& x : g_ph) x = 0;
    g_phN = 0;
    return buf;
}

static void wait_fence(UINT64 v) {
    if (g_fence->GetCompletedValue() >= v) return;
    g_fence->SetEventOnCompletion(v, g_ev);
    WaitForSingleObject(g_ev, 1000);
}

static bool init_device() {
    // shortcut: default adapter, the GPU of the primary display; pick the headset's adapter (IVRSystem::GetOutputDevice) on multi-GPU systems.
    HMODULE m = LoadLibraryA("d3d12.dll");
    auto create = m ? (PFN_D3D12_CREATE_DEVICE)GetProcAddress(m, "D3D12CreateDevice") : nullptr;
    if (!create || FAILED(create(nullptr, D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&g_dev)))) {
        log_printf("GPU copy: no D3D12 device\n");
        return false;
    }
    D3D12_COMMAND_QUEUE_DESC qd{};
    qd.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
    if (FAILED(g_dev->CreateCommandQueue(&qd, IID_PPV_ARGS(&g_q9))) || FAILED(g_dev->CreateCommandQueue(&qd, IID_PPV_ARGS(&g_q))) ||
        FAILED(g_dev->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&g_fence)))) {
        log_printf("GPU copy: could not create queues\n");
        return false;
    }
    g_ev = CreateEventA(nullptr, FALSE, FALSE, nullptr);
    for (auto& c : g_cmd) {
        if (FAILED(g_dev->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&c.alloc))) ||
            FAILED(g_dev->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, c.alloc, nullptr, IID_PPV_ARGS(&c.list))))
            return false;
        c.list->Close();
    }
    vr_gpu_init(g_dev, g_q, g_fence);
    return true;
}

void* gpu_create_d3d9(void* d3d9Module, unsigned sdk) {
    static bool tried, ok;
    if (!tried) {
        tried = true;
        ok = GetProcAddress((HMODULE)d3d9Module, "Direct3DCreate9On12") && init_device();
    }
    if (!ok) return nullptr;
    auto create = (PFN_Direct3DCreate9On12)GetProcAddress((HMODULE)d3d9Module, "Direct3DCreate9On12");
    D3D9ON12_ARGS a{};
    a.Enable9On12 = TRUE;
    a.pD3D12Device = g_dev;
    a.ppD3D12Queues[0] = g_q9;
    a.NumQueues = 1;
    return create(sdk, &a, 1);
}

bool gpu_on() { return g_dev != nullptr; }
void gpu_set_pair_copy(bool on) { g_pairCopy = on; }

void gpu_release() {
    bool any = false;
    for (int e = 0; e < 2; e++) any |= g_slots[e].rt || g_oldRt[e];
    if (any) wait_fence(g_fv);  // nothing of ours may still use them
    for (int e = 0; e < 2; e++) {
        if (g_slots[e].rt) g_slots[e].rt->Release();
        g_slots[e] = RtSlot{};
        if (g_oldRt[e]) g_oldRt[e]->Release();
        g_oldRt[e] = nullptr;
    }
    if (g_on12) g_on12->Release();
    g_on12 = nullptr;
    g_rtDev = nullptr;
}

static D3D12_RESOURCE_BARRIER transition(ID3D12Resource* r, D3D12_RESOURCE_STATES from, D3D12_RESOURCE_STATES to) {
    D3D12_RESOURCE_BARRIER b{};
    b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    b.Transition.pResource = r;
    b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    b.Transition.StateBefore = from;
    b.Transition.StateAfter = to;
    return b;
}

bool gpu_capture(void* device9, int eye, unsigned pair, const FramePose& pose) {
    double t0 = qpc_ms();
    struct Add { double t; ~Add() { g_ph[P_CAPTURE] += qpc_ms() - t; } } add{ t0 };
    IDirect3DDevice9* d = (IDirect3DDevice9*)device9;
    IDirect3DSurface9* bb = nullptr;
    if (FAILED(d->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &bb))) return false;
    D3DSURFACE_DESC dd;
    bb->GetDesc(&dd);
    if (!g_slots[0].rt || g_rtDev != d || g_w != dd.Width || g_h != dd.Height) {
        gpu_release();
        // A8R8G8B8: an X8 back buffer becomes B8G8R8A8 in D3D12, the format SteamVR takes (StretchRect sets alpha).
        bool ok = SUCCEEDED(d->QueryInterface(__uuidof(IDirect3DDevice9On12), (void**)&g_on12));
        for (auto& r : g_slots)
            ok = ok && SUCCEEDED(d->CreateRenderTarget(dd.Width, dd.Height, D3DFMT_A8R8G8B8, D3DMULTISAMPLE_NONE, 0, FALSE, &r.rt, nullptr));
        if (!ok) {
            static int fails;
            if (fails++ < 3) log_printf("GPU copy: no 9On12 interface or render target on device %p\n", (void*)d);
            gpu_release();
            bb->Release();
            return false;
        }
        g_rtDev = d;
        g_w = dd.Width;
        g_h = dd.Height;
        log_printf("GPU copy: %ux%u through D3D9On12\n", g_w, g_h);
    }
    const int e = eye & 1;
    if (g_oldRt[e] && g_fence->GetCompletedValue() >= g_oldRtFence[e]) {
        g_oldRt[e]->Release();
        g_oldRt[e] = nullptr;
    }
    RtSlot& r = g_slots[e];
    if (++r.uses >= kRtFrames && !g_oldRt[e]) {
        IDirect3DSurface9* fresh = nullptr;
        if (SUCCEEDED(d->CreateRenderTarget(dd.Width, dd.Height, D3DFMT_A8R8G8B8, D3DMULTISAMPLE_NONE, 0, FALSE, &fresh, nullptr))) {
            g_oldRt[e] = r.rt;
            g_oldRtFence[e] = g_fv;  // our last copy from it
            r.rt = fresh;
            r.uses = 0;
        }
    }
    HRESULT hr = d->StretchRect(bb, nullptr, r.rt, nullptr, D3DTEXF_NONE);
    bb->Release();
    r.captured = SUCCEEDED(hr);
    r.eye = eye;
    r.pair = pair;
    r.pose = pose;
    if (!r.captured) {
        static int fails;
        if (fails++ < 3) log_printf("GPU copy: StretchRect failed 0x%08lx\n", hr);
    }
    return r.captured;
}

// One eye's render target out of 9On12, copied into an eye image on our queue, handed back.
static bool copy_out(RtSlot& r) {
    r.captured = false;
    double t0 = qpc_ms();
    ID3D12Resource* src = nullptr;
    if (FAILED(g_on12->UnwrapUnderlyingResource(r.rt, g_q, IID_PPV_ARGS(&src)))) {
        static int fails;
        if (fails++ < 3) log_printf("GPU copy: unwrap failed\n");
        return false;
    }
    double t1 = qpc_ms();
    g_ph[P_UNWRAP] += t1 - t0;
    D3D12_RESOURCE_DESC sd = src->GetDesc();
    ID3D12Resource* dst = nullptr;
    int slot = vr_gpu_begin(r.eye, (int)sd.Width, (int)sd.Height, (int)sd.Format, (void**)&dst);

    Cmd& c = g_cmd[g_cmdN++ & 3];
    wait_fence(c.fv);  // two pairs old: done long ago
    c.alloc->Reset();
    c.list->Reset(c.alloc, nullptr);
    if (dst) {
        D3D12_RESOURCE_BARRIER b[2] = { transition(src, D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_COPY_SOURCE),
                                        transition(dst, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_COPY_DEST) };
        c.list->ResourceBarrier(2, b);
        c.list->CopyResource(dst, src);
        for (auto& x : b) std::swap(x.Transition.StateBefore, x.Transition.StateAfter);
        c.list->ResourceBarrier(2, b);
    }
    c.list->Close();
    ID3D12CommandList* lists[] = { c.list };
    g_q->ExecuteCommandLists(1, lists);
    g_q->Signal(g_fence, ++g_fv);
    c.fv = g_fv;
    double t2 = qpc_ms();
    g_ph[P_RECORD] += t2 - t1;
    UINT64 fv = g_fv;
    ID3D12Fence* f = g_fence;
    g_on12->ReturnUnderlyingResource(r.rt, 1, &fv, &f);  // 9On12 waits for our copy before writing it again
    src->Release();
    if (dst) vr_gpu_end(r.eye, slot, r.pair, r.pose, g_fv);
    g_ph[P_RETURN] += qpc_ms() - t2;
    return dst != nullptr;
}

int gpu_finish() {
    // Both eyes of a pair at once, after its right eye. Taking a frame out of 9On12 makes the game thread
    // wait until 9On12's worker has caught up (~2 ms, the copy's biggest cost); once per pair, that wait
    // is paid half as often. The headset shows a pair only once both eyes are there, so the left eye
    // waiting a frame in its render target delays nothing.
    if (g_pairCopy && !g_slots[1].captured) return -1;
    g_phN += g_pairCopy ? 2 : 1;  // phase times are reported per frame
    int last = -1;
    for (int e = 0; e < 2; e++)
        if (g_slots[e].captured && copy_out(g_slots[e])) last = e;
    // One pair in flight at most: nothing else holds the game back now that it skips the desktop
    // Present, and pairs queued deeper only add latency.
    double t3 = qpc_ms();
    if (g_fv > 2) wait_fence(g_fv - 2);
    g_ph[P_THROTTLE] += qpc_ms() - t3;
    return last;
}

void gpu_read_eye(int eye, void (*write)(const void* bits, int pitch, int w, int h)) {
    ID3D12Resource* tex = (ID3D12Resource*)vr_gpu_newest(eye);
    if (!tex) return;
    D3D12_RESOURCE_DESC td = tex->GetDesc();
    D3D12_PLACED_SUBRESOURCE_FOOTPRINT fp;
    UINT64 total = 0;
    g_dev->GetCopyableFootprints(&td, 0, 1, 0, &fp, nullptr, nullptr, &total);
    D3D12_HEAP_PROPERTIES hp{};
    hp.Type = D3D12_HEAP_TYPE_READBACK;
    D3D12_RESOURCE_DESC bd{};
    bd.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    bd.Width = total;
    bd.Height = bd.DepthOrArraySize = bd.MipLevels = 1;
    bd.SampleDesc.Count = 1;
    bd.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    ID3D12Resource* buf = nullptr;
    if (FAILED(g_dev->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &bd, D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&buf)))) return;
    Cmd& c = g_cmd[g_cmdN++ & 3];
    wait_fence(c.fv);
    c.alloc->Reset();
    c.list->Reset(c.alloc, nullptr);
    D3D12_RESOURCE_BARRIER b = transition(tex, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_COPY_SOURCE);
    c.list->ResourceBarrier(1, &b);
    D3D12_TEXTURE_COPY_LOCATION from{}, to{};
    from.pResource = tex;
    from.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
    to.pResource = buf;
    to.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
    to.PlacedFootprint = fp;
    c.list->CopyTextureRegion(&to, 0, 0, 0, &from, nullptr);
    std::swap(b.Transition.StateBefore, b.Transition.StateAfter);
    c.list->ResourceBarrier(1, &b);
    c.list->Close();
    ID3D12CommandList* lists[] = { c.list };
    g_q->ExecuteCommandLists(1, lists);
    g_q->Signal(g_fence, ++g_fv);
    c.fv = g_fv;
    wait_fence(g_fv);
    void* p = nullptr;
    if (SUCCEEDED(buf->Map(0, nullptr, &p))) {
        write((const char*)p + fp.Offset, (int)fp.Footprint.RowPitch, (int)td.Width, (int)td.Height);
        buf->Unmap(0, nullptr);
    }
    buf->Release();
}
