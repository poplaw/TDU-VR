// DXVK path (DXVK=1 in tdu_vr.ini): the game's D3D9 runs on DXVK. Each frame the back buffer is copied
// (StretchRect) into a render target of ours made with transfer-source usage, which is then switched to
// TRANSFER_SRC_OPTIMAL, the layout SteamVR reads, on DXVK's command stream. When the pair's right eye is
// in, DXVK's commands are flushed to its queue and the pair goes to the presenter, which submits those
// Vulkan images to SteamVR on the same queue under DXVK's queue lock. One queue orders it all: the
// compositor's copy comes after our transition, and our next writes come after the compositor's copy.
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <d3d9.h>
#include <cstdint>
#include "vk.h"

// Vulkan types used by DXVK's interop interfaces (no Vulkan SDK needed). Non-dispatchable handles are
// 64-bit integers in 32-bit builds, dispatchable ones are pointers.
typedef struct VkInstance_T* VkInstance;
typedef struct VkPhysicalDevice_T* VkPhysicalDevice;
typedef struct VkDevice_T* VkDevice;
typedef struct VkQueue_T* VkQueue;
typedef uint64_t VkImage;
typedef int VkImageLayout;
typedef uint32_t VkImageUsageFlags;
struct VkImageSubresourceRange { uint32_t aspectMask, baseMipLevel, levelCount, baseArrayLayer, layerCount; };
struct VkImageCreateInfo;
static const VkImageLayout kTransferSrc = 6;  // VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL
static const uint32_t kFormatB8G8R8A8 = 44;   // VK_FORMAT_B8G8R8A8_UNORM, what DXVK makes of A8R8G8B8

// DXVK's D3D9 interop interfaces (src/d3d9/d3d9_interfaces.h, v3.1.1): same IIDs and vtable order.
MIDL_INTERFACE("d56344f5-8d35-46fd-806d-94c351b472c1")
ID3D9VkInteropTexture : public IUnknown {
    virtual HRESULT STDMETHODCALLTYPE GetVulkanImageInfo(VkImage* pHandle, VkImageLayout* pLayout, VkImageCreateInfo* pInfo) = 0;
};
struct D3D9VkExtImageDesc {
    D3DRESOURCETYPE Type;
    UINT Width, Height, Depth, MipLevels;
    DWORD Usage;
    D3DFORMAT Format;
    D3DPOOL Pool;
    D3DMULTISAMPLE_TYPE MultiSample;
    DWORD MultiSampleQuality;
    bool Discard, IsAttachmentOnly, IsLockable;
    VkImageUsageFlags ImageUsage;
};
static_assert(sizeof(D3D9VkExtImageDesc) == 48, "layout of DXVK's struct (MinGW, 32-bit)");
MIDL_INTERFACE("2eaa4b89-0107-4bdb-87f7-0f541c493ce0")
ID3D9VkInteropDevice : public IUnknown {
    virtual void STDMETHODCALLTYPE GetVulkanHandles(VkInstance* pInstance, VkPhysicalDevice* pPhysDev, VkDevice* pDevice) = 0;
    virtual void STDMETHODCALLTYPE GetSubmissionQueue(VkQueue* pQueue, uint32_t* pQueueIndex, uint32_t* pQueueFamilyIndex) = 0;
    virtual void STDMETHODCALLTYPE TransitionTextureLayout(ID3D9VkInteropTexture* pTexture, const VkImageSubresourceRange* pSubresources,
                                                           VkImageLayout OldLayout, VkImageLayout NewLayout) = 0;
    virtual void STDMETHODCALLTYPE FlushRenderingCommands() = 0;
    virtual void STDMETHODCALLTYPE LockSubmissionQueue() = 0;
    virtual void STDMETHODCALLTYPE ReleaseSubmissionQueue() = 0;
    virtual void STDMETHODCALLTYPE LockDevice() = 0;
    virtual void STDMETHODCALLTYPE UnlockDevice() = 0;
    virtual bool STDMETHODCALLTYPE WaitForResource(IDirect3DResource9* pResource, DWORD MapFlags) = 0;
    virtual HRESULT STDMETHODCALLTYPE CreateImage(const D3D9VkExtImageDesc* desc, IDirect3DResource9** ppResult) = 0;
};

static IDirect3DDevice9* g_dev;      // device the interface below belongs to
static ID3D9VkInteropDevice* g_ix;  // null: not DXVK (or not tried yet for g_dev)
static VkQueue g_queue;

struct Slot {
    IDirect3DTexture9* tex;
    IDirect3DSurface9* surf;
    ID3D9VkInteropTexture* itex;
    VkImage image;
    VkImageLayout native;  // DXVK's own layout for it
    bool readable;         // currently in TRANSFER_SRC_OPTIMAL
    UINT w, h;
};
static Slot g_slots[2][4];  // by eye, by presenter slot
struct Pending { int slot; unsigned pair; FramePose pose; bool ok; };
static Pending g_pend[2];

static void lock_queue(void*) { if (g_ix) g_ix->LockSubmissionQueue(); }
static void unlock_queue(void*) { if (g_ix) g_ix->ReleaseSubmissionQueue(); }

static void release_slot(Slot& s) {
    if (s.itex) s.itex->Release();
    if (s.surf) s.surf->Release();
    if (s.tex) s.tex->Release();
    s = Slot{};
}

void vk_release() {
    if (!g_ix) { g_dev = nullptr; return; }
    vr_vk_reset();  // the presenter stops submitting our images
    // The compositor's copies were queued on DXVK's queue outside DXVK's tracking: let them finish
    // before the images go away.
    typedef int(__stdcall * WaitIdle)(VkQueue);
    static WaitIdle waitIdle = (WaitIdle)GetProcAddress(GetModuleHandleA("vulkan-1.dll"), "vkQueueWaitIdle");
    if (waitIdle && g_queue) {
        g_ix->LockSubmissionQueue();
        waitIdle(g_queue);
        g_ix->ReleaseSubmissionQueue();
    }
    for (auto& eye : g_slots)
        for (auto& s : eye) release_slot(s);
    g_pend[0].ok = g_pend[1].ok = false;
    g_ix->Release();
    g_ix = nullptr;
    g_dev = nullptr;
}

static bool init(IDirect3DDevice9* d) {
    if (g_dev == d) return g_ix != nullptr;
    vk_release();
    g_dev = d;
    if (FAILED(d->QueryInterface(__uuidof(ID3D9VkInteropDevice), (void**)&g_ix))) {
        g_ix = nullptr;
        log_printf("DXVK copy: the device has no Vulkan interop interface\n");
        return false;
    }
    VkInstance inst = nullptr;
    VkPhysicalDevice phys = nullptr;
    VkDevice dev = nullptr;
    uint32_t index = 0, family = 0;
    g_ix->GetVulkanHandles(&inst, &phys, &dev);
    g_ix->GetSubmissionQueue(&g_queue, &index, &family);
    vr_vk_init(inst, phys, dev, g_queue, family, lock_queue, unlock_queue, nullptr);
    log_printf("DXVK copy: Vulkan device %p, queue %p (family %u)\n", (void*)dev, (void*)g_queue, family);
    return true;
}

static bool make_slot(Slot& s, UINT w, UINT h) {
    release_slot(s);
    D3D9VkExtImageDesc desc{};
    desc.Type = D3DRTYPE_TEXTURE;
    desc.Width = w;
    desc.Height = h;
    desc.Depth = 1;
    desc.MipLevels = 1;
    desc.Usage = D3DUSAGE_RENDERTARGET;
    desc.Format = D3DFMT_A8R8G8B8;  // StretchRect sets alpha; SteamVR takes B8G8R8A8
    desc.Pool = D3DPOOL_DEFAULT;
    desc.MultiSample = D3DMULTISAMPLE_NONE;
    desc.ImageUsage = 0x1 | 0x4;  // TRANSFER_SRC (the compositor copies from it) | SAMPLED
    IDirect3DResource9* res = nullptr;
    if (FAILED(g_ix->CreateImage(&desc, &res)) || !res) return false;
    bool ok = SUCCEEDED(res->QueryInterface(__uuidof(IDirect3DTexture9), (void**)&s.tex)) &&
              SUCCEEDED(s.tex->GetSurfaceLevel(0, &s.surf)) &&
              SUCCEEDED(s.surf->QueryInterface(__uuidof(ID3D9VkInteropTexture), (void**)&s.itex)) &&
              SUCCEEDED(s.itex->GetVulkanImageInfo(&s.image, &s.native, nullptr));
    res->Release();
    if (!ok) { release_slot(s); return false; }
    s.w = w;
    s.h = h;
    return true;
}

bool vk_capture(void* device9, int eye, unsigned pair, const FramePose& pose) {
    IDirect3DDevice9* d = (IDirect3DDevice9*)device9;
    if (!init(d)) return false;
    IDirect3DSurface9* bb = nullptr;
    if (FAILED(d->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &bb))) return true;
    D3DSURFACE_DESC bd;
    bb->GetDesc(&bd);
    const int e = eye & 1;
    int si = vr_vk_begin(e);
    Slot& s = g_slots[e][si];
    if ((!s.tex || s.w != bd.Width || s.h != bd.Height) && !make_slot(s, bd.Width, bd.Height)) {
        static int fails;
        if (fails++ < 3) log_printf("DXVK copy: could not make a %ux%u eye image\n", bd.Width, bd.Height);
        bb->Release();
        return true;
    }
    const VkImageSubresourceRange all{ 1 /* color */, 0, 1, 0, 1 };
    if (s.readable) g_ix->TransitionTextureLayout(s.itex, &all, kTransferSrc, s.native);  // back to DXVK's layout first
    HRESULT hr = d->StretchRect(bb, nullptr, s.surf, nullptr, D3DTEXF_NONE);
    bb->Release();
    g_ix->TransitionTextureLayout(s.itex, &all, s.native, kTransferSrc);
    s.readable = true;
    g_pend[e] = Pending{ si, pair, pose, SUCCEEDED(hr) };
    if (e == 1) {
        // The pair is complete: its commands go to the queue, then the presenter may submit it.
        g_ix->FlushRenderingCommands();
        for (int k = 0; k < 2; k++) {
            if (!g_pend[k].ok) continue;
            const Slot& p = g_slots[k][g_pend[k].slot];
            vr_vk_end(k, g_pend[k].slot, p.image, (int)p.w, (int)p.h, kFormatB8G8R8A8, g_pend[k].pair, g_pend[k].pose);
            g_pend[k].ok = false;
        }
    }
    return true;
}
