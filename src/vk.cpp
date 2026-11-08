// DXVK path (DXVK=1 in tdu_vr.ini): the game's D3D9 runs on DXVK (Vulkan), and the eye images reach
// SteamVR through Direct3D 12, the same way as on D3D9On12. SteamVR never touches DXVK's queue: when it
// did (Vulkan textures submitted on DXVK's queue), WaitGetPoses and PostPresentHandoff used that queue from
// the presenter thread, which lost the Vulkan device, and serializing them stalled DXVK ~90 ms per frame.
//
// Per frame (game thread, before Present): the back buffer is copied (StretchRect) into the eye's texture
// of ours, which DXVK then switches to TRANSFER_SRC_OPTIMAL; the game's Present puts those commands on
// DXVK's queue. One frame later the pair is "bridged": one vkQueueSubmit on DXVK's queue (under DXVK's
// queue lock, nothing else inside) copies both eyes into two D3D12 textures imported into DXVK's Vulkan
// device, waiting for and signalling shared fences on the GPU. Our D3D12 queue waits for that fence, copies
// into the presenter's eye images, and the presenter hands them to SteamVR as on the D3D9On12 path.
// A frame later because DXVK's flush only queues its work for a submission thread: by the next frame
// it has reached the queue, so our copy comes after it.
// Our textures are DXVK shared textures: DXVK moves ordinary images in memory (new Vulkan handle); shared
// ones are never moved.
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <d3d9.h>
#include <d3d12.h>
#include <cstdint>
#include <utility>
#include "vk.h"

// ---- the Vulkan subset used here (no Vulkan SDK needed; values and layouts from the specification) ----
// Dispatchable handles are pointers, non-dispatchable ones 64-bit integers in 32-bit builds.
typedef struct VkInstance_T* VkInstance;
typedef struct VkPhysicalDevice_T* VkPhysicalDevice;
typedef struct VkDevice_T* VkDevice;
typedef struct VkQueue_T* VkQueue;
typedef struct VkCommandBuffer_T* VkCommandBuffer;
typedef uint64_t VkImage, VkDeviceMemory, VkSemaphore, VkFence, VkCommandPool, VkBuffer, VkDeviceSize;
typedef int VkResult, VkImageLayout;
typedef uint32_t VkFlags;
typedef void(__stdcall* PFN_vkVoidFunction)();
#define VKAPI __stdcall

enum : int {
    ST_SUBMIT_INFO = 4, ST_MEMORY_ALLOCATE_INFO = 5, ST_FENCE_CREATE_INFO = 8, ST_SEMAPHORE_CREATE_INFO = 9,
    ST_IMAGE_CREATE_INFO = 14, ST_COMMAND_POOL_CREATE_INFO = 39, ST_COMMAND_BUFFER_ALLOCATE_INFO = 40,
    ST_COMMAND_BUFFER_BEGIN_INFO = 42, ST_IMAGE_MEMORY_BARRIER = 45,
    ST_EXTERNAL_MEMORY_IMAGE_CREATE_INFO = 1000072001, ST_IMPORT_MEMORY_WIN32_HANDLE_INFO = 1000073000,
    ST_MEMORY_WIN32_HANDLE_PROPERTIES = 1000073002, ST_IMPORT_SEMAPHORE_WIN32_HANDLE_INFO = 1000078000,
    ST_MEMORY_DEDICATED_ALLOCATE_INFO = 1000127001, ST_SEMAPHORE_TYPE_CREATE_INFO = 1000207002,
    ST_TIMELINE_SEMAPHORE_SUBMIT_INFO = 1000207003,
};
static const VkImageLayout L_UNDEFINED = 0, L_GENERAL = 1, L_TRANSFER_SRC = 6, L_TRANSFER_DST = 7;
static const uint32_t kMemD3D12Resource = 0x40, kSemD3D12Fence = 0x8;
static const uint32_t kQueueIgnored = ~0u, kQueueExternal = ~1u;
static const VkFlags A_TRANSFER_READ = 0x800, A_TRANSFER_WRITE = 0x1000, A_MEMORY_WRITE = 0x10000;
static const VkFlags S_TRANSFER = 0x1000, S_BOTTOM = 0x2000, S_ALL = 0x10000;

struct VkExtent3D { uint32_t width, height, depth; };
struct VkOffset3D { int32_t x, y, z; };
struct VkImageSubresourceLayers { VkFlags aspectMask; uint32_t mipLevel, baseArrayLayer, layerCount; };
struct VkImageSubresourceRange { VkFlags aspectMask; uint32_t baseMipLevel, levelCount, baseArrayLayer, layerCount; };
struct VkImageCopy { VkImageSubresourceLayers srcSubresource; VkOffset3D srcOffset; VkImageSubresourceLayers dstSubresource; VkOffset3D dstOffset; VkExtent3D extent; };
struct VkImageCreateInfo {
    int sType; const void* pNext; VkFlags flags; int imageType; int format; VkExtent3D extent; uint32_t mipLevels, arrayLayers;
    int samples; int tiling; VkFlags usage; int sharingMode; uint32_t queueFamilyIndexCount; const uint32_t* pQueueFamilyIndices; VkImageLayout initialLayout;
};
struct VkExternalMemoryImageCreateInfo { int sType; const void* pNext; VkFlags handleTypes; };
struct VkMemoryRequirements { VkDeviceSize size, alignment; uint32_t memoryTypeBits; };
struct VkMemoryAllocateInfo { int sType; const void* pNext; VkDeviceSize allocationSize; uint32_t memoryTypeIndex; };
struct VkImportMemoryWin32HandleInfoKHR { int sType; const void* pNext; VkFlags handleType; HANDLE handle; LPCWSTR name; };
struct VkMemoryDedicatedAllocateInfo { int sType; const void* pNext; VkImage image; VkBuffer buffer; };
struct VkMemoryWin32HandlePropertiesKHR { int sType; void* pNext; uint32_t memoryTypeBits; };
struct VkSemaphoreTypeCreateInfo { int sType; const void* pNext; int semaphoreType; uint64_t initialValue; };
struct VkSemaphoreCreateInfo { int sType; const void* pNext; VkFlags flags; };
struct VkImportSemaphoreWin32HandleInfoKHR { int sType; const void* pNext; VkSemaphore semaphore; VkFlags flags; VkFlags handleType; HANDLE handle; LPCWSTR name; };
struct VkFenceCreateInfo { int sType; const void* pNext; VkFlags flags; };
struct VkCommandPoolCreateInfo { int sType; const void* pNext; VkFlags flags; uint32_t queueFamilyIndex; };
struct VkCommandBufferAllocateInfo { int sType; const void* pNext; VkCommandPool commandPool; int level; uint32_t commandBufferCount; };
struct VkCommandBufferBeginInfo { int sType; const void* pNext; VkFlags flags; const void* pInheritanceInfo; };
struct VkImageMemoryBarrier {
    int sType; const void* pNext; VkFlags srcAccessMask, dstAccessMask; VkImageLayout oldLayout, newLayout;
    uint32_t srcQueueFamilyIndex, dstQueueFamilyIndex; VkImage image; VkImageSubresourceRange subresourceRange;
};
struct VkSubmitInfo {
    int sType; const void* pNext; uint32_t waitSemaphoreCount; const VkSemaphore* pWaitSemaphores; const VkFlags* pWaitDstStageMask;
    uint32_t commandBufferCount; const VkCommandBuffer* pCommandBuffers; uint32_t signalSemaphoreCount; const VkSemaphore* pSignalSemaphores;
};
struct VkTimelineSemaphoreSubmitInfo {
    int sType; const void* pNext; uint32_t waitSemaphoreValueCount; const uint64_t* pWaitSemaphoreValues;
    uint32_t signalSemaphoreValueCount; const uint64_t* pSignalSemaphoreValues;
};
static_assert(sizeof(VkImageMemoryBarrier) == 64 && sizeof(VkMemoryAllocateInfo) == 24 && sizeof(VkImportSemaphoreWin32HandleInfoKHR) == 32 &&
              sizeof(VkCommandBufferAllocateInfo) == 24 && sizeof(VkSemaphoreTypeCreateInfo) == 24, "32-bit Vulkan struct layouts");

static struct {
    PFN_vkVoidFunction(VKAPI* GetDeviceProcAddr)(VkDevice, const char*);
    VkResult(VKAPI* CreateImage)(VkDevice, const VkImageCreateInfo*, const void*, VkImage*);
    void(VKAPI* DestroyImage)(VkDevice, VkImage, const void*);
    void(VKAPI* GetImageMemoryRequirements)(VkDevice, VkImage, VkMemoryRequirements*);
    VkResult(VKAPI* GetMemoryWin32HandlePropertiesKHR)(VkDevice, VkFlags, HANDLE, VkMemoryWin32HandlePropertiesKHR*);
    VkResult(VKAPI* AllocateMemory)(VkDevice, const VkMemoryAllocateInfo*, const void*, VkDeviceMemory*);
    void(VKAPI* FreeMemory)(VkDevice, VkDeviceMemory, const void*);
    VkResult(VKAPI* BindImageMemory)(VkDevice, VkImage, VkDeviceMemory, VkDeviceSize);
    VkResult(VKAPI* CreateSemaphore)(VkDevice, const VkSemaphoreCreateInfo*, const void*, VkSemaphore*);
    void(VKAPI* DestroySemaphore)(VkDevice, VkSemaphore, const void*);
    VkResult(VKAPI* ImportSemaphoreWin32HandleKHR)(VkDevice, const VkImportSemaphoreWin32HandleInfoKHR*);
    VkResult(VKAPI* CreateFence)(VkDevice, const VkFenceCreateInfo*, const void*, VkFence*);
    void(VKAPI* DestroyFence)(VkDevice, VkFence, const void*);
    VkResult(VKAPI* WaitForFences)(VkDevice, uint32_t, const VkFence*, uint32_t, uint64_t);
    VkResult(VKAPI* ResetFences)(VkDevice, uint32_t, const VkFence*);
    VkResult(VKAPI* CreateCommandPool)(VkDevice, const VkCommandPoolCreateInfo*, const void*, VkCommandPool*);
    void(VKAPI* DestroyCommandPool)(VkDevice, VkCommandPool, const void*);
    VkResult(VKAPI* AllocateCommandBuffers)(VkDevice, const VkCommandBufferAllocateInfo*, VkCommandBuffer*);
    VkResult(VKAPI* ResetCommandBuffer)(VkCommandBuffer, VkFlags);
    VkResult(VKAPI* BeginCommandBuffer)(VkCommandBuffer, const VkCommandBufferBeginInfo*);
    VkResult(VKAPI* EndCommandBuffer)(VkCommandBuffer);
    void(VKAPI* CmdPipelineBarrier)(VkCommandBuffer, VkFlags, VkFlags, VkFlags, uint32_t, const void*, uint32_t, const void*, uint32_t, const VkImageMemoryBarrier*);
    void(VKAPI* CmdCopyImage)(VkCommandBuffer, VkImage, VkImageLayout, VkImage, VkImageLayout, uint32_t, const VkImageCopy*);
    VkResult(VKAPI* QueueSubmit)(VkQueue, uint32_t, const VkSubmitInfo*, VkFence);
    VkResult(VKAPI* QueueWaitIdle)(VkQueue);
} vk;

// ---- DXVK's D3D9 interop interfaces (src/d3d9/d3d9_interfaces.h, v3.1.1): same IIDs and vtable order ----
MIDL_INTERFACE("d56344f5-8d35-46fd-806d-94c351b472c1")
ID3D9VkInteropTexture : public IUnknown {
    virtual HRESULT STDMETHODCALLTYPE GetVulkanImageInfo(VkImage* pHandle, VkImageLayout* pLayout, VkImageCreateInfo* pInfo) = 0;
};
MIDL_INTERFACE("2eaa4b89-0107-4bdb-87f7-0f541c493ce0")
ID3D9VkInteropDevice : public IUnknown {
    virtual void STDMETHODCALLTYPE GetVulkanHandles(VkInstance* pInstance, VkPhysicalDevice* pPhysDev, VkDevice* pDevice) = 0;
    virtual void STDMETHODCALLTYPE GetSubmissionQueue(VkQueue* pQueue, uint32_t* pQueueIndex, uint32_t* pQueueFamilyIndex) = 0;
    virtual void STDMETHODCALLTYPE TransitionTextureLayout(ID3D9VkInteropTexture* pTexture, const VkImageSubresourceRange* pSubresources,
                                                           VkImageLayout OldLayout, VkImageLayout NewLayout) = 0;
    virtual void STDMETHODCALLTYPE FlushRenderingCommands() = 0;
    virtual void STDMETHODCALLTYPE LockSubmissionQueue() = 0;
    virtual void STDMETHODCALLTYPE ReleaseSubmissionQueue() = 0;
    // (LockDevice, UnlockDevice, WaitForResource, CreateImage follow; not used)
};

// ---- state ----
static IDirect3DDevice9* g_dev9;     // device the interop interface below belongs to
static ID3D9VkInteropDevice* g_ix;  // null: not DXVK (or not tried for g_dev9)
static bool g_broken;               // set up failed: stay off for this device
static VkDevice g_vkDev;
static VkQueue g_queue;
static uint32_t g_family;
static VkCommandPool g_pool;
static VkCommandBuffer g_cmd[2];
static VkFence g_cmdFence[2];
static unsigned g_cmdN;

// D3D12 side: our device and queue (the presenter submits the eye images on it), and two shared fences:
// g_fV signalled by our Vulkan copy, g_fD by our D3D12 copy out of the bridge textures.
static ID3D12Device* g_d12;
static ID3D12CommandQueue* g_q;
static ID3D12Fence *g_fV, *g_fD;
static VkSemaphore g_semV, g_semD;
static UINT64 g_vV, g_vD;
struct Cmd12 { ID3D12CommandAllocator* alloc; ID3D12GraphicsCommandList* list; UINT64 fv; };
static Cmd12 g_c12[4];
static unsigned g_c12N;
static HANDLE g_ev;

// The bridge: per eye one D3D12 texture shared with DXVK's Vulkan device.
struct Bridge { ID3D12Resource* tex; VkImage image; VkDeviceMemory mem; bool used; };
static Bridge g_br[2];
static UINT g_brW, g_brH;

// Our DXVK textures, one per eye, and the pair waiting to be bridged.
struct Slot { IDirect3DTexture9* tex; IDirect3DSurface9* surf; ID3D9VkInteropTexture* itex; VkImage image; VkImageLayout native; bool readable; UINT w, h; };
static Slot g_slots[2];
struct Pending { unsigned pair; FramePose pose; bool ok; };
static Pending g_pend[2];
static bool g_pairReady;  // both eyes of g_pend are captured: bridge at the next frame

static void wait12(ID3D12Fence* f, UINT64 v) {
    if (f->GetCompletedValue() >= v) return;
    f->SetEventOnCompletion(v, g_ev);
    WaitForSingleObject(g_ev, 1000);
}

static void release_bridge() {
    for (auto& b : g_br) {
        if (b.image) vk.DestroyImage(g_vkDev, b.image, nullptr);
        if (b.mem) vk.FreeMemory(g_vkDev, b.mem, nullptr);
        if (b.tex) b.tex->Release();
        b = Bridge{};
    }
    g_brW = g_brH = 0;
}

static void release_slots() {
    for (auto& s : g_slots) {
        if (s.itex) s.itex->Release();
        if (s.surf) s.surf->Release();
        if (s.tex) s.tex->Release();
        s = Slot{};
    }
    g_pend[0].ok = g_pend[1].ok = g_pairReady = false;
}

void vk_release() {
    if (g_ix) {
        // Our copies on DXVK's queue and on our D3D12 queue must be done before their images go away.
        g_ix->LockSubmissionQueue();
        if (vk.QueueWaitIdle) vk.QueueWaitIdle(g_queue);
        g_ix->ReleaseSubmissionQueue();
        if (g_fD) wait12(g_fD, g_vD);
        release_bridge();
        for (int i = 0; i < 2; i++) {
            if (g_cmdFence[i]) vk.DestroyFence(g_vkDev, g_cmdFence[i], nullptr);
            g_cmdFence[i] = 0;
        }
        if (g_pool) vk.DestroyCommandPool(g_vkDev, g_pool, nullptr);
        g_pool = 0;
        if (g_semV) vk.DestroySemaphore(g_vkDev, g_semV, nullptr);
        if (g_semD) vk.DestroySemaphore(g_vkDev, g_semD, nullptr);
        g_semV = g_semD = 0;
        release_slots();
        g_ix->Release();
        g_ix = nullptr;
    }
    g_dev9 = nullptr;
    g_broken = false;
}

// D3D12 device, queue, shared fences, command lists: once per process. The presenter uses the device
// and queue for every DXVK device of the game.
static bool init_d3d12() {
    if (g_d12) return true;
    HMODULE m = LoadLibraryA("d3d12.dll");
    auto create = m ? (PFN_D3D12_CREATE_DEVICE)GetProcAddress(m, "D3D12CreateDevice") : nullptr;
    if (!create || FAILED(create(nullptr, D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&g_d12)))) { g_d12 = nullptr; return false; }
    D3D12_COMMAND_QUEUE_DESC qd{};
    qd.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
    if (FAILED(g_d12->CreateCommandQueue(&qd, IID_PPV_ARGS(&g_q))) || FAILED(g_d12->CreateFence(0, D3D12_FENCE_FLAG_SHARED, IID_PPV_ARGS(&g_fV))) ||
        FAILED(g_d12->CreateFence(0, D3D12_FENCE_FLAG_SHARED, IID_PPV_ARGS(&g_fD))))
        return false;
    for (auto& c : g_c12) {
        if (FAILED(g_d12->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&c.alloc))) ||
            FAILED(g_d12->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, c.alloc, nullptr, IID_PPV_ARGS(&c.list))))
            return false;
        c.list->Close();
    }
    g_ev = CreateEventA(nullptr, FALSE, FALSE, nullptr);
    vr_gpu_init(g_d12, g_q, g_fD);  // the presenter shows an eye image once g_fD passes its copy
    return true;
}

static bool load_vulkan(VkInstance inst) {
    HMODULE m = GetModuleHandleA("vulkan-1.dll");  // loaded by DXVK
    auto gipa = m ? (PFN_vkVoidFunction(VKAPI*)(VkInstance, const char*))GetProcAddress(m, "vkGetInstanceProcAddr") : nullptr;
    if (!gipa) return false;
    vk.GetDeviceProcAddr = (decltype(vk.GetDeviceProcAddr))gipa(inst, "vkGetDeviceProcAddr");
    if (!vk.GetDeviceProcAddr) return false;
    bool ok = true;
#define VKLOAD(name) ok &= (*(PFN_vkVoidFunction*)&vk.name = vk.GetDeviceProcAddr(g_vkDev, "vk" #name)) != nullptr
    VKLOAD(CreateImage); VKLOAD(DestroyImage); VKLOAD(GetImageMemoryRequirements); VKLOAD(GetMemoryWin32HandlePropertiesKHR);
    VKLOAD(AllocateMemory); VKLOAD(FreeMemory); VKLOAD(BindImageMemory); VKLOAD(CreateSemaphore); VKLOAD(DestroySemaphore);
    VKLOAD(ImportSemaphoreWin32HandleKHR); VKLOAD(CreateFence); VKLOAD(DestroyFence); VKLOAD(WaitForFences); VKLOAD(ResetFences);
    VKLOAD(CreateCommandPool); VKLOAD(DestroyCommandPool); VKLOAD(AllocateCommandBuffers); VKLOAD(ResetCommandBuffer);
    VKLOAD(BeginCommandBuffer); VKLOAD(EndCommandBuffer); VKLOAD(CmdPipelineBarrier); VKLOAD(CmdCopyImage); VKLOAD(QueueSubmit);
    VKLOAD(QueueWaitIdle);
#undef VKLOAD
    return ok;
}

static VkSemaphore import_fence(ID3D12Fence* f) {
    HANDLE h = nullptr;
    if (FAILED(g_d12->CreateSharedHandle(f, nullptr, GENERIC_ALL, nullptr, &h))) return 0;
    VkSemaphoreTypeCreateInfo type{ ST_SEMAPHORE_TYPE_CREATE_INFO, nullptr, 1 /* timeline */, 0 };
    VkSemaphoreCreateInfo sci{ ST_SEMAPHORE_CREATE_INFO, &type, 0 };
    VkSemaphore sem = 0;
    bool ok = vk.CreateSemaphore(g_vkDev, &sci, nullptr, &sem) == 0;
    VkImportSemaphoreWin32HandleInfoKHR imp{ ST_IMPORT_SEMAPHORE_WIN32_HANDLE_INFO, nullptr, sem, 0, kSemD3D12Fence, h, nullptr };
    ok = ok && vk.ImportSemaphoreWin32HandleKHR(g_vkDev, &imp) == 0;
    CloseHandle(h);
    if (!ok && sem) { vk.DestroySemaphore(g_vkDev, sem, nullptr); sem = 0; }
    return sem;
}

static bool init(IDirect3DDevice9* d) {
    if (g_dev9 == d) return g_ix && !g_broken;
    vk_release();
    g_dev9 = d;
    if (FAILED(d->QueryInterface(__uuidof(ID3D9VkInteropDevice), (void**)&g_ix))) {
        g_ix = nullptr;
        log_printf("DXVK copy: the device has no Vulkan interop interface\n");
        return false;
    }
    VkInstance inst = nullptr;
    VkPhysicalDevice phys = nullptr;
    uint32_t index = 0;
    g_ix->GetVulkanHandles(&inst, &phys, &g_vkDev);
    g_ix->GetSubmissionQueue(&g_queue, &index, &g_family);
    const char* fail = nullptr;
    if (!init_d3d12()) fail = "no D3D12 device";
    else if (!load_vulkan(inst)) fail = "Vulkan functions missing";
    else if (!(g_semV = import_fence(g_fV)) || !(g_semD = import_fence(g_fD))) fail = "could not share the D3D12 fences with Vulkan";
    if (!fail) {
        VkCommandPoolCreateInfo pci{ ST_COMMAND_POOL_CREATE_INFO, nullptr, 0x2 /* reset command buffer */, g_family };
        VkCommandBufferAllocateInfo cai{ ST_COMMAND_BUFFER_ALLOCATE_INFO, nullptr, 0, 0 /* primary */, 2 };
        VkFenceCreateInfo fci{ ST_FENCE_CREATE_INFO, nullptr, 0x1 /* signaled */ };
        bool ok = vk.CreateCommandPool(g_vkDev, &pci, nullptr, &g_pool) == 0;
        cai.commandPool = g_pool;
        ok = ok && vk.AllocateCommandBuffers(g_vkDev, &cai, g_cmd) == 0;
        for (int i = 0; i < 2 && ok; i++) ok = vk.CreateFence(g_vkDev, &fci, nullptr, &g_cmdFence[i]) == 0;
        if (!ok) fail = "no Vulkan command buffers";
    }
    if (fail) {
        log_printf("DXVK copy: %s; no images for the headset\n", fail);
        g_broken = true;
        return false;
    }
    log_printf("DXVK copy: Vulkan device %p, queue %p (family %u), bridged to D3D12\n", (void*)g_vkDev, (void*)g_queue, g_family);
    return true;
}

static bool make_bridge(UINT w, UINT h) {
    release_bridge();
    for (auto& b : g_br) {
        D3D12_HEAP_PROPERTIES hp{};
        hp.Type = D3D12_HEAP_TYPE_DEFAULT;
        D3D12_RESOURCE_DESC rd{};
        rd.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
        rd.Width = w;
        rd.Height = h;
        rd.DepthOrArraySize = rd.MipLevels = 1;
        rd.Format = DXGI_FORMAT_B8G8R8A8_UNORM;
        rd.SampleDesc.Count = 1;
        HANDLE sh = nullptr;
        if (FAILED(g_d12->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_SHARED, &rd, D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS(&b.tex))) ||
            FAILED(g_d12->CreateSharedHandle(b.tex, nullptr, GENERIC_ALL, nullptr, &sh)))
            return false;
        VkExternalMemoryImageCreateInfo ext{ ST_EXTERNAL_MEMORY_IMAGE_CREATE_INFO, nullptr, kMemD3D12Resource };
        VkImageCreateInfo ici{ ST_IMAGE_CREATE_INFO, &ext, 0, 1 /* 2D */, 44 /* B8G8R8A8_UNORM */, { w, h, 1 }, 1, 1, 1, 0 /* optimal */,
                               0x1 | 0x2 /* transfer src | dst */, 0 /* exclusive */, 0, nullptr, L_UNDEFINED };
        VkMemoryRequirements req{};
        VkMemoryWin32HandlePropertiesKHR props{ ST_MEMORY_WIN32_HANDLE_PROPERTIES, nullptr, 0 };
        bool ok = vk.CreateImage(g_vkDev, &ici, nullptr, &b.image) == 0;
        if (ok) {
            vk.GetImageMemoryRequirements(g_vkDev, b.image, &req);
            ok = vk.GetMemoryWin32HandlePropertiesKHR(g_vkDev, kMemD3D12Resource, sh, &props) == 0;
        }
        uint32_t bits = req.memoryTypeBits & props.memoryTypeBits, type = 0;
        while (bits && !(bits & 1)) { bits >>= 1; type++; }
        ok = ok && bits;
        VkMemoryDedicatedAllocateInfo ded{ ST_MEMORY_DEDICATED_ALLOCATE_INFO, nullptr, b.image, 0 };
        VkImportMemoryWin32HandleInfoKHR imp{ ST_IMPORT_MEMORY_WIN32_HANDLE_INFO, &ded, kMemD3D12Resource, sh, nullptr };
        VkMemoryAllocateInfo mai{ ST_MEMORY_ALLOCATE_INFO, &imp, req.size, type };
        ok = ok && vk.AllocateMemory(g_vkDev, &mai, nullptr, &b.mem) == 0 && vk.BindImageMemory(g_vkDev, b.image, b.mem, 0) == 0;
        CloseHandle(sh);
        if (!ok) return false;
    }
    g_brW = w;
    g_brH = h;
    return true;
}

static bool make_slot(Slot& s, UINT w, UINT h) {
    if (s.itex) s.itex->Release();
    if (s.surf) s.surf->Release();
    if (s.tex) s.tex->Release();
    s = Slot{};
    // DXVK gives every D3D9 image transfer usage. The share handle makes it shared: never moved in memory.
    // (DXVK accepts one on a plain, non-Ex device; A8R8G8B8: StretchRect sets alpha.)
    HANDLE share = nullptr;
    bool ok = SUCCEEDED(g_dev9->CreateTexture(w, h, 1, D3DUSAGE_RENDERTARGET, D3DFMT_A8R8G8B8, D3DPOOL_DEFAULT, &s.tex, &share)) &&
              SUCCEEDED(s.tex->GetSurfaceLevel(0, &s.surf)) &&
              SUCCEEDED(s.surf->QueryInterface(__uuidof(ID3D9VkInteropTexture), (void**)&s.itex)) &&
              SUCCEEDED(s.itex->GetVulkanImageInfo(&s.image, &s.native, nullptr));
    if (!ok) return false;
    s.w = w;
    s.h = h;
    return true;
}

// One vkQueueSubmit on DXVK's queue: both eyes' DXVK textures into the bridge textures, after our previous
// D3D12 copy out of them (g_semD) and signalling g_semV; then our D3D12 queue copies them on into the
// presenter's eye images.
static void bridge_pair() {
    g_pairReady = false;
    const Slot& s0 = g_slots[0];
    if (!g_pend[0].ok || !g_pend[1].ok || s0.w != g_slots[1].w || s0.h != g_slots[1].h) return;
    if ((g_brW != s0.w || g_brH != s0.h) && !make_bridge(s0.w, s0.h)) {
        static int fails;
        if (fails++ < 3) log_printf("DXVK copy: could not share a %ux%u D3D12 texture with Vulkan\n", s0.w, s0.h);
        release_bridge();
        return;
    }
    // Vulkan: record and submit.
    const unsigned ci = g_cmdN++ & 1;
    VkCommandBuffer cb = g_cmd[ci];
    vk.WaitForFences(g_vkDev, 1, &g_cmdFence[ci], 1, 1000000000ull);  // two pairs old: long done
    vk.ResetFences(g_vkDev, 1, &g_cmdFence[ci]);
    vk.ResetCommandBuffer(cb, 0);
    VkCommandBufferBeginInfo bi{ ST_COMMAND_BUFFER_BEGIN_INFO, nullptr, 0x1 /* one time */, nullptr };
    vk.BeginCommandBuffer(cb, &bi);
    const VkImageSubresourceRange all{ 1 /* color */, 0, 1, 0, 1 };
    VkImageMemoryBarrier pre[4], post[4];
    for (int e = 0; e < 2; e++) {
        // DXVK's writes to its texture (in TRANSFER_SRC_OPTIMAL since our transition) become visible to the copy.
        pre[e] = VkImageMemoryBarrier{ ST_IMAGE_MEMORY_BARRIER, nullptr, A_MEMORY_WRITE, A_TRANSFER_READ, L_TRANSFER_SRC, L_TRANSFER_SRC,
                                       kQueueIgnored, kQueueIgnored, g_slots[e].image, all };
        // The bridge texture comes back from D3D12 (contents not needed).
        pre[2 + e] = VkImageMemoryBarrier{ ST_IMAGE_MEMORY_BARRIER, nullptr, 0, A_TRANSFER_WRITE, L_UNDEFINED, L_TRANSFER_DST,
                                           g_br[e].used ? kQueueExternal : kQueueIgnored, g_br[e].used ? g_family : kQueueIgnored, g_br[e].image, all };
        // Then the bridge texture goes to D3D12, and DXVK's later writes to its texture wait for our read.
        post[e] = VkImageMemoryBarrier{ ST_IMAGE_MEMORY_BARRIER, nullptr, A_TRANSFER_READ, 0, L_TRANSFER_SRC, L_TRANSFER_SRC,
                                        kQueueIgnored, kQueueIgnored, g_slots[e].image, all };
        post[2 + e] = VkImageMemoryBarrier{ ST_IMAGE_MEMORY_BARRIER, nullptr, A_TRANSFER_WRITE, 0, L_TRANSFER_DST, L_GENERAL,
                                            g_family, kQueueExternal, g_br[e].image, all };
        g_br[e].used = true;
    }
    vk.CmdPipelineBarrier(cb, S_ALL, S_TRANSFER, 0, 0, nullptr, 0, nullptr, 4, pre);
    for (int e = 0; e < 2; e++) {
        VkImageCopy region{ { 1, 0, 0, 1 }, { 0, 0, 0 }, { 1, 0, 0, 1 }, { 0, 0, 0 }, { s0.w, s0.h, 1 } };
        vk.CmdCopyImage(cb, g_slots[e].image, L_TRANSFER_SRC, g_br[e].image, L_TRANSFER_DST, 1, &region);
    }
    vk.CmdPipelineBarrier(cb, S_TRANSFER, S_ALL | S_BOTTOM, 0, 0, nullptr, 0, nullptr, 4, post);
    vk.EndCommandBuffer(cb);
    const uint64_t waitD = g_vD, signalV = ++g_vV;
    VkTimelineSemaphoreSubmitInfo tsi{ ST_TIMELINE_SEMAPHORE_SUBMIT_INFO, nullptr, 1, &waitD, 1, &signalV };
    const VkFlags waitStage = S_TRANSFER;
    VkSubmitInfo si{ ST_SUBMIT_INFO, &tsi, 1, &g_semD, &waitStage, 1, &cb, 1, &g_semV };
    g_ix->LockSubmissionQueue();  // only our submit inside: DXVK's submission thread waits microseconds
    VkResult r = vk.QueueSubmit(g_queue, 1, &si, g_cmdFence[ci]);
    g_ix->ReleaseSubmissionQueue();
    if (r != 0) {
        static int fails;
        if (fails++ < 3) log_printf("DXVK copy: vkQueueSubmit failed (%d)\n", r);
        return;
    }

    // D3D12: after the Vulkan copy, the bridge textures into the presenter's eye images.
    g_q->Wait(g_fV, signalV);
    Cmd12& c = g_c12[g_c12N++ & 3];
    wait12(g_fD, c.fv);  // four pairs old: long done
    c.alloc->Reset();
    c.list->Reset(c.alloc, nullptr);
    ID3D12Resource* dst[2] = {};
    int slot[2];
    for (int e = 0; e < 2; e++) {
        slot[e] = vr_gpu_begin(e, (int)s0.w, (int)s0.h, DXGI_FORMAT_B8G8R8A8_UNORM, (void**)&dst[e]);
        if (!dst[e]) continue;
        D3D12_RESOURCE_BARRIER b[2] = {};
        for (auto& x : b) { x.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION; x.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES; }
        b[0].Transition.pResource = g_br[e].tex;
        b[0].Transition.StateBefore = D3D12_RESOURCE_STATE_COMMON;
        b[0].Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_SOURCE;
        b[1].Transition.pResource = dst[e];
        b[1].Transition.StateBefore = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
        b[1].Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_DEST;
        c.list->ResourceBarrier(2, b);
        c.list->CopyResource(dst[e], g_br[e].tex);
        for (auto& x : b) std::swap(x.Transition.StateBefore, x.Transition.StateAfter);
        c.list->ResourceBarrier(2, b);
    }
    c.list->Close();
    ID3D12CommandList* lists[] = { c.list };
    g_q->ExecuteCommandLists(1, lists);
    g_q->Signal(g_fD, ++g_vD);
    c.fv = g_vD;
    for (int e = 0; e < 2; e++)
        if (dst[e]) vr_gpu_end(e, slot[e], g_pend[e].pair, g_pend[e].pose, g_vD);
}

void vk_read_eye(int eye, void (*write)(const void* bits, int pitch, int w, int h)) {
    ID3D12Resource* tex = g_d12 ? (ID3D12Resource*)vr_gpu_newest(eye) : nullptr;
    if (!tex) return;
    D3D12_RESOURCE_DESC td = tex->GetDesc();
    D3D12_PLACED_SUBRESOURCE_FOOTPRINT fp;
    UINT64 total = 0;
    g_d12->GetCopyableFootprints(&td, 0, 1, 0, &fp, nullptr, nullptr, &total);
    D3D12_HEAP_PROPERTIES hp{};
    hp.Type = D3D12_HEAP_TYPE_READBACK;
    D3D12_RESOURCE_DESC bd{};
    bd.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    bd.Width = total;
    bd.Height = bd.DepthOrArraySize = bd.MipLevels = 1;
    bd.SampleDesc.Count = 1;
    bd.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    ID3D12Resource* buf = nullptr;
    if (FAILED(g_d12->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &bd, D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&buf)))) return;
    Cmd12& c = g_c12[g_c12N++ & 3];
    wait12(g_fD, c.fv);
    c.alloc->Reset();
    c.list->Reset(c.alloc, nullptr);
    D3D12_RESOURCE_BARRIER b{};
    b.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    b.Transition.pResource = tex;
    b.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
    b.Transition.StateBefore = D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE;
    b.Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_SOURCE;
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
    g_q->Signal(g_fD, ++g_vD);
    c.fv = g_vD;
    wait12(g_fD, g_vD);
    void* p = nullptr;
    if (SUCCEEDED(buf->Map(0, nullptr, &p))) {
        write((const char*)p + fp.Offset, (int)fp.Footprint.RowPitch, (int)td.Width, (int)td.Height);
        buf->Unmap(0, nullptr);
    }
    buf->Release();
}

bool vk_capture(void* device9, int eye, unsigned pair, const FramePose& pose) {
    IDirect3DDevice9* d = (IDirect3DDevice9*)device9;
    if (!init(d)) return g_ix != nullptr;  // DXVK but broken: nothing more to do; not DXVK: let the caller try
    IDirect3DSurface9* bb = nullptr;
    if (FAILED(d->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &bb))) return true;
    D3DSURFACE_DESC bd;
    bb->GetDesc(&bd);
    const int e = eye & 1;
    Slot& s = g_slots[e];
    if (g_pairReady && (!s.tex || s.w != bd.Width || s.h != bd.Height)) g_pairReady = false;  // its image is about to change
    if ((!s.tex || s.w != bd.Width || s.h != bd.Height) && !make_slot(s, bd.Width, bd.Height)) {
        static int fails;
        if (fails++ < 3) log_printf("DXVK copy: could not make a %ux%u eye texture\n", bd.Width, bd.Height);
        bb->Release();
        return true;
    }
    // The previous pair, captured a frame ago (its commands are on DXVK's queue by now), before this
    // frame overwrites its left eye.
    if (g_pairReady) bridge_pair();
    const VkImageSubresourceRange all{ 1, 0, 1, 0, 1 };
    if (s.readable) g_ix->TransitionTextureLayout(s.itex, &all, L_TRANSFER_SRC, s.native);  // back to DXVK's layout first
    HRESULT hr = d->StretchRect(bb, nullptr, s.surf, nullptr, D3DTEXF_NONE);
    bb->Release();
    g_ix->TransitionTextureLayout(s.itex, &all, s.native, L_TRANSFER_SRC);
    s.readable = true;
    g_pend[e] = Pending{ pair, pose, SUCCEEDED(hr) };
    if (e == 1) g_pairReady = g_pend[0].ok && g_pend[0].pair == pair;  // bridged at the next frame
    return true;
}
