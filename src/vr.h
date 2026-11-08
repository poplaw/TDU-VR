#pragma once
#include "vrmath.h"

struct EyeView {
    Mat4 invEye;              // game view space -> this eye's view space (row-vector, right-handed)
    float l, r, up, down;     // eye frustum tangents (l < 0)
    float lookYaw, lookPitch; // head look for the game's own camera (radians, +yaw = left, +pitch = up)
};

struct FramePose { float m[3][4]; };  // raw seated HMD pose a frame was rendered with

void log_printf(const char* fmt, ...);
float vr_ini_float(const char* key, float def);                // [VR] key in tdu_vr.ini
bool vr_init();                                                // false = no headset / SteamVR; mod stays passthrough
bool vr_no_hmd();                                              // TDUVR_NOHMD set: simulated headset, no SteamVR
// Pose for the frame about to render: fills pose, frustum and the head look to give the game
// (lookYaw/lookPitch, clamped to maxYaw/maxPitch). newPair: the first eye of a left/right pair
// samples a fresh head pose; the second eye reuses it, so both images of a pair carry the same pose.
// tilt: the game camera's built-in pitch relative to the axis the game yaws around (TDU's in-car
// camera looks 4.7 degrees down and yaws around the car's up axis), radians, negative = down.
void vr_begin_frame(int eye, bool newPair, float maxYaw, float maxPitch, float tilt, EyeView* out, FramePose* pose);
// invEye for this frame, given the head look the game actually applied to its camera (0, 0 if
// none): that rotation is taken out, so the rewrite only adds roll, position and eye offset.
void vr_eye_for_look(int eye, float yaw, float pitch, float tilt, EyeView* out);
// Hand a finished eye image to the presenter thread, which shows the newest complete left/right
// pair to SteamVR every headset frame. Never blocks on the headset.
void vr_publish(int eye, unsigned pair, const FramePose& pose, const void* bgra, int pitch, int w, int h);
void vr_recenter();
const char* vr_status();                                       // SteamVR / presenter summary for the log
bool vr_hmd_present();                                         // a headset is there (or TDUVR_NOHMD): worth setting up VR

// GPU path (gpu.cpp, D3D9On12): eye images are D3D12 textures filled on the GPU by the game thread.
void vr_gpu_init(void* device12, void* queue12, void* fence12);  // presenter submits on queue12
// A texture to copy the next image of this eye into (D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE before
// and after the copy); returns its slot. Never one the presenter is showing.
int vr_gpu_begin(int eye, int w, int h, int dxgiFormat, void** tex12);
// The copy into that slot is queued and done once fence12 reaches fenceValue: the presenter may show it.
void vr_gpu_end(int eye, int slot, unsigned pair, const FramePose& pose, unsigned long long fenceValue);
void* vr_gpu_newest(int eye);                                  // newest finished texture of an eye, or null

// DXVK path (vk.cpp): eye images are DXVK's Vulkan images (TRANSFER_SRC_OPTIMAL when handed over),
// submitted by the presenter on DXVK's queue between lock(ctx) and unlock(ctx).
void vr_vk_init(void* instance, void* physDev, void* device, void* queue, unsigned queueFamily, void (*lock)(void*),
                void (*unlock)(void*), void* ctx);
int vr_vk_begin(int eye);  // a slot to fill, never one the presenter is showing; not shown until vr_vk_end
// Commands filling the image are on DXVK's queue (flushed): the presenter may submit it.
void vr_vk_end(int eye, int slot, unsigned long long image, int w, int h, unsigned vkFormat, unsigned pair, const FramePose& pose);
void vr_vk_reset();        // forget all images; returns once the presenter is not submitting any

// Sampling profiler (prof.cpp, TDUVR_PROFILE): off unless the variable is set.
void prof_init();
void prof_frame(bool inCar);
