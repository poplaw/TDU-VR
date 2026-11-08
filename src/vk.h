#pragma once
// DXVK path: the game's D3D9 runs on DXVK (Vulkan), and each finished frame goes from the back buffer to
// a Vulkan image that SteamVR reads, without leaving the GPU.
#include "vr.h"

// Before the game's Present (the back buffer still holds the frame): copy it into an eye texture of ours.
// A pair is copied on into the presenter's D3D12 eye images at the next frame (see vk.cpp).
// false: the device is not a DXVK device (no Vulkan interop), nothing was done.
bool vk_capture(void* device9, int eye, unsigned pair, const FramePose& pose);
// Drop what belongs to the device (before Reset, or a new device).
void vk_release();
// Read the newest image of an eye back to the CPU (slow, for TDUVR_DUMP): calls write(bits, pitch, w, h).
void vk_read_eye(int eye, void (*write)(const void* bits, int pitch, int w, int h));
