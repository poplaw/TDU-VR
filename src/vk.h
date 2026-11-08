#pragma once
// DXVK path: the game's D3D9 runs on DXVK (Vulkan), and each finished frame goes from the back buffer to
// a Vulkan image that SteamVR reads, without leaving the GPU.
#include "vr.h"

// Before the game's Present (the back buffer still holds the frame): copy it into an eye image of ours.
// Once the pair's right eye is in, DXVK's commands are flushed and the pair goes to the presenter.
// false: the device is not a DXVK device (no Vulkan interop), nothing was done.
bool vk_capture(void* device9, int eye, unsigned pair, const FramePose& pose);
// Drop what belongs to the device (before Reset, or a new device).
void vk_release();
