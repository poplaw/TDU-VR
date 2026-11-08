#pragma once
// D3D9On12 path: the game's D3D9 device runs on a D3D12 device of ours, so each finished frame goes
// from the back buffer to an eye image that SteamVR reads, without leaving the GPU.
#include "vr.h"

// An IDirect3D9 whose devices run on our D3D12 device, or null when D3D9On12 is unavailable (then the
// caller uses plain d3d9 and the CPU copy). d3d9: the system d3d9.dll.
void* gpu_create_d3d9(void* d3d9Module, unsigned sdk);
bool gpu_on();  // gpu_create_d3d9 succeeded at least once
// Two halves around the game's Present. Before it (the back buffer still holds the frame): copy the
// back buffer into the render target of its eye. After it, once the pair's right eye is in: take both
// render targets' D3D12 resources out (this waits until 9On12's worker thread has caught up with the
// game, ~2 ms) and copy them into eye images on the GPU. gpu_finish returns the last eye it delivered,
// or -1 (nothing, or a left eye waiting for its right one).
bool gpu_capture(void* device9, int eye, unsigned pair, const FramePose& pose);
int gpu_finish();
void gpu_set_pair_copy(bool on);  // A/B test: false = copy every frame
// Drop what belongs to the device (before Reset, or a new device).
void gpu_release();
const char* gpu_status();  // time per copy phase since the last call, for the log
// Read the newest image of an eye back to the CPU (slow, for TDUVR_DUMP): calls write(bits, pitch, w, h).
void gpu_read_eye(int eye, void (*write)(const void* bits, int pitch, int w, int h));
