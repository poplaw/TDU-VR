#pragma once
// D3D9On12 path: the game's D3D9 device runs on a D3D12 device of ours, so each finished frame goes
// from the back buffer to an eye image that SteamVR reads, without leaving the GPU.
#include "vr.h"

// An IDirect3D9 whose devices run on our D3D12 device, or null when D3D9On12 is unavailable (then the
// caller uses plain d3d9 and the CPU copy). d3d9: the system d3d9.dll.
void* gpu_create_d3d9(void* d3d9Module, unsigned sdk);
bool gpu_on();  // gpu_create_d3d9 succeeded at least once
// Two halves around the game's Present. Before it (the back buffer still holds the frame): copy the
// back buffer into a render target of ours. After it: take the render target's D3D12 resource out
// (this waits until 9On12's worker thread has caught up with the game, ~1.5 ms) and copy it into an
// eye image on the GPU.
bool gpu_capture(void* device9);
bool gpu_finish(int eye, unsigned pair, const FramePose& pose);
// Drop what belongs to the device (before Reset, or a new device).
void gpu_release();
const char* gpu_status();  // time per copy phase since the last call, for the log
// Read the newest image of an eye back to the CPU (slow, for TDUVR_DUMP): calls write(bits, pitch, w, h).
void gpu_read_eye(int eye, void (*write)(const void* bits, int pitch, int w, int h));
