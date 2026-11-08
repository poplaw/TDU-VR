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
