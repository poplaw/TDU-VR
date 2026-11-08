# TDU VR (SteamVR, 6DOF)

A `d3d9.dll` proxy for Test Drive Unlimited (PC, 2007 exe). It puts the game in a headset through SteamVR with full 6DOF head tracking and stereo.

## How it works
- The game's vertex shaders carry D3DX constant tables (`gVSWorldViewProject`, `gVSWorldView`, `gView`, `gViewProject`, `gProj`, `gInvView`, ...). The proxy reads them in `CreateVertexShader`, then rewrites those registers before every draw:
  - `*View` → `M * invEye`
  - `*ViewProj` → `M * inv(P) * invEye * Peye`
  - `Proj` → `Peye`
  - `InvView` → `E * M`
- **`P` is worked out per draw.** TDU draws different parts of the scene with different depth ranges (for example near 2 m with infinite far, and near 0.8 m with far 40 km) and changes FOV between views. So one shared projection is wrong.
  - With a matrix pair, it's exact: `inv(WorldView) * WorldViewProj`, or `inv(View) * ViewProj`.
  - With only a combined matrix, the depth terms are recovered from that matrix and the FOV comes from the main camera.
  - 2D and orthographic draws fail these checks and are left alone.
- **`Peye` is the headset's off-center frustum**, with the draw's own depth range kept. TDU's camera is right-handed and looks down −Z.
- **Only main-view passes are rewritten:** the render target must have the back buffer's shape and the viewport must be (nearly) full. Cube maps, shadow maps, mirrors and menu car previews are skipped.
- **2D draws go onto a virtual screen.** Menus, HUD and cursor are flat draws with orthographic matrices. They're moved onto a flat screen fixed in front of you (`UiFov` wide, `UiDistance` away, recentered with F8) and projected correctly for each eye. Without this, the same picture lands on different directions in each off-center eye image and you see it double. Post-processing quads read the game's render targets, so they're recognized and left full-frame.
  - The game layers its menus with the depth test (tiny z values, buttons in front of the bars behind them), so the screen keeps each draw's own depth. Flattening it hid the bottom-bar buttons ("Wyjście", "Powrót", ...).
  - In menus the game never clears the frame, because its full-screen background normally covers it. On the virtual screen that background covers only part of the frame, so the mod clears the frame after each Present. Otherwise the other eye's older image showed through as a second menu.
- **Head look.** The game has TrackIR head-look built in: it reads a yaw and pitch, clamps them, and turns its in-car camera. The mod writes the headset's yaw and pitch there each frame, at addresses checked against this exe's camera code before use, so the in-car camera points where you look. The rewrite then removes exactly that rotation and only adds roll, position and eye offset.
  - Why: the game runs a sphere-cull test (0x50b3b0) against the in-car camera for a handful of objects per frame, about 44° either side of where that camera looks; head look keeps it centered on your view. The rest of the world does not depend on the camera's direction or FOV (tested: with the camera turned 86° away, or its FOV forced to 10°, everything ahead is still drawn).
  - Measured on the in-car camera: angles apply on the next frame with no smoothing; yaw is clamped at 97° and pitch at 30°; the game's pitch sign is inverted.
  - The camera looks 4.7° down and yaws around the car's up axis. The mod splits the head pose in that same frame.
  - The game skips its camera update on roughly 1 in 9 frames. So the mod reads the angles the camera actually used back from the camera object at the first draw of each frame, and corrects the eye matrices if they differ. The object is found by a short memory scan while a tiny, changing look is fed (under 0.05°). Until then, and whenever that camera stops following (menus, other camera views), no rotation is taken out, so head tracking works normally there.
  - The first-draw readback is not always what gets rendered. While the head turns, about 15% of the frames that start a pair (the left eye) are drawn with the previous frame's camera, although the camera object already holds the new one (never seen on right-eye frames). So the mod finds the camera each frame was actually drawn with and matches it by orientation against the camera object's state now and at the last 4 first draws (`+0x70`), then uses that state's look.
  - That camera must be known before the world is drawn. The game's own `InvView`/`View` only shows up about 1,100 draws into the frame, after the roads, trees and buildings. Matching there left all of those 1–3° off on those left-eye frames: ghosting and trailing while turning the head. So the camera is read from the first draw whose world matrix doesn't rotate (ground pieces; in the car, the very first 3D draw). Its `WorldView` 3x3 rows are the view rotation's, scaled. The pick is checked once more at the first `InvView`/`View`.
  - The match allows about 1°: while driving, the game sometimes draws a state from between two first draws (the car has moved a little), and while the head turns the looks to choose from differ by more than that.
  - Log line `head look:` shows: `rendered with another look than read` (frames corrected, per eye), `unmatched` (no exact match; the nearest state within 1° is used), `known by 3D draw` (should be about 0) and `early pick corrected` (should stay 0).
  - The menu/HUD screen stays fixed in front of you; it doesn't turn with the game camera.
- **Positions never depend on noisy depth terms.** Per-draw depth terms carry float noise (the near plane wanders by about 1.5%). WorldView is rebuilt from WorldViewProj using only the FOV scales, so the depth terms only affect depth. They're snapped to the frame's few real depth ranges, so each range gets one bit-identical frustum.
- **Stereo is alternate-eye rendering:** even frames are rendered for one eye, odd frames for the other. Both frames of a pair use the same head pose, and the presenter only shows a left/right pair from the same pair. So it doesn't matter whether SteamVR reprojects each eye with its own pose or with one shared pose.
- **A presenter thread feeds SteamVR.** The back buffer is copied through system memory with a one-frame lag and handed to a presenter thread. Every headset frame, that thread submits the newest image of each eye with its render pose. So the compositor always gets a complete frame on time and only has to reproject head motion. Meanwhile the game runs at its own rate and never waits on the headset.
- Quest via Link, Air Link or Virtual Desktop works through SteamVR. SteamVR's OpenXR runtime has no 32-bit support, so OpenVR is used.

## Install
Needs the PC version of Test Drive Unlimited (the 2007 exe this was made against), SteamVR and a headset. Quest works through Link, Air Link or Virtual Desktop.

1. Get `d3d9.dll`, `openvr_api.dll` and `tdu_vr.ini` from the release zip, or build them: run `build.bat` (needs Visual Studio with C++ tools; set `TDU_DIR` to the game folder to also test against the game's shaders).
2. Copy those three files next to `TestDriveUnlimited.exe`.
3. Start SteamVR, then the game. The mod forces windowed mode and turns V-Sync off, since the game has no options for either. If SteamVR finds no headset at that moment, the mod stays out of the way and the game runs as normal. In the house the game opens a tiny second display device (120x61). The mod leaves it alone; taking it over used to crash the game a few seconds after arriving.
4. With `AutoDrive=1` (the default) you need no menus. About 8 s after the main menu shows, the mod presses Enter ("Kontynuuj grę"), then about 10 s after the house shows, Backspace ("Wyjście"), and you're in your car, about a minute after launch. It presses keys only while the game window is in front, and stops after 5 minutes. Set `AutoDrive=0` to use the menus yourself.
5. Sit in the in-car view and press **F8** to recenter.

To uninstall, delete those three files.

## Settings that matter
Each eye gets every other game frame, so each eye's picture updates at **half the game's FPS**. TDU at 1920x1080 runs at about 90–100 fps with the mod on an RTX 4070 Super, so each eye gets about 45 new images per second. Head rotation is still reprojected at the full headset rate.

**Frame generation must be off.** Both SteamVR's **Motion Smoothing** and Virtual Desktop's **SSW** force the app to half rate and invent the in-between frames from motion. With alternate-eye images that warps, stretches and wobbles the world.

The `SteamVR:` part of the status lines in `tdu_vr.log` should read `throttled 0, motion smoothing off`. If you see `presenter 60 Hz, throttled 1, motion smoothing ACTIVE` even at an idle loading screen, one of them is still on.

- **SteamVR → Settings → Video:** turn **Motion Smoothing off**, globally or per application for this game.
- **Virtual Desktop → Streaming:** set **Synchronous Spacewarp (SSW)** to **Disabled**.
- **Headset refresh rate:** 90 or 72 Hz is easier on the GPU than 120.
- **In-game:** 1920x1080 or lower, not your desktop's 5120x1440. Lower shadows and reflections too: about 300–450 of the roughly 3,000 draws per frame go into reflection and shadow maps.

## Testing without the headset
Environment variables, set before launching the game:
- `TDUVR_FAKEHEAD=<degrees>,<seconds>`: the head looks straight ahead and turns left/right by that much once per period (`0,4` holds still). Otherwise a headset lying on the desk points anywhere.
- `TDUVR_DUMP=<file.bmp>,<seconds>`: writes one eye image exactly as sent to the headset. `,+<seconds>` counts from when the in-car camera is found.
- `TDUVR_LOOKTEST=1`: cycles fixed head-look angles and logs the camera the game produced.

With `AutoDrive=1`, a launch reaches the in-car view on its own. Keep the game window in front: the game reads the keyboard through DirectInput, so only real (injected) key presses work.

## Known limits
- **Edge culling.** Objects culled by the game's sphere test (about 44° either side of where the game camera looks) can pop at the outer edges of the headset view (up to 55° out) and past the head-look limits (97° yaw, 30° pitch). Widening the game's FOV is not a fix: its camera FOV (set at 0x404840, called from 0x8d696e) also drives level of detail, and at 120° distant signs vanish and trees switch models.
- **HUD and menu sharpness.** The virtual screen is drawn at the game's resolution and then squeezed into part of the eye image, so small text can be soft. Raising `UiFov` makes it bigger. Screen-space effects drawn as 2D with plain textures, such as lens flares, would land on the virtual screen too.
- **Alternate-eye artifacts.** At high speed, side scenery can shimmer, because each eye's image comes from a different frame.
- **CPU copy.** It no longer stalls, but at high resolutions it still costs a few milliseconds per frame. The next step is a GPU-only copy through D3D9On12.
- **Lighting.** Per-pixel lighting, specular and mirrors use the original camera, so small errors appear when you lean far.
