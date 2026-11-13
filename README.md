# Hat Chop for The Sims 3

An ASI plugin that gives The Sims 3 automatic hat chops, like The Sims 4. When a Sim wears an accessory hat, the hair under it is cut away at draw time, so it no longer pokes through the hat. Take the hat off and the full hairstyle comes back.

It works with any hairstyle and any accessory hat on the hat bone. No hair or hat package is edited, and there are no per-hair chop meshes.

Version 7.2.

## Chop types

| Type | Name | Used for | What happens to the hair |
|---|---|---|---|
| 1 | Cap chop | flatter hats: baseball cap, fedora, newsboy | cut almost level, a little lower at the back |
| 2 | Beanie chop | hats low at the back: beanie, cowboy hat, sun hat | cut sloping down toward the back |
| 3 | Bald | full hats: hijabs, head scarves, wraps, turbans, bonnets, durags | all scalp hair is hidden (beards stay) |

Types 1 and 2 follow the two cut shapes The Sims 4 itself uses. The plugin picks one per hat from the hat's shape: the hat's lower edge is measured in 36 slices around the head. The chosen cut is lifted to sit just inside that edge, so no hair shows below the brim and no bald band shows above it.

Type 3 is used only for meshes that carry the full-hat marker (see below).

## What counts as a hat

- **Bone:** the accessory must be weighted to `b__Hat_grip__`.
- **Coverage:** it must cover at least 90% of the top of the scalp, tested with straight-down rays.

Circlets, tiaras, animal ears, earmuffs and headbands therefore leave the hair alone. A hat split into several meshes counts when the pieces cover the scalp together.

## Ages

Adults, teens, elders, children and toddlers. Children and toddlers are found by the height of the face mesh. All measurements are then scaled with EA's own head proportions for that age.

## Requirements

- The Sims 3, EA App version: `TS3.exe` 1.69, PE timestamp `6707155C`. On any other executable the plugin logs `OFF` and does nothing.
- A 32-bit ASI loader, for example [Ultimate ASI Loader](https://github.com/ThirteenAG/Ultimate-ASI-Loader). It can also start as a MonoPatcher plugin.

## Install

1. Close the game.
2. Copy `HatChop.asi` and `HatChop.ini` into `The Sims 3\Game\Bin`, next to `TS3.exe`.
3. Start the game and put a hat on a Sim.

To remove it, delete both files.

## Settings (`HatChop.ini`)

| Key | Default | |
|---|---|---|
| `Enabled` | 1 | 0 turns the plugin off |
| `LogParts` | 0 | 1 writes extra mesh details to the log |

## Log

`Game\Bin\HatChop.log` records the following:

- the start line (`Hat Chop 7.2 ASI starting`);
- each hat decision: chosen cut, lift, time taken;
- accessories skipped as non-hats;
- the reason whenever a chop is skipped.

## Full-hat marker (for CC creators)

To make a hat use type 3 (all hair hidden), add three extra vertices at the end of one of its meshes, plus one zero-area triangle using them:

- The last three vertices of the mesh have the same position.
- Their x is the float with bit pattern `0x38F0C0DE`, and their z the float with bit pattern `0xB8FA11CA`. Any y works; put them inside the head so they are never seen.
- The mesh must not use the SimHair shader. Its bones do not matter, so draped scarves on the neck bones work too.

## Building

Builds with 32-bit MinGW-w64 gcc (MSYS2 package `mingw-w64-i686-gcc`):

- Windows: `build.bat` (set `GCC` if your gcc is not at `C:\msys64\mingw32\bin\gcc.exe`)
- MSYS2 MINGW32 shell: `./build.sh`

Output: `bin/HatChop.asi`. The plugin uses strict SSE2 math (`-msse2 -mfpmath=sse`), so its results do not depend on the x87 precision Direct3D 9 sets.

## Source

| File | Contents |
|---|---|
| `src/hat_chop_hat_fit.c` | build entry: version 7.2 settings |
| `src/hat_chop.c` | game hooks, settings, log, hat and full-hat detection |
| `src/hat_chop_hat_fit.h` | hat test (scalp coverage) and hat-edge fit |
| `src/hat_chop_age.h` | child and toddler detection and scaling |
| `src/hat_chop_profile_*.h` | the two cut shapes, head calibration, chop, index rewrite, result cache |
| `src/hat_chop_bisect.h`, `src/hat_chop_fixed_cut.h` | exact cut along the plane, and the fallback cut |
| `src/hat_chop_skin_guard.h` | keeps reshaped hair from sinking into the forehead |
| `src/hat_chop_ray_index.h` | height and direction buckets for fast ray tests |
| `src/hat_chop_payload.h`, `src/hat_chop_indices.h` | reading and writing the game's packed vertex and index buffers |
| `src/hat_chop_gravity.h`, `src/hat_chop_cut.h` | the older straight-down chop helpers, still shared by the code above |

### Data from The Sims 4

`src/hat_chop_profile_meshes.h` and `src/hat_chop_profile_refs.h` were generated from The Sims 4 game files. They hold the cut0/cut1 meshes of EA's `yfHair_LongRocker` hairstyle, EA's `yfHead` mesh, and measurements taken from them. These are the reference shapes for chop types 1 and 2, and they are compiled into `HatChop.asi`. That content belongs to Electronic Arts.

## Known limits

- Hat_Resize slider changes are not read; the chop uses the hat at its default size.
- Hats on other bones (for example `b__Glasses__`) are not recognized.
- Hats tilted back more steeply than either The Sims 4 cut can leave a few strands at the back brim.
- Large vertex moves keep the original normals, so shading can differ slightly near the cut.

## Credits

- The two-cut method follows mauvemorn's Sims 4 Studio tutorial [How to make hat chops for TS4 hair](https://sims4studio.com/thread/26873/make-hat-chops-ts4-hair).
- [Ultimate ASI Loader](https://github.com/ThirteenAG/Ultimate-ASI-Loader) by ThirteenAG.

The Sims is a trademark of Electronic Arts. This project is not made or endorsed by EA.
