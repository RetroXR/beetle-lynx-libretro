#ifndef __LYNX_LINK__
#define __LYNX_LINK__

#include <libretro.h>

#include "../state.h"

void lynx_link_init(retro_environment_t env);
void lynx_link_start(void);
void lynx_link_stop(void);
/* After anything that moves gSystemCycleCount other than running: a reset, a
 * state load. */
void lynx_link_resync(void);
/* Called after every CSystem::Update(). */
void lynx_link_ran(void);

/* Fixed-window frames (core option lynx_fixed_frames), which netplay pins on.
 *
 * Stock ends a retro_run when THIS unit's display finishes a frame, which is set
 * by the game's own timers and by when the unit was switched on. Two cabled
 * units' frames therefore end at unrelated instants on the wire, and there is no
 * moment at which "every machine at frame N" names one point in the cable's
 * time -- which is what a netplay rollback has to restore the whole cable to.
 * In this mode every retro_run is the same slice of the Lynx's 16 MHz clock, so
 * every unit's frame N ends on the same tick of the bus.
 *
 * begin is told how many cycles the frame has left; the link meets its peers at
 * both edges and never asks the bus for more than the frame, so a peer that has
 * stopped at the edge never holds this one up short of it. */
extern bool lynx_fixed_frames;
/* The window's rate: the 75 Hz the core reports, or 60 with lynx_force_60hz. */
extern unsigned lynx_fixed_fps;
/* Returns the cycles this frame is to run: `cycles` (what the last window
 * left), or a whole `window` for a machine the bus is about to anchor. */
uint32 lynx_link_frame_begin(uint32 cycles, uint32 window);
void lynx_link_frame_end(void);

/* The link's side of a savestate. Under fixed frames it is restored with the
 * rest of the machine (netplay restores the bus to the same instant); otherwise
 * a load keeps the link clock running forward, as it always has. */
int lynx_link_state_action(StateMem *sm, int load, int data_only);

#endif
