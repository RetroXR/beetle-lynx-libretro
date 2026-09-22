#ifndef __LYNX_LINK__
#define __LYNX_LINK__

#include <libretro.h>

void lynx_link_init(retro_environment_t env);
void lynx_link_start(void);
void lynx_link_stop(void);
/* After anything that moves gSystemCycleCount other than running: a reset, a
 * state load. */
void lynx_link_resync(void);
/* Called after every CSystem::Update(). */
void lynx_link_ran(void);

#endif
