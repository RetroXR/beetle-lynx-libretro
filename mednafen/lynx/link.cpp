/* ComLynx, carried over the frontend's link bus.
 *
 * ComLynx is one open-collector wire shared by every Lynx on the cable, driven
 * by Mikey's UART. There is no master and no shared clock: a unit clocks its
 * own bytes out at the rate Timer 4 sets, and every unit on the wire hears every
 * byte -- its own included, which Mikey already models as a loopback. So the
 * cable here is a broadcast of bytes, and what one machine sends lands in the
 * receive queue of every other.
 *
 * A byte is sent the moment the guest writes it, stamped with the tick its stop
 * bit leaves the wire -- known then, from Timer 4's rate -- and each receiver
 * latches it when its own clock reaches that tick, exactly when a unit on a real
 * cable would. Games settle who is who and start a match with bursts of bytes
 * answered within a byte or two, so anything later than the wire (a grain of
 * latency, or Mikey's 44-tick gap between queued loopback bytes) sends each unit
 * off alone. A grain shorter than the shortest byte keeps every stamp reachable.
 *
 * It is one wire, so every unit must hear the same bytes in the same order --
 * its own included -- and two frames on it at once are one frame: the line is
 * open-collector, so their bits AND. A game counting players or electing a
 * leader reads back what it sent to find out whether it collided. So while
 * cabled, this unit's own byte goes into the same inbox as everyone else's
 * instead of Mikey's instant loopback, bytes are ordered by the tick they end,
 * and frames that overlap are merged. A byte is latched at its stop bit with
 * whatever overlaps it that has arrived by then: a peer that starts in the last
 * grain of it may not have, and is heard as its own frame. Holding the byte a
 * grain to be sure makes a unit hear its own echo late, which games that check
 * the echo for a collision do not forgive.
 *
 * Every machine rendezvouses each grain, so a byte lands at the same emulated
 * moment whatever the host's threads did -- which is what netplay relies on. */

#include <string.h>

#include <libretro.h>

#include "system.h"
#include "link.h"
#include "link_interface.h"

extern retro_log_printf_t log_cb;
extern CSystem *lynxie;

/* Wire name. A peer with any other id is never joined to this one. */
#define LINK_PROTOCOL "comlynx-1"

/* gSystemCycleCount's 16 MHz. */
#define LINK_CLOCK_RATE ((uint64_t)16000000)

/* 128 us. The fastest ComLynx byte, 62500 baud, takes 176 us (2816 cycles);
 * the grain must be shorter than that for a byte stamped at its stop bit to be
 * inside every peer's horizon when it is sent. */
#define LINK_GRAIN ((uint64_t)2048)

/* A jump in gSystemCycleCount longer than this is not running: a state load. */
#define LINK_MAX_STEP 16000000u

#define LINK_MSG_SIZE 8
#define LINK_MSG_BYTE 1

#define INBOX_MAX 256

static const struct retro_link_interface *link_if;
static struct retro_link_interface link_storage;
static retro_link_port_t *link_port;
static bool anchored;
static unsigned peers;

static uint64_t now;
static uint64_t limit;
static uint32 last_cycle;

static uint16 inbox[INBOX_MAX];
static uint64_t inbox_tick[INBOX_MAX];
static uint32 inbox_len[INBOX_MAX];
static unsigned inbox_count;

static void say(enum retro_log_level level, const char *msg, unsigned a, int b);

/* Oldest end first; equal ends keep arrival order. */
static void inbox_put(uint16 data, uint64_t tick, uint32 len)
{
   unsigned i;
   if (inbox_count >= INBOX_MAX)
   {
      say(RETRO_LOG_WARN, "ComLynx: inbox full, byte dropped%u%d\n", 0, 0);
      return;
   }
   i = inbox_count;
   while (i > 0 && inbox_tick[i - 1] > tick)
   {
      inbox[i] = inbox[i - 1];
      inbox_tick[i] = inbox_tick[i - 1];
      inbox_len[i] = inbox_len[i - 1];
      i--;
   }
   inbox[i] = data;
   inbox_tick[i] = tick;
   inbox_len[i] = len;
   inbox_count++;
}

static void inbox_drop(unsigned n)
{
   inbox_count -= n;
   memmove(&inbox[0], &inbox[n], inbox_count * sizeof(inbox[0]));
   memmove(&inbox_tick[0], &inbox_tick[n], inbox_count * sizeof(inbox_tick[0]));
   memmove(&inbox_len[0], &inbox_len[n], inbox_count * sizeof(inbox_len[0]));
}

static unsigned sent, took;

static void say(enum retro_log_level level, const char *msg, unsigned a, int b)
{
   if (log_cb)
      log_cb(level, msg, a, b);
}

static void on_tx(int data, uint32 objref)
{
   uint8 msg[LINK_MSG_SIZE];
   uint64_t at;
   uint32 len;
   (void)objref;

   if (!link_port || peers < 2)
      return;

   /* The first rendezvous anchors the origin the bus measures ticks from; a
    * message sent before it lands in the peer's far future. Asking for where
    * this machine already stands is granted at once. */
   if (!anchored)
   {
      link_if->advance(link_port, now, now + LINK_GRAIN, now, 0);
      anchored = true;
   }

   msg[0] = LINK_MSG_BYTE;
   msg[1] = 0;
   msg[2] = data & 0xFF;
   msg[3] = (data >> 8) & 0xFF;   /* parity bit, or the break code */
   /* now lags by the instruction doing the write; the byte ends one frame on. */
   len = lynxie->mMikie->ComLynxByteCycles();
   at = now + (uint32)(gSystemCycleCount - last_cycle) + len;
   if (at < now + LINK_GRAIN)
      at = now + LINK_GRAIN;
   msg[4] = len & 0xFF;
   msg[5] = (len >> 8) & 0xFF;
   msg[6] = (len >> 16) & 0xFF;
   msg[7] = (len >> 24) & 0xFF;
   /* This unit hears itself on the same wire, in the same order. */
   inbox_put(data & 0xFFFF, at, len);
   if (link_if->send(link_port, at, RETRO_LINK_BROADCAST, msg, sizeof(msg)))
      sent++;
}

void lynx_link_init(retro_environment_t env)
{
   memset(&link_storage, 0, sizeof(link_storage));
   link_if = NULL;
   if (env(RETRO_ENVIRONMENT_GET_LINK_INTERFACE, &link_storage) ||
       env(RETRO_ENVIRONMENT_GET_LINK_INTERFACE_FINAL, &link_storage))
      link_if = &link_storage;
}

void lynx_link_start(void)
{
   if (!link_if || link_port || !lynxie)
      return;
   link_port = link_if->attach(0, LINK_PROTOCOL, LINK_CLOCK_RATE);
   anchored = false;
   peers = 0;
   inbox_count = 0;
   limit = now;
   last_cycle = gSystemCycleCount;
   if (!link_port)
   {
      say(RETRO_LOG_WARN, "ComLynx: the frontend refused port %u%d\n", 0, 0);
      return;
   }
   lynxie->mMikie->ComLynxTxStartCallback(on_tx, 0);
   lynxie->mMikie->ComLynxCable(0);
}

void lynx_link_stop(void)
{
   if (!link_port)
      return;
   if (lynxie)
      lynxie->mMikie->ComLynxTxStartCallback(NULL, 0);
   link_if->detach(link_port);
   link_port = NULL;
   inbox_count = 0;
   peers = 0;
}

void lynx_link_resync(void)
{
   /* The link clock carries on -- the bus must never see it go backwards --
    * and only what was in flight to the old machine goes. */
   last_cycle = gSystemCycleCount;
   inbox_count = 0;
}

static void pump(void)
{
   uint8 buf[16];
   uint64_t tick;
   unsigned from;
   size_t len = sizeof(buf);

   while (link_if->recv(link_port, &tick, &from, buf, &len))
   {
      if (len == LINK_MSG_SIZE && buf[0] == LINK_MSG_BYTE)
         inbox_put(buf[2] | (buf[3] << 8), tick,
                   buf[4] | (buf[5] << 8) | (buf[6] << 16) | ((uint32)buf[7] << 24));
      len = sizeof(buf);
   }
}

static void refresh_peers(void)
{
   unsigned count = 0;
   int id = link_if->peers(link_port, &count);
   unsigned was = peers;

   peers = (id < 0) ? 0 : count;
   if (peers != was)
   {
      /* Bytes on their way to the last cable belong to it, not to this one. */
      inbox_count = 0;
      anchored = false;
      /* NOEXP: a game can see a lead in the socket. */
      lynxie->mMikie->ComLynxCable(peers >= 2);
      lynxie->mMikie->ComLynxExternalLoopback(peers >= 2);
      say(RETRO_LOG_WARN, "ComLynx: %u machine(s) on the wire, this one is %d\n",
          peers, id);
   }
}

static void rendezvous(void)
{
   uint32_t wake = RETRO_LINK_WAKE_NONE;
   uint64_t grant;

   refresh_peers();
   grant = link_if->advance(link_port, now, now + LINK_GRAIN, now + LINK_GRAIN, &wake);
   anchored = true;
   pump();

   if (grant == RETRO_LINK_UNBOUNDED)
      limit = now + LINK_GRAIN;       /* uncabled: look again a grain from now */
   else if (grant > now)
      limit = grant;
   else
      limit = now + 1;                /* woken without a grant: ask again at once */
}

void lynx_link_ran(void)
{
   uint32 step;

   if (!link_port)
      return;
   step = gSystemCycleCount - last_cycle;
   last_cycle = gSystemCycleCount;
   if (step > LINK_MAX_STEP)
      step = 0;
   now += step;

   while (inbox_count && inbox_tick[0] <= now)
   {
      /* Every frame that overlaps the first on the wire is the same frame:
       * the line is low wherever any unit drives it low. */
      uint16 wire = inbox[0];
      unsigned n = 1;
      bool brk = (inbox[0] & 0x8000) != 0;
      while (n < inbox_count && inbox_tick[n] < inbox_tick[0] + inbox_len[n])
      {
         wire &= inbox[n];
         brk = brk || (inbox[n] & 0x8000);
         n++;
      }
      if (brk)
         wire = 0x8000;
      lynxie->mMikie->ComLynxRxWire(wire);
      took++;
      inbox_drop(n);
   }

   if (now >= limit)
      rendezvous();
}
