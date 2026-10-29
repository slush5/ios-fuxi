// SPDX-License-Identifier: BSD-2-Clause-Patent
// Ported from UniXNU drivers/NativeMobileDisplay/NativeLegacyDisplay.cpp (BSD-2-Clause-Patent)
// See LICENSE.NativeMobileDisplay. Adapted demand-driven timer policy; no private ABI offsets.
// The layer format filter and the TE line gate below are original Fuxi code.
#ifndef FUXI_UNIXNU_DISPLAY_H
#define FUXI_UNIXNU_DISPLAY_H
#include <stdint.h>

static inline int
fuxi_unixnu_tick_wanted(int off, int needsBlack, int dirty, int gpu, int notify)
{
	return gpu || notify || (off ? needsBlack : dirty);
}

static inline uint64_t
fuxi_unixnu_next_tick(uint64_t next, uint64_t period, uint64_t now)
{
	/* Skip missed periods; never queue a catch-up storm after a slow copy. */
	next += period;
	return next <= now ? now + period : next;
}

/*
 * Layer formats the CPU copy refuses: 1 the first time a format is seen, so
 * the caller logs each one once. Remembers up to n formats (count <= n).
 */
static inline int
fuxi_unixnu_format_new(uint32_t *seen, uint32_t *count, uint32_t n, uint32_t format)
{
	for (uint32_t i = 0; i < *count && i < n; i++) {
		if (seen[i] == format) {
			return 0;
		}
	}
	if (*count >= n) {
		return 0;
	}
	seen[*count] = format;
	*count += 1;
	return 1;
}

/*
 * TE line gate (sweetfb_tegate). A display line counter (the INTF tear-check
 * line count) advances while the panel buffer is fetched and drops back (a
 * "wrap") once per frame. The CPU copy of a frame starts only while the
 * counter is inside a window of lines at that wrap, so the top-to-bottom copy
 * runs ahead of the fetch instead of across it.
 * Calibration feeds fuxi_unixnu_te_sample() with (time, line) pairs over a
 * few frames; fuxi_unixnu_te_finish() then sets the counter range
 * lo..lo+span-1, the wrap period and the time per line while the counter
 * advances (tpl, 16.16 fixed point). The anchor is the time at which the
 * counter was at lo in the latest frame seen advancing; the window opens
 * start lines after it, once per period, and lasts window lines. Times are
 * mach absolute units. The caller supplies the register read, the clock and
 * an uncoalesced deadline sleep; nothing here touches hardware.
 * A software vsync that presents at a fixed tick would meet the window at a
 * slowly drifting phase and wait up to a period per frame (and, once wait plus
 * copy overruns the tick, lock to every other window): fuxi_unixnu_te_tick()
 * therefore places each tick te->early before a window opening.
 */
#define FUXI_UNIXNU_TE_WRAP     64u     /* a drop of at least this many lines is a wrap */
#define FUXI_UNIXNU_TE_STREAK   120u    /* consecutive misses that turn the gate off */

struct fuxi_unixnu_te_cal {
	uint64_t last, wrap_first, wrap_last, count_ticks, count_lines;
	uint32_t lo, hi, prev, wrap_line, samples, wraps, stalls;
};

struct fuxi_unixnu_te {
	uint32_t (*read_line)(void);            /* 32-bit counter register read; unset: gate off */
	uint64_t (*now)(void);
	void (*sleep_until)(uint64_t deadline);
	uint64_t period, tpl, anchor, guard, budget, early, interval;
	uint64_t wait_sum, wait_max;
	uint32_t lo, span, start, window;
	uint32_t hits, misses, streak, disabled;
};

/*
 * The gate is configured while other threads may already test it (the vsync
 * timer is armed before calibration). The configuring thread stores every
 * other field first and read_line last, with release order
 * (fuxi_unixnu_te_publish); users test it with an acquire load
 * (fuxi_unixnu_te_on) before reading any other field, so a gate seen set is
 * seen fully configured. read_line never changes once set.
 */
static inline void
fuxi_unixnu_te_publish(struct fuxi_unixnu_te *te, uint32_t (*fn)(void))
{
	__atomic_store_n(&te->read_line, fn, __ATOMIC_RELEASE);
}

/* 1: published and not disabled. */
static inline int
fuxi_unixnu_te_on(const struct fuxi_unixnu_te *te)
{
	return __atomic_load_n(&te->read_line, __ATOMIC_ACQUIRE) && !te->disabled;
}

static inline void
fuxi_unixnu_te_sample(struct fuxi_unixnu_te_cal *c, uint64_t now, uint32_t line)
{
	if (c->samples == 0) {
		c->lo = c->hi = line;
	} else if (line + FUXI_UNIXNU_TE_WRAP <= c->prev) {
		if (c->wraps == 0) {
			c->wrap_first = now;
		}
		c->wraps++;
		c->wrap_last = now;
		c->wrap_line = line;
	} else if (line > c->prev) {
		c->count_ticks += now - c->last;
		c->count_lines += line - c->prev;
	} else if (line == c->prev) {
		c->stalls++;
	}
	if (line < c->lo) {
		c->lo = line;
	}
	if (line > c->hi) {
		c->hi = line;
	}
	c->samples++;
	c->prev = line;
	c->last = now;
}

/* 1: a periodic counter within [min_period, max_period]; te gets range, period, tpl and anchor. */
static inline int
fuxi_unixnu_te_finish(const struct fuxi_unixnu_te_cal *c, struct fuxi_unixnu_te *te,
    uint64_t min_period, uint64_t max_period)
{
	if (c->wraps < 2 || c->hi < c->lo + FUXI_UNIXNU_TE_WRAP || c->count_lines == 0 ||
	    c->count_ticks == 0 || c->count_ticks > (UINT64_MAX >> 16) || max_period > (UINT64_MAX >> 20)) {
		return 0;
	}
	uint64_t period = (c->wrap_last - c->wrap_first) / (c->wraps - 1);
	uint32_t span = c->hi - c->lo + 1;
	uint64_t tpl = (c->count_ticks << 16) / c->count_lines;
	/* The counter must cover its range within a period (one eighth of slack). */
	if (period < min_period || period > max_period || tpl == 0 ||
	    tpl > ((period + period / 8) << 16) / span) {
		return 0;
	}
	te->period = period;
	te->tpl = tpl;
	te->lo = c->lo;
	te->span = span;
	uint64_t back = ((uint64_t)(c->wrap_line - c->lo) * tpl) >> 16;
	te->anchor = c->wrap_last > back ? c->wrap_last - back : 0;
	return 1;
}

/* Window of window lines (1..span) opening lead lines (0..span-1) before the wrap. */
static inline void
fuxi_unixnu_te_window(struct fuxi_unixnu_te *te, uint32_t window, uint32_t lead)
{
	if (window == 0) {
		window = 1;
	}
	if (window > te->span) {
		window = te->span;
	}
	if (lead >= te->span) {
		lead = te->span - 1;
	}
	te->window = window;
	te->start = lead ? te->span - lead : 0;
}

/* Line -> position 0..span-1 from lo; out-of-range values clamp to the ends. */
static inline uint32_t
fuxi_unixnu_te_pos(const struct fuxi_unixnu_te *te, uint32_t line)
{
	if (line <= te->lo) {
		return 0;
	}
	line -= te->lo;
	return line < te->span ? line : te->span - 1;
}

static inline int
fuxi_unixnu_te_inside(const struct fuxi_unixnu_te *te, uint32_t pos)
{
	return (pos + te->span - te->start) % te->span < te->window;
}

/* Earliest time, not before now, at which the window is predicted open. */
static inline uint64_t
fuxi_unixnu_te_target(const struct fuxi_unixnu_te *te, uint64_t now)
{
	uint64_t length = ((uint64_t)te->window * te->tpl) >> 16;
	if (te->anchor == 0 || te->anchor > now) {
		/* No phase yet: poll, in steps short enough not to step over the window. */
		return now + (length / 2 < te->period / 16 ? length / 2 : te->period / 16);
	}
	uint64_t open = te->anchor + (((uint64_t)te->start * te->tpl) >> 16);
	uint64_t close = open + length;
	if (close <= now) {
		open += ((now - close) / te->period + 1) * te->period;
	}
	return open > now ? open : now;
}

/*
 * One read of the counter: its position, and *when it was read. A counter
 * that advances (strictly between the ends of its range) re-anchors the phase.
 */
static inline uint32_t
fuxi_unixnu_te_observe(struct fuxi_unixnu_te *te, uint64_t *when)
{
	uint32_t pos = fuxi_unixnu_te_pos(te, te->read_line() & 0xffffu);
	uint64_t now = te->now();
	if (pos > 0 && pos + 1 < te->span) {
		uint64_t back = ((uint64_t)pos * te->tpl) >> 16;
		if (now > back) {
			te->anchor = now - back;
		}
	}
	*when = now;
	return pos;
}

/*
 * Next software vsync deadline: te->early before the first window opening at
 * least half a period away, and no earlier than te->interval (the plain tick
 * period) minus half a period, so a faster counter never ticks faster than
 * the plain tick (a 120 Hz counter keeps a 60 Hz tick). No phase yet: that
 * distance from now.
 */
static inline uint64_t
fuxi_unixnu_te_tick(struct fuxi_unixnu_te *te)
{
	uint64_t now;
	(void)fuxi_unixnu_te_observe(te, &now);
	uint64_t from = now + (te->interval > te->period ? te->interval - te->period / 2 : te->period / 2);
	if (te->anchor == 0 || te->anchor > now) {
		return from;
	}
	uint64_t open = te->anchor + (((uint64_t)te->start * te->tpl) >> 16);
	if (open < from) {
		open += (from - open + te->period - 1) / te->period * te->period;
	}
	return open - te->early > now ? open - te->early : from;
}

/*
 * Wait until the counter is inside the window, at most te->budget: sleep to
 * te->guard before the predicted opening, then poll. Counts a hit or a miss;
 * FUXI_UNIXNU_TE_STREAK misses in a row set disabled. Returns the time waited.
 */
static inline uint64_t
fuxi_unixnu_te_gate(struct fuxi_unixnu_te *te)
{
	uint64_t begin = te->now(), now = begin, end = begin + te->budget;
	for (;;) {
		uint32_t pos = fuxi_unixnu_te_observe(te, &now);
		if (fuxi_unixnu_te_inside(te, pos)) {
			te->hits++;
			te->streak = 0;
			break;
		}
		if (now >= end) {
			te->misses++;
			if (++te->streak >= FUXI_UNIXNU_TE_STREAK) {
				te->disabled = 1;
			}
			break;
		}
		uint64_t target = fuxi_unixnu_te_target(te, now);
		if (target > end) {
			target = end;
		}
		/* Hysteresis: once within 1.5 guard, poll; a re-anchor jitter of a line never re-sleeps. */
		if (target > now + te->guard + te->guard / 2) {
			te->sleep_until(target - te->guard);
		}
	}
	uint64_t waited = now - begin;
	te->wait_sum += waited;
	if (waited > te->wait_max) {
		te->wait_max = waited;
	}
	return waited;
}
#endif
