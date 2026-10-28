/* Pure helpers of osfmk/arm64/fuxi_idle.c (patch 0105), shared with the host
 * test tools/test-fuxi-idle.py. No kernel headers: integers in, text out.
 *
 * Ported from HackiOS hackios-xnu-kernel xnu/files/bsd/dev/redwood_cpufreq.c
 * (cpuload=MS: busy share = wall time minus idle time) (APSL 2.0).
 * SPDX-License-Identifier: APSL-2.0
 */
#ifndef FUXI_IDLE_LOGIC_H
#define FUXI_IDLE_LOGIC_H

#define FI_MAX_CPUS     8
#define FI_PERIOD_MAX   3600u   /* seconds */

/* One CPU at one instant. Times are mach_absolute_time() units. */
struct fi_sample {
	uint64_t idle, user, sys;       /* recount: idle time, user and system run time */
	uint64_t irq, ipi, tmr;         /* cpu_stat: irq_ex_cnt, ipi_cnt, timer_cnt */
	int valid;                      /* the processor was registered when sampled */
};

/* fuxicpuload=N: N seconds between lines, 0 = off, capped at an hour. */
static inline unsigned
fi_period(unsigned seconds)
{
	return seconds > FI_PERIOD_MAX ? FI_PERIOD_MAX : seconds;
}

static inline uint64_t
fi_delta(uint64_t now, uint64_t then)
{
	return now >= then ? now - then : 0;
}

/* Busy percent of a wall interval from the idle time in it (interrupts count as busy). */
static inline unsigned
fi_busy_pct(uint64_t wall, uint64_t idle)
{
	if (!wall) {
		return 0;
	}
	if (idle >= wall) {
		return 0;
	}
	return (unsigned)(((wall - idle) * 100 + wall / 2) / wall);
}

/* Share of the wall interval, in percent, of a run time (user or system). */
static inline unsigned
fi_share_pct(uint64_t wall, uint64_t part)
{
	if (!wall) {
		return 0;
	}
	if (part >= wall) {
		return 100;
	}
	return (unsigned)((part * 100 + wall / 2) / wall);
}

/* Events per second over wall_ms milliseconds. */
static inline uint64_t
fi_rate(uint64_t events, uint64_t wall_ms)
{
	if (!wall_ms) {
		return 0;
	}
	return (events * 1000 + wall_ms / 2) / wall_ms;
}

static inline void
fi_append(char *buf, unsigned size, unsigned *at, const char *text)
{
	while (*text && *at + 1 < size) {
		buf[(*at)++] = *text++;
	}
	buf[*at < size ? *at : size - 1] = 0;
}

static inline void
fi_append_u(char *buf, unsigned size, unsigned *at, uint64_t v)
{
	char digits[24];
	unsigned n = 0;
	do {
		digits[n++] = (char)('0' + v % 10);
		v /= 10;
	} while (v && n < sizeof(digits));
	char out[24];
	for (unsigned i = 0; i < n; i++) {
		out[i] = digits[n - 1 - i];
	}
	out[n] = 0;
	fi_append(buf, size, at, out);
}

/*
 * One log line:
 *   FUXI IDLE: cpuload 10s up 600s avg 9% irq 5200 ipi 8400 tmr 3100 | c0 23% u5 s17 i812 p1934 t377 | c1 - | ...
 * avg is over the CPUs that were valid at both samples; irq/ipi/tmr totals are
 * per second. A CPU that came online between the samples prints "-".
 * Returns the line length (the buffer is always terminated).
 */
static inline unsigned
fi_format(char *buf, unsigned size, unsigned period, uint64_t up_s, uint64_t wall, uint64_t wall_ms,
    const struct fi_sample *prev, const struct fi_sample *cur, unsigned ncpu)
{
	unsigned at = 0, counted = 0, busy_sum = 0;
	uint64_t irq = 0, ipi = 0, tmr = 0;
	if (!size) {
		return 0;
	}
	buf[0] = 0;
	if (ncpu > FI_MAX_CPUS) {
		ncpu = FI_MAX_CPUS;
	}
	for (unsigned c = 0; c < ncpu; c++) {
		if (!prev[c].valid || !cur[c].valid) {
			continue;
		}
		busy_sum += fi_busy_pct(wall, fi_delta(cur[c].idle, prev[c].idle));
		irq += fi_delta(cur[c].irq, prev[c].irq);
		ipi += fi_delta(cur[c].ipi, prev[c].ipi);
		tmr += fi_delta(cur[c].tmr, prev[c].tmr);
		counted++;
	}
	fi_append(buf, size, &at, "FUXI IDLE: cpuload ");
	fi_append_u(buf, size, &at, period);
	fi_append(buf, size, &at, "s up ");
	fi_append_u(buf, size, &at, up_s);
	fi_append(buf, size, &at, "s avg ");
	fi_append_u(buf, size, &at, counted ? (busy_sum + counted / 2) / counted : 0);
	fi_append(buf, size, &at, "% irq ");
	fi_append_u(buf, size, &at, fi_rate(irq, wall_ms));
	fi_append(buf, size, &at, " ipi ");
	fi_append_u(buf, size, &at, fi_rate(ipi, wall_ms));
	fi_append(buf, size, &at, " tmr ");
	fi_append_u(buf, size, &at, fi_rate(tmr, wall_ms));
	for (unsigned c = 0; c < ncpu; c++) {
		fi_append(buf, size, &at, " | c");
		fi_append_u(buf, size, &at, c);
		if (!prev[c].valid || !cur[c].valid) {
			fi_append(buf, size, &at, " -");
			continue;
		}
		fi_append(buf, size, &at, " ");
		fi_append_u(buf, size, &at, fi_busy_pct(wall, fi_delta(cur[c].idle, prev[c].idle)));
		fi_append(buf, size, &at, "% u");
		fi_append_u(buf, size, &at, fi_share_pct(wall, fi_delta(cur[c].user, prev[c].user)));
		fi_append(buf, size, &at, " s");
		fi_append_u(buf, size, &at, fi_share_pct(wall, fi_delta(cur[c].sys, prev[c].sys)));
		fi_append(buf, size, &at, " i");
		fi_append_u(buf, size, &at, fi_rate(fi_delta(cur[c].irq, prev[c].irq), wall_ms));
		fi_append(buf, size, &at, " p");
		fi_append_u(buf, size, &at, fi_rate(fi_delta(cur[c].ipi, prev[c].ipi), wall_ms));
		fi_append(buf, size, &at, " t");
		fi_append_u(buf, size, &at, fi_rate(fi_delta(cur[c].tmr, prev[c].tmr), wall_ms));
	}
	return at;
}

/*
 * IPI diagnostics (fuxicpuload) and the idle L1 flush (fuxinoidleflush).
 * FI_SIGP_* mirror osfmk/arm/cpu_internal.h (fuxi_idle.c asserts they match).
 */
#define FI_SIGP_DEC             0x00000001u
#define FI_SIGP_XCALL           0x00000004u
#define FI_SIGP_AST             0x00000008u
#define FI_SIGP_XCALL_IMM       0x00000200u
#define FI_SIGP_DISABLED        0x80000000u

/* Per-CPU event counters; they only increase. Times are mach_absolute_time() units. */
struct fi_xcount {
	uint64_t nop;           /* IPIs with no signal bit: a SIGPnop idle kick (or a foreign SGI) */
	uint64_t ast, xc, dec;  /* IPIs carrying SIGPast, SIGPxcall/xcallImm, SIGPdec */
	uint64_t other;         /* IPIs carrying any other bit (debug, kppet) */
	uint64_t sgi;           /* SGIs other than SWEET_IPI_SGI (also counted as nop) */
	uint64_t idles;         /* cpu_idle() WFI entries */
	uint64_t flush;         /* time spent in platform_cache_idle_enter() */
};

/* Classify one IPI by the cpu_signal bits the handler found pending. */
static inline void
fi_classify(struct fi_xcount *c, unsigned signal)
{
	signal &= ~FI_SIGP_DISABLED;
	if (!signal) {
		c->nop++;
		return;
	}
	if (signal & FI_SIGP_AST) {
		c->ast++;
	}
	if (signal & (FI_SIGP_XCALL | FI_SIGP_XCALL_IMM)) {
		c->xc++;
	}
	if (signal & FI_SIGP_DEC) {
		c->dec++;
	}
	if (signal & ~(FI_SIGP_AST | FI_SIGP_XCALL | FI_SIGP_XCALL_IMM | FI_SIGP_DEC)) {
		c->other++;
	}
}

static inline void
fi_header(char *buf, unsigned size, unsigned *at, const char *what, unsigned period)
{
	buf[0] = 0;
	*at = 0;
	fi_append(buf, size, at, "FUXI IDLE: ");
	fi_append(buf, size, at, what);
	fi_append(buf, size, at, " ");
	fi_append_u(buf, size, at, period);
	fi_append(buf, size, at, "s");
}

static inline void
fi_field(char *buf, unsigned size, unsigned *at, const char *key, uint64_t v)
{
	fi_append(buf, size, at, key);
	fi_append_u(buf, size, at, v);
}

/*
 * Received IPIs per second by kind, one group per CPU:
 *   FUXI IDLE: ipirx 10s | c0 n2900 a41 x0 d0 o0 g0 | c1 - | ...
 * n = no signal bit (idle kick), a = AST, x = xcall, d = decrementer,
 * o = other bits, g = foreign SGI id (a subset of n). One IPI can carry
 * several bits, so n+a+x+d+o can exceed the p rate of the cpuload line.
 */
static inline unsigned
fi_format_ipirx(char *buf, unsigned size, unsigned period, uint64_t wall_ms,
    const struct fi_xcount *prev, const struct fi_xcount *cur, const int *valid, unsigned ncpu)
{
	unsigned at = 0;
	if (!size) {
		return 0;
	}
	fi_header(buf, size, &at, "ipirx", period);
	if (ncpu > FI_MAX_CPUS) {
		ncpu = FI_MAX_CPUS;
	}
	for (unsigned c = 0; c < ncpu; c++) {
		fi_field(buf, size, &at, " | c", c);
		if (!valid[c]) {
			fi_append(buf, size, &at, " -");
			continue;
		}
		fi_field(buf, size, &at, " n", fi_rate(fi_delta(cur[c].nop, prev[c].nop), wall_ms));
		fi_field(buf, size, &at, " a", fi_rate(fi_delta(cur[c].ast, prev[c].ast), wall_ms));
		fi_field(buf, size, &at, " x", fi_rate(fi_delta(cur[c].xc, prev[c].xc), wall_ms));
		fi_field(buf, size, &at, " d", fi_rate(fi_delta(cur[c].dec, prev[c].dec), wall_ms));
		fi_field(buf, size, &at, " o", fi_rate(fi_delta(cur[c].other, prev[c].other), wall_ms));
		fi_field(buf, size, &at, " g", fi_rate(fi_delta(cur[c].sgi, prev[c].sgi), wall_ms));
	}
	return at;
}

/* Ticks to nanoseconds with the interval's own ratio (no timebase constant). */
static inline uint64_t
fi_ticks_ns(uint64_t ticks, uint64_t wall, uint64_t wall_ns)
{
	if (!wall) {
		return 0;
	}
	return (uint64_t)(((unsigned __int128)ticks * wall_ns + wall / 2) / wall);
}

/*
 * WFI idle entries per second and the L1 D-cache flush done before each:
 *   FUXI IDLE: idleflush 10s on | c0 n3007 a812 m2 | ...
 * n = idle entries per second, a = average ns per flush, m = flush time in
 * per mille of the interval. "off" (fuxinoidleflush=1) prints n only.
 */
static inline unsigned
fi_format_flush(char *buf, unsigned size, unsigned period, int flushing, uint64_t wall, uint64_t wall_ns,
    const struct fi_xcount *prev, const struct fi_xcount *cur, const int *valid, unsigned ncpu)
{
	unsigned at = 0;
	uint64_t wall_ms = wall_ns / 1000000u;
	if (!size) {
		return 0;
	}
	fi_header(buf, size, &at, "idleflush", period);
	fi_append(buf, size, &at, flushing ? " on" : " off");
	if (ncpu > FI_MAX_CPUS) {
		ncpu = FI_MAX_CPUS;
	}
	for (unsigned c = 0; c < ncpu; c++) {
		fi_field(buf, size, &at, " | c", c);
		if (!valid[c]) {
			fi_append(buf, size, &at, " -");
			continue;
		}
		uint64_t n = fi_delta(cur[c].idles, prev[c].idles);
		uint64_t t = fi_delta(cur[c].flush, prev[c].flush);
		fi_field(buf, size, &at, " n", fi_rate(n, wall_ms));
		if (flushing) {
			fi_field(buf, size, &at, " a", n ? fi_ticks_ns(t, wall, wall_ns) / n : 0);
			fi_field(buf, size, &at, " m", !wall ? 0 : t >= wall ? 1000 : (t * 1000 + wall / 2) / wall);
		}
	}
	return at;
}

/*
 * Scheduler IPI attribution: who (task of the thread running on the CPU that
 * asked for the IPI) woke whom (task of the thread the IPI is for; -1 = no
 * thread, e.g. a rebalance). Fixed tables, linear scan; overflow is "rest".
 */
#define FI_WAKE_SLOTS   32
#define FI_WAKE_MERGE   64
#define FI_WAKE_TOP     4       /* keeps the line near 256 bytes */
#define FI_NAME         17      /* MAXCOMLEN + 1 */

struct fi_wake {
	int32_t waker, woken;
	uint64_t count;
};

/* Add n to the (waker, woken) slot; returns 0 when the table is full. */
static inline int
fi_wake_add(struct fi_wake *t, unsigned slots, int32_t waker, int32_t woken, uint64_t n)
{
	for (unsigned i = 0; i < slots; i++) {
		if (!t[i].count) {
			t[i].waker = waker;
			t[i].woken = woken;
			t[i].count = n;
			return 1;
		}
		if (t[i].waker == waker && t[i].woken == woken) {
			t[i].count += n;
			return 1;
		}
	}
	return 0;
}

/* Indices of up to k largest slots, largest first (ties: lower index). Returns how many. */
static inline unsigned
fi_wake_top(const struct fi_wake *t, unsigned slots, unsigned *out, unsigned k)
{
	unsigned found = 0;
	while (found < k) {
		unsigned best = slots;
		for (unsigned i = 0; i < slots; i++) {
			int taken = 0;
			for (unsigned j = 0; j < found; j++) {
				taken |= out[j] == i;
			}
			if (!taken && t[i].count && (best == slots || t[i].count > t[best].count)) {
				best = i;
			}
		}
		if (best == slots) {
			break;
		}
		out[found++] = best;
	}
	return found;
}

static inline void
fi_append_task(char *buf, unsigned size, unsigned *at, const char *name, int32_t pid)
{
	fi_append(buf, size, at, name[0] ? name : "?");
	fi_append(buf, size, at, "[");
	if (pid < 0) {
		fi_append(buf, size, at, "-");
	} else {
		fi_append_u(buf, size, at, (uint64_t)pid);
	}
	fi_append(buf, size, at, "]");
}

/*
 *   FUXI IDLE: ipiwake 10s imm 40 idle 9100 def 0 | launchd[1]>mediaserverd[93] 3100 | ... | rest 12
 * Totals by sched_ipi_type (immediate, idle, deferred) and the top pairs,
 * all per second; rest = the other pairs plus those that did not fit.
 * names[2*i] / names[2*i+1] are the waker / woken names of top[i] ("" = unknown).
 */
static inline unsigned
fi_format_wake(char *buf, unsigned size, unsigned period, uint64_t wall_ms, const uint64_t *types,
    const struct fi_wake *t, unsigned slots, const unsigned *top, unsigned ntop,
    const char (*names)[FI_NAME], uint64_t lost)
{
	unsigned at = 0;
	uint64_t shown = 0, all = lost;
	if (!size) {
		return 0;
	}
	for (unsigned i = 0; i < slots; i++) {
		all += t[i].count;
	}
	fi_header(buf, size, &at, "ipiwake", period);
	fi_field(buf, size, &at, " imm ", fi_rate(types[0], wall_ms));
	fi_field(buf, size, &at, " idle ", fi_rate(types[1], wall_ms));
	fi_field(buf, size, &at, " def ", fi_rate(types[2], wall_ms));
	for (unsigned i = 0; i < ntop; i++) {
		const struct fi_wake *w = &t[top[i]];
		fi_append(buf, size, &at, " | ");
		fi_append_task(buf, size, &at, names[2 * i], w->waker);
		fi_append(buf, size, &at, ">");
		fi_append_task(buf, size, &at, names[2 * i + 1], w->woken);
		fi_field(buf, size, &at, " ", fi_rate(w->count, wall_ms));
		shown += w->count;
	}
	fi_field(buf, size, &at, " | rest ", fi_rate(all - shown, wall_ms));
	return at;
}

/* fuxinosleep / fuxixpcid / fuxinoidleflush: a boot-arg value is on only when it is exactly 1. */
static inline int
fi_flag(int present, unsigned value)
{
	return present && value == 1;
}

/* Log the 1st, 2nd, 4th, 8th ... event of a repeating refusal. */
static inline int
fi_log_this(uint64_t count)
{
	return count && !(count & (count - 1));
}

/*
 * fuxiclk=N clock meter (perf_plan.md K3). Every N seconds each online CPU in
 * turn runs a chain of dependent 64-bit adds with interrupts masked and times
 * it with mach_absolute_time(). A510, A715, A710 and X3 retire one dependent
 * add per cycle, so adds per microsecond is the core clock in MHz: the level
 * the bootloader left, or lower when the hardware limiter (LMh) throttles.
 * A run is FI_CLK_PASSES passes over FI_CLK_BLOCK unrolled adds. After a
 * one-pass warm-up a CPU does up to FI_CLK_RUNS timed runs and starts no run
 * once FI_CLK_BUDGET_US have passed, so it stays masked for less than the
 * budget plus one run plus the warm-up: at most 0.51 ms at 307.2 MHz (the
 * lowest SM8550 CPU level), 0.4 ms from 1.31 GHz up, 1.8 ms even at the
 * 19.2 MHz crystal.
 */
#define FI_CLK_BLOCK            1024    /* adds per pass, 4 KB of code; a plain int for .rept */
#define FI_CLK_PASSES           32u     /* passes per run: 32768 adds */
#define FI_CLK_RUNS             16u     /* at most 524288 timed adds per CPU */
#define FI_CLK_BUDGET_US        400u    /* no new run after this much masked time */
#define FI_CLK_LINE             192u    /* one FUXI CLK line, 8 CPUs at 10 digits fit */

/* Time another run: fewer than FI_CLK_RUNS done and the masked time still under budget. */
static inline int
fi_clk_more(unsigned runs, uint64_t elapsed, uint64_t budget)
{
	return runs < FI_CLK_RUNS && elapsed < budget;
}

/* Adds per microsecond, rounded: the clock in MHz at one add per cycle. 0 = nothing timed. */
static inline unsigned
fi_clk_mhz(uint64_t adds, uint64_t ns)
{
	uint64_t mhz;

	if (!ns || adds > UINT64_MAX / 1000) {
		return 0;
	}
	mhz = (adds * 1000 + ns / 2) / ns;
	return mhz > 0xffffffffu ? 0xffffffffu : (unsigned)mhz;
}

/*
 *   FUXI CLK: 60s | c0 1786 | c1 1786 | c2 2803 | c3 2803 | c4 2803 | c5 2803 | c6 3187
 * MHz per logical CPU, the numbering of the cpuload line (salami: c0-c1 A510,
 * c2-c3 A715, c4-c5 A710, c6 X3). 0 (offline, still registering, or the
 * switch to it failed) prints "-". Returns the line length (always terminated).
 */
static inline unsigned
fi_format_clk(char *buf, unsigned size, unsigned period, const unsigned *mhz, unsigned ncpu)
{
	unsigned at = 0;

	if (!size) {
		return 0;
	}
	buf[0] = 0;
	fi_append(buf, size, &at, "FUXI CLK: ");
	fi_append_u(buf, size, &at, period);
	fi_append(buf, size, &at, "s");
	if (ncpu > FI_MAX_CPUS) {
		ncpu = FI_MAX_CPUS;
	}
	for (unsigned c = 0; c < ncpu; c++) {
		fi_field(buf, size, &at, " | c", c);
		if (!mhz[c]) {
			fi_append(buf, size, &at, " -");
			continue;
		}
		fi_field(buf, size, &at, " ", mhz[c]);
	}
	return at;
}

#endif /* FUXI_IDLE_LOGIC_H */
