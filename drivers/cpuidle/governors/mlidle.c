// SPDX-License-Identifier: GPL-2.0

#include <linux/cpuidle.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/sched/cputime.h>
#include "../../kernel/sched/sched.h"
#include <linux/sched/clock.h>
#include <linux/sched/topology.h>
#include <linux/tick.h>
#include "final_8d_mlidle_forest.h"

#define NR_FEATURES 7

/*
 * The PULSE value is added to metrics when they grow and the DECAY_SHIFT value
 * is used for decreasing metrics on a regular basis.
 */
#define PULSE		1024
#define DECAY_SHIFT	3

/*
 * Number of the most recent idle duration values to take into consideration for
 * the detection of recent early wakeup patterns.
 */
#define NR_RECENT	9

/**
 * struct mlidle_bin - Metrics used by the mlidle cpuidle governor.
 * @intercepts: The "intercepts" metric.
 * @hits: The "hits" metric.
 * @recent: The number of recent "intercepts".
 */
struct mlidle_bin {
	unsigned int intercepts;
	unsigned int hits;
	unsigned int recent;
};

/**
 * struct mlidle_cpu - CPU data used by the mlidle cpuidle governor.
 * @time_span_ns: Time between idle state selection and post-wakeup update.
 * @sleep_length_ns: Time till the closest timer event (at the selection time).
 * @state_bins: Idle state data bins for this CPU.
 * @total: Grand total of the "intercepts" and "hits" metrics for all bins.
 * @next_recent_idx: Index of the next @recent_idx entry to update.
 * @recent_idx: Indices of bins corresponding to recent "intercepts".
 */
struct mlidle_cpu {
	s64 time_span_ns;
	s64 sleep_length_ns;
	struct mlidle_bin state_bins[CPUIDLE_STATE_MAX];
	unsigned int total;
	int next_recent_idx;
	int recent_idx[NR_RECENT];
	unsigned long max_cap;
  u32 rq_cpu_time;
  u32 ttwu_count;
  u64 sleep_id;
};

static DEFINE_PER_CPU(struct mlidle_cpu, mlidle_cpus);

/**
 * mlidle_update - Update CPU metrics after wakeup.
 * @drv: cpuidle driver containing state data.
 * @dev: Target CPU.
 */
static void mlidle_update(struct cpuidle_driver *drv, struct cpuidle_device *dev)
{
	struct mlidle_cpu *cpu_data = per_cpu_ptr(&mlidle_cpus, dev->cpu);
	int i, idx_timer = 0, idx_duration = 0, hit = -1;
	u64 measured_ns;

	if (cpu_data->time_span_ns >= cpu_data->sleep_length_ns) {
		/*
		 * One of the safety nets has triggered or the wakeup was close
		 * enough to the closest timer event expected at the idle state
		 * selection time to be discarded.
		 */
		measured_ns = U64_MAX;
	} else {
		u64 lat_ns = drv->states[dev->last_state_idx].exit_latency_ns;

		/*
		 * The computations below are to determine whether or not the
		 * (saved) time till the next timer event and the measured idle
		 * duration fall into the same "bin", so use last_residency_ns
		 * for that instead of time_span_ns which includes the cpuidle
		 * overhead.
		 */
		measured_ns = dev->last_residency_ns;
		/*
		 * The delay between the wakeup and the first instruction
		 * executed by the CPU is not likely to be worst-case every
		 * time, so take 1/2 of the exit latency as a very rough
		 * approximation of the average of it.
		 */
		if (measured_ns >= lat_ns)
			measured_ns -= lat_ns / 2;
		else
			measured_ns /= 2;
	}

	cpu_data->total = 0;

	/*
	 * Decay the "hits" and "intercepts" metrics for all of the bins and
	 * find the bins that the sleep length and the measured idle duration
	 * fall into.
	 */
	for (i = 0; i < drv->state_count; i++) {
		s64 target_residency_ns = drv->states[i].target_residency_ns;
		struct mlidle_bin *bin = &cpu_data->state_bins[i];

		bin->hits -= bin->hits >> DECAY_SHIFT;
		bin->intercepts -= bin->intercepts >> DECAY_SHIFT;

		cpu_data->total += bin->hits + bin->intercepts;

		if (target_residency_ns <= cpu_data->sleep_length_ns) {
			idx_timer = i;
			if (target_residency_ns <= measured_ns)
				idx_duration = i;
		}
	}

	i = cpu_data->next_recent_idx++;
	if (cpu_data->next_recent_idx >= NR_RECENT)
		cpu_data->next_recent_idx = 0;

	if (cpu_data->recent_idx[i] >= 0)
		cpu_data->state_bins[cpu_data->recent_idx[i]].recent--;

	/*
	 * If the measured idle duration falls into the same bin as the sleep
	 * length, this is a "hit", so update the "hits" metric for that bin.
	 * Otherwise, update the "intercepts" metric for the bin fallen into by
	 * the measured idle duration.
	 */
	if (idx_timer == idx_duration) {
		cpu_data->state_bins[idx_timer].hits += PULSE;
		cpu_data->recent_idx[i] = -1;
    hit = 1;
	} else {
		cpu_data->state_bins[idx_duration].intercepts += PULSE;
		cpu_data->state_bins[idx_duration].recent++;
		cpu_data->recent_idx[i] = idx_duration;
    hit = 0;
	}

  trace_printk(
      "cpu=%d sleep_id=%llu hit=%d timer_state=%d duration_state=%d sleep_length=%lld time_span=%lld measured=%lld\n",
      dev->cpu, cpu_data->sleep_id, hit, idx_timer, idx_duration, cpu_data->sleep_length_ns, cpu_data->time_span_ns, measured_ns
  );

	cpu_data->total += PULSE;
}

/**
 * mlidle_find_shallower_state - Find shallower idle state matching given duration.
 * @drv: cpuidle driver containing state data.
 * @dev: Target CPU.
 * @state_idx: Index of the capping idle state.
 * @duration_ns: Idle duration value to match.
 * @no_poll: Don't consider polling states.
 */
static int mlidle_find_shallower_state(struct cpuidle_driver *drv,
                    struct cpuidle_device *dev, int state_idx,
                    s64 duration_ns, bool no_poll)
{
    int i;

    for (i = state_idx - 1; i >= 0; i--) {
        if (dev->states_usage[i].disable ||
                (no_poll && drv->states[i].flags & CPUIDLE_FLAG_POLLING))
            continue;

        state_idx = i;
        if (drv->states[i].target_residency_ns <= duration_ns)
            break;
    }
    return state_idx;
}

/**
 * mlidle_select - Selects the next idle state to enter.
 * @drv: cpuidle driver containing state data.
 * @dev: Target CPU.
 * @stop_tick: Indication on whether or not to stop the scheduler tick.
 */
static int mlidle_select(struct cpuidle_driver *drv, struct cpuidle_device *dev,
		      bool *stop_tick)
{
	struct mlidle_cpu *cpu_data = per_cpu_ptr(&mlidle_cpus, dev->cpu);
	ktime_t delta_tick;
	s64 duration_ns;
	int idx;
  struct rq *rq = cpu_rq(dev->cpu);
  s64 features[NR_FEATURES];
  u32 util, rq_cpu_time_delta, ttwu_count_delta;

	if (dev->last_state_idx >= 0) {
		mlidle_update(drv, dev);
		dev->last_state_idx = -1;
	}

  cpu_data->sleep_id++;
  if (cpu_data->sleep_id == U64_MAX)
    cpu_data->sleep_id = 0;

	cpu_data->time_span_ns = local_clock();

	duration_ns = tick_nohz_get_sleep_length(&delta_tick);
	cpu_data->sleep_length_ns = duration_ns;

  util = sched_cpu_util(dev->cpu);
  rq_cpu_time_delta = rq->rq_cpu_time - cpu_data->rq_cpu_time;
  ttwu_count_delta = rq->ttwu_count - cpu_data->ttwu_count;

  /* save previous rq_cpu_time & ttwu_count */
  cpu_data->rq_cpu_time = rq->rq_cpu_time;
  cpu_data->ttwu_count = rq->ttwu_count;

  trace_printk(
      "cpu=%d sleep_id=%llu sleep_length_ns=%lld max_cap=%lu util=%u s0hit=%u s0int=%u s0rec=%u s1hit=%u rq_cpu_time=%u ttwu_count=%u\n",
      dev->cpu, cpu_data->sleep_id, duration_ns, cpu_data->max_cap, util,
      cpu_data->state_bins[0].hits, cpu_data->state_bins[0].intercepts, cpu_data->state_bins[0].recent,
      cpu_data->state_bins[1].hits, rq_cpu_time_delta, ttwu_count_delta
  );

	/* Check if there is any choice in the first place. */
	if (drv->state_count < 2) {
		idx = 0;
		goto end;
	}
	if (!dev->states_usage[0].disable) {
		idx = 0;
		if (drv->states[1].target_residency_ns > duration_ns)
			goto end;
	}

  /* Prepare the features array for the ML model */
  features[0] = cpu_data->sleep_length_ns;
  features[1] = util;
  features[2] = cpu_data->max_cap;
  features[3] = cpu_data->state_bins[0].intercepts;
  features[4] = cpu_data->state_bins[1].hits;
  features[5] = rq_cpu_time_delta;
  features[6] = ttwu_count_delta;

  /* Use the trained model to predict the idle state */
  idx = mlidle_forest_predict(features, NR_FEATURES);
  trace_printk("cpu=%d sleep_id=%llu predicted_state=%d\n", dev->cpu, cpu_data->sleep_id, idx);

end:
	/*
	 * Don't stop the tick if the selected state is a polling one or if the
	 * expected idle duration is shorter than the tick period length.
	 */
	if (((drv->states[idx].flags & CPUIDLE_FLAG_POLLING) ||
	    duration_ns < TICK_NSEC) && !tick_nohz_tick_stopped()) {
		*stop_tick = false;

		/*
		 * The tick is not going to be stopped, so if the target
		 * residency of the state to be returned is not within the time
		 * till the closest timer including the tick, try to correct
		 * that.
		 */
		if (idx > 0 &&
		    drv->states[idx].target_residency_ns > delta_tick)
			idx = mlidle_find_shallower_state(drv, dev, idx, delta_tick, false);
	}

	return idx;
}

/**
 * mlidle_reflect - Note that governor data for the CPU need to be updated.
 * @dev: Target CPU.
 * @state: Entered state.
 */
static void mlidle_reflect(struct cpuidle_device *dev, int state)
{
	struct mlidle_cpu *cpu_data = per_cpu_ptr(&mlidle_cpus, dev->cpu);

	dev->last_state_idx = state;
	/*
	 * If the wakeup was not "natural", but triggered by one of the safety
	 * nets, assume that the CPU might have been idle for the entire sleep
	 * length time.
	 */
	if (dev->poll_time_limit ||
	    (tick_nohz_idle_got_tick() && cpu_data->sleep_length_ns > TICK_NSEC)) {
		dev->poll_time_limit = false;
		cpu_data->time_span_ns = cpu_data->sleep_length_ns;
	} else {
		cpu_data->time_span_ns = local_clock() - cpu_data->time_span_ns;
	}

  trace_printk(
      "cpu=%d sleep_id=%llu sleep_length_ns=%lld time_span_ns=%lld state=%d\n",
      dev->cpu, cpu_data->sleep_id, cpu_data->sleep_length_ns, cpu_data->time_span_ns, state
  );
}

/**
 * mlidle_enable_device - Initialize the governor's data for the target CPU.
 * @drv: cpuidle driver (not used).
 * @dev: Target CPU.
 */
static int mlidle_enable_device(struct cpuidle_driver *drv,
			     struct cpuidle_device *dev)
{
	struct mlidle_cpu *cpu_data = per_cpu_ptr(&mlidle_cpus, dev->cpu);
	unsigned long max_capacity = arch_scale_cpu_capacity(dev->cpu);
  struct rq *rq = cpu_rq(dev->cpu);
	int i;

	memset(cpu_data, 0, sizeof(*cpu_data));
  cpu_data->max_cap = max_capacity;
  cpu_data->sleep_id = 0;
  cpu_data->rq_cpu_time = rq->rq_cpu_time;
  cpu_data->ttwu_count = rq->ttwu_count;

	for (i = 0; i < NR_RECENT; i++)
		cpu_data->recent_idx[i] = -1;

	return 0;
}

static struct cpuidle_governor mlidle_governor = {
	.name =		"mlidle",
	.rating =	19,
	.enable =	mlidle_enable_device,
	.select =	mlidle_select,
	.reflect =	mlidle_reflect,
};

static int __init mlidle_governor_init(void)
{
	return cpuidle_register_governor(&mlidle_governor);
}

postcore_initcall(mlidle_governor_init);
