// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023-2024 Sultan Alsawaf <sultan@kerneltoast.com>.
 */

/**
 * DOC: Capacity Aware Superset Scheduler (CASS) description
 *
 * The Capacity Aware Superset Scheduler (CASS) optimizes runqueue selection of
 * CFS tasks. By using CPU capacity as a basis for comparing the relative
 * utilization between different CPUs, CASS fairly balances load across CPUs of
 * varying capacities. This results in improved multi-core performance,
 * especially when CPUs are overutilized because CASS doesn't clip a CPU's
 * utilization when it eclipses the CPU's capacity.
 *
 * As a superset of capacity aware scheduling, CASS implements a hierarchy of
 * criteria to determine the better CPU to wake a task upon between CPUs that
 * have the same relative utilization. This way, single-core performance,
 * latency, and cache affinity are all optimized where possible.
 *
 * CASS doesn't feature explicit energy awareness but its basic load balancing
 * principle results in decreased overall energy, often better than what is
 * possible with explicit energy awareness. By fairly balancing load based on
 * relative utilization, all CPUs are kept at their lowest P-state necessary to
 * satisfy the overall load at any given moment.
 */

struct cass_cpu_cand {
	int cpu;
	unsigned int exit_lat;
	unsigned long cap, cap_max, cap_no_therm, cap_orig;
	unsigned long eff_util, hard_util, util;
};

/*
 * cass_cpu_util - Compute utilization and available capacity of a candidate CPU
 * @c:       Candidate CPU structure
 * @this_cpu: Current CPU executing the function
 * @sync:    Whether to synchronize utilization with the current task
 *
 * This function calculates the effective utilization of a CPU candidate,
 * taking into account real-time (RT), deadline (DL), and IRQ utilization.
 * It also adjusts the utilization if the candidate CPU is the current CPU
 * and the current task is not a real-time task.
 */
static __always_inline void cass_cpu_util(struct cass_cpu_cand *c, int this_cpu, bool sync)
{
	struct rq *rq = cpu_rq(c->cpu);
	struct cfs_rq *cfs_rq = &rq->cfs;
	unsigned long est = READ_ONCE(cfs_rq->avg.util_est.enqueued);

	/* Get utilization from CFS runqueue */
	c->util = READ_ONCE(cfs_rq->avg.util_avg);
	if (sched_feat(UTIL_EST) && est > c->util) {
		sync = false;
		c->util = est;
	}

	/* Subtract the current task's utilization if running on this CPU */
	if (sync && c->cpu == this_cpu && !rt_task(current))
		c->util -= min(c->util, task_util(current));

	/* Compute real-time, deadline, and IRQ utilization */
	c->hard_util = cpu_util_rt(rq) + cpu_util_dl(rq) + cpu_util_irq(rq);

	/* Compute available capacity considering real-time load */
	c->cap = c->cap_max - min(c->hard_util, c->cap_max - 1);
	c->cap_no_therm = c->cap_orig - min(c->hard_util, c->cap_orig - 1);
}

/*
 * cass_prime_cpu - Determine if a CPU is a prime candidate
 * @c: Candidate CPU structure
 *
 * This function returns true if the CPU is the last CPU in the system
 * and its capacity differs from the second-to-last CPU, making it an
 * exceptional candidate.
 */
static __always_inline bool cass_prime_cpu(const struct cass_cpu_cand *c)
{
	return (c->cpu == nr_cpu_ids - 1) && 
	       (arch_scale_cpu_capacity(nr_cpu_ids - 2) != SCHED_CAPACITY_SCALE);
}

/*
 * cass_cpu_better - Compare two CPUs and determine the better candidate
 * @a: First CPU candidate
 * @b: Second CPU candidate
 * @p_util: Predicted utilization of the task
 * @this_cpu: Current executing CPU
 * @prev_cpu: CPU on which the task was last running
 * @sync: Whether the task wakeup is synchronized
 *
 * This function compares two CPU candidates and determines which one
 * is more suitable for scheduling the task.
 */
static __always_inline bool cass_cpu_better(const struct cass_cpu_cand *a,
					    const struct cass_cpu_cand *b, unsigned long p_util,
					    int this_cpu, int prev_cpu, bool sync)
{
#define cass_cmp(a, b) ((a) - (b))
	long res;

	/* Compare efficiency based on utilization and capacity */
	if ((res = cass_cmp(b->eff_util / b->cap_max, a->eff_util / a->cap_max)) ||
	    (b->eff_util > b->cap_max && a->eff_util > a->cap_max &&
	     (res = cass_cmp(b->eff_util * SCHED_CAPACITY_SCALE / b->cap_max,
			     a->eff_util * SCHED_CAPACITY_SCALE / a->cap_max))) ||
	    /* Check if the CPU can fit the task's predicted utilization */
	    (res = cass_cmp(fits_capacity(p_util, a->cap_max),
			    fits_capacity(p_util, b->cap_max))) ||
	    /* Prefer prime CPUs */
	    (res = cass_cmp(cass_prime_cpu(b), cass_prime_cpu(a))) ||
	    /* Prefer CPUs with lower utilization */
	    (res = cass_cmp(b->util, a->util)) ||
	    /* Prefer CPUs with lower exit latency */
	    (res = cass_cmp(!!a->exit_lat, !!b->exit_lat)) ||
	    /* Prefer current CPU if sync is enabled */
	    (sync && ((res = cass_cmp(a->cpu, this_cpu)) || !cass_cmp(b->cpu, this_cpu))) ||
	    /* Prefer CPUs with higher capacity */
	    (res = cass_cmp(a->cap, b->cap)) ||
	    (res = cass_cmp(b->exit_lat, a->exit_lat)) ||
	    /* Prefer previous CPU to minimize migration */
	    (res = cass_cmp(a->cpu, prev_cpu)) ||
	    /* Prefer CPUs that share cache with previous CPU */
	    (res = cass_cmp(cpus_share_cache(a->cpu, prev_cpu),
			    cpus_share_cache(b->cpu, prev_cpu))))
		return res > 0;

	return false;
}

/*
 * cass_best_cpu - Find the best CPU for a task
 * @p: Task structure
 * @prev_cpu: Previous CPU on which the task was running
 * @sync: Whether the task wakeup is synchronized
 * @rt: Whether the task is a real-time task
 *
 * This function selects the most efficient CPU for the given task.
 */
static int cass_best_cpu(struct task_struct *p, int prev_cpu, bool sync, bool rt)
{
	struct cass_cpu_cand cands[2], *best = cands;
	int this_cpu = raw_smp_processor_id();
	unsigned long p_util = rt ? 0 : task_util_est(p);
	unsigned long uc_min = uclamp_eff_value(p, UCLAMP_MIN);
	bool has_idle = false;
	int cidx = 0, cpu;

	for_each_cpu_and(cpu, p->cpus_ptr, cpu_active_mask) {
		struct cass_cpu_cand *curr = &cands[cidx];
		struct rq *rq = cpu_rq(cpu);

		curr->cpu = cpu;
		curr->cap_orig = arch_scale_cpu_capacity(cpu);
		curr->cap_max = curr->cap_orig - thermal_load_avg(rq);

		if (curr->cap_max < uc_min && curr->cap_max < best->cap_max)
			continue;

		/* Identify idle CPUs */
		if ((sync && cpu == this_cpu && rq->nr_running == 1) ||
		    available_idle_cpu(cpu) || sched_idle_cpu(cpu)) {
			if (!uc_min && !cass_prime_cpu(curr)) {
				if (!has_idle)
					best = curr;
				has_idle = true;
			}
			curr->exit_lat = 1 + (idle_get_state(rq) ? idle_get_state(rq)->exit_latency : 0);
		} else {
			if (has_idle)
				continue;
			curr->exit_lat = 0;
		}

		cass_cpu_util(curr, this_cpu, sync);
		if (cpu != task_cpu(p))
			curr->util += p_util;

		curr->eff_util = max(curr->util + curr->hard_util, uc_min);
		curr->util = max(curr->util * SCHED_CAPACITY_SCALE / curr->cap_no_therm, uc_min);

		if (best == curr || cass_cpu_better(curr, best, p_util, this_cpu, prev_cpu, sync)) {
			best = curr;
			cidx ^= 1;
		}
	}

	return best->cpu;
}

/* Entry point for fair and real-time task selection */
static int cass_select_task_rq_fair(struct task_struct *p, int prev_cpu, int sd_flag, int wake_flags)
{
	return cass_best_cpu(p, prev_cpu, wake_flags & WF_SYNC, false);
}

int cass_select_task_rq_rt(struct task_struct *p, int prev_cpu, int sd_flag, int wake_flags)
{
	return cass_best_cpu(p, prev_cpu, wake_flags & WF_SYNC, true);
}
