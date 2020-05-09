#ifndef LITMUS_RT_BANDWIDTH_H
#define LITMUS_RT_BANDWIDTH_H

#include <litmus/preempt.h>

#define TRACE_BANDWIDTH_STATE_CHANGE(x, y, task)				\
	TRACE_TASK(task, "job:%d bw_mask:0x%x %d(%s) -> %d(%s)\n",	\
		    tsk_rt(task)->job_params.job_no, 			\
			tsk_rt(task)->job_params.bw_partitions,	\
			(x), bw_state_name(x),					\
		    (y), bw_state_name(y))

typedef struct  {
	int 			cpu;
	uint32_t 		used_bw; 		/* currently used bw partition */
} cpu_bw_entry_t;

/* lock_cache_partitions
 * lock cp_mask for cpu so that only cpu can use cp_mask
 * NOTE:
 * 1) rt.lock is grabbed by the caller so that
 *    scheduler on diff CPUs do not have race condition
 * 2) We have race condition when user write to /proc/sys
 *    As long as users do not write to /proc/sys, we are safe
 */
void lock_bw_partitions(int cpu, uint32_t cp_mask, struct task_struct *tsk, rt_domain_t *rt);

/* unlock_cache_partitions
 * unlock cp_mask for cpu so that other cpus can use cp_mask
 */
void unlock_bw_partitions(int cpu, uint32_t cp_mask, rt_domain_t *rt);

int count_set_bits(uint32_t bitmask);

#endif
