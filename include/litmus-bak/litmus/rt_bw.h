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

/* task t cache_state should be s
 */
//static inline void check_bw_state(struct task_struct *t, cache_state_t s);

/* set_cache_state
 * Change job.cache_state to new state
 */
//static inline void set_bw_state(struct task_struct *task, cache_state_t s);

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

/* set_cache_config
 * Check task.cache_state is correct before change it
 * Change job.cache_state to new state
 * Change PL310 cache partition register
 * 		a) CACHE_IN_USE: lock job.cache_partitions on task.core
 * 		b) CACHE_CLEARED: unlock job.cache_partitions on task.core
 * Change rt.used_cache_partitions when 
 * 		a) new partitions are CACHE_IN_USE or old 
 * 		b) cache partitions are CACHE_CLEARED
 * NOTE:
 * 		a) IF s == CACHE_WILL_USE, before call this func
 * 		   job.cache_partitions must be setup 
 * 		   rt_param.linked_on must be setup
 * 		b) IF s == CACHE_WILL_CLEAR | CACHE_CLEARED
 * 		   rt_param.scheduled_on must NOT be cleared
 * 		   job.cache_partitions do not have to be cleared since
 * 		   cache_state can indicate that info is useless.
 * rt_domain_t.lock is grabbed by the caller
 */
//void set_bw_config(rt_domain_t *rt, struct task_struct *task, cache_state_t s);

#endif
