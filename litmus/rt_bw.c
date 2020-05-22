#include <linux/spinlock.h>
#include <linux/percpu.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/list.h>

#include <litmus/litmus.h>
#include <litmus/jobs.h>
#include <litmus/sched_plugin.h>
#include <litmus/edf_common.h>
#include <litmus/sched_trace.h>
#include <litmus/trace.h>
#include <litmus/debug_trace.h>

#include <litmus/preempt.h>
#include <litmus/budget.h>

#include <litmus/rt_bw.h>

/* CPU bandwidth status for all bandwidth-aware scheduler */
DEFINE_PER_CPU(cpu_bw_entry_t, cpu_bw_entries);

/* input
 * cpu: the cpu to check
 * bw_mask: the bw_mask the cpu has
 */
static inline int check_bw_status_invariant(int cpu, uint32_t bw_mask)
{
	int i;
	cpu_bw_entry_t *bw_entry_tmp, *bw_entry;
	
	bw_entry = &per_cpu(cpu_bw_entries, cpu);

	for_each_online_cpu(i) {
		bw_entry_tmp = &per_cpu(cpu_bw_entries, i);
		if (i != cpu && (bw_entry_tmp->used_bw & bw_mask))
		{
			TRACE("[BUG]Lock [P%d], Detect overlap BW: [P%d] used_bw:0x%x, [P%d] used_bw:0x%x, NR_CPUS=%d\n",
				   cpu, i, bw_entry_tmp->used_bw, cpu, bw_entry->used_bw, NR_CPUS);
			return 0;
		}
	}

	return 1;
}

/* lock_bw_partitions
 * lock bw_mask for cpu so that only cpu can use bw_mask
 * NOTE:
 * 1) rt.lock is grabbed by the caller so that
 *    scheduler on different CPUs do not have race condition
 * 2) We have race condition when user write to /proc/sys
 *    As long as users do not write to /proc/sys, we are safe
 *
 * tsk: lock bandwidth partition for task tsk
 */
void lock_bw_partitions(int cpu, uint32_t bw_mask, struct task_struct *tsk, rt_domain_t *rt)
{
	cpu_bw_entry_t *bw_entry;
    int ret = 0, i;

	if (cpu == NO_CPU)
	{
		TRACE("[BUG] try to lock 0x%x on NO_CPU\n", bw_mask);
	} else
	{
		bw_entry = &per_cpu(cpu_bw_entries, cpu);
		if (bw_entry->used_bw != 0)
		{
			TRACE("[BUG][P%d] has locked bw 0x%x before try to lock bw 0x%x\n",
				  bw_entry->cpu, bw_entry->used_bw, bw_mask);
		}
		ret = check_bw_status_invariant(cpu, bw_mask);
		//if (ret)
	//	{
			bw_entry->used_bw = bw_mask;
			for (i = 0; i < MAX_BANDWIDTH_PARTITIONS; i++)
			{
				if (bw_entry->used_bw & (1<<i) & MAX_BANDWIDTH_PARTITIONS)
				{
					rt->bw2taskmap[i] = tsk->pid;
				}
			}
			rt->used_bw_partitions |= bw_mask;
		//}
	}
	return;
}

/* unlock_bw_partitions
 * unlock bw_mask for cpu so that other cpus can use bw_mask
 */
void unlock_bw_partitions(int cpu, uint32_t bw_mask, rt_domain_t *rt)
{
	cpu_bw_entry_t *bw_entry;
    int ret, i;

	if (cpu == NO_CPU) 	
    {
		TRACE("[BUG] try to unlock 0x%x on NO_CPU\n", bw_mask);
	} else 	
    {
		bw_entry = &per_cpu(cpu_bw_entries, cpu);
		if (bw_entry->used_bw != bw_mask)
		{
			TRACE("[BUG][P%d] has locked BW partitions 0x%x before try to unlock BW partitions 0x%x\n",
				  bw_entry->cpu, bw_entry->used_bw, bw_mask);
		}
		ret = check_bw_status_invariant(cpu, bw_mask);
		for (i = 0; i < MAX_BANDWIDTH_PARTITIONS; i++)
		{
			if (bw_entry->used_bw & (1<<i) & MAX_BANDWIDTH_PARTITIONS)
			{
				rt->bw2taskmap[i] = 0;
			}
		}
		bw_entry->used_bw = 0;
		rt->used_bw_partitions &= (BANDWIDTH_PARTITIONS_MASK & ~bw_mask);
	}

	return;
}

int count_set_bits(uint32_t bitmask)
{
	int i = 0;
	int num_bits = 0;
	for (i = 0; i < MAX_NUM_BANDWIDTH_PARTITIONS; i++)
	{
		if (bitmask & (1 << i))
			num_bits++;
	}
	return num_bits;
}

