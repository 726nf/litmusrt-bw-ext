/*
 * litmus/sched_gsn_edfbw.c
 *
 * Implementation of the GSN-EDFBW scheduling algorithm.
 * Copy from litmus/sched_gsn_edf.c
 *
 * This version uses the simple approach and serializes all scheduling
 * decisions by the use of a queue lock. This is probably not the
 * best way to do it, but it should suffice for now.
 */

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

#include <litmus/preempt.h>
#include <litmus/budget.h>

#include <litmus/bheap.h>

#ifdef CONFIG_SCHED_CPU_AFFINITY
#include <litmus/affinity.h>
#endif

/* to set up domain/cpu mappings */
#include <litmus/litmus_proc.h>

#include <linux/module.h>

#include <litmus/rt_bw.h>

/* Overview of GSN-EDFBW operations.
 *
 * This description only covers how the individual operations are 
 * implemented in LITMUS.
 *
 * link_task_to_cpu(T, cpu) 	- Low-level operation to update the linkage
 *                                structure (NOT the actually scheduled
 *                                task). If there is another linked task To
 *                                already it will set To->linked_on = NO_CPU
 *                                (thereby removing its association with this
 *                                CPU). However, it will not requeue the
 *                                previously linked task (if any). It will set
 *                                T's state to not completed and check whether
 *                                it is already running somewhere else. If T
 *                                is scheduled somewhere else it will link
 *                                it to that CPU instead (and pull the linked
 *                                task to cpu). T may be NULL.
 *
 * unlink(T)			- Unlink removes T from all scheduler data
 *                                structures. If it is linked to some CPU it
 *                                will link NULL to that CPU. If it is
 *                                currently queued in the gsnedfbw queue it will
 *                                be removed from the rt_domain. It is safe to
 *                                call unlink(T) if T is not linked. T may not
 *                                be NULL.
 *
 * requeue(T)			- Requeue will insert T into the appropriate
 *                                queue. If the system is in real-time mode and
 *                                the T is released already, it will go into the
 *                                ready queue. If the system is not in
 *                                real-time mode is T, then T will go into the
 *                                release queue. If T's release time is in the
 *                                future, it will go into the release
 *                                queue. That means that T's release time/job
 *                                no/etc. has to be updated before requeu(T) is
 *                                called. It is not safe to call requeue(T)
 *                                when T is already queued. T may not be NULL.
 *
 * gsnedfbw_job_arrival(T)	- This is the catch all function when T enters
 *                                the system after either a suspension or at a
 *                                job release. It will queue T (which means it
 *                                is not safe to call gsnedfbw_job_arrival(T) if
 *                                T is already queued) and then check whether a
 *                                preemption is necessary. If a preemption is
 *                                necessary it will update the linkage
 *                                accordingly and cause scheduled to be called
 *                                (either with an IPI or need_resched). It is
 *                                safe to call gsnedfbw_job_arrival(T) if T's
 *                                next job has not been actually released yet
 *                                (releast time in the future). T will be put
 *                                on the release queue in that case.
 *
 * curr_job_completion()	- Take care of everything that needs to be done
 *                                to prepare the current task for its next
 *                                release and place it in the right queue with
 *                                gsnedfbw_job_arrival().
 *
 *
 * When we now that T is linked to CPU then link_task_to_cpu(NULL, CPU) is
 * equivalent to unlink(T). Note that if you unlink a task from a CPU none of
 * the functions will automatically propagate pending task from the ready queue
 * to a linked task. This is the job of the calling function ( by means of
 * __take_ready).
 */

/* cpu_entry_t - maintain the linked and scheduled state
 */
typedef struct  {
	int 			cpu;
    struct task_struct* preempting; /*only RT tasks, RT task that preempt a RT task through taking its bandwidth partitions */
	struct task_struct*	linked;		/* only RT tasks, RT task that link on CPU */
	struct task_struct*	scheduled;	/* only RT tasks, RT task that schedule on CPU */
	struct bheap_node*	hn;

} cpu_entry_t;
DEFINE_PER_CPU(cpu_entry_t, gsnedfbw_cpu_entries);

cpu_entry_t* gsnedfbw_cpus[NR_CPUS];

DECLARE_PER_CPU(cpu_bw_entry_t, cpu_bw_entries);

/* the cpus queue themselves according to priority in here */
static struct bheap_node gsnedfbw_heap_node[NR_CPUS];
static struct bheap      gsnedfbw_cpu_heap;

static rt_domain_t gsnedfbw;
#define gsnedfbw_lock (gsnedfbw.ready_lock)
#define gsnedfbw_bw_lock (gsnedfbw.bw_lock)

static cpu_entry_t* standby_cpus[NR_CPUS];

/* Uncomment this if you want to see all scheduling decisions in the
 * TRACE() log.
*/
//#define WANT_ALL_SCHED_EVENTS

static int cpu_lower_prio(struct bheap_node *_a, struct bheap_node *_b)
{
	cpu_entry_t *a, *b;
	a = _a->value;
	b = _b->value;
	/* Note that a and b are inverted: we want the lowest-priority CPU at
	 * the top of the heap.
	 */
	return edf_higher_prio(b->linked, a->linked);
}

/* update_cpu_position - Move the cpu entry to the correct place to maintain
 *                       order in the cpu queue. Caller must hold gsnedfbw lock.
 */
static void update_cpu_position(cpu_entry_t *entry)
{
	if (likely(bheap_node_in_heap(entry->hn)))
		bheap_delete(cpu_lower_prio, &gsnedfbw_cpu_heap, entry->hn);
	bheap_insert(cpu_lower_prio, &gsnedfbw_cpu_heap, entry->hn);
}

/* caller must hold gsnedfbw lock */
static cpu_entry_t* lowest_prio_cpu(void)
{
	struct bheap_node* hn;
	hn = bheap_peek(cpu_lower_prio, &gsnedfbw_cpu_heap);
	return hn->value;
}

static void remove_cpu(cpu_entry_t *entry)
{
	if (likely(bheap_node_in_heap(entry->hn)))
		bheap_delete(cpu_lower_prio, &gsnedfbw_cpu_heap, entry->hn);
}

/* link_task_to_cpu - Update the link of a CPU.
 *                    Handles the case where the to-be-linked task is already
 *                    scheduled on a different CPU.
 */
static noinline void link_task_to_cpu(struct task_struct* linked, cpu_entry_t *entry)
{
	cpu_entry_t *sched;
	struct task_struct* tmp;
	int on_cpu;

	BUG_ON(linked && !is_realtime(linked));

	/* Currently linked task is set to be unlinked. */
	if (entry->linked) {
		entry->linked->rt_param.linked_on = NO_CPU;
	}

	/* Link new task to CPU. */
	if (linked) {
		/* handle task is already scheduled somewhere! */
		on_cpu = linked->rt_param.scheduled_on;
		if (on_cpu != NO_CPU) {
			sched = &per_cpu(gsnedfbw_cpu_entries, on_cpu);
			/* this should only happen if not linked already */
			BUG_ON(sched->linked == linked);

			/* If we are already scheduled on the CPU to which we
			 * wanted to link, we don't need to do the swap --
			 * we just link ourselves to the CPU and depend on
			 * the caller to get things right.
			 */
			if (entry != sched) {
				TRACE_TASK(linked,
					   "already scheduled on %d, updating link.\n",
					   sched->cpu);
				tmp = sched->linked;
				linked->rt_param.linked_on = sched->cpu;
				sched->linked = linked;
				update_cpu_position(sched);
				linked = tmp;
			}
		}
		if (linked) /* might be NULL due to swap */
			linked->rt_param.linked_on = entry->cpu;
	}
	entry->linked = linked;
#ifdef WANT_ALL_SCHED_EVENTS
	if (linked)
		TRACE_TASK(linked, "linked to %d.\n", entry->cpu);
	else
		TRACE("NULL linked to %d.\n", entry->cpu);
#endif
	update_cpu_position(entry);
}

/* unlink - Make sure a task is not linked any longer to an entry
 *          where it was linked before. Must hold gsnedfbw_lock.
 */
static noinline void unlink(struct task_struct* t)
{
	cpu_entry_t *entry;

	if (t->rt_param.linked_on != NO_CPU) {
		/* unlink */
		entry = &per_cpu(gsnedfbw_cpu_entries, t->rt_param.linked_on);

		TRACE("asking for gsnedfbw_bw_lock\n");
		raw_spin_lock(&gsnedfbw_bw_lock);

		unlock_bw_partitions(entry->cpu, tsk_rt(t)->job_params.bw_partitions, &gsnedfbw);
		tsk_rt(t)->job_params.bw_partitions = 0;
		tsk_rt(t)->job_params.num_using_bw_partitions = 0;

		TRACE("release gsnedfbw_bw_lock\n");
		raw_spin_unlock(&gsnedfbw_bw_lock);

		t->rt_param.linked_on = NO_CPU;
		link_task_to_cpu(NULL, entry);

	} else if (is_queued(t)) {
		/* This is an interesting situation: t is scheduled,
		 * but was just recently unlinked.  It cannot be
		 * linked anywhere else (because then it would have
		 * been relinked to this CPU), thus it must be in some
		 * queue. We must remove it from the list in this
		 * case.
		 */
		remove(&gsnedfbw, t);
	}
}


/* preempt - force a CPU to reschedule
 */
static void preempt(cpu_entry_t *entry)
{
	preempt_if_preemptable(entry->scheduled, entry->cpu);
}

/* requeue - Put an unlinked task into gsn-edfbw domain.
 *           Caller must hold gsnedfbw_lock.
 */
static noinline void requeue(struct task_struct* task)
{
	BUG_ON(!task);
	/* sanity check before insertion */
	BUG_ON(is_queued(task));

	if (is_early_releasing(task) || is_released(task, litmus_clock()))
		__add_ready(&gsnedfbw, task);
	else {
		/* it has got to wait */
		add_release(&gsnedfbw, task);
	}
}

#ifdef CONFIG_SCHED_CPU_AFFINITY
static cpu_entry_t* gsnedfbw_get_nearest_available_cpu(cpu_entry_t *start)
{
	cpu_entry_t *affinity;

	get_nearest_available_cpu(affinity, start, gsnedfbw_cpu_entries,
#ifdef CONFIG_RELEASE_MASTER
			gsnedfbw.release_master,
#else
			NO_CPU,
#endif
			cpu_online_mask);

	return(affinity);
}
#endif

static int check_for_bw_preemptions(cpu_entry_t *entry, struct task_struct *task)
{
	int num_used_bw_partitions = 0;
	int num_bw_to_use = 0;
	int bw_ok = 0, i, cpu = 0;
	uint32_t bw_mask_to_use = 0;
	rt_domain_t *rt = &gsnedfbw;
	struct list_head *iter, *tmp;

	INIT_LIST_HEAD(&tsk_rt(task)->standby_list);
	num_used_bw_partitions = count_set_bits(rt->used_bw_partitions & BANDWIDTH_PARTITIONS_MASK);

	if (MAX_NUM_BANDWIDTH_PARTITIONS - num_used_bw_partitions
			>= tsk_rt(task)->task_params.num_bw_partitions)
	{
		bw_ok = 1;
		for (i = 0; i < MAX_NUM_BANDWIDTH_PARTITIONS; i++)
		{
			if (num_bw_to_use >= tsk_rt(task)->task_params.num_bw_partitions)
				break;
			if (!(rt->used_bw_partitions & (1<<i) & BANDWIDTH_PARTITIONS_MASK))
			{
				if (bw_mask_to_use & (1<<i) & BANDWIDTH_PARTITIONS_MASK)
					TRACE_TASK(task, "bw_mask_to_use=0x%x double set i=%d\n",
							   bw_mask_to_use, i);
				bw_mask_to_use |= (1<<i) & BANDWIDTH_PARTITIONS_MASK;
				num_bw_to_use++;
			}
		}

		TRACE_TASK(task, "Enough idle bandwidth, bw_ok=%d, bw_mask_to_use=0x%x, cpu=%d\n",
				bw_ok, bw_mask_to_use, entry->cpu);
	}else
	{
		bw_ok = 0;

		//BUG_ON(num_bw_to_use);

		if (num_bw_to_use) {
			TRACE("[BUG] trying to preempt bandwidth but already has num_bw_to_use=%d\n", num_bw_to_use);
		}

		/* take idle bandwidth partitions first */
		for (i = 0; i < MAX_NUM_BANDWIDTH_PARTITIONS; i++)
		{
			if (num_bw_to_use >= tsk_rt(task)->task_params.num_bw_partitions)
				break;
			if (!(rt->used_bw_partitions & (1<<i) & BANDWIDTH_PARTITIONS_MASK))
			{
				if (bw_mask_to_use & (1<<i) & BANDWIDTH_PARTITIONS_MASK)
					TRACE_TASK(task, "bw_mask_to_use=0x%x double set i=%d\n",
							   bw_mask_to_use, i);
				bw_mask_to_use |= (1<<i) & BANDWIDTH_PARTITIONS_MASK;
				num_bw_to_use++;
			}
		}
		TRACE_TASK(task, "take idle bandwidth 0x%x, num_bw_to_use=%d\n", bw_mask_to_use, num_bw_to_use);

		do {
			cpu_entry_t *last = lowest_prio_cpu();
			struct task_struct *cur;

			cpu++;

			if (!last)
				break;

			cur = last->linked;

			if (!edf_higher_prio(task, cur))
				break;

			standby_cpus[last->cpu] = last;
			if (last->hn->value != NULL)
				remove_cpu(last);

			if (!cur)
				break;

			num_bw_to_use += tsk_rt(cur)->task_params.num_bw_partitions;

			if (num_bw_to_use <= tsk_rt(task)->task_params.num_bw_partitions)
			{
				if (bw_mask_to_use & tsk_rt(cur)->job_params.bw_partitions)
					TRACE_TASK(task, "[BUG] preempt %s/%d/%d bw 0x%x but already has bw 0x%x\n",
				   			   cur->comm, cur->pid, tsk_rt(cur)->job_params.job_no,
							   tsk_rt(cur)->job_params.bw_partitions, bw_mask_to_use);
				bw_mask_to_use |= tsk_rt(cur)->job_params.bw_partitions;
			} else
			{
				num_bw_to_use -= tsk_rt(cur)->task_params.num_bw_partitions;

				for (i = 0; i < MAX_NUM_BANDWIDTH_PARTITIONS; i++)
				{
					if (tsk_rt(cur)->job_params.bw_partitions & (1<<i))
					{
						if (num_bw_to_use >= tsk_rt(task)->task_params.num_bw_partitions)
							break;
						if (bw_mask_to_use & (1<<i))
							TRACE_TASK(task, "[BUG] preempt %s/%d/%d bw 0x%x (i=%d) but already has bw 0x%x\n",
							   cur->comm, cur->pid, tsk_rt(cur)->job_params.job_no,
							   tsk_rt(cur)->job_params.bw_partitions, i, bw_mask_to_use);
						bw_mask_to_use |= (1<<i);
						num_bw_to_use++;
					}
				}
			}

			list_add(&tsk_rt(cur)->standby_list, &tsk_rt(task)->standby_list);

			if (num_bw_to_use >= tsk_rt(task)->task_params.num_bw_partitions)
				break;

		}while (cpu <= NR_CPUS);

		if (num_bw_to_use >= tsk_rt(task)->task_params.num_bw_partitions)
		{
			bw_ok = 1;
			list_for_each_safe(iter, tmp, &tsk_rt(task)->standby_list)
			{
				struct rt_param *rt_cur = list_entry(iter, struct rt_param, standby_list);
				struct task_struct *tsk_cur = list_entry(rt_cur, struct task_struct, rt_param);
				if (tsk_cur->pid != task->pid)
				{
					cpu_entry_t *cpu_entry = gsnedfbw_cpus[rt_cur->linked_on];
					list_del_init(&rt_cur->standby_list);

					TRACE_TASK(task, "asking for gsnedfbw_bw_lock\n");
					raw_spin_lock(&gsnedfbw_bw_lock);

					unlock_bw_partitions(cpu_entry->cpu, tsk_rt(tsk_cur)->job_params.bw_partitions, &gsnedfbw);
					tsk_rt(cpu_entry->linked)->job_params.bw_partitions = 0;

					TRACE_TASK(task, "release gsnedfbw_bw_lock\n");
					raw_spin_unlock(&gsnedfbw_bw_lock);

					if (cpu_entry->cpu != entry->cpu)
					{
						/* requeue the linked task; scheduled task is requeued at schedule() */
						if (requeue_preempted_job(cpu_entry->linked))
							requeue(cpu_entry->linked);
						link_task_to_cpu(NULL, cpu_entry);
						cpu_entry->preempting = task;
						preempt(cpu_entry);
					}
				}
			}

			TRACE_TASK(task, "Enough idle bandwidth through preemption, bw_ok=%d, bw_mask_to_use=0x%x, cpu=%d\n",
					bw_ok, bw_mask_to_use, entry->cpu);
		} else {
			list_for_each_safe(iter, tmp, &tsk_rt(task)->standby_list) {
				list_del_init(iter);
			}
		}


		for_each_online_cpu(cpu)
		{
			if (standby_cpus[cpu] != NULL)
				update_cpu_position(standby_cpus[cpu]);
		}

		memset(&standby_cpus, 0, sizeof(standby_cpus));
		INIT_LIST_HEAD(&tsk_rt(task)->standby_list);
	}

	if (bw_ok)
	{
		tsk_rt(task)->job_params.bw_partitions = bw_mask_to_use;

		if (entry->linked && is_realtime(entry->linked))
		{
			if (requeue_preempted_job(entry->linked))
				requeue(entry->linked);
		}

		TRACE("asking for gsnedfbw_bw_lock\n");
		raw_spin_lock(&gsnedfbw_bw_lock);

		lock_bw_partitions(entry->cpu, tsk_rt(task)->job_params.bw_partitions, task, &gsnedfbw);
		tsk_rt(task)->job_params.num_using_bw_partitions = num_bw_to_use;

		TRACE("release gsnedfbw_bw_lock\n");
		raw_spin_unlock(&gsnedfbw_bw_lock);
	}

	return bw_ok;
}

/* check for any necessary preemptions */
static void check_for_preemptions(void)
{
	struct task_struct *task;
	cpu_entry_t *last;
	int ret = 0;

#ifdef CONFIG_PREFER_LOCAL_LINKING
	cpu_entry_t *local;

	/* Before linking to other CPUs, check first whether the local CPU is
	 * idle. */
	local = this_cpu_ptr(&gsnedfbw_cpu_entries);
	task  = __peek_ready(&gsnedfbw);

	if (task && !local->linked
#ifdef CONFIG_RELEASE_MASTER
	    && likely(local->cpu != gsnedfbw.release_master)
#endif
		) {
		TRACE_TASK(task, "linking to local CPU %d to avoid IPI\n", local->cpu);

		ret = check_for_bw_preemptions(local, task);

		if (ret != 0)
		{
			task = __take_ready(&gsnedfbw);
			link_task_to_cpu(task, local);
			preempt(local);
		}
	}
#endif

	for (last = lowest_prio_cpu();
	     edf_preemption_needed(&gsnedfbw, last->linked);
	     last = lowest_prio_cpu()) {
		/* preemption necessary */
		task = __peek_ready(&gsnedfbw);
		if (!task)
			break;
		TRACE("check_for_preemptions: attempting to link task %d to P%d\n",
		      task->pid, last->cpu);

#ifdef CONFIG_SCHED_CPU_AFFINITY
		{
			cpu_entry_t *affinity =
					gsnedfbw_get_nearest_available_cpu(
						&per_cpu(gsnedfbw_cpu_entries, task_cpu(task)));
			if (affinity)
				last = affinity;
		}
#endif

		ret = check_for_bw_preemptions(last, task);

		if (ret != 0)
		{
			task = __take_ready(&gsnedfbw);
			link_task_to_cpu(task, last);
			preempt(last);
		} else
			break;
	}
}

/* gsnedfbw_job_arrival: task is either resumed or released */
static noinline void gsnedfbw_job_arrival(struct task_struct* task)
{
	BUG_ON(!task);

	TRACE_TASK(task, "gsnedfbw_job_arrival %s/%d/%d\n",
			   task->comm, task->pid,  tsk_rt(task)->job_params.job_no);
	INIT_LIST_HEAD(&tsk_rt(task)->standby_list);

	requeue(task);
	check_for_preemptions();
}

static void gsnedfbw_release_jobs(rt_domain_t* rt, struct bheap* tasks)
{
	unsigned long flags;

	TRACE("gsnedfbw_release_jobs\n");
	TRACE("asking for gsnedfbw_lock\n");
	raw_spin_lock_irqsave(&gsnedfbw_lock, flags);

	__merge_ready(rt, tasks);
	check_for_preemptions();

	TRACE("release gsnedfbw_lock\n");
	raw_spin_unlock_irqrestore(&gsnedfbw_lock, flags);
}

/* caller holds gsnedfbw_lock */
static noinline void curr_job_completion(int forced)
{
	struct task_struct *t = current;
	BUG_ON(!t);

	sched_trace_task_completion(t, forced);

	TRACE_TASK(t, "job_completion(forced=%d).\n", forced);

	/* set flags */
	tsk_rt(t)->completed = 0;
	/* prepare for next period */
	prepare_for_next_period(t);
	if (is_early_releasing(t) || is_released(t, litmus_clock()))
		sched_trace_task_release(t);
	/* unlink */
	unlink(t);
	/* requeue
	 * But don't requeue a blocking task. */
	if (is_current_running())
		gsnedfbw_job_arrival(t);
}

/* gsnedfbw_check_sched_invariant
 * Check sched invariant at end of gsnedfbw_schedule
 * gsnedfbw.lock is grabbed by caller
 * Invariant: At the end of schedule() on a CPU,
 * the current linked task on the CPU should NOT be preemptable
 * by the top task in ready_queue
 */
void gsnedfbw_check_sched_invariant(void)
{
	int i;
	cpu_entry_t *entry = NULL;
	struct task_struct *task = NULL,  *qtask = NULL;
	struct task_struct *preempted_task = NULL;
	rt_domain_t *rt = &gsnedfbw;
	int num_used_bw = 0, num_avail_bw = 0;
	int cpu_ok = 0;
	int preempted_cpu = -1;

	qtask = __peek_ready(&gsnedfbw);
	
	/* No ready RT task */
	if (!qtask)
		return;

	/* Top ready task has higher priority? */
	entry = this_cpu_ptr(&gsnedfbw_cpu_entries);
	task = entry->linked;
	if (edf_higher_prio(qtask, task))
	{
		cpu_ok = 1;
		preempted_task = task;
		preempted_cpu = entry->cpu;
	}

	num_used_bw = count_set_bits(rt->used_bw_partitions & BANDWIDTH_PARTITIONS_MASK);
	num_avail_bw = MAX_NUM_BANDWIDTH_PARTITIONS - num_used_bw;
	
	for (i = 0; i < NR_CPUS; i++)
	{
		entry = gsnedfbw_cpus[i];
		/* entry may have picked a task but not schedule yet */
		task = entry->linked;
		if (!task || !is_realtime(task))
			continue;
		if (edf_higher_prio(qtask, task))
				num_avail_bw += tsk_rt(task)->task_params.num_bw_partitions;
	}

	if (cpu_ok && num_avail_bw >= tsk_rt(qtask)->task_params.num_bw_partitions)
	{
		if (preempted_task && is_realtime(preempted_task))
			TRACE_TASK(qtask, "[ERROR] can preempt %s/%d/%d on P%d rt.bw=0x%x qtask.num_bw=%d\n",
				   preempted_task->comm, preempted_task->pid,
				   tsk_rt(preempted_task)->job_params.job_no, preempted_cpu,
				   rt->used_bw_partitions, tsk_rt(qtask)->task_params.num_bw_partitions);
		if (preempted_task && !is_realtime(preempted_task))
			TRACE_TASK(qtask, "[ERROR] can preempt %s/%d on P%d rt.bw=0x%x qtask.num_bw=%d\n",
				   preempted_task->comm, preempted_task->pid, preempted_cpu,
				   rt->used_bw_partitions, tsk_rt(qtask)->task_params.num_bw_partitions);
		if (!preempted_task)
			TRACE_TASK(qtask, "[ERROR] can preempt NULL on P%d rt.bw=0x%x qtask.num_bw=%d\n",
				   preempted_cpu, rt->used_bw_partitions, tsk_rt(qtask)->task_params.num_bw_partitions);
	}

	return;
}

/* Getting schedule() right is a bit tricky. schedule() may not make any
 * assumptions on the state of the current task since it may be called for a
 * number of reasons. The reasons include a scheduler_tick() determined that it
 * was necessary, because sys_exit_np() was called, because some Linux
 * subsystem determined so, or even (in the worst case) because there is a bug
 * hidden somewhere. Thus, we must take extreme care to determine what the
 * current state is.
 *
 * The CPU could currently be scheduling a task (or not), be linked (or not).
 *
 * The following assertions for the scheduled task could hold:
 *
 *  - !is_running(scheduled)        // the job blocks
 *	- scheduled->timeslice == 0	// the job completed (forcefully)
 *	- is_completed()		// the job completed (by syscall)
 * 	- linked != scheduled		// we need to reschedule (for any reason)
 * 	- is_np(scheduled)		// rescheduling must be delayed,
 *					   sys_exit_np must be requested
 *
 * Any of these can occur together.
 */
static struct task_struct* gsnedfbw_schedule(struct task_struct * prev)
{
	rt_domain_t *rt = &gsnedfbw;
	cpu_entry_t* entry = this_cpu_ptr(&gsnedfbw_cpu_entries);
	int out_of_time, sleep, preempt, np, exists, blocks, finish;
	struct task_struct* next = NULL;

#ifdef CONFIG_RELEASE_MASTER
	/* Bail out early if we are the release master.
	 * The release master never schedules any real-time tasks.
	 */
	if (unlikely(gsnedfbw.release_master == entry->cpu)) {
		sched_state_task_picked();
		return NULL;
	}
#endif

	raw_spin_lock(&gsnedfbw_lock);

	/* sanity checking */
	BUG_ON(entry->scheduled && entry->scheduled != prev);
	BUG_ON(entry->scheduled && !is_realtime(prev));
	BUG_ON(is_realtime(prev) && !entry->scheduled);

	/* (0) Determine state */
	exists      = entry->scheduled != NULL;
	blocks      = exists && !is_current_running();
	out_of_time = exists && budget_enforced(entry->scheduled)
		&& budget_exhausted(entry->scheduled);
	np 	    = exists && is_np(entry->scheduled);
	sleep	    = exists && is_completed(entry->scheduled);
	preempt     = entry->scheduled != entry->linked;
	finish 	= 0;

#ifdef WANT_ALL_SCHED_EVENTS
	TRACE_TASK(prev, "invoked gsnedfbw_schedule.\n");
#endif

	if (exists)
		TRACE_TASK(prev,
			   "blocks:%d out_of_time:%d np:%d sleep:%d preempt:%d "
			   "state:%d sig:%d bw:0x%x rt.bw:0x%x\n",
			   blocks, out_of_time, np, sleep, preempt,
			   prev->state, signal_pending(prev),
			   tsk_rt(prev)->job_params.bw_partitions,
			   rt->used_bw_partitions);
	if (entry->linked && preempt)
		TRACE_TASK(prev, "will be preempted by %s/%d\n",
			   entry->linked->comm, entry->linked->pid);
	
	/* If a task blocks we have no choice but to reschedule. */
	if (blocks)
		unlink(entry->scheduled);

	/* Request a sys_exit_np() call if we would like to preempt but cannot.
	 * We need to make sure to update the link structure anyway in case
	 * that we are still linked. Multiple calls to request_exit_np() don't
	 * hurt.
	 */
	if (np && (out_of_time || preempt || sleep)) 
	{
		unlink(entry->scheduled);
		request_exit_np(entry->scheduled);
	}

	/* Any task that is preemptable and either exhausts its execution
	 * budget or wants to sleep completes. We may have to reschedule after
	 * this. Don't do a job completion if we block (can't have timers running
	 * for blocked jobs).
	 */
	if (exists && !np && (out_of_time || sleep) && !blocks)
	{
		finish = 1;
		curr_job_completion(!sleep);
	}

	/* Be preempted */
	if (exists && !np && !(out_of_time || sleep) && !blocks && entry->linked != entry->scheduled) {
		entry->scheduled->rt_param.job_params.bw_partitions = 0;
	}

	/* Link pending task if we became unlinked.
 	 * But do not link if the core is preempted only via bandwidth
	 */
	if (!entry->linked)
	{
		/* scheduled RT task is preempted due to bandwidth if
 		 * is_realtime(entry->scheduled) &&
 		 * entry->scheduled != entry->linked;
 		 * The preempted RT task will be handled in the rest of this function.
 		 * otherwise, check if another RT task can run on the CPU
 		 * Note: Even when we consider priority inversion,
 		 * 		 logic here is still correct.
 		 */
		if (!exists || !is_realtime(entry->scheduled))
		{
			check_for_preemptions();
		}
	}

	/* The final scheduling decision. Do we need to switch for some reason?
	 * If linked is different from scheduled, then select linked as next.
	 */
	if ((!np || blocks) && entry->linked != entry->scheduled) {
		if (entry->scheduled) {
			/* not gonna be scheduled soon */
			entry->scheduled->rt_param.scheduled_on = NO_CPU;
			tsk_rt(entry->scheduled)->job_params.bw_partitions = 0;
			TRACE_TASK(entry->scheduled, "scheduled_on = NO_CPU, rt->used_bw_mask=0x%x should exclude job.bw_mask=0x%x\n",
					   rt->used_bw_partitions, entry->scheduled->rt_param.job_params.bw_partitions);
			/* Trace when preempted via bandwidth by another CPU */
			if (!blocks && !entry->linked && !finish)
			{
				if (!entry->preempting)
					TRACE_TASK(entry->scheduled, "[BUG] preempted by NULL.\n");
				else
				{
					TRACE_TASK(entry->scheduled, "preempted by %s/%d/%d due to bandwidth preemption\n",
				   	       entry->preempting->comm, entry->preempting->pid,
						   tsk_rt(entry->preempting)->job_params.job_no);
					entry->preempting = NULL;
				}
			}
		}
		/* Schedule a linked job? */
		if (entry->linked) {
			entry->linked->rt_param.scheduled_on = entry->cpu;
			next = entry->linked;
			TRACE_TASK(next, "scheduled_on = P%d, rt.used_bw_mask=0x%x should include job.bw_mask=0x%x\n",
					   smp_processor_id(), rt->used_bw_partitions, tsk_rt(next)->job_params.bw_partitions);
		}
	} else if (exists)
		/* Only override Linux scheduler if we have a real-time task
		 * scheduled that needs to continue.
		 */
		next = prev;

	sched_state_task_picked();

	/* Check correctness of scheduler
  	 * NOTE: TODO: avoid such check in non-debug mode */
	//gsnedfbw_check_sched_invariant();

	raw_spin_unlock(&gsnedfbw_lock);

#ifdef WANT_ALL_SCHED_EVENTS
	TRACE_TASK(next, "gsnedfbw_lock released\n");

	if (next)
		TRACE_TASK(next, "scheduled at %llu\n", litmus_clock());
	else if (exists && !next)
		TRACE("becomes idle at %llu.\n", litmus_clock());
	else
		TRACE("idle stays idle at %llu.\n", litmus_clock());
#endif

	return next;

}

/* _finish_switch - we just finished the switch away from prev
 */
static void gsnedfbw_finish_switch(struct task_struct *prev)
{
	cpu_entry_t *entry = this_cpu_ptr(&gsnedfbw_cpu_entries);

	entry->scheduled = is_realtime(current) ? current : NULL;
#ifdef WANT_ALL_SCHED_EVENTS
	TRACE_TASK(current, "switched to \n");
	TRACE_TASK(prev, "switched away from\n");
#endif
}


/*	Prepare a task for running in RT mode
 */
static void gsnedfbw_task_new(struct task_struct * t, int on_rq, int is_scheduled)
{
	unsigned long 		flags;
	cpu_entry_t* 		entry;

	TRACE("gsn edfbw: task new %d\n", t->pid);

	TRACE("asking for gsnedfbw_lock\n");
	raw_spin_lock_irqsave(&gsnedfbw_lock, flags);

	tsk_rt(t)->job_params.bw_partitions = 0;
	INIT_LIST_HEAD(&tsk_rt(t)->standby_list);

	/* setup job params */
	release_at(t, litmus_clock());

	if (is_scheduled) {
		entry = &per_cpu(gsnedfbw_cpu_entries, task_cpu(t));
		BUG_ON(entry->scheduled);

#ifdef CONFIG_RELEASE_MASTER
		if (entry->cpu != gsnedfbw.release_master) {
#endif
			entry->scheduled = t;
			tsk_rt(t)->scheduled_on = task_cpu(t);
#ifdef CONFIG_RELEASE_MASTER
		} else {
			/* do not schedule on release master */
			preempt(entry); /* force resched */
			tsk_rt(t)->scheduled_on = NO_CPU;
		}
#endif
	} else {
		t->rt_param.scheduled_on = NO_CPU;
	}

	t->rt_param.linked_on = NO_CPU;

	if (on_rq || is_scheduled)
		gsnedfbw_job_arrival(t);

	TRACE("release gsnedfbw_lock\n");
	raw_spin_unlock_irqrestore(&gsnedfbw_lock, flags);
}

static void gsnedfbw_task_wake_up(struct task_struct *task)
{
	unsigned long flags;
	lt_t now;

	TRACE_TASK(task, "wake_up at %llu, bw_mask=0x%x\n",
			   litmus_clock(), tsk_rt(task)->job_params.bw_partitions);

	TRACE("asking for gsnedfbw_lock\n");
	raw_spin_lock_irqsave(&gsnedfbw_lock, flags);
	now = litmus_clock();
	if (is_sporadic(task) && is_tardy(task, now)) {
		/* new sporadic release */
		release_at(task, now);
		sched_trace_task_release(task);
	}
	gsnedfbw_job_arrival(task);

	TRACE("release gsnedfbw_lock\n");
	raw_spin_unlock_irqrestore(&gsnedfbw_lock, flags);
}

static void gsnedfbw_task_block(struct task_struct *t)
{
	rt_domain_t *rt = &gsnedfbw;
	unsigned long flags;

	TRACE_TASK(t, "block at %llu, bw_mask=0x%x\n",
			   litmus_clock(), tsk_rt(t)->job_params.bw_partitions);

	/* unlink if necessary */
	TRACE("asking for gsnedfbw_lock\n");
	raw_spin_lock_irqsave(&gsnedfbw_lock, flags);
	unlink(t);
	TRACE_TASK(t, "blocked, rt.used_bw_mask=0x%x should not include job.bw_mask=0x%x\n",
			   rt->used_bw_partitions, tsk_rt(t)->job_params.bw_partitions);
	/* schedule point when task is blocked */
	check_for_preemptions();

	TRACE("release gsnedfbw_lock\n");
	raw_spin_unlock_irqrestore(&gsnedfbw_lock, flags);

	BUG_ON(!is_realtime(t));
}


static void gsnedfbw_task_exit(struct task_struct * t)
{
	rt_domain_t *rt = &gsnedfbw;
	unsigned long flags;

	/* unlink if necessary */
	TRACE("asking for gsnedfbw_lock\n");
	raw_spin_lock_irqsave(&gsnedfbw_lock, flags);
	unlink(t);
	if (tsk_rt(t)->scheduled_on != NO_CPU) {
		gsnedfbw_cpus[tsk_rt(t)->scheduled_on]->scheduled = NULL;
		tsk_rt(t)->scheduled_on = NO_CPU;
	}
	TRACE_TASK(t, "exit, used_bw_mask=0x%x cleared by job.bw_mask=0x%x\n",
			   rt->used_bw_partitions, tsk_rt(t)->job_params.bw_partitions);
	tsk_rt(t)->job_params.bw_partitions = 0;
	/* schedule point when task is blocked */
	check_for_preemptions();
	
	TRACE("release gsnedfbw_lock\n");
	raw_spin_unlock_irqrestore(&gsnedfbw_lock, flags);

	BUG_ON(!is_realtime(t));
        TRACE_TASK(t, "RIP\n");
}


static long gsnedfbw_admit_task(struct task_struct* tsk)
{
		INIT_LIST_HEAD(&tsk_rt(tsk)->standby_list);
    	TRACE_TASK(tsk, "is admitted, num_bw=%d, job.bw_mask=0x%x (should be 0x0)\n",
				   tsk_rt(tsk)->task_params.num_bw_partitions, tsk_rt(tsk)->job_params.bw_partitions);
		return 0;
}

static struct domain_proc_info gsnedfbw_domain_proc_info;
static long gsnedfbw_get_domain_proc_info(struct domain_proc_info **ret)
{
	*ret = &gsnedfbw_domain_proc_info;
	return 0;
}

static void gsnedfbw_setup_domain_proc(void)
{
	int i, cpu;
	int release_master =
#ifdef CONFIG_RELEASE_MASTER
			atomic_read(&release_master_cpu);
#else
		NO_CPU;
#endif
	int num_rt_cpus = num_online_cpus() - (release_master != NO_CPU);
	struct cd_mapping *map;

	memset(&gsnedfbw_domain_proc_info, 0, sizeof(gsnedfbw_domain_proc_info));
	init_domain_proc_info(&gsnedfbw_domain_proc_info, num_rt_cpus, 1);
	gsnedfbw_domain_proc_info.num_cpus = num_rt_cpus;
	gsnedfbw_domain_proc_info.num_domains = 1;

	gsnedfbw_domain_proc_info.domain_to_cpus[0].id = 0;
	for (cpu = 0, i = 0; cpu < num_online_cpus(); ++cpu) {
		if (cpu == release_master)
			continue;
		map = &gsnedfbw_domain_proc_info.cpu_to_domains[i];
		map->id = cpu;
		cpumask_set_cpu(0, map->mask);
		++i;

		/* add cpu to the domain */
		cpumask_set_cpu(cpu,
			gsnedfbw_domain_proc_info.domain_to_cpus[0].mask);
	}
}

static long gsnedfbw_activate_plugin(void)
{
	int cpu;
	cpu_entry_t *entry;
	cpu_bw_entry_t *bw_entry;

	bheap_init(&gsnedfbw_cpu_heap);
#ifdef CONFIG_RELEASE_MASTER
	gsnedfbw.release_master = atomic_read(&release_master_cpu);
#endif

	for_each_online_cpu(cpu) {
		entry = &per_cpu(gsnedfbw_cpu_entries, cpu);
		bheap_node_init(&entry->hn, entry);
		entry->linked    = NULL;
		entry->scheduled = NULL;
		bw_entry = &per_cpu(cpu_bw_entries, cpu);
		bw_entry->cpu = cpu;
		bw_entry->used_bw = 0;
#ifdef CONFIG_RELEASE_MASTER
		if (cpu != gsnedfbw.release_master) {
#endif
			TRACE("GSN-EDFBW: Initializing CPU #%d.\n", cpu);
			update_cpu_position(entry);
#ifdef CONFIG_RELEASE_MASTER
		} else {
			TRACE("GSN-EDFBW: CPU %d is release master.\n", cpu);
		}
#endif
	}

	gsnedfbw_setup_domain_proc();
	gsnedfbw.used_bw_partitions = 0;

	return 0;
}

static long gsnedfbw_deactivate_plugin(void)
{
	destroy_domain_proc_info(&gsnedfbw_domain_proc_info);
	return 0;
}

/*	Plugin object	*/
static struct sched_plugin gsn_edfbw_plugin __cacheline_aligned_in_smp = {
	.plugin_name		= "GSN-EDFBW",
	.finish_switch		= gsnedfbw_finish_switch,
	.task_new		= gsnedfbw_task_new,
	.complete_job		= complete_job,
	.task_exit		= gsnedfbw_task_exit,
	.schedule		= gsnedfbw_schedule,
	.task_wake_up		= gsnedfbw_task_wake_up,
	.task_block		= gsnedfbw_task_block,
	.admit_task		= gsnedfbw_admit_task,
	.activate_plugin	= gsnedfbw_activate_plugin,
	.deactivate_plugin	= gsnedfbw_deactivate_plugin,
	.get_domain_proc_info	= gsnedfbw_get_domain_proc_info
};


static int __init init_gsn_edfbw(void)
{
	int cpu;
	cpu_entry_t *entry;
	cpu_bw_entry_t *bw_entry;

	memset(&standby_cpus, 0, sizeof(standby_cpus));

	bheap_init(&gsnedfbw_cpu_heap);
	/* initialize CPU state */
	for (cpu = 0; cpu < NR_CPUS; cpu++)  {
		entry = &per_cpu(gsnedfbw_cpu_entries, cpu);
		gsnedfbw_cpus[cpu] = entry;
		entry->cpu = cpu;
		entry->hn = &gsnedfbw_heap_node[cpu];
		bheap_node_init(&entry->hn, entry);
		bw_entry = &per_cpu(cpu_bw_entries, cpu);
		bw_entry->cpu = cpu;
		bw_entry->used_bw = 0;
	}

	gsnedfbw.used_bw_partitions = 0;
	memset(gsnedfbw.bw2taskmap, 0, sizeof(gsnedfbw.bw2taskmap));

	edf_domain_init(&gsnedfbw, NULL, gsnedfbw_release_jobs);
	return register_sched_plugin(&gsn_edfbw_plugin);
}

module_init(init_gsn_edfbw);
