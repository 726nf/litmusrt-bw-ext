/*
 * Constant definitions related to
 * scheduling policy.
 */

#ifndef _LINUX_LITMUS_H_
#define _LINUX_LITMUS_H_

#include <litmus/ctrlpage.h>

#ifdef CONFIG_RELEASE_MASTER
extern atomic_t release_master_cpu;
#endif

/* in_list - is a given list_head queued on some list?
 */
static inline int in_list(struct list_head* list)
{
	return !(  /* case 1: deleted */
		   (list->next == LIST_POISON1 &&
		    list->prev == LIST_POISON2)
		 ||
		   /* case 2: initialized */
		   (list->next == list &&
		    list->prev == list)
		);
}

struct task_struct* __waitqueue_remove_first(wait_queue_head_t *wq);

#define NO_CPU			0xffffffff

void litmus_fork(struct task_struct *tsk);
void litmus_exec(void);
/* clean up real-time state of a task */
void litmus_clear_state(struct task_struct *dead_tsk);
void exit_litmus(struct task_struct *dead_tsk);

/* Prevent the plugin from being switched-out from underneath a code
 * path. Might sleep, so may be called only from non-atomic context. */
void litmus_plugin_switch_disable(void);
void litmus_plugin_switch_enable(void);

long litmus_admit_task(struct task_struct *tsk);
void litmus_exit_task(struct task_struct *tsk);
void litmus_dealloc(struct task_struct *tsk);
void litmus_do_exit(struct task_struct *tsk);
int litmus_be_migrate_to(int cpu);

#define is_realtime(t) 		((t)->policy == SCHED_LITMUS)
#define rt_transition_pending(t) \
	((t)->rt_param.transition_pending)

#define tsk_rt(t)		(&(t)->rt_param)

/*	Realtime utility macros */
#ifdef CONFIG_LITMUS_LOCKING
#define is_priority_boosted(t)  (tsk_rt(t)->priority_boosted)
#define get_boost_start(t)  (tsk_rt(t)->boost_start_time)
#else
#define is_priority_boosted(t)  0
#define get_boost_start(t)      0
#endif


/* task_params macros */
#define get_exec_cost(t)  	(tsk_rt(t)->task_params.exec_cost)
#define get_rt_period(t)	(tsk_rt(t)->task_params.period)
#define get_rt_relative_deadline(t)	(tsk_rt(t)->task_params.relative_deadline)
#define get_rt_phase(t)		(tsk_rt(t)->task_params.phase)
#define get_partition(t) 	(tsk_rt(t)->task_params.cpu)
#define get_priority(t) 	(tsk_rt(t)->task_params.priority)
#define get_class(t)        (tsk_rt(t)->task_params.cls)
#define get_release_policy(t) (tsk_rt(t)->task_params.release_policy)

/* job_param macros */
#define get_exec_time(t)    (tsk_rt(t)->job_params.exec_time)
#define get_deadline(t)		(tsk_rt(t)->job_params.deadline)
#define get_release(t)		(tsk_rt(t)->job_params.release)
#define get_lateness(t)		(tsk_rt(t)->job_params.lateness)

/* release policy macros */
#define is_periodic(t)		(get_release_policy(t) == TASK_PERIODIC)
#define is_sporadic(t)		(get_release_policy(t) == TASK_SPORADIC)
#ifdef CONFIG_ALLOW_EARLY_RELEASE
#define is_early_releasing(t)	(get_release_policy(t) == TASK_EARLY)
#else
#define is_early_releasing(t)	(0)
#endif

#define is_hrt(t)     		\
	(tsk_rt(t)->task_params.cls == RT_CLASS_HARD)
#define is_srt(t)     		\
	(tsk_rt(t)->task_params.cls == RT_CLASS_SOFT)
#define is_be(t)      		\
	(tsk_rt(t)->task_params.cls == RT_CLASS_BEST_EFFORT)

/* Our notion of time within LITMUS: kernel monotonic time. */
static inline lt_t litmus_clock(void)
{
	return ktime_to_ns(ktime_get());
}

/* A macro to convert from nanoseconds to ktime_t. */
#define ns_to_ktime(t)		ktime_add_ns(ktime_set(0, 0), t)

#define is_released(t, now)	\
	(lt_before_eq(get_release(t), now))
#define is_tardy(t, now)    \
	(lt_before_eq(tsk_rt(t)->job_params.deadline, now))

/* real-time comparison macros */
#define earlier_deadline(a, b) (lt_before(\
	(a)->rt_param.job_params.deadline,\
	(b)->rt_param.job_params.deadline))
#define earlier_release(a, b)  (lt_before(\
	(a)->rt_param.job_params.release,\
	(b)->rt_param.job_params.release))

void preempt_if_preemptable(struct task_struct* t, int on_cpu);

#define bheap2task(hn) ((struct task_struct*) hn->value)

static inline int is_present(struct task_struct* t)
{
	return t && tsk_rt(t)->present;
}

static inline int is_completed(struct task_struct* t)
{
	return t && tsk_rt(t)->completed;
}


/* Used to convert ns-specified execution costs and periods into
 * integral quanta equivalents.
 */
#define LITMUS_QUANTUM_LENGTH_NS (CONFIG_LITMUS_QUANTUM_LENGTH_US * 1000ULL)

/* make the unit explicit */
typedef unsigned long quanta_t;

enum round {
	FLOOR,
	CEIL
};

static inline quanta_t time2quanta(lt_t time, enum round round)
{
	s64  quantum_length = LITMUS_QUANTUM_LENGTH_NS;

	if (do_div(time, quantum_length) && round == CEIL)
		time++;
	return (quanta_t) time;
}

static inline lt_t quanta2time(quanta_t quanta)
{
	return quanta * LITMUS_QUANTUM_LENGTH_NS;
}

/* By how much is cpu staggered behind CPU 0? */
u64 cpu_stagger_offset(int cpu);

static inline struct control_page* get_control_page(struct task_struct *t)
{
	return tsk_rt(t)->ctrl_page;
}

static inline int has_control_page(struct task_struct* t)
{
	return tsk_rt(t)->ctrl_page != NULL;
}


#ifdef CONFIG_SCHED_OVERHEAD_TRACE

#define TS_SYSCALL_IN_START						\
	if (has_control_page(current)) {				\
		__TS_SYSCALL_IN_START(&get_control_page(current)->ts_syscall_start); \
	}

#define TS_SYSCALL_IN_END						\
	if (has_control_page(current)) {				\
		unsigned long flags;					\
		uint64_t irqs;						\
		local_irq_save(flags);					\
		irqs = get_control_page(current)->irq_count -		\
			get_control_page(current)->irq_syscall_start;	\
		__TS_SYSCALL_IN_END(&irqs);				\
		local_irq_restore(flags);				\
	}

#else

#define TS_SYSCALL_IN_START
#define TS_SYSCALL_IN_END

#endif

#ifdef CONFIG_SMP

/*
 * struct hrtimer_start_on_info - timer info on remote cpu
 * @timer:	timer to be triggered on remote cpu
 * @time:	time event
 * @mode:	timer mode
 * @csd:	smp_call_function parameter to call hrtimer_pull on remote cpu
 */
struct hrtimer_start_on_info {
	struct hrtimer		*timer;
	ktime_t			time;
	enum hrtimer_mode	mode;
	struct call_single_data csd;
};

void hrtimer_pull(void *csd_info);
extern void hrtimer_start_on(int cpu, struct hrtimer_start_on_info *info,
			struct hrtimer *timer, ktime_t time,
			const enum hrtimer_mode mode);

#endif

#endif
