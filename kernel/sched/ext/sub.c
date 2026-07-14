// SPDX-License-Identifier: GPL-2.0
/*
 * BPF extensible scheduler class: Documentation/scheduler/sched-ext.rst
 *
 * Sub-scheduler hierarchy support.
 *
 * A sub-scheduler is an scx_sched attached to a cgroup subtree under another
 * scx_sched. This file holds the sub-scheduler implementation: the scheduler
 * tree walk, capability delegation, per-shard cap state and its sync, and the
 * sub-scheduler enable/disable paths. The core dispatch/enqueue machinery it
 * builds on lives in ext.c.
 *
 * Copyright (c) 2026 Meta Platforms, Inc. and affiliates.
 * Copyright (c) 2026 Tejun Heo <tj@kernel.org>
 */
#include <linux/rhashtable.h>
#include "internal.h"
#include "cid.h"
#include "arena.h"
#include "sub.h"

#ifdef CONFIG_EXT_SUB_SCHED

/**
 * scx_skip_subtree_pre - Skip @pos's subtree in a pre-order walk
 * @pos: current position
 * @root: walk root
 *
 * In a walk started by scx_next_descendant_pre(), continue past @pos's subtree:
 * return @pos's next sibling, or the closest ancestor's next sibling, or NULL
 * if @pos's subtree is the last under @root. Same locking rules.
 */
struct scx_sched *scx_skip_subtree_pre(struct scx_sched *pos, struct scx_sched *root)
{
	struct scx_sched *next;

	lockdep_assert(lockdep_is_held(&scx_enable_mutex) ||
		       lockdep_is_held(&scx_sched_lock) ||
		       rcu_read_lock_any_held());

	while (pos != root) {
		next = list_next_or_null_rcu(&scx_parent(pos)->children, &pos->sibling,
					     struct scx_sched, sibling);
		if (next)
			return next;
		pos = scx_parent(pos);
	}
	return NULL;
}

/**
 * scx_next_descendant_pre - find the next descendant for pre-order walk
 * @pos: the current position (%NULL to initiate traversal)
 * @root: sched whose descendants to walk
 *
 * To be used by scx_for_each_descendant_pre(). Find the next descendant to
 * visit for pre-order traversal of @root's descendants. @root is included in
 * the iteration and the first node to be visited.
 */
struct scx_sched *scx_next_descendant_pre(struct scx_sched *pos, struct scx_sched *root)
{
	struct scx_sched *next;

	lockdep_assert(lockdep_is_held(&scx_enable_mutex) ||
		       lockdep_is_held(&scx_sched_lock) ||
		       rcu_read_lock_any_held());

	/* if first iteration, visit @root */
	if (!pos)
		return root;

	/* visit the first child if exists */
	next = list_first_or_null_rcu(&pos->children, struct scx_sched, sibling);
	if (next)
		return next;

	/* no child, visit my or the closest ancestor's next sibling */
	return scx_skip_subtree_pre(pos, root);
}

static struct scx_sched *scx_find_sub_sched(u64 cgroup_id)
{
	return rhashtable_lookup(&scx_sched_hash, &cgroup_id,
				 scx_sched_hash_params);
}

void scx_set_task_sched(struct task_struct *p, struct scx_sched *sch)
{
	rcu_assign_pointer(p->scx.sched, sch);
}

struct cgroup *sch_cgroup(struct scx_sched *sch)
{
	return sch->cgrp;
}

/* for each descendant of @cgrp including self, set ->scx_sched to @sch */
void set_cgroup_sched(struct cgroup *cgrp, struct scx_sched *sch)
{
	struct cgroup *pos;
	struct cgroup_subsys_state *css;

	cgroup_for_each_live_descendant_pre(pos, css, cgrp)
		rcu_assign_pointer(pos->scx_sched, sch);
}

static void free_pshard(struct scx_pshard *pshard)
{
	struct scx_caps_updated *cu;

	if (!pshard)
		return;
	cu = &pshard->caps_updated;
	if (cu->cmask_arena_out)
		scx_arena_free(pshard->sch, cu->cmask_arena_out,
			       struct_size_t(struct scx_cmask, bits,
					     SCX_CMASK_NR_WORDS(pshard->nr_cids)));
	kfree(pshard);
}

void scx_free_pshards(struct scx_sched *sch)
{
	s32 si;

	if (!sch->pshard)
		return;
	for (si = 0; si < sch->nr_pshards; si++)
		free_pshard(sch->pshard[si]);
	kfree(sch->pshard);
}

static struct scx_pshard *alloc_pshard(struct scx_sched *sch, s32 shard_idx, s32 node)
{
	const struct scx_cid_shard *shard = &scx_cid_shard_ranges[shard_idx];
	size_t cmask_size = struct_size_t(struct scx_cmask, bits,
					  SCX_CMASK_NR_WORDS(shard->nr_cids));
	struct scx_pshard *pshard;
	struct scx_caps_updated *cu;
	s32 i;

	pshard = kzalloc_node(sizeof(*pshard), GFP_KERNEL, node);
	if (!pshard)
		return NULL;

	raw_spin_lock_init(&pshard->lock);
	pshard->sch = sch;
	pshard->base = shard->base_cid;
	pshard->nr_cids = shard->nr_cids;

	for (i = 0; i < __SCX_NR_CAPS; i++)
		scx_cmask_init(&pshard->caps[i].cmask, shard->base_cid, shard->nr_cids);

	cu = &pshard->caps_updated;
	raw_spin_lock_init(&cu->lock);
	INIT_LIST_HEAD(&cu->node_in_flight);
	__scx_cmask_init(&cu->cmask, shard->base_cid, shard->nr_cids, SCX_CID_SHARD_MAX_CPUS);

	cu->cmask_arena_out = scx_arena_alloc(sch, cmask_size);
	if (!cu->cmask_arena_out) {
		free_pshard(pshard);
		return NULL;
	}

	scx_cmask_init(cu->cmask_arena_out, shard->base_cid, shard->nr_cids);

	return pshard;
}

s32 scx_alloc_pshards(struct scx_sched *sch)
{
	struct scx_pshard **pshard;
	s32 si;

	if (!sch->is_cid_type || !sch->arena_pool)
		return 0;

	pshard = kzalloc_objs(pshard[0], scx_nr_cid_shards, GFP_KERNEL);
	if (!pshard)
		return -ENOMEM;

	for (si = 0; si < scx_nr_cid_shards; si++) {
		pshard[si] = alloc_pshard(sch, si, scx_shard_node[si]);
		if (!pshard[si]) {
			while (--si >= 0)
				free_pshard(pshard[si]);
			kfree(pshard);
			return -ENOMEM;
		}
	}

	sch->nr_pshards = scx_nr_cid_shards;
	/*
	 * Publish only after every entry is built so a reader observing
	 * @sch->pshard never sees a partially-filled array. Pair the store
	 * with a barrier and READ_ONCE() on the read side.
	 */
	smp_wmb();
	WRITE_ONCE(sch->pshard, pshard);
	return 0;
}

/*
 * Seed the root's caps fully. Root owns all cids on all caps at enable time.
 * Children acquire caps via scx_bpf_sub_grant().
 */
void scx_init_root_caps(struct scx_sched *sch)
{
	s32 si, i;

	for (si = 0; si < sch->nr_pshards; si++) {
		struct scx_pshard *ps = sch->pshard[si];

		for (i = 0; i < __SCX_NR_CAPS; i++)
			scx_cmask_fill(&ps->caps[i].cmask);
	}
}

/* record a caps change, see struct scx_caps_updated */
static void caps_updated_record(struct scx_pshard *ps, const struct scx_cmask *cids, u64 caps,
				struct list_head *to_deliver)
{
	struct scx_caps_updated *cu = &ps->caps_updated;

	guard(raw_spinlock)(&cu->lock);
	scx_cmask_or(&cu->cmask, cids);
	cu->caps |= caps;
	if (list_empty(&cu->node_in_flight))
		list_add_tail(&cu->node_in_flight, to_deliver);
}

/* deliver queued caps_updated callbacks, see struct scx_caps_updated */
static void caps_updated_deliver(struct list_head *to_deliver)
{
	struct scx_caps_updated *cu, *tmp;

	list_for_each_entry_safe(cu, tmp, to_deliver, node_in_flight) {
		struct scx_pshard *ps = container_of(cu, struct scx_pshard, caps_updated);
		struct scx_sched *sch = ps->sch;

		while (true) {
			u64 caps = 0;

			/*
			 * During enable, has_op is set after ops.sub_attach(),
			 * so !has_op means the op is absent or the sched isn't
			 * live yet - e.g. caps grant from ops.sub_attach().
			 * Either way don't consume - leave for
			 * scx_sub_seed_caps() to deliver once live.
			 */
			scoped_guard (raw_spinlock, &cu->lock) {
				if (cu->caps && SCX_HAS_OP(sch, sub_caps_updated) &&
				    likely(!READ_ONCE(sch->aborting))) {
					struct scx_cmask_ref ref;

					caps = cu->caps;
					scx_cmask_ref_init_kern(sch, cu->cmask_arena_out,
								ps->base, ps->nr_cids, &ref);
					scx_cmask_ref_copy(&ref, &cu->cmask);
					scx_cmask_clear(&cu->cmask);
					cu->caps = 0;
				} else {
					list_del_init(&cu->node_in_flight);
				}
			}
			if (!caps)
				break;

			/* caps != 0 only when deliverable (has_op, above) */
			SCX_CALL_OP(sch, sub_caps_updated, NULL,
				    scx_kaddr_to_arena(sch, cu->cmask_arena_out),
				    caps);
		}
	}
}

/*
 * Deliver caps owed to @sch that couldn't be delivered earlier (e.g. a grant
 * taken during its sub_attach(), before has_op was set). Called once @sch is
 * enabled.
 */
static void scx_sub_seed_caps(struct scx_sched *sch)
{
	LIST_HEAD(to_deliver);
	s32 si;

	guard(irqsave)();

	for (si = 0; si < sch->nr_pshards; si++) {
		struct scx_pshard *ps = sch->pshard[si];
		struct scx_caps_updated *cu = &ps->caps_updated;

		scoped_guard (raw_spinlock, &cu->lock) {
			if (cu->caps && list_empty(&cu->node_in_flight))
				list_add_tail(&cu->node_in_flight, &to_deliver);
		}
	}
	caps_updated_deliver(&to_deliver);
}

static DECLARE_WAIT_QUEUE_HEAD(scx_unlink_waitq);

void drain_descendants(struct scx_sched *sch)
{
	/*
	 * Child scheds that finished the critical part of disabling will take
	 * themselves off @sch->children. Wait for it to drain. As propagation
	 * is recursive, empty @sch->children means that all proper descendant
	 * scheds reached unlinking stage.
	 */
	wait_event(scx_unlink_waitq, list_empty(&sch->children));
}

static void scx_fail_parent(struct scx_sched *sch,
			    struct task_struct *failed, s32 fail_code)
{
	struct scx_sched *parent = scx_parent(sch);
	struct scx_task_iter sti;
	struct task_struct *p;

	scx_error(parent, "ops.init_task() failed (%d) for %s[%d] while disabling a sub-scheduler",
		  fail_code, failed->comm, failed->pid);

	/*
	 * Once $parent is bypassed, it's safe to put SCX_TASK_NONE tasks into
	 * it. This may cause downstream failures on the BPF side but $parent is
	 * dying anyway.
	 */
	scx_bypass(parent, true);

	scx_task_iter_start(&sti, sch->cgrp);
	while ((p = scx_task_iter_next_locked(&sti))) {
		if (scx_task_on_sched(parent, p))
			continue;

		scoped_guard (sched_change, p, DEQUEUE_SAVE | DEQUEUE_MOVE) {
			scx_disable_and_exit_task(sch, p);
			scx_set_task_sched(p, parent);
		}
	}
	scx_task_iter_stop(&sti);
}

void scx_sub_disable(struct scx_sched *sch)
{
	struct scx_sched *parent = scx_parent(sch);
	struct scx_task_iter sti;
	struct task_struct *p;
	int ret;

	/*
	 * Guarantee forward progress and wait for descendants to be disabled.
	 * To limit disruptions, $parent is not bypassed. Tasks are fully
	 * prepped and then inserted back into $parent.
	 */
	scx_bypass(sch, true);
	drain_descendants(sch);

	/*
	 * Here, every runnable task is guaranteed to make forward progress and
	 * we can safely use blocking synchronization constructs. Actually
	 * disable ops.
	 */
	mutex_lock(&scx_enable_mutex);
	percpu_down_write(&scx_fork_rwsem);
	scx_cgroup_lock();

	set_cgroup_sched(sch_cgroup(sch), parent);

	scx_task_iter_start(&sti, sch->cgrp);
	while ((p = scx_task_iter_next_locked(&sti))) {
		struct rq *rq;
		struct rq_flags rf;

		/* filter out duplicate visits */
		if (scx_task_on_sched(parent, p))
			continue;

		/*
		 * By the time control reaches here, all descendant schedulers
		 * should already have been disabled.
		 */
		WARN_ON_ONCE(!scx_task_on_sched(sch, p));

		/*
		 * @p is pinned by the iter: css_task_iter_next() takes a
		 * reference and holds it until the next iter_next() call, so
		 * @p->usage is guaranteed > 0.
		 */
		get_task_struct(p);

		scx_task_iter_unlock(&sti);

		/*
		 * $p is READY or ENABLED on @sch. Initialize for $parent,
		 * disable and exit from @sch, and then switch over to $parent.
		 *
		 * If a task fails to initialize for $parent, the only available
		 * action is disabling $parent too. While this allows disabling
		 * of a child sched to cause the parent scheduler to fail, the
		 * failure can only originate from ops.init_task() of the
		 * parent. A child can't directly affect the parent through its
		 * own failures.
		 */
		ret = __scx_init_task(parent, p, false);
		if (ret) {
			scx_fail_parent(sch, p, ret);
			put_task_struct(p);
			break;
		}

		rq = task_rq_lock(p, &rf);

		if (scx_get_task_state(p) == SCX_TASK_DEAD) {
			/*
			 * sched_ext_dead() raced us between __scx_init_task()
			 * and this rq lock and ran exit_task() on @sch (the
			 * sched @p was on at that point), not on $parent.
			 * $parent's just-completed init is owed an exit_task()
			 * and we issue it here.
			 */
			scx_sub_init_cancel_task(parent, p);
			task_rq_unlock(rq, p, &rf);
			put_task_struct(p);
			continue;
		}

		scoped_guard (sched_change, p, DEQUEUE_SAVE | DEQUEUE_MOVE) {
			/*
			 * $p is initialized for $parent and still attached to
			 * @sch. Disable and exit for @sch, switch over to
			 * $parent, override the state to READY to account for
			 * $p having already been initialized, and then enable.
			 */
			scx_disable_and_exit_task(sch, p);
			scx_set_task_state(p, SCX_TASK_INIT_BEGIN);
			scx_set_task_state(p, SCX_TASK_INIT);
			scx_set_task_sched(p, parent);
			scx_set_task_state(p, SCX_TASK_READY);
			scx_enable_task(parent, p);
		}

		task_rq_unlock(rq, p, &rf);
		put_task_struct(p);
	}
	scx_task_iter_stop(&sti);

	scx_disable_dump(sch);

	scx_cgroup_unlock();
	percpu_up_write(&scx_fork_rwsem);

	/*
	 * All tasks are moved off of @sch but there may still be on-going
	 * operations (e.g. ops.select_cpu()). Drain them by flushing RCU. Use
	 * the expedited version as ancestors may be waiting in bypass mode.
	 * Also, tell the parent that there is no need to keep running bypass
	 * DSQs for us.
	 */
	synchronize_rcu_expedited();
	scx_disable_bypass_dsp(sch);

	scx_unlink_sched(sch);

	mutex_unlock(&scx_enable_mutex);

	/*
	 * @sch is now unlinked from the parent's children list. Notify and call
	 * ops.sub_detach/exit(). Note that ops.sub_detach/exit() must be called
	 * after unlinking and releasing all locks. See scx_claim_exit().
	 */
	wake_up_all(&scx_unlink_waitq);

	if (parent->ops.sub_detach && sch->sub_attached) {
		struct scx_sub_detach_args sub_detach_args = {
			.ops = &sch->ops,
			.cgroup_path = sch->cgrp_path,
		};
		SCX_CALL_OP(parent, sub_detach, NULL,
			    &sub_detach_args);
	}

	scx_log_sched_disable(sch);

	if (sch->ops.exit)
		SCX_CALL_OP(sch, exit, NULL, sch->exit_info);

	/*
	 * @sch's non-ops programs such as timers and tracers can fire after
	 * ops.exit(). Now that exit is complete, stop scx_prog_sched() from
	 * resolving to @sch and drain in-flight resolvers.
	 */
	WRITE_ONCE(sch->dead, true);
	synchronize_rcu();

	if (sch->sub_kset)
		kobject_del(&sch->sub_kset->kobj);
	/* not added if enable failed before scx_sched_sysfs_add() */
	if (sch->kobj.state_in_sysfs)
		kobject_del(&sch->kobj);
}

/* verify that a scheduler can be attached to @cgrp and return the parent */
static struct scx_sched *find_parent_sched(struct cgroup *cgrp)
{
	struct scx_sched *parent = cgrp->scx_sched;
	struct scx_sched *pos;

	lockdep_assert_held(&scx_sched_lock);

	/* can't attach twice to the same cgroup */
	if (parent->cgrp == cgrp)
		return ERR_PTR(-EBUSY);

	/* does $parent allow sub-scheds? */
	if (!parent->ops.sub_attach)
		return ERR_PTR(-EOPNOTSUPP);

	/* can't insert between $parent and its exiting children */
	list_for_each_entry(pos, &parent->children, sibling)
		if (cgroup_is_descendant(pos->cgrp, cgrp))
			return ERR_PTR(-EBUSY);

	return parent;
}

static bool assert_task_ready_or_enabled(struct task_struct *p)
{
	u32 state = scx_get_task_state(p);

	switch (state) {
	case SCX_TASK_READY:
	case SCX_TASK_ENABLED:
		return true;
	default:
		WARN_ONCE(true, "sched_ext: Invalid task state %d for %s[%d] during enabling sub sched",
			  state, p->comm, p->pid);
		return false;
	}
}

void scx_sub_enable_workfn(struct kthread_work *work)
{
	struct scx_enable_cmd *cmd = container_of(work, struct scx_enable_cmd, work);
	struct sched_ext_ops *ops = cmd->ops;
	struct cgroup *cgrp;
	struct scx_sched *parent, *sch;
	struct scx_task_iter sti;
	struct task_struct *p;
	s32 i, ret;

	mutex_lock(&scx_enable_mutex);

	if (!scx_enabled()) {
		ret = -ENODEV;
		goto out_unlock;
	}

	/* See scx_root_enable_workfn() for the @ops->priv check. */
	if (rcu_access_pointer(ops->priv)) {
		ret = -EBUSY;
		goto out_unlock;
	}

	cgrp = cgroup_get_from_id(ops->sub_cgroup_id);
	if (IS_ERR(cgrp)) {
		ret = PTR_ERR(cgrp);
		goto out_unlock;
	}

	raw_spin_lock_irq(&scx_sched_lock);
	parent = find_parent_sched(cgrp);
	if (IS_ERR(parent)) {
		raw_spin_unlock_irq(&scx_sched_lock);
		ret = PTR_ERR(parent);
		goto out_put_cgrp;
	}
	kobject_get(&parent->kobj);
	raw_spin_unlock_irq(&scx_sched_lock);

	/* scx_alloc_and_add_sched() consumes @cgrp whether it succeeds or not */
	sch = scx_alloc_and_add_sched(cmd, cgrp, parent);
	kobject_put(&parent->kobj);
	if (IS_ERR(sch)) {
		ret = PTR_ERR(sch);
		goto out_unlock;
	}

	/*
	 * Validate before scx_link_sched() publishes @sch, so an invalid sub
	 * never becomes visible with an unallocated pshard.
	 */
	ret = scx_validate_ops(sch, ops);
	if (ret)
		goto err_disable;

	/*
	 * Allocate pshard[] before scx_link_sched() publishes @sch into the
	 * parent's RCU children list. A concurrent revoke walking the tree
	 * would otherwise dereference sch->pshard[si] while it's still NULL.
	 * Unlike the root path, the cid shard layout is stable at this point.
	 *
	 * scx_alloc_pshards() skips allocation when @sch's arena pool isn't
	 * initialized, so scx_arena_pool_init() must run first.
	 */
	ret = scx_arena_pool_init(sch);
	if (ret)
		goto err_disable;

	ret = scx_alloc_pshards(sch);
	if (ret)
		goto err_disable;

	ret = scx_link_sched(sch);
	if (ret)
		goto err_disable;

	ret = scx_sched_sysfs_add(sch);
	if (ret)
		goto err_disable;

	if (sch->level >= SCX_SUB_MAX_DEPTH) {
		scx_error(sch, "max nesting depth %d violated",
			  SCX_SUB_MAX_DEPTH);
		goto err_disable;
	}

	if (sch->ops.init) {
		ret = SCX_CALL_OP_RET(sch, init, NULL);
		if (ret) {
			ret = scx_ops_sanitize_err(sch, "init", ret);
			scx_error(sch, "ops.init() failed (%d)", ret);
			goto err_disable;
		}
		sch->exit_info->flags |= SCX_EFLAG_INITIALIZED;
	}

	ret = scx_set_cmask_scratch_alloc(sch);
	if (ret)
		goto err_disable;

	struct scx_sub_attach_args sub_attach_args = {
		.ops = &sch->ops,
		.cgroup_path = sch->cgrp_path,
	};

	ret = SCX_CALL_OP_RET(parent, sub_attach, NULL,
			      &sub_attach_args);
	if (ret) {
		ret = scx_ops_sanitize_err(sch, "sub_attach", ret);
		scx_error(sch, "parent rejected (%d)", ret);
		goto err_disable;
	}
	sch->sub_attached = true;

	scx_bypass(sch, true);

	for (i = SCX_OPI_BEGIN; i < SCX_OPI_END; i++)
		if (((void (**)(void))ops)[i])
			set_bit(i, sch->has_op);

	percpu_down_write(&scx_fork_rwsem);
	scx_cgroup_lock();

	/*
	 * Set cgroup->scx_sched's and check CSS_ONLINE. Either we see
	 * !CSS_ONLINE or scx_cgroup_lifetime_notify() sees and shoots us down.
	 */
	set_cgroup_sched(sch_cgroup(sch), sch);
	if (!(cgrp->self.flags & CSS_ONLINE)) {
		scx_error(sch, "cgroup is not online");
		goto err_unlock_and_disable;
	}

	/*
	 * Initialize tasks for the new child $sch without exiting them for
	 * $parent so that the tasks can always be reverted back to $parent
	 * sched on child init failure.
	 */
	WARN_ON_ONCE(scx_enabling_sub_sched);
	scx_enabling_sub_sched = sch;

	scx_task_iter_start(&sti, sch->cgrp);
	while ((p = scx_task_iter_next_locked(&sti))) {
		struct rq *rq;
		struct rq_flags rf;

		/*
		 * Task iteration may visit the same task twice when racing
		 * against exiting. Use %SCX_TASK_SUB_INIT to mark tasks which
		 * finished __scx_init_task() and skip if set.
		 *
		 * A task may exit and get freed between __scx_init_task()
		 * completion and scx_enable_task(). In such cases,
		 * scx_disable_and_exit_task() must exit the task for both the
		 * parent and child scheds.
		 */
		if (p->scx.flags & SCX_TASK_SUB_INIT)
			continue;

		/* @p is pinned by the iter; see scx_sub_disable() */
		get_task_struct(p);

		if (!assert_task_ready_or_enabled(p)) {
			ret = -EINVAL;
			goto abort;
		}

		scx_task_iter_unlock(&sti);

		/*
		 * As $p is still on $parent, it can't be transitioned to INIT.
		 * Let's worry about task state later. Use __scx_init_task().
		 */
		ret = __scx_init_task(sch, p, false);
		if (ret)
			goto abort;

		rq = task_rq_lock(p, &rf);

		if (scx_get_task_state(p) == SCX_TASK_DEAD) {
			/*
			 * sched_ext_dead() raced us between __scx_init_task()
			 * and this rq lock and ran exit_task() on $parent (the
			 * sched @p was on at that point), not on @sch. @sch's
			 * just-completed init is owed an exit_task() and we
			 * issue it here.
			 */
			scx_sub_init_cancel_task(sch, p);
			task_rq_unlock(rq, p, &rf);
			put_task_struct(p);
			continue;
		}

		p->scx.flags |= SCX_TASK_SUB_INIT;
		task_rq_unlock(rq, p, &rf);

		put_task_struct(p);
	}
	scx_task_iter_stop(&sti);

	/*
	 * All tasks are prepped. Disable/exit tasks for $parent and enable for
	 * the new @sch.
	 */
	scx_task_iter_start(&sti, sch->cgrp);
	while ((p = scx_task_iter_next_locked(&sti))) {
		/*
		 * Use clearing of %SCX_TASK_SUB_INIT to detect and skip
		 * duplicate iterations.
		 */
		if (!(p->scx.flags & SCX_TASK_SUB_INIT))
			continue;

		scoped_guard (sched_change, p, DEQUEUE_SAVE | DEQUEUE_MOVE) {
			/*
			 * $p must be either READY or ENABLED. If ENABLED,
			 * __scx_disabled_and_exit_task() first disables and
			 * makes it READY. However, after exiting $p, it will
			 * leave $p as READY.
			 */
			assert_task_ready_or_enabled(p);
			__scx_disable_and_exit_task(parent, p);

			/*
			 * $p is now only initialized for @sch and READY, which
			 * is what we want. Assign it to @sch and enable.
			 */
			scx_set_task_sched(p, sch);
			scx_enable_task(sch, p);

			p->scx.flags &= ~SCX_TASK_SUB_INIT;
		}
	}
	scx_task_iter_stop(&sti);

	scx_enabling_sub_sched = NULL;

	scx_cgroup_unlock();
	percpu_up_write(&scx_fork_rwsem);

	scx_bypass(sch, false);

	/* @sch is enabled; deliver any caps owed since its sub_attach() */
	scx_sub_seed_caps(sch);

	pr_info("sched_ext: BPF sub-scheduler \"%s\" enabled\n", sch->ops.name);
	kobject_uevent(&sch->kobj, KOBJ_ADD);
	ret = 0;
	goto out_unlock;

out_put_cgrp:
	cgroup_put(cgrp);
out_unlock:
	mutex_unlock(&scx_enable_mutex);
	cmd->ret = ret;
	return;

abort:
	put_task_struct(p);
	scx_task_iter_stop(&sti);

	/*
	 * Undo __scx_init_task() for tasks we marked. scx_enable_task() never
	 * ran for @sch on them, so calling scx_disable_task() here would invoke
	 * ops.disable() without a matching ops.enable(). scx_enabling_sub_sched
	 * must stay set until SUB_INIT is cleared from every marked task -
	 * scx_disable_and_exit_task() reads it when a task exits concurrently.
	 */
	scx_task_iter_start(&sti, sch->cgrp);
	while ((p = scx_task_iter_next_locked(&sti))) {
		if (p->scx.flags & SCX_TASK_SUB_INIT) {
			scx_sub_init_cancel_task(sch, p);
			p->scx.flags &= ~SCX_TASK_SUB_INIT;
		}
	}
	scx_task_iter_stop(&sti);
	scx_enabling_sub_sched = NULL;
err_unlock_and_disable:
	/* we'll soon enter disable path, keep bypass on */
	scx_cgroup_unlock();
	percpu_up_write(&scx_fork_rwsem);
err_disable:
	mutex_unlock(&scx_enable_mutex);
	/*
	 * Some enable failures only return an errno (e.g. -ENOMEM from an
	 * allocation) without calling scx_error(). Record it so
	 * scx_flush_disable_work() runs the disable and ops.exit() fires.
	 */
	scx_error(sch, "scx_sub_enable() failed (%d)", ret);
	scx_flush_disable_work(sch);
	cmd->ret = 0;
}

static s32 scx_cgroup_lifetime_notify(struct notifier_block *nb,
				      unsigned long action, void *data)
{
	struct cgroup *cgrp = data;
	struct cgroup *parent = cgroup_parent(cgrp);

	if (!cgroup_on_dfl(cgrp))
		return NOTIFY_OK;

	switch (action) {
	case CGROUP_LIFETIME_ONLINE:
		/* inherit ->scx_sched from $parent */
		if (parent)
			rcu_assign_pointer(cgrp->scx_sched, parent->scx_sched);
		break;
	case CGROUP_LIFETIME_OFFLINE:
		/* if there is a sched attached, shoot it down */
		if (cgrp->scx_sched && cgrp->scx_sched->cgrp == cgrp)
			scx_exit(cgrp->scx_sched, SCX_EXIT_UNREG_KERN,
				 SCX_ECODE_RSN_CGROUP_OFFLINE,
				 "cgroup %llu going offline", cgroup_id(cgrp));
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block scx_cgroup_lifetime_nb = {
	.notifier_call = scx_cgroup_lifetime_notify,
};

static s32 __init scx_cgroup_lifetime_notifier_init(void)
{
	return blocking_notifier_chain_register(&cgroup_lifetime_notifier,
						&scx_cgroup_lifetime_nb);
}
core_initcall(scx_cgroup_lifetime_notifier_init);

static void scx_pstack_recursion(struct bpf_prog *prog, const char *op)
{
	struct scx_sched *sch;

	guard(rcu)();
	sch = scx_prog_sched(prog->aux);
	if (unlikely(!sch))
		return;

	scx_error(sch, "%s recursion detected", op);
}

void scx_pstack_recursion_on_dispatch(struct bpf_prog *prog)
{
	scx_pstack_recursion(prog, "dispatch");
}

void scx_pstack_recursion_on_caps_updated(struct bpf_prog *prog)
{
	scx_pstack_recursion(prog, "sub_caps_updated");
}

__bpf_kfunc_start_defs();

/**
 * scx_bpf_sub_dispatch - Trigger dispatching on a child scheduler
 * @cgroup_id: cgroup ID of the child scheduler to dispatch
 * @aux: implicit BPF argument to access bpf_prog_aux hidden from BPF progs
 *
 * Allows a parent scheduler to trigger dispatching on one of its direct
 * child schedulers. The child scheduler runs its dispatch operation to
 * move tasks from dispatch queues to the local runqueue.
 *
 * Returns: true on success, false if cgroup_id is invalid, not a direct
 * child, or caller lacks dispatch permission.
 */
__bpf_kfunc bool scx_bpf_sub_dispatch(u64 cgroup_id, const struct bpf_prog_aux *aux)
{
	struct rq *this_rq = this_rq();
	struct scx_sched *parent, *child;

	guard(rcu)();
	parent = scx_prog_sched(aux);
	if (unlikely(!parent))
		return false;

	child = scx_find_sub_sched(cgroup_id);

	if (unlikely(!child))
		return false;

	if (unlikely(scx_parent(child) != parent)) {
		scx_error(parent, "trying to dispatch a distant sub-sched on cgroup %llu",
			  cgroup_id);
		return false;
	}

	return scx_dispatch_sched(child, this_rq, this_rq->scx.sub_dispatch_prev,
				  true);
}

/* Validate common inputs. On success, *parent_out and *child_out are set. */
static s32 sub_cap_preamble(u64 cgroup_id, u64 caps, const struct bpf_prog_aux *aux,
			    struct scx_sched **parent_out, struct scx_sched **child_out)
{
	struct scx_sched *parent, *child;

	parent = scx_prog_sched(aux);
	if (unlikely(!parent))
		return -ENODEV;

	if (!scx_is_cid_type()) {
		scx_error(parent, "sub-cap kfuncs require a cid-form scheduler");
		return -EOPNOTSUPP;
	}

	child = scx_find_sub_sched(cgroup_id);
	if (unlikely(!child))
		return -ENODEV;

	if (unlikely(scx_parent(child) != parent)) {
		scx_error(parent, "%s: sub-%llu is not a direct child",
			  parent->cgrp_path, cgroup_id);
		return -EINVAL;
	}

	if (unlikely(caps & ~__SCX_CAP_ALL)) {
		scx_error(parent, "invalid caps 0x%llx", caps);
		return -EINVAL;
	}

	*parent_out = parent;
	*child_out = child;
	return 0;
}

/**
 * scx_bpf_sub_grant - Grant @caps on @cmask__ign's cids to a direct child
 * @cgroup_id: cgroup id of the direct child sub-sched
 * @caps: bitmask of SCX_CAP_* to grant
 * @cmask__ign: cid cmask to grant @caps on (arena pointer)
 * @denied_out__ign: optional arena cmask accumulating refused cids
 * @aux: implicit BPF argument
 *
 * A cid in @cmask__ign is granted to the child only if the parent holds every
 * requested cap on it. Refused cids are OR'd into @denied_out__ign when
 * provided. Refusals outside @denied_out__ign's range are not recorded.
 *
 * All-or-nothing keeps the caller-visible result binary per cid, so
 * @denied_out__ign is one mask to interpret rather than a per-cap matrix.
 *
 * Return 0 on full success, -EPERM if any cid was refused, or a negative
 * errno on other failures.
 */
__bpf_kfunc s32 scx_bpf_sub_grant(u64 cgroup_id, u64 caps,
				  const struct scx_cmask *cmask__ign,
				  struct scx_cmask *denied_out__ign,
				  const struct bpf_prog_aux *aux)
{
	struct scx_cmask_ref ref, denied_ref;
	struct scx_sched *parent, *child;
	bool any_denied = false;
	LIST_HEAD(to_deliver);
	s32 si, ret;

	guard(irqsave)();

	ret = sub_cap_preamble(cgroup_id, caps, aux, &parent, &child);
	if (ret)
		return ret;

	ret = scx_cmask_ref_init(parent, cmask__ign, &ref);
	if (ret) {
		scx_error(parent, "invalid cmask (%d)", ret);
		return ret;
	}

	if (denied_out__ign) {
		ret = scx_cmask_ref_init(parent, denied_out__ign, &denied_ref);
		if (ret) {
			scx_error(parent, "invalid denied_out (%d)", ret);
			return ret;
		}
	}

	/* apply the grant one shard at a time */
	for (si = ref.shard_first; si < ref.shard_end; si++) {
		SCX_CMASK_DEFINE_SHARD(slice, 0, SCX_CID_SHARD_MAX_CPUS);
		struct scx_pshard *pps = parent->pshard[si];
		struct scx_pshard *cps = child->pshard[si];
		u64 granted_caps = 0;
		u32 cap_bit;

		scx_cmask_ref_shard(&ref, si, slice);
		if (scx_cmask_empty(slice))
			continue;

		SCX_CMASK_DEFINE_SHARD(granted_cids, slice->base, slice->nr_cids);
		SCX_CMASK_DEFINE_SHARD(changed_cids, slice->base, slice->nr_cids);
		SCX_CMASK_DEFINE_SHARD(delta, slice->base, slice->nr_cids);

		scx_cmask_copy(granted_cids, slice);

		scoped_guard (raw_spinlock, &pps->lock) {
			guard(raw_spinlock_nested)(&cps->lock);

			/*
			 * Narrow granted_cids to cids the parent holds every
			 * requested cap on. All-or-nothing per cid.
			 */
			scx_for_each_cap_bit(cap_bit, caps)
				scx_cmask_and(granted_cids, &pps->caps[cap_bit].cmask);

			/*
			 * For each requested cap, fold the newly-set cids into
			 * the child and accumulate the delta.
			 */
			scx_for_each_cap_bit(cap_bit, caps) {
				struct scx_cmask *ccm = &cps->caps[cap_bit].cmask;

				scx_cmask_copy(delta, granted_cids);
				scx_cmask_andnot(delta, ccm);
				if (scx_cmask_empty(delta))
					continue;

				scx_cmask_or(ccm, delta);
				scx_cmask_or(changed_cids, delta);
				granted_caps |= BIT_U64(cap_bit);
			}

			if (granted_caps)
				caps_updated_record(cps, changed_cids, granted_caps,
						    &to_deliver);
		}

		/* record cids that didn't make it through into @denied_out */
		if (!scx_cmask_subset(slice, granted_cids)) {
			any_denied = true;
			if (denied_out__ign) {
				SCX_CMASK_DEFINE_SHARD(denied, slice->base, slice->nr_cids);

				scx_cmask_copy(denied, slice);
				scx_cmask_andnot(denied, granted_cids);
				scx_cmask_ref_or(&denied_ref, denied);
			}
		}
	}

	caps_updated_deliver(&to_deliver);

	return any_denied ? -EPERM : 0;
}

/**
 * scx_bpf_sub_revoke - Revoke @caps on @cmask__ign's cids from @child
 * @cgroup_id: cgroup id of the direct child sub-sched
 * @caps: bitmask of SCX_CAP_* to revoke
 * @cmask__ign: cid cmask to revoke @caps on (arena pointer)
 * @aux: implicit BPF argument
 *
 * Clear @caps bits on @cmask__ign from the child named by @cgroup_id and all
 * its descendants. The origin parent's pshard lock is held across the subtree
 * walk so a concurrent grant from the origin parent observes the revoked
 * state.
 */
__bpf_kfunc void scx_bpf_sub_revoke(u64 cgroup_id, u64 caps,
				    const struct scx_cmask *cmask__ign,
				    const struct bpf_prog_aux *aux)
{
	struct scx_cmask_ref ref;
	struct scx_sched *parent, *child, *pos;
	LIST_HEAD(to_deliver);
	s32 si, ret;

	guard(irqsave)();

	if (sub_cap_preamble(cgroup_id, caps, aux, &parent, &child))
		return;

	ret = scx_cmask_ref_init(parent, cmask__ign, &ref);
	if (ret) {
		scx_error(parent, "invalid cmask (%d)", ret);
		return;
	}

	/* per-shard, walk child's subtree and clear @caps */
	for (si = ref.shard_first; si < ref.shard_end; si++) {
		SCX_CMASK_DEFINE_SHARD(slice, 0, SCX_CID_SHARD_MAX_CPUS);

		scx_cmask_ref_shard(&ref, si, slice);
		if (scx_cmask_empty(slice))
			continue;

		/*
		 * Pre-order with subtree skip: a descendant that cleared
		 * nothing means no descendant of it can hold @caps on these
		 * cids either.
		 */
		guard(raw_spinlock)(&parent->pshard[si]->lock);
		pos = scx_next_descendant_pre(NULL, child);
		while (pos) {
			struct scx_pshard *ps = pos->pshard[si];
			SCX_CMASK_DEFINE_SHARD(changed_cids, slice->base, slice->nr_cids);
			SCX_CMASK_DEFINE_SHARD(delta, slice->base, slice->nr_cids);
			u64 revoked_caps = 0;
			u32 cap_bit;

			scoped_guard (raw_spinlock_nested, &ps->lock) {
				/*
				 * For each cap, clear lost cids and accumulate
				 * the per-cap diff for notification.
				 */
				scx_for_each_cap_bit(cap_bit, caps) {
					struct scx_cmask *cm = &ps->caps[cap_bit].cmask;

					scx_cmask_copy(delta, cm);
					scx_cmask_and(delta, slice);
					if (scx_cmask_empty(delta))
						continue;

					scx_cmask_andnot(cm, delta);
					scx_cmask_or(changed_cids, delta);
					revoked_caps |= BIT_U64(cap_bit);
				}

				if (revoked_caps)
					caps_updated_record(ps, changed_cids, revoked_caps,
							    &to_deliver);
			}

			if (revoked_caps)
				pos = scx_next_descendant_pre(pos, child);
			else
				pos = scx_skip_subtree_pre(pos, child);
		}
	}

	caps_updated_deliver(&to_deliver);
}

/**
 * scx_bpf_sub_caps - Read self's or a direct child's cap cmasks
 * @cgroup_id: 0 for self, or a direct child's cgroup id
 * @caps: one or more SCX_CAP_* bits
 * @out__ign: arena cmask to receive the union of @caps within its range
 * @aux: implicit BPF argument
 *
 * Read the cap cmasks granted on each cid for self (@cgroup_id 0) or a direct
 * child - the literal granted set. A sched can read only itself or a direct
 * child.
 *
 * Return 0, -ENODEV if @cgroup_id names no direct child, or -EINVAL on bad
 * inputs.
 */
__bpf_kfunc s32 scx_bpf_sub_caps(u64 cgroup_id, u64 caps, struct scx_cmask *out__ign,
				 const struct bpf_prog_aux *aux)
{
	struct scx_cmask_ref ref;
	struct scx_sched *sch, *target;
	struct scx_pshard **pshard;
	s32 si, ret;

	guard(irqsave)();

	sch = scx_prog_sched(aux);
	if (unlikely(!sch))
		return -ENODEV;

	if (!scx_is_cid_type()) {
		scx_error(sch, "sub-cap kfuncs require a cid-form scheduler");
		return -EOPNOTSUPP;
	}

	if (unlikely(caps & ~__SCX_CAP_ALL)) {
		scx_error(sch, "invalid caps 0x%llx", caps);
		return -EINVAL;
	}

	/* @cgroup_id 0 reads self, otherwise a direct child */
	if (cgroup_id) {
		target = scx_find_sub_sched(cgroup_id);
		if (unlikely(!target))
			return -ENODEV;
		if (unlikely(scx_parent(target) != sch)) {
			scx_error(sch, "%s: sub-%llu is not a direct child",
				  sch->cgrp_path, cgroup_id);
			return -EINVAL;
		}
	} else {
		target = sch;
	}

	/*
	 * The target's caps storage may not be set up yet (e.g. a self-read
	 * during ops.init_cids()). Pairs with the publish in
	 * scx_alloc_pshards(): a non-NULL pshard has every element set.
	 */
	pshard = READ_ONCE(target->pshard);
	if (unlikely(!pshard)) {
		scx_error(sch, "scx_bpf_sub_caps() called before caps storage is initialized");
		return -ENODEV;
	}

	ret = scx_cmask_ref_init(sch, out__ign, &ref);
	if (ret) {
		scx_error(sch, "invalid out (%d)", ret);
		return ret;
	}

	for (si = ref.shard_first; si < ref.shard_end; si++) {
		const struct scx_cid_shard *shard = &scx_cid_shard_ranges[si];
		SCX_CMASK_DEFINE_SHARD(local_out, shard->base_cid, shard->nr_cids);
		u32 cap_bit;

		scx_for_each_cap_bit(cap_bit, caps)
			scx_cmask_or(local_out, &pshard[si]->caps[cap_bit].cmask);
		scx_cmask_ref_copy(&ref, local_out);
	}
	return 0;
}

__bpf_kfunc_end_defs();

#endif	/* CONFIG_EXT_SUB_SCHED */
