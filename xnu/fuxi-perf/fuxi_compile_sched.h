/* Original Fuxi scheduler helpers, included after thread_bind_internal.
 * Governor mutex -> thread locks -> pset lock, never the reverse.
 * Physical Aff1 is translated; logical CPU 7 does not exist on this board.
 */
#if defined(FUXI)
int fuxi_compile_bind(unsigned domains);
void fuxi_compile_unbind(thread_t thread, int physical);
int fuxi_compile_alive(thread_t thread);
int fuxi_compile_online(int physical);
void fuxi_compile_migrate(void);

int
fuxi_compile_online(int physical)
{
 int cpu = ml_get_cpu_number((uint32_t)physical << 8);
 processor_t p = cpu < 0 ? PROCESSOR_NULL : cpu_to_processor(cpu);
 /* cpu_data is published before processor_init() sets processor_set, and
  * fuxicores without fuxiperf polls while SweetCPU registers CPUs. A CPU
  * still registering is offline; its pset, once set, never changes. */
 if (p == PROCESSOR_NULL || !p->processor_set) return 0;
 spl_t s = splsched();
 pset_lock(p->processor_set);
 int online = p->state >= PROCESSOR_IDLE;
 pset_unlock(p->processor_set);
 splx(s);
 return online;
}

int
fuxi_compile_bind(unsigned domains)
{
 thread_t self = current_thread();
 for (int physical = 7; physical >= 3; physical--) {
  if (!(domains & (physical == 7 ? 4u : 2u))) continue;
  if (!fuxi_compile_online(physical)) continue;
  spl_t s = splsched();
  thread_lock(self);
  /* Same locking as thread_bind(), with eligibility checked atomically.
   * Preserve existing binding; avoid a racing realtime-policy assertion. */
  int eligible = self->bound_processor == PROCESSOR_NULL &&
      self->sched_pri < BASEPRI_RTQUEUES;
  if (eligible)
   (void)thread_bind_internal(self,cpu_to_processor(ml_get_cpu_number((uint32_t)physical << 8)));
  thread_unlock(self);
  splx(s);
  if (!eligible) return -2; /* realtime or already bound: lease unbound */
  return physical;
 }
 return -1;
}

void
fuxi_compile_migrate(void)
{
 /* Outside the governor mutex, so timeout/offline cleanup can unbind even
  * while migration is pending. Binding alone does not move the caller. */
 thread_block(THREAD_CONTINUE_NULL);
}

int
fuxi_compile_alive(thread_t thread)
{
 thread_mtx_lock(thread);
 int alive = thread->active;
 thread_mtx_unlock(thread);
 return alive;
}

void
fuxi_compile_unbind(thread_t thread, int physical)
{
 int cpu = ml_get_cpu_number((uint32_t)physical << 8);
 processor_t expected = cpu < 0 ? PROCESSOR_NULL : cpu_to_processor(cpu);
 spl_t s = splsched();
 thread_lock(thread);
 if (expected != PROCESSOR_NULL && thread->bound_processor == expected) {
  /* Same remove/reinsert discipline as the VM bind group. Clearing a
   * binding is safe for a running/in-flight thread; no remote rebind. */
  boolean_t removed = thread_run_queue_remove(thread);
  if (thread->sched_pri < BASEPRI_RTQUEUES) {
   (void)thread_bind_internal(thread, PROCESSOR_NULL);
  } else {
   /* Policy can change after admission. Clearing an existing binding must
    * not hit thread_bind_internal's assertion against binding RT threads.
    * Both RT and normal runqueues use the remove/reinsert APIs above/below. */
   thread->bound_processor = PROCESSOR_NULL;
  }
  if (removed) thread_run_queue_reinsert(thread, SCHED_PREEMPT | SCHED_TAILQ);
 }
 thread_unlock(thread);
 splx(s);
}
#endif
