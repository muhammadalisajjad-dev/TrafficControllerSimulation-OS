# OS Project Spring 2026 — Two intersections (F10 / F11)

Console simulation with **pthread vehicle threads**, **anonymous semaphores** for each parking lot (10 spots + 5 queue slots), and **two controller processes** created with `fork()` that coordinate emergencies over **two unidirectional pipes** (bidirectional IPC).

## Build

```bash
make
```

Requires GCC with POSIX threads and C11 (`sem_init` process-shared semaphores live inside `mmap`).

## Run

```bash
./simulation          # default 15 vehicles
./simulation 10     # up to 15 vehicles (values outside range reset to 15)
```

Stop early with **Ctrl+C** (`SIGINT`): shutdown flag is raised; finish behavior depends on thread state (normally join vehicles after natural completion).

## Files

| File | Role |
|------|------|
| `simulation.c` | Parent: mmap shared state, pipes, fork controllers, vehicle pthreads, parking semaphores |
| `controller.c` | Per-intersection traffic phases + pipe protocol + emergency clearing |
| `shm.c` | Process-shared mutex/cond/emergency semaphores initialization |
| `shared_state.h`, `common.h`, `ipc.h`, `controller.h`, `shm.h` | Shared definitions |

## Rubric mapping (short)

- **Multi-process:** `fork()` twice for F10/F11; `pipe()` pair for F10→F11 and F11→F10 messages.
- **Threads:** One pthread per vehicle (`NUM_VEHICLES` default 15), staggered `usleep` arrivals.
- **Parking:** Per intersection `sem_t` spots (10) and queue (5); `sem_trywait` immediate spot else queue or fail; **never** acquired while holding intersection reservation (parking runs before crossing).
- **Traffic:** NS/EW phases with opposing straight concurrency; emergency vehicles bypass phase and trigger peer clearing before connector travel.
- **Cleanup:** `pthread_join`, `sem_destroy` parking + shared emergency semaphores, `munmap`, `waitpid`.
