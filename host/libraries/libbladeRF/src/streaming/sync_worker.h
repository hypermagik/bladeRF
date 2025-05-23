/*
 * Copyright (C) 2014 Nuand LLC
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 */

#ifndef STREAMING_SYNC_WORKER_H_
#define STREAMING_SYNC_WORKER_H_

#include "host_config.h"
#include "sync.h"
#include <libbladeRF.h>
#include "thread.h"

#if BLADERF_OS_WINDOWS || BLADERF_OS_OSX
#include "clock_gettime.h"
#else
#include <time.h>
#endif

/* Worker lifetime:
 *
 * STARTUP --+--> IDLE --> RUNNING --+--> SHUTTING_DOWN --> STOPPED
 *           ^----------------------/
 */

/* Request flags */
#define SYNC_WORKER_START (1 << 0)
#define SYNC_WORKER_STOP (1 << 1)

typedef enum {
    SYNC_WORKER_STATE_STARTUP,
    SYNC_WORKER_STATE_IDLE,
    SYNC_WORKER_STATE_RUNNING,
    SYNC_WORKER_STATE_SHUTTING_DOWN,
    SYNC_WORKER_STATE_STOPPED
} sync_worker_state;

struct sync_worker {
    THREAD thread;

    struct bladerf_stream *stream;
    bladerf_stream_cb cb;

    /* These items should be accessed while holding state_lock */
    sync_worker_state state;
    int err_code;
    MUTEX state_lock;
    COND state_changed;           /* Worker thread uses this to inform a
                                   * waiting main thread about a state
                                   * change */

    /* The requests lock should always be acquired AFTER
     * the sync->buf_mgmt.lock
     */
    unsigned int requests;
    COND requests_pending;
    MUTEX request_lock;
};

/**
 * Create a launch a worker thread. It will enter the IDLE state upon
 * executing.
 *
 * @param   s   Sync handle containing worker to initialize
 *
 * @return 0 on success, BLADERF_ERR_* on failure
 */
int sync_worker_init(struct bladerf_sync *s);

/**
 * Shutdown and deinitialize
 *
 * @param       w       Worker to deinitialize
 * @param[in]   lock    Acquired to signal `cond` if non-NULL
 * @param[in]   cond    If non-NULL, this is signaled after requesting the
 *                      worker to shut down, waking a potentially blocked
 *                      workers.
 */
void sync_worker_deinit(struct sync_worker *w,
                        MUTEX *lock,
                        COND *cond);

/**
 * Wait for state change with optional timeout
 *
 * @param       w           Worker to wait for
 * @param[in]   state       State to wait for
 * @param[in]   timeout_ms  Timeout in ms. 0 implies "wait forever"
 *
 * @return 0 on success, BLADERF_ERR_TIMEOUT on timeout, BLADERF_ERR_UNKNOWN on
 * other errors
 */
int sync_worker_wait_for_state(struct sync_worker *w,
                               sync_worker_state state,
                               unsigned int timeout_ms);

/**
 * Get the worker's current state.
 *
 * @param       w           Worker to query
 * @param[out]  err_code    Stream error code (libbladeRF error code value).
 *                          Querying this value will reset the interal error
 *                          code value.
 *
 * @return Worker's current state
 */
sync_worker_state sync_worker_get_state(struct sync_worker *w, int *err_code);

/**
 * Submit a request to the worker task
 *
 * @param       w           Worker to send request to
 * @param[in]   request     Bitmask of requests to submit
 */
void sync_worker_submit_request(struct sync_worker *w, unsigned int request);

#endif
