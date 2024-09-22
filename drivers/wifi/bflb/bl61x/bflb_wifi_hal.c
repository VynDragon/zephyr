/*
 * Copyright (c) 2024 MASSDRIVER EI
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bflb_bl61x_wifi_hal, CONFIG_WIFI_LOG_LEVEL);

#include "rtos_def.h"
#include "rtos_al.h"
#include "platform_al.h"

K_HEAP_DEFINE(bl61x_wifi_heap, CONFIG_WIFI_BFLB_BL61X_HEAP_SIZE);
K_THREAD_STACK_DEFINE(bl61x_wifi_stack, CONFIG_WIFI_BFLB_BL61X_STACK_SIZE);

uint32_t rtos_now(bool isr)
{
	return k_uptime_get_32();
}

void *rtos_malloc(uint32_t size)
{
	return k_heap_alloc(&bl61x_wifi_heap, size, K_NO_WAIT);
}

void *rtos_calloc(uint32_t nb_elt, uint32_t size)
{
	return k_heap_alloc(&bl61x_wifi_heap, size * nb_elt, K_NO_WAIT);
}

void rtos_free(void *ptr)
{
	k_heap_free(&bl61x_wifi_heap, ptr);
}

int rtos_task_create(rtos_task_fct func,
                     const char * const name,
                     enum rtos_task_id task_id,
                     const uint16_t stack_depth,
                     void * const params,
                     rtos_prio prio,
                     rtos_task_handle * const task_handle)
{
	struct k_thread *thread = rtos_malloc(sizeof(struct k_thread));
	thread->custom_data = rtos_malloc(sizeof(struct k_sem));

	k_thread_create(thread, bl61x_wifi_stack, stack_depth, func, params, NULL, NULL, prio, 0, K_NO_WAIT);
	k_thread_name_set(thread, name);
	k_sem_init(thread->custom_data, 0, 256);
	if (task_handle) {
		*task_handle = thread;
	 }
	return 0;
}

void rtos_task_delete(rtos_task_handle task_handle)
{
	if (task_handle == 0) {
		/*struct k_thread cpy = *k_current_get();
		rtos_free(k_current_get()->custom_data);
		rtos_free(k_current_get());
		k_thread_abort(&cpy);*/
		return;
	}
	k_thread_abort(task_handle);
	rtos_free(task_handle->custom_data);
	rtos_free(task_handle);
}

void rtos_task_suspend(int duration)
{
	k_msleep(duration);
}

int rtos_task_init_notification(rtos_task_handle task)
{
	return 0;
}

int rtos_task_wait_notification(int timeout)
{
	int		ret = 0;
	uint32_t	cnt = k_sem_count_get(k_thread_custom_data_get());

	if (timeout == 0) {
		ret = k_sem_take(k_thread_custom_data_get(), K_NO_WAIT);
	} else if (timeout == -1) {
		ret = k_sem_take(k_thread_custom_data_get(), K_FOREVER);
	} else {
		ret = k_sem_take(k_thread_custom_data_get(), K_MSEC(timeout));
	}

	if (ret < 0) {
		return 0;
	}
	k_sem_reset(k_thread_custom_data_get());
	return (cnt == 0 ? 1 : cnt);
}

void rtos_task_notify(rtos_task_handle task, bool isr)
{
	k_sem_give(task->custom_data);
}

void rtos_priority_set(rtos_task_handle handle, rtos_prio priority)
{
	k_thread_priority_set(handle, priority);
}

rtos_task_handle rtos_get_task_handle(void)
{
	return k_current_get();
}

int rtos_queue_create(int elt_size, int nb_elt, rtos_queue *queue)
{
	*queue = rtos_malloc(sizeof(struct k_msgq));

	if (*queue == NULL) {
		return -1;
	}

	k_msgq_alloc_init(*queue, elt_size, nb_elt);

	return 0;
}

void rtos_queue_delete(rtos_queue queue)
{
	k_msgq_cleanup(queue);
	rtos_free(queue);
}

bool rtos_queue_is_empty(rtos_queue queue)
{
	return k_msgq_num_used_get(queue) == 0 ? true : false;
}

bool rtos_queue_is_full(rtos_queue queue)
{
	return k_msgq_num_free_get(queue) == 0 ? true : false;
}

int rtos_queue_cnt(rtos_queue queue)
{
	return k_msgq_num_used_get(queue);
}

int rtos_queue_write(rtos_queue queue, void *msg, int timeout, bool isr)
{
	int ret = 0;

	if (isr) {
		ret = k_msgq_put(queue, msg, K_NO_WAIT);
	} else if (timeout == 0) {
		ret = k_msgq_put(queue, msg, K_NO_WAIT);
	} else if (timeout == -1) {
		ret = k_msgq_put(queue, msg, K_FOREVER);
	} else {
		ret = k_msgq_put(queue, msg, K_MSEC(timeout));
	}
	return ret;
}

int rtos_queue_read(rtos_queue queue, void *msg, int timeout, bool isr)
{
	int ret = 0;

	if (isr) {
		ret = k_msgq_get(queue, msg, K_NO_WAIT);
	} else if (timeout == 0) {
		ret = k_msgq_get(queue, msg, K_NO_WAIT);
	} else if (timeout == -1) {
		ret = k_msgq_get(queue, msg, K_FOREVER);
	} else {
		ret = k_msgq_get(queue, msg, K_MSEC(timeout));
	}
	return ret;
}

int rtos_semaphore_create(rtos_semaphore *semaphore, int max_count, int init_count)
{
	*semaphore = rtos_malloc(sizeof(struct k_sem));

	if (*semaphore == NULL) {
		return -1;
	}

	k_sem_init(*semaphore, init_count, max_count);
	return 0;
}

int rtos_semaphore_get_count(rtos_semaphore semaphore)
{
	return k_sem_count_get(semaphore);
}

void rtos_semaphore_delete(rtos_semaphore semaphore)
{
	rtos_free(semaphore);
}

int rtos_semaphore_wait(rtos_semaphore semaphore, int timeout)
{
	int	ret = 0;

	if (timeout == 0) {
		ret = k_sem_take(semaphore, K_NO_WAIT);
	} else if (timeout == -1) {
		ret = k_sem_take(semaphore, K_FOREVER);
	} else {
		ret = k_sem_take(semaphore, K_MSEC(timeout));
	}

	return ret;
}

int rtos_semaphore_signal(rtos_semaphore semaphore, bool isr)
{
	k_sem_give(semaphore);
	return 0;
}

int rtos_mutex_create(rtos_mutex *mutex) {
	*mutex = rtos_malloc(sizeof(struct k_mutex));

	if (*mutex == NULL) {
		return -1;
	}

	k_mutex_init(*mutex);
	return 0;
}

void rtos_mutex_delete(rtos_mutex mutex)
{
	rtos_free(mutex);
}

void rtos_mutex_lock(rtos_mutex mutex)
{
	k_mutex_lock(mutex, K_FOREVER);
}

void rtos_mutex_unlock(rtos_mutex mutex)
{
	k_mutex_unlock(mutex);
}

rtos_task_handle rtos_mutex_get_holder(rtos_mutex mutex)
{
	return mutex->owner;
}

int rtos_init(void)
{
	/* mamma mia we have already done the initialization */
	return 0;
}

void platform_post_event(int catalogue, int code, ...)
{
	bflb_wifi_event_handler(catalogue, code);
}

void *platform_malloc(uint32_t size)
{
	return rtos_malloc(size);
}

void platform_free(void* mem_ptr)
{
	rtos_free(mem_ptr);
}
