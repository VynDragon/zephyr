#include <time.h>


#include <zephyr/kernel.h>
#include <zephyr/kernel/thread.h>


#include <bl_os_adapter.h>
#include <bl_os_log.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bflb_rf_hal, CONFIG_LOG_DEFAULT_LEVEL);


#define MSGQ_MAX 32
#define WORK_QUEUE_STACK 512
#define REGULAR_PRIORITY 5
#define REGULAR_TIMEOUT_MS 150

struct bflb_wifi_callback_data {
	void *func;
	void *data;
};


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl_os_assert_func
 *
 * Description:
 *   Delete timer and free resource
 *
 * Input Parameters:
 *   file  - assert file
 *   line  - assert line
 *   func  - assert function
 *   expr  - assert condition
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bflb_os_adapter_assert_func(const char *file, int line,
                       const char *func, const char *expr)
{
	LOG_ERR("Assert failed in %s, %s:%d (%s)",
	func, file, line, expr);
}

/****************************************************************************
 * Name: bl_os_event_create
 *
 * Description:
 *   Create event group
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Event group data pointer
 *
 ****************************************************************************/

void *bflb_os_adapter_event_create(void)
{
  return (void *)0;
}

/****************************************************************************
 * Name: bl_os_event_delete
 *
 * Description:
 *   Delete event and free resource
 *
 * Input Parameters:
 *   event  - event data point
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bflb_os_adapter_event_delete(void *event)
{
}

/****************************************************************************
 * Name: bl_os_event_send
 *
 * Description:
 *   Set event bits
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Event value after setting
 *
 ****************************************************************************/

uint32_t bflb_os_adapter_event_send(void *event, uint32_t bits)
{
  return 0;
}

/****************************************************************************
 * Name: bl_os_event_wait
 *
 * Description:
 *   Delete timer and free resource
 *
 * Input Parameters:
 *   event
 *   bits_to_wait_for
 *   clear_on_exit
 *   wait_for_all_bits
 *   block_time_tick
 *
 * Returned Value:
 *   Current event value
 *
 ****************************************************************************/

uint32_t bflb_os_adapter_event_wait(void *event,
                          uint32_t bits_to_wait_for,
                          int clear_on_exit,
                          int wait_for_all_bits,
                          uint32_t block_time_tick)
{
  return 0;
}

/****************************************************************************
 * Name: bl_os_event_register
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int bflb_os_adapter_event_register(int type, void *cb, void *arg)
{
  return 0;
}

/****************************************************************************
 * Name: bl_os_event_notify
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int bflb_os_adapter_event_notify(int evt, int val)
{
//	bflb_wifi_event_handler(evt, val);
	return 0;
}

/****************************************************************************
 * Name: bl_os_task_create
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int bflb_os_adapter_task_create(const char *name,
                      void *entry,
                      uint32_t stack_depth,
                      void *param,
                      uint32_t prio,
                      void *task_handle)
{
	task_handle = k_malloc(sizeof(struct k_thread));
	k_thread_create((struct k_thread *)task_handle,
			k_thread_stack_alloc(stack_depth, 0),
			stack_depth, entry,
			param, NULL, NULL,
			prio, 0, K_NO_WAIT);
	return 0;
}

/****************************************************************************
 * Name: bl_os_task_delete
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void bflb_os_adapter_task_delete(void *task_handle)
{
	struct k_thread *handle = task_handle;

	k_thread_abort(handle);
	//k_thread_stack_free(handle->stack_obj);
	k_free(handle);
}

/****************************************************************************
 * Name: bl_os_task_get_current_task
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void *bflb_os_adapter_task_get_current_task(void)
{
	return k_current_get();
}

/****************************************************************************
 * Name: bl_os_task_notify_create
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void *bflb_os_adapter_task_notify_create(void)
{
	struct k_sem *sem= k_malloc(sizeof(struct k_sem));
	if (k_sem_init(sem, 0, 1) != 0) {
		LOG_ERR("Malloc fail task_notify_create");
		return NULL;
	}
	return sem;
}

/****************************************************************************
 * Name: bl_os_task_get_current_task
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void bflb_os_adapter_task_notify(void *task_handle)
{
	k_sem_give((struct k_sem *)task_handle);
}

/****************************************************************************
 * Name: bl_os_task_get_current_task
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void bflb_os_adapter_task_wait(void *task_handle, uint32_t tick)
{
	k_sem_take((struct k_sem *)task_handle, K_TICKS(tick));
}

/****************************************************************************
 * Name: bl_os_api_init
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int bflb_os_adapter_api_init(void)
{
  return 0;
}

/****************************************************************************
 * Name: bl_os_lock_gaint
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void bflb_os_adapter_lock_gaint(void)
{
}

/****************************************************************************
 * Name: bl_os_unlock_gaint
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void bflb_os_adapter_unlock_gaint(void)
{
}

/****************************************************************************
 * Name: bl_os_enter_critical
 *
 * Description:
 *   Enter critical state
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   CPU PS value
 *
 ****************************************************************************/

uint32_t bflb_os_adapter_enter_critical(void)
{
	return irq_lock();
}

/****************************************************************************
 * Name: bl_os_exit_critical
 *
 * Description:
 *   Exit from critical state
 *
 * Input Parameters:
 *   level - CPU PS value
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bflb_os_adapter_exit_critical(uint32_t level)
{
	irq_unlock(level);
}

/****************************************************************************
 * Name: bl_os_msleep
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int bflb_os_adapter_msleep(long msec)
{
	k_timepoint_t end_timeout = sys_timepoint_calc(K_MSEC(msec));

	while (!sys_timepoint_expired(end_timeout)) {
		k_yield();
	}
	return 0;
}

/****************************************************************************
 * Name: bl_os_sleep
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int bflb_os_adapter_sleep(unsigned int seconds)
{
  	k_timepoint_t end_timeout = sys_timepoint_calc(K_SECONDS(seconds));

	while (!sys_timepoint_expired(end_timeout)) {
		k_yield();
	}
	return 0;
}

/****************************************************************************
 * Name: bl_os_printf
 *
 * Description:
 *   Output format string and its arguments
 *
 * Input Parameters:
 *   format - format string
 *
 * Returned Value:
 *   0
 *
 ****************************************************************************/

void bflb_os_adapter_printf(const char *__fmt, ...)
{
	va_list arg;

	va_start(arg, __fmt);
	vprintf(__fmt, arg);
	va_end(arg);
}

/****************************************************************************
 * Name: bl_os_malloc
 *
 * Description:
 *   Allocate a block of memory
 *
 * Input Parameters:
 *   size - memory size
 *
 * Returned Value:
 *   Memory pointer
 *
 ****************************************************************************/

void *bflb_os_adapter_malloc(unsigned int size)
{
	return k_malloc(size);
}

/****************************************************************************
 * Name: bl_os_free
 *
 * Description:
 *   Free a block of memory
 *
 * Input Parameters:
 *   ptr - memory block
 *
 * Returned Value:
 *   No
 *
 ****************************************************************************/

void bflb_os_adapter_free(void *ptr)
{
	k_free(ptr);
}

/****************************************************************************
 * Name: bl_os_zalloc
 *
 * Description:
 *   Allocate a block of memory
 *
 * Input Parameters:
 *   size - memory size
 *
 * Returned Value:
 *   Memory pointer
 *
 ****************************************************************************/

void *bflb_os_adapter_zalloc(unsigned int size)
{
	uint8_t *d= k_malloc(size);
	memset(d, 0, size);
	return d;
}

/****************************************************************************
 * Name: bl_os_update_time
 *
 * Description:
 *   Transform ticks to time and add this time to timespec value
 *
 * Input Parameters:
 *   timespec - Input timespec data pointer
 *   ticks    - System ticks
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void bflb_os_adapter_update_time(struct timespec *timespec, uint32_t ticks)
{
	uint32_t tmp;

	tmp = ticks / CONFIG_SYS_CLOCK_TICKS_PER_SEC;
	timespec->tv_sec += tmp;

	/* zephyr runs at 1000 Hz typically */
	ticks -= CONFIG_SYS_CLOCK_TICKS_PER_SEC * tmp;
	tmp = (ticks * 1000000) / (CONFIG_SYS_CLOCK_TICKS_PER_SEC / 1000);
	timespec->tv_nsec += tmp;
}

/****************************************************************************
 * Name: bl_os_errno_trans
 *
 * Description:
 *   Transform from nuttx Os error code to Wi-Fi adapter error code
 *
 * Input Parameters:
 *   ret - NuttX error code
 *
 * Returned Value:
 *   Wi-Fi adapter error code
 *
 ****************************************************************************/

static inline int32_t bflb_os_adapter_errno_trans(int ret)
{
	if (!ret)
	{
	return true;
	}
	else
	{
	return false;
	}
}

/****************************************************************************
 * Name: bl_os_mq_creat
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void *bflb_os_adapter_mq_creat(uint32_t queue_len, uint32_t item_size)
{
	struct k_msgq *q = k_malloc(sizeof(struct k_msgq));
	void *q_buff = k_malloc(item_size * MSGQ_MAX);


	k_msgq_init(q, q_buff, item_size, MSGQ_MAX);

	return q;
}

/****************************************************************************
 * Name: bl_os_mq_delete
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void bflb_os_adapter_mq_delete(void *mq)
{
	struct k_msgq *q = mq;

	k_free(q->buffer_start);
	k_free(q);
}

/****************************************************************************
 * Name: bl_os_mq_send_generic
 *
 * Description:
 *   Generic send message to queue within a certain period of time
 *
 * Input Parameters:
 *   queue - Message queue data pointer
 *   item  - Message data pointer
 *   ticks - Wait ticks
 *   prio  - Message priority
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

int bflb_os_adapter_mq_send_wait(void *queue, void *item, uint32_t len,
                       uint32_t ticks, int prio)
{
	struct k_msgq *q = queue;
	int ret = 0;

	if (ticks == BL_OS_WAITING_FOREVER || ticks == 0)
	{
		/* Wi-Fi interrupt function will call this adapter function to send
		* message to message queue, so here we should call kernel API
		* instead of application API
		*/

		ret = k_msgq_put(q, item, K_FOREVER);
		if (ret < 0)
		{
			LOG_ERR("ERROR: Failed to send message to mqueue error=%d\n",
			ret);
		}
	}
	else
	{
		ret = k_msgq_put(q, item, K_TICKS(ticks));
		if (ret < 0)
		{
			LOG_ERR("ERROR: Failed to timedsend message to mqueue error=%d\n",
			ret);
		}
	}

	return bflb_os_adapter_errno_trans(ret);
}

/****************************************************************************
 * Name: bl_os_mq_send
 *
 * Description:
 *   Send message of low priority to queue within a certain period of time
 *
 * Input Parameters:
 *   queue - Message queue data pointer
 *   item  - Message data pointer
 *   ticks - Wait ticks
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

int bflb_os_adapter_mq_send(void *queue, void *item, uint32_t len)
{
  return bflb_os_adapter_mq_send_wait(queue, item, len, BL_OS_WAITING_FOREVER, 0);
}

/****************************************************************************
 * Name: bl_os_mq_recv
 *
 * Description:
 *   Receive message from queue within a certain period of time
 *
 * Input Parameters:
 *   queue - Message queue data pointer
 *   item  - Message data pointer
 *   ticks - Wait ticks
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

int bflb_os_adapter_mq_recv(void *queue, void *item, uint32_t len, uint32_t ticks)
{
	struct k_msgq *q = queue;
	int ret = 0;

	if (ticks == BL_OS_WAITING_FOREVER || ticks == 0)
	{
		/* Wi-Fi interrupt function will call this adapter function to send
		* message to message queue, so here we should call kernel API
		* instead of application API
		*/

		ret = k_msgq_get(q, item, K_FOREVER);
		if (ret < 0)
		{
			LOG_ERR("ERROR: Failed to send message to mqueue error=%d\n",
			ret);
		}
	}
	else
	{
		ret = k_msgq_get(q, item, K_TICKS(ticks));
		if (ret < 0)
		{
			LOG_ERR("ERROR: Failed to timedsend message to mqueue error=%d\n",
			ret);
		}
	}

	return bflb_os_adapter_errno_trans(ret);
}

/****************************************************************************
 * Name: bl_os_timer_callback
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void bflb_os_adapter_timer_callback(struct k_timer *timer)
{
	struct bflb_wifi_callback_data *data = k_timer_user_data_get(timer);

	if (data != NULL && data->func) {
		((void (*)(void *))data->func)(data->data);
	}
}

/****************************************************************************
 * Name: bl_os_timer_create
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void *bflb_os_adapter_timer_create(void *func, void *argv)
{
	struct k_timer *timer = k_malloc(sizeof(struct k_timer));
	struct bflb_wifi_callback_data *data = k_malloc(sizeof(struct bflb_wifi_callback_data));
	data->func = func;
	data->data = argv;
	k_timer_init(timer, bflb_os_adapter_timer_callback, NULL);
	k_timer_user_data_set(timer, data);
	return timer;
}

/****************************************************************************
 * Name: bl_os_timer_delete
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int bflb_os_adapter_timer_delete(void *timerid, uint32_t tick)
{
	struct k_timer *timer = timerid;
	void *data = k_timer_user_data_get(timer);

	k_timer_stop(timer);
	if (data != NULL) {
		k_free(data);
	}
	k_free(timer);
	return 0;
}

/****************************************************************************
 * Name: os_timer_start_once
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int bflb_os_adapter_timer_start_once(void *timerid, long t_sec, long t_nsec)
{
	struct k_timer *timer = timerid;

	k_timer_start(timer, K_MSEC(t_sec * 1000 + t_nsec / 1000000), K_NO_WAIT);
	return 0;
}

/****************************************************************************
 * Name: os_timer_start_periodic
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int bflb_os_adapter_timer_start_periodic(void *timerid, long t_sec, long t_nsec)
{
	struct k_timer *timer = timerid;

	k_timer_start(timer, K_MSEC(t_sec * 1000 + t_nsec / 1000000), K_MSEC(t_sec * 1000 + t_nsec /
1000000));
	return 0;
}

/****************************************************************************
 * Name: bl_os_workqueue_create
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void *bflb_os_adapter_workqueue_create(void)
{
	return k_current_get();
}

/****************************************************************************
 * Name: bl_os_workqueue_submit_hpwork
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int bflb_os_adapter_workqueue_submit_hpwork(void *work,
                                  void *worker,
                                  void *argv,
                                  long tick)
{
	bflb_os_adapter_task_notify(work);
	return 0;
}

/****************************************************************************
 * Name: bl_os_workqueue_submit_lpwork
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int bflb_os_adapter_workqueue_submit_lpwork(void *work,
                                  void *worker,
                                  void *argv,
                                  long tick)
{
	bflb_os_adapter_task_notify(work);
	return 0;
}

/****************************************************************************
 * Name: bl_os_clock_gettime_ms
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

uint64_t bflb_os_adapter_clock_gettime_ms(void)
{
	return k_uptime_get();
}

/****************************************************************************
 * Name: bl_os_get_tick
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

uint32_t bflb_os_adapter_get_tick()
{
	return k_uptime_ticks();
}

/****************************************************************************
 * Name: bl_os_isr_adpt_cb
 *
 * Description:
 *   Wi-Fi interrupt adapter callback function
 *
 * Input Parameters:
 *   arg - interrupt adapter private data
 *
 * Returned Value:
 *   0 on success
 *
 ****************************************************************************/

static void bflb_os_adapter_isr_adpt_cb(const void *arg)
{
  struct bflb_wifi_callback_data *adapter = (struct bflb_wifi_callback_data *)arg;

  ((void (*)(void*))adapter->func)(adapter->data);
}

/****************************************************************************
 * Name: bl_os_irq_attach
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void bflb_os_adapter_irq_attach(int32_t n, void *f, void *arg)
{
	int ret;
	struct bflb_wifi_callback_data *adapter;

	adapter = k_malloc(sizeof(struct bflb_wifi_callback_data));

	if (!adapter)
	{
		LOG_ERR("IRQ adapter malloc fail");
	}

	adapter->func = f;
	adapter->data  = arg;

	ret = irq_connect_dynamic(n, REGULAR_PRIORITY, bflb_os_adapter_isr_adpt_cb, (void*)adapter,
0);
	LOG_ERR("IRQ adapter attach fail");
	if (ret == 0)
	{
		LOG_ERR("IRQ adapter attach fail");
	}
}

/****************************************************************************
 * Name: bl_os_irq_enable
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void bflb_os_adapter_irq_enable(int32_t n)
{
	irq_enable(n);
}

void up_enable_irq(int32_t n)
{
	irq_enable(n);
}

/****************************************************************************
 * Name: bl_os_irq_disable
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

void bflb_os_adapter_irq_disable(int32_t n)
{
	irq_disable(n);
}

void up_disable_irq(int32_t n)
{
	irq_disable(n);
}


/****************************************************************************
 * Name: bl_os_mutex_create
 *
 * Description:
 *   Create mutex
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Mutex data pointer
 *
 ****************************************************************************/

void *bflb_os_adapter_mutex_create(void)
{
	int ret;
	struct k_mutex *mutex;

	mutex = k_malloc(sizeof(struct k_mutex));
	if (!mutex)
	{
		LOG_ERR("mutex adapter malloc fail");
		return NULL;
	}

	ret = k_mutex_init(mutex);
	if (ret != 0)
	{
		LOG_ERR("mutex adapter config fail");
		k_free(mutex);
		return NULL;
	}

	return mutex;
}

/****************************************************************************
 * Name: bl_os_mutex_delete
 *
 * Description:
 *   Delete mutex
 *
 * Input Parameters:
 *   mutex_data - mutex data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bflb_os_adapter_mutex_delete(void *mutex_data)
{
	struct k_mutex *mutex = (struct k_mutex *)mutex_data;

	/* mutex cannot be destroyed */
	k_free(mutex);
}

/****************************************************************************
 * Name: bl_os_mutex_lock
 *
 * Description:
 *   Lock mutex
 *
 * Input Parameters:
 *   mutex_data - mutex data pointer
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

int32_t bflb_os_adapter_mutex_lock(void *mutex_data)
{
	int ret;
	struct k_mutex *mutex = (struct k_mutex *)mutex_data;

	ret = k_mutex_lock(mutex, K_MSEC(REGULAR_TIMEOUT_MS));
	if (ret != 0)
	{
		LOG_ERR("mutex lock failed");
	}

	return bflb_os_adapter_errno_trans(ret);
}

/****************************************************************************
 * Name: bl_os_mutex_unlock
 *
 * Description:
 *   Lock mutex
 *
 * Input Parameters:
 *   mutex_data - mutex data pointer
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

int32_t bflb_os_adapter_mutex_unlock(void *mutex_data)
{
	int ret;
	struct k_mutex *mutex = (struct k_mutex *)mutex_data;

	ret = k_mutex_unlock(mutex);
	if (ret != 0)
	{
		LOG_ERR("mutex unlock failed");
	}

	return bflb_os_adapter_errno_trans(ret);
}

/****************************************************************************
 * Name: bl_os_sem_create
 *
 * Description:
 *   Create and initialize semaphore
 *
 * Input Parameters:
 *   max  - No mean
 *   init - semaphore initialization value
 *
 * Returned Value:
 *   Semaphore data pointer
 *
 ****************************************************************************/

void *bflb_os_adapter_sem_create(uint32_t init)
{
	struct k_sem *sem = k_malloc(sizeof(struct k_sem));
	if (k_sem_init(sem, 0, 1) != 0) {
		LOG_ERR("Malloc fail sem_create");
		return NULL;
	}
	return sem;
}

/****************************************************************************
 * Name: bl_os_sem_delete
 *
 * Description:
 *   Delete semaphore
 *
 * Input Parameters:
 *   semphr - Semaphore data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bflb_os_adapter_sem_delete(void *semphr)
{
	k_free(semphr);
}

/****************************************************************************
 * Name: bl_os_sem_take
 *
 * Description:
 *   Wait semaphore within a certain period of time
 *
 * Input Parameters:
 *   semphr - Semaphore data pointer
 *   ticks  - Wait system ticks
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

int32_t bflb_os_adapter_sem_take(void *semphr, uint32_t ticks)
{
	if (k_sem_take((struct k_sem *)semphr, K_TICKS(ticks)) == 0)
		return true;
	return false;
}

/****************************************************************************
 * Name: bl_os_sem_give
 *
 * Description:
 *   Post semaphore
 *
 * Input Parameters:
 *   semphr - Semaphore data pointer
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

int32_t bflb_os_adapter_sem_give(void *semphr)
{
	k_sem_give((struct k_sem *)semphr);
	return 1;
}

/****************************************************************************
 * Name: bl_os_log_writev
 *
 * Description:
 *   Output log with by format string and its arguments
 *
 * Input Parameters:
 *   level  - log level, no mean here
 *   tag    - log TAG, no mean here
 *   file   - file name
 *   line   - assert line
 *   format - format string
 *   args   - arguments list
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

/*static void bflb_os_adapter_log_writev(uint32_t level,
                             const char *tag,
                             const char *file,
                             int line,
                             const char *format,
                             va_list args)
{
  switch (level)
    {
    case LOG_LEVEL_ERROR:
      {
        LOG_ERR(format, args);
        break;
      }

    case LOG_LEVEL_WARN:
      {
         LOG_WRN(format, args);
        break;
      }

    case LOG_LEVEL_INFO:
      {
        LOG_INF(format, args);
        break;
      }
    }
}*/

/****************************************************************************
 * Name: bl_os_log_write
 *
 * Description:
 *   Output log with by format string and its arguments
 *
 * Input Parameters:
 *   level  - log level, no mean here
 *   file   - file name
 *   line   - assert line
 *   tag    - log TAG, no mean here
 *   format - format string
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bflb_os_adapter_log_write(uint32_t level,
                     const char *tag,
                     const char *file,
                     int line,
                     const char *format,
                     ...)
{
	va_list list;
	va_start(list, format);
	vprintf(format, list);
	va_end(list);
}



bl_ops_funcs_t g_bl_ops_funcs =
{
  ._version = BL_OS_ADAPTER_VERSION,
  ._printf = bflb_os_adapter_printf,
  ._assert = bflb_os_adapter_assert_func,
  ._init = bflb_os_adapter_api_init,
  ._enter_critical = bflb_os_adapter_enter_critical,
  ._exit_critical = bflb_os_adapter_exit_critical,
  ._msleep = bflb_os_adapter_msleep,
  ._sleep = bflb_os_adapter_sleep,
  ._event_group_create = bflb_os_adapter_event_create,
  ._event_group_delete = bflb_os_adapter_event_delete,
  ._event_group_send = bflb_os_adapter_event_send,
  ._event_group_wait = bflb_os_adapter_event_wait,
  ._event_register = bflb_os_adapter_event_register,
  ._event_notify = bflb_os_adapter_event_notify,
  ._task_create = bflb_os_adapter_task_create,
  ._task_delete = bflb_os_adapter_task_delete,
  ._task_get_current_task = bflb_os_adapter_task_get_current_task,
  ._task_notify_create = bflb_os_adapter_task_notify_create,
  ._task_notify = bflb_os_adapter_task_notify,
  ._task_wait = bflb_os_adapter_task_wait,
  ._lock_gaint = bflb_os_adapter_lock_gaint,
  ._unlock_gaint = bflb_os_adapter_unlock_gaint,
  ._irq_attach = bflb_os_adapter_irq_attach,
  ._irq_enable = bflb_os_adapter_irq_enable,
  ._irq_disable = bflb_os_adapter_irq_disable,
  ._workqueue_create = bflb_os_adapter_workqueue_create,
  ._workqueue_submit_hp = bflb_os_adapter_workqueue_submit_hpwork,
  ._workqueue_submit_lp = bflb_os_adapter_workqueue_submit_lpwork,
  ._timer_create = bflb_os_adapter_timer_create,
  ._timer_delete = bflb_os_adapter_timer_delete,
  ._timer_start_once = bflb_os_adapter_timer_start_once,
  ._timer_start_periodic = bflb_os_adapter_timer_start_periodic,
  ._sem_create = bflb_os_adapter_sem_create,
  ._sem_delete = bflb_os_adapter_sem_delete,
  ._sem_take = bflb_os_adapter_sem_take,
  ._sem_give = bflb_os_adapter_sem_give,
  ._mutex_create = bflb_os_adapter_mutex_create,
  ._mutex_delete = bflb_os_adapter_mutex_delete,
  ._mutex_lock = bflb_os_adapter_mutex_lock,
  ._mutex_unlock = bflb_os_adapter_mutex_unlock,
  ._queue_create = bflb_os_adapter_mq_creat,
  ._queue_delete = bflb_os_adapter_mq_delete,
  ._queue_send_wait = bflb_os_adapter_mq_send_wait,
  ._queue_send = bflb_os_adapter_mq_send,
  ._queue_recv = bflb_os_adapter_mq_recv,
  ._malloc = bflb_os_adapter_malloc,
  ._free = bflb_os_adapter_free,
  ._zalloc = bflb_os_adapter_zalloc,
  ._get_time_ms = bflb_os_adapter_clock_gettime_ms,
  ._get_tick = bflb_os_adapter_get_tick,
  ._log_write = bflb_os_adapter_log_write
};
