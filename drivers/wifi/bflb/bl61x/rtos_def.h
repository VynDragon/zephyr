/**
 * @file rtos_def.h
 * @brief
 *
 * Copyright (c) 2023 Bouffalolab team
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 */

#ifndef RTOS_DEF_H_
#define RTOS_DEF_H_

#include <zephyr/kernel.h>

#include <stdbool.h>
/*
 * DEFINITIONS
 ****************************************************************************************
 */
/// RTOS task handle
typedef k_tid_t			rtos_task_handle;

/// RTOS priority
typedef int			rtos_prio;

/// RTOS task function
typedef k_thread_entry_t	rtos_task_fct;

/// RTOS queue
typedef struct k_msgq*		rtos_queue;

/// RTOS semaphore
typedef struct k_sem*		rtos_semaphore;

/// RTOS mutex
typedef struct k_mutex*		rtos_mutex;


#define RTOS_WORD_SIZE 4

#define WL_API_RMEM_ADDR  0x20010600

#endif // RTOS_DEF_H_
