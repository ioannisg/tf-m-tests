/*
 * Copyright (c) 2017-2021, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "tfm_api.h"
#include "cmsis_os2.h"
#include "tfm_integ_test.h"
#include "tfm_ns_svc.h"
#include "tfm_ns_interface.h"
#if defined(TEST_FRAMEWORK_NS) || defined(TEST_FRAMEWORK_S)
#include "test_framework_integ_test.h"
#endif
#ifdef PSA_API_TEST_NS
#include "psa_api_test.h"
#endif
#include "tfm_plat_ns.h"
#include "driver/Driver_USART.h"
#include "device_cfg.h"
#ifdef TFM_MULTI_CORE_TOPOLOGY
#include "tfm_multi_core_api.h"
#include "tfm_ns_mailbox.h"
#endif
#include "tfm_log.h"
#include "uart_stdout.h"

#ifdef TFM_PSA_API
#include "psa_manifest/sid.h"
#endif

#include <cmsis.h>

/**
 * \brief Modified table template for user defined SVC functions
 *
 * \details RTX has a weak definition of osRtxUserSVC, which
 *          is overridden here
 */
#if (defined(__ARMCC_VERSION) && (__ARMCC_VERSION == 6110004))
/* Workaround needed for a bug in Armclang 6.11, more details at:
 * http://www.keil.com/support/docs/4089.htm
 */
__attribute__((section(".gnu.linkonce")))
#endif
extern void * const osRtxUserSVC[1+USER_SVC_COUNT];
       void * const osRtxUserSVC[1+USER_SVC_COUNT] = {
  (void *)USER_SVC_COUNT,

#define X(SVC_ENUM, SVC_HANDLER) (void*)SVC_HANDLER,

    /* SVC API for Services */
#ifdef TFM_NS_CLIENT_IDENTIFICATION
    LIST_SVC_NSPM
#endif

#undef X

/*
 * (void *)user_function1,
 *  ...
 */
};

/**
 * \brief List of RTOS thread attributes
 */
#if defined(TEST_FRAMEWORK_NS) || defined(TEST_FRAMEWORK_S) \
 || defined(PSA_API_TEST_NS)
static const osThreadAttr_t thread_attr = {
    .name = "test_thread",
    .stack_size = 4096U
};
#else
static const osThreadAttr_t thread_attr = {
    .name = "main_application_thread",
    .stack_size = 4096U
};
#endif

#ifdef TFM_MULTI_CORE_NS_OS_MAILBOX_THREAD
static osThreadFunc_t mailbox_thread_func = tfm_ns_mailbox_thread_runner;
static const osThreadAttr_t mailbox_thread_attr = {
    .name = "mailbox_thread",
    .stack_size = 1024U
};
#endif

/**
 * \brief Static globals to hold RTOS related quantities,
 *        main thread
 */
#if defined(TEST_FRAMEWORK_NS) || defined(TEST_FRAMEWORK_S) \
 || defined(PSA_API_TEST_NS)
static osThreadFunc_t thread_func;
#else
static osThreadFunc_t main_application_thread;
#endif

#ifdef TFM_MULTI_CORE_TOPOLOGY
static struct ns_mailbox_queue_t ns_mailbox_queue;

static void tfm_ns_multi_core_boot(void)
{
    int32_t ret;

    LOG_MSG("Non-secure code running on non-secure core.");

    if (tfm_ns_wait_for_s_cpu_ready()) {
        LOG_MSG("Error sync'ing with secure core.");

        /* Avoid undefined behavior after multi-core sync-up failed */
        for (;;) {
        }
    }

    ret = tfm_ns_mailbox_init(&ns_mailbox_queue);
    if (ret != MAILBOX_SUCCESS) {
        LOG_MSG("Non-secure mailbox initialization failed.");

        /* Avoid undefined behavior after NS mailbox initialization failed */
        for (;;) {
        }
    }
}
#else
extern uint32_t tfm_ns_interface_init(void);
#endif

/**
 * \brief Platform peripherals and devices initialization.
 *        Can be overridden for platform specific initialization.
 *
 * \return  ARM_DRIVER_OK if the initialization succeeds
 */
__WEAK int32_t tfm_ns_platform_init(void)
{
    stdio_init();

    return ARM_DRIVER_OK;
}

/**
 * \brief Platform peripherals and devices de-initialization.
 *        Can be overridden for platform specific initialization.
 *
 * \return  ARM_DRIVER_OK if the de-initialization succeeds
 */
__WEAK int32_t tfm_ns_platform_uninit(void)
{
    stdio_uninit();

    return ARM_DRIVER_OK;
}

void main_thread(void *argument)
{
    UNUSED_VARIABLE(argument);

    printf("Enable use of FP registers\r\n");
    SCB->CPACR |= (0x3 << 20) | (0x3 | 22);

    printf("Enable automatic state preservation and lazy stacking\n");
    FPU->FPCCR = FPU_FPCCR_ASPEN_Msk | FPU_FPCCR_LSPEN_Msk;

    /* Confirm CONTROL.FPCA has not been set yet */
    printf("CONTROL_NS.FPCA: %u\r\n", (__get_CONTROL() & CONTROL_FPCA_Msk) >> CONTROL_FPCA_Pos);

    /* Enable Interrupt */
    printf("TIMER1IRQn %u\n", TIMER1_IRQn);
    NVIC_ClearPendingIRQ(TIMER1_IRQn);
    NVIC_EnableIRQ(TIMER1_IRQn);
    __DSB();
    __ISB();

    printf("Make use of FP registers\n");

    uint32_t result;
    __ASM volatile ("VMRS %0, fpscr" : "=r" (result) );
    __DSB();
    __ISB();
    /* Confirm CONTROL.FPCA is now been set */
    printf("CONTROL_NS.FPCA: %u\r\n", (__get_CONTROL() & CONTROL_FPCA_Msk) >> CONTROL_FPCA_Pos);

    /* Call a Secure function while CONTROL.FPCA is set. */
    (void)psa_connect(IPC_SERVICE_TEST_PSA_ACCESS_APP_READ_ONLY_MEM_SID,
                         IPC_SERVICE_TEST_PSA_ACCESS_APP_READ_ONLY_MEM_VERSION);

    /* The Secure domain will simulate a NS IRQ firing, which will
     * attempt to use the FP registers.
     */

    /* Test completes here. However, the System
     * crashes before seeing this message.
     */
    printf("TEST COMPLETED SUCCESSFULLY\n");
}

/**
 * \brief main() function
 */
#ifndef __GNUC__
__attribute__((noreturn))
#endif
int main(void)
{
    if (tfm_ns_platform_init() != ARM_DRIVER_OK) {
        /* Avoid undefined behavior if platform init failed */
        while(1);
    }

    (void) osKernelInitialize();

#ifdef TFM_MULTI_CORE_TOPOLOGY
    tfm_ns_multi_core_boot();
#else
    /* Initialize the TFM NS interface */
    tfm_ns_interface_init();
#endif

#ifdef TFM_MULTI_CORE_NS_OS_MAILBOX_THREAD
    (void) osThreadNew(mailbox_thread_func, NULL, &mailbox_thread_attr);
#endif

#if defined(TEST_FRAMEWORK_NS) || defined(TEST_FRAMEWORK_S)
    thread_func = test_app;
#elif defined(PSA_API_TEST_NS)
    thread_func = psa_api_test;
#else
    main_application_thread = main_thread;
#endif

#if defined(TEST_FRAMEWORK_NS) || defined(TEST_FRAMEWORK_S) \
 || defined(PSA_API_TEST_NS)
    (void) osThreadNew(thread_func, NULL, &thread_attr);
#else
    (void) osThreadNew(main_application_thread, NULL, &thread_attr);
#endif

    LOG_MSG("Non-Secure system starting...\r\n");
    (void) osKernelStart();

    /* Reached only in case of error */
    for (;;) {
    }
}

void TIMER1_Handler(void)
{
    uint32_t result;

    /* ISR makes use of FP registers. If LSPACT is set,
     * state preservation will be activated.
     */
    __ASM volatile ("VMRS %0, fpscr" : "=r" (result) );
    __DSB();
    __ISB();

    (void)result;
}
