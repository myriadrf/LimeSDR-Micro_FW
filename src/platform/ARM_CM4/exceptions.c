/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2017, 2021 NXP
 */

#include "config.h"
#include "debug_console.h"
#include "exceptions.h"
#include "core_cm4.h"
#include "io.h"

#include "log.h"

static void prvPrintUsageFaultReason( uint32_t ulVal )
{
    log_err("Usage fault: ");

    if( ( ulVal & ( 1 << 0 ) ) )
    {
        log_err("Undefined instruction" LOG_EOL);
    }
    else if( ( ulVal & ( 1 << 1 ) ) )
    {
        log_err("Invalid state Usage Fault" LOG_EOL);
    }
    else if( ( ulVal & ( 1 << 2 ) ) )
    {
        log_err("Invalid PC load UsageFault caused by invalid EXC_RETURN value" LOG_EOL);
    }
    else if( ( ulVal & ( 1 << 3 ) ) )
    {
        log_err("Coprocessor access error" LOG_EOL);
    }
    else if( ( ulVal & ( 1 << 8 ) ) )
    {
        log_err("Unaligned access error" LOG_EOL);
    }
    else if( ( ulVal & ( 1 << 9 ) ) )
    {
        log_err("Divide by zero" LOG_EOL);
    }
}

static void prvPrintBusFaultReason( uint32_t ulVal )
{
    log_err("Bus fault: ");

    if( ( ulVal & ( 1 << 0 ) ) )
    {
        log_err("This bit indicates a bus fault on an instruction prefetch" LOG_EOL);
    }
    else if( ( ulVal & ( 1 << 2 ) ) )
    {
        log_err("Imprecise data access error" LOG_EOL);
    }
    else if( ( ulVal & ( 1 << 3 ) ) )
    {
        log_err("bus fault has occurred on exception return" LOG_EOL);
    }
    else if( ( ulVal & ( 1 << 4 ) ) )
    {
        log_err("Bus fault has occurred on exception entry" LOG_EOL);
    }
    else if( ( ulVal & ( 1 << 5 ) ) )
    {
        log_err(" fault occurred during floating-point lazy state preservation" LOG_EOL);
    }
    else if( ( ulVal & ( 1 << 7 ) ) && ( ulVal & ( 1 << 1 ) ) )
    {
        log_err("Precise data access error. Bus Fault Address Register is: %x" LOG_EOL, SCB->BFAR);
    }
}

static void prvPrintMemMngFaultReason( uint32_t ulVal )
{
    log_err("Memory fault: ");

    if( ( ulVal & ( 1 << 0 ) ) )
    {
        log_err("Instruction access violation" LOG_EOL);
    }
    else if( ( ulVal & ( 1 << 1 ) ) )
    {
        log_err("Data access violation" LOG_EOL);
    }
    else if( ( ulVal & ( 1 << 3 ) ) )
    {
        log_err("Memory Management Fault on unstacking for a return from exception" LOG_EOL);
    }
    else if( ( ulVal & ( 1 << 4 ) ) )
    {
        log_err("Memory Management Fault on stacking for exception entry" LOG_EOL);
    }
    else if( ( ulVal & ( 1 << 5 ) ) )
    {
        log_err("Memory Management Fault during floating point lazy state preservation" LOG_EOL);
    }

    if( ( ulVal & ( 1 << 7 ) ) )
    {
        log_err("Memory management fault address : %x" LOG_EOL, SCB->MMFAR);
    }
}

static void prvGetHardFaultReason( void )
{
    uint32_t ulCFSRValue;

    log_err("SCB->CFSR = 0x%08x" LOG_EOL, SCB->CFSR);
    log_err("SCB->HFSR = 0x%08x" LOG_EOL, SCB->HFSR);

    ulCFSRValue = SCB->CFSR;

    if( ( SCB->HFSR & SCB_HFSR_DEBUGEVT_MSK ) )
    {
        log_err("Debug Event Hard Fault" LOG_EOL);
        log_err("SCB->DFSR = 0x%08x\n", SCB->DFSR);
    }
    else if( ( SCB->HFSR & SCB_HFSR_VECTTBL_MSK ) )
    {
        log_err("Fault was due to vector table read on exception processing" LOG_EOL);
    }
    else if( ( SCB->HFSR & SCB_HFSR_FORCED_MSK ) )
    {
        log_err("Forced Hard Fault" LOG_EOL);

        if( ( SCB->CFSR & SCB_CFSR_USGFAULTSR_MSK ) )
        {
            ulCFSRValue >>= SCB_CFSR_USGFAULTSR_POS;
            prvPrintUsageFaultReason( ulCFSRValue );
        }
        else if( ( SCB->CFSR & SCB_CFSR_BUSFAULTSR_MSK ) )
        {
            ulCFSRValue >>= SCB_CFSR_BUSFAULTSR_POS;
            prvPrintBusFaultReason( ulCFSRValue );
        }
        else if( ( SCB->CFSR & SCB_CFSR_MEMFAULTSR_MSK ) )
        {
            ulCFSRValue >>= SCB_CFSR_MEMFAULTSR_POS;
            prvPrintMemMngFaultReason( ulCFSRValue );
        }
    }

    isb();
    dmb();

    while( 1 )
    {
    }
}

static void prvDumpStackFrame( uint32_t * pulMSP )
{
    volatile uint32_t stacked_r0;
    volatile uint32_t stacked_r1;
    volatile uint32_t stacked_r2;
    volatile uint32_t stacked_r3;
    volatile uint32_t stacked_r12;
    volatile uint32_t stacked_lr;
    volatile uint32_t stacked_pc;
    volatile uint32_t stacked_psr;

    stacked_r0 = ( ( uint32_t ) pulMSP[ 0 ] );
    stacked_r1 = ( ( uint32_t ) pulMSP[ 1 ] );
    stacked_r2 = ( ( uint32_t ) pulMSP[ 2 ] );
    stacked_r3 = ( ( uint32_t ) pulMSP[ 3 ] );
    stacked_r12 = ( ( uint32_t ) pulMSP[ 4 ] );
    stacked_lr = ( ( uint32_t ) pulMSP[ 5 ] );
    stacked_pc = ( ( uint32_t ) pulMSP[ 6 ] );
    stacked_psr = ( ( uint32_t ) pulMSP[ 7 ] );

    log_err("========STACK FRAME=========" LOG_EOL);
    log_err("\tr0 : %x" LOG_EOL, stacked_r0);
    log_err("\tr1 : %x" LOG_EOL, stacked_r1);
    log_err("\tr2 : %x" LOG_EOL, stacked_r2);
    log_err("\tr3 : %x" LOG_EOL, stacked_r3);
    log_err("\tr12 : %x" LOG_EOL, stacked_r12);
    log_err("\tlr : %x" LOG_EOL, stacked_lr);
    log_err("\tpc : %x" LOG_EOL, stacked_pc);
    log_err("\tpsr : %x" LOG_EOL, stacked_psr);
    log_err("=====END OF STACK FRAME======" LOG_EOL);
}

void vAnalyzeFault( uint32_t * pulMSP,
                    uint32_t ulFaultAddr )
{
    prvDumpStackFrame( pulMSP );
    log_err(LOG_EOL "Fault Address : %x" LOG_EOL, ulFaultAddr);
    prvGetHardFaultReason();
}

void vHardFaultHandler( void )
{
    __asm volatile
    (
        " tst lr, #4		\n"
        " ite eq		\n"
        " mrseq r0, msp        \n"
        " mrsne r0, psp	\n"
        " ldr r1, [r0,#24]	\n"
        " b vAnalyzeFault	\n"
    );
}

void vEnableExceptions( void )
{
    SCB->CCR |= ENABLE_DIV_UA_EXCEPTIONS;
}
