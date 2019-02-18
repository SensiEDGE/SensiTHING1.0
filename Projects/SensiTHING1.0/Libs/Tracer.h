/***************************************************************************//**
 * @file    Tracer.h
 * @brief   Provides tracing output into serial port. Standard output methods for
 *          reporting debug information, warnings and errors, which can be easily
 *          be turned on/off.
 *
 *******************************************************************************
 * COPYRIGHT(c) 2019 SensiEDGE
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of SensiEDGE nor the names of its contributors may
 *      be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *
 * Usage.
 * -# Initialize the serial port using system driver. Add retarget stdout to a serial port.
 * -# Uses the TRACE_DEBUG(), TRACE_INFO(), TRACE_WARNING(), TRACE_ERROR()
 *    TRACE_FATAL() macros to output traces throughout the program.
 * -# Each type of trace has a level : Debug 5, Info 4, Warning 3, Error 2
 *    and Fatal 1. Disable a group of traces by changing the value of
 *    TRACE_LEVEL during compilation; traces with a level bigger than TRACE_LEVEL
 *    are not generated. To generate no trace, use the reserved value 0.
 * -# Trace disabling can be static or dynamic. If dynamic disabling is selected
 *    the trace level can be modified in runtime. If static disabling is selected
 *    the disabled traces are not compiled.
 *
 * Trace level description.
 * -# TRACE_DEBUG (5): Traces whose only purpose is for debugging the program,
 *    and which do not produce meaningful information otherwise.
 * -# TRACE_INFO (4): Informational trace about the program execution. Should
 *    enable the user to see the execution flow.
 * -# TRACE_WARNING (3): Indicates that a minor error has happened. In most case
 *    it can be discarded safely; it may even be expected.
 * -# TRACE_ERROR (2): Indicates an error which may not stop the program execution,
 *    but which indicates there is a problem with the code.
 * -# TRACE_FATAL (1): Indicates a major error which prevents the program from going
 *    any further.
 *
 * @endverbatim
 *
 ******************************************************************************/


#ifndef _TRACER_H_
#define _TRACER_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdio.h>


/* Global Definitions. */

#define TRACE_LEVEL_DEBUG				5
#define TRACE_LEVEL_INFO				4
#define TRACE_LEVEL_WARNING				3
#define TRACE_LEVEL_ERROR				2
#define TRACE_LEVEL_FATAL				1
#define TRACE_LEVEL_NO_TRACE			0

#if !defined(TRACE_LEVEL)    
#define TRACE_LEVEL     TRACE_LEVEL_NO_TRACE
#endif

#if defined(NOTRACE)
#error "Error: NOTRACE has to be not defined !"
#endif

#undef NOTRACE
#if (TRACE_LEVEL == TRACE_LEVEL_NO_TRACE)
#define NOTRACE
#endif


/**
 * @brief      Outputs a formatted string using <printf> if the log level is high enough.
 *             Can be disabled by defining TRACE_LEVEL=0 during compilation.
 * @param[in]  format - Formatted string to output.
 * @param[in]  ... - Additional parameters depending on formatted string.
 */

#if defined(NOTRACE)

// Empty macro
#define TRACE_DEBUG(...)      { }	do { } while(0)
#define TRACE_INFO(...)       { }	do { } while(0)
#define TRACE_WARNING(...)    { }	do { } while(0)              
#define TRACE_ERROR(...)      { }	do { } while(0)
#define TRACE_FATAL(...)      { while(1); }	do { } while(0)

#define TRACE_DEBUG_WP(...)   { }	do { } while(0)
#define TRACE_INFO_WP(...)    { }	do { } while(0)
#define TRACE_WARNING_WP(...) { }	do { } while(0)
#define TRACE_ERROR_WP(...)   { }	do { } while(0)
#define TRACE_FATAL_WP(...)   { while(1); }	do { } while(0)

#else

// Trace compilation depends on TRACE_LEVEL value
#if (TRACE_LEVEL >= TRACE_LEVEL_DEBUG)
#define TRACE_DEBUG(...)      { printf("-D- " __VA_ARGS__); }	do { } while(0)
#define TRACE_DEBUG_WP(...)   { printf(__VA_ARGS__); }	do { } while(0)
#else
#define TRACE_DEBUG(...)      { }	do { } while(0)
#define TRACE_DEBUG_WP(...)   { }	do { } while(0)
#endif

#if (TRACE_LEVEL >= TRACE_LEVEL_INFO)
#define TRACE_INFO(...)       { printf("-I- " __VA_ARGS__);  }	do { } while(0)
#define TRACE_INFO_WP(...)    { printf(__VA_ARGS__); }	do { } while(0)
#else
#define TRACE_INFO(...)       { }	do { } while(0)
#define TRACE_INFO_WP(...)    { }	do { } while(0)
#endif

#if (TRACE_LEVEL >= TRACE_LEVEL_WARNING)
#define TRACE_WARNING(...)    { printf("-W- " __VA_ARGS__);  }	do { } while(0)
#define TRACE_WARNING_WP(...) { printf(__VA_ARGS__);  }	do { } while(0)
#else
#define TRACE_WARNING(...)    { }	do { } while(0)
#define TRACE_WARNING_WP(...) { }	do { } while(0)
#endif

#if (TRACE_LEVEL >= TRACE_LEVEL_ERROR)
#define TRACE_ERROR(...)      { printf("-E- " __VA_ARGS__);  }	do { } while(0)
#define TRACE_ERROR_WP(...)   { printf(__VA_ARGS__);  }	do { } while(0)
#else
#define TRACE_ERROR(...)      { }	do { } while(0)
#define TRACE_ERROR_WP(...)   { }	do { } while(0)
#endif

#if (TRACE_LEVEL >= TRACE_LEVEL_FATAL)
#define TRACE_FATAL(...)      { printf("-F- " __VA_ARGS__);  while(1); }	do { } while(0)
#define TRACE_FATAL_WP(...)   { printf(__VA_ARGS__);  while(1); }	do { } while(0)
#else
#define TRACE_FATAL(...)      { while(1); }	do { } while(0)
#define TRACE_FATAL_WP(...)   { while(1); }	do { } while(0)
#endif

#endif


#ifdef __cplusplus
}
#endif

#endif /* _TRACER_H_ */
