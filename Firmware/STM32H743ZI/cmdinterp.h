/** \file cmdinterp.h
 * Command interpretation and execution.
 *
 * Command models are inspired by the
 * [SCPI](https://en.wikipedia.org/wiki/Standard_Commands_for_Programmable_Instruments)
 * protocol.
 *
 * User shall supply a function to get next character from stream for interpretation.
 *
 * Command+parameters start with ':' and ends with ';'.
 * No delimiters immediately following ':' or preceding ';' are allowed.
 * Commands that end with '?' shall expect a return to be sent back.
 * '_', ',', space/tab, '\r', and '\n' are delimiters, the consecutive sequence of which
 * (any combination) are considered as one.
 * Integer numbers of decimal (??), hexadecimal (#Hxx or #hxx), octal (#Oxx or #oxx),
 * and binary (#Bxx or #bxx) are accepted.
 * Floating point numbers are accepted.
 * '!' to kill the interpreter.
 *
 * Example
 *     :WR_#h51,-0.13e6  ;
 */
#ifndef __CMDINTERP_H__
#define __CMDINTERP_H__

/* On AVR (Arduino), place PROGMEM after var decl. and before = will
 * cause the var to be stored in flash instead of sram.  On STM32,
 * static const var achieves the same effect. */
#if !defined(PROGMEM)
  #define PROGMEM
#endif
/** Maximum number of parameters that can follow a command. */
#define CMDINTERP_NPARAM_MAX 8
/** Maximum number of characters in a command. */
#define CMDINTERP_CMDLEN_MAX 32

#ifdef __cplusplus
extern "C" {
#endif

/** Data (storage) type. */
typedef union { char c; int i; float f; } cmdinterp_data_t;
typedef cmdinterp_data_t (*cmdinterp_cmdfunc_t)();
/** Function type to get next-character. */
typedef int (*cmdinterp_nextc_func_t)(void *s);
/** Interpreter states */
typedef enum { CMDINTERP_STATE_WAIT, CMDINTERP_STATE_GETCMD, CMDINTERP_STATE_GETPARAM,
    CMDINTERP_STATE_RUNCMD, CMDINTERP_STATE_ERROR } cmdinterp_state_t;
/** Interpreter data types */
typedef enum { CMDINTERP_DTYPE_CHAR, CMDINTERP_DTYPE_INT, CMDINTERP_DTYPE_FLOAT }
    cmdinterp_dtype_t;
/** Command-function mapping. */
typedef struct
{
    const char *s;
    cmdinterp_cmdfunc_t f;
    int nparam;
} cmdinterp_funcmap_t;
/** Structure to hold all global states of the interpreter. */
typedef struct
{
    void *s;                                         /**< Source stream of commands. */
    cmdinterp_nextc_func_t nextc;                    /**< Function to get next-character. */
    const cmdinterp_funcmap_t *cmdfuncmap;           /**< Command-function map. */
    int nparam;                                      /**< Number of Parameters found. */
    cmdinterp_data_t param[CMDINTERP_NPARAM_MAX];    /**< Parameters following a command. */
    cmdinterp_dtype_t paramdt[CMDINTERP_NPARAM_MAX]; /**< Parameter data type. */
    char cmd[CMDINTERP_CMDLEN_MAX];                  /**< Current command. */
    char tmp[CMDINTERP_CMDLEN_MAX];                  /**< Temporary buffer. */
    char msg[CMDINTERP_CMDLEN_MAX];                  /**< Message to be sent back. */
    cmdinterp_state_t state;                         /**< Interpreter state. */
} cmdinterp_t;

/** Initialize the interpreter.
 * @param[out] hdl initialized handle.
 * @param[in] s stream to get next-character from.
 * @param[in] nextc function to get next-character.
 * @param[in] cmdfuncmap command-function map.
 * @return 0: failure, 1: success.
 */
int cmdinterp_init(cmdinterp_t *hdl, void *s, cmdinterp_nextc_func_t nextc,
                   const cmdinterp_funcmap_t *cmdfuncmap);
/** Run the interpreter.
 * @return 1 on success (when '\0' was received) even if erroneous commands were processed.
 *         0 on catastrophic failure.
 */
int cmdinterp_run(cmdinterp_t *hdl);

#ifdef __cplusplus
}
#endif
#endif /* __CMDINTERP_H__ */
