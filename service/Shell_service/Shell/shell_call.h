#ifndef __SHELL_CALL_H
#define __SHELL_CALL_H

#include "shell.h"
#define COM1SHELL_MSG_BANNER   "\r\n---------- Hoolink Shell ----------\r\n"
#define COM1SHELL_MSG_PROMPT   "$>"

eExecStatus shell_set_process( int ac,
                      signed char *av[],
                      signed char **ppcStringReply);

eExecStatus shell_show_process( int ac, 
                          signed char *av[],
                          signed char **ppcStringReply);

eExecStatus shell_ctrl_process( int ac, 
                          signed char *av[],
                          signed char **ppcStringReply);

eExecStatus shell_log_process( int ac,
                                signed char *av[],
                                signed char **ppcStringReply);

void shell_printf(char *format, ...);

#endif