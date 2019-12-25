#ifndef __SHELL_H__
#define __SHELL_H__

#define CR                    '\r'
#define LF                    '\n'
#define CTRL_C                0x03
#define BKSPACE_CHAR          '\b'
#define ABORT_CHAR            CTRL_C
#define CRLF                  "\r\n"
#define CTRL_Q                0x11
#define QUIT_APPEND           CTRL_Q

#define PLUS                    '+'

#define ESC                   '\x1B'
#define MI_B                  '['

#define OK_CRLF               "OK"CRLF
#define ERROR_CRLF            "Error"CRLF

#define SHELL_ERRMSG_NOTIMPLEMENTED   "NOT IMPLEMENTED"CRLF
#define SHELL_ERRMSG_CONFIGERROR      ERROR_CRLF"COULD NOT SET CONFIG"CRLF
#define SHELL_ERRMSG_MEMALLOC         ERROR_CRLF"Memory allocation failed"CRLF
#define SHELL_ERRMSG_MAINTENANCEMODE  ERROR_CRLF"Command unavailable in maintenance mode"CRLF
#define SHELL_MSG_REBOOT              "Reboot to take effect"CRLF
#define SHELL_MSG_CONFIG_SET          "config updated"CRLF

#define SHELL_MAX_MSGOUT_LEN   128

/* Type definitions. */
#define portCHAR		char
#define portFLOAT		float
#define portDOUBLE		double
#define portLONG		long
#define portSHORT		short
#define portSTACK_TYPE	uint32_t
#define portBASE_TYPE	long

#define pdFALSE			( ( BaseType_t ) 0 )
#define pdTRUE			( ( BaseType_t ) 1 )

//命令未找到
#define SHELL_MSG_CMDNOTFOUND \
        (char *)"Error:Command not found"CRLF;

//参数错误应答
#define SHELL_MSG_SYNTAXERROR \
        (char *)"Error:Syntax error"CRLF;
        
//权限拒绝应答
#define SHELL_MSG_PERMISSIONDENIED \
        (char *)"Error:Permission denied"CRLF;
        
typedef enum {
   SHELL_EXECSTATUS_OK,               // The command was successfully executed.
   SHELL_EXECSTATUS_OK_NO_FREE,       // The command was successfully executed, but the caller has not to free buffer.
   SHELL_EXECSTATUS_KO,               // The command execution failed.
} eExecStatus;

typedef eExecStatus (*pfShellCmd) ( int ac, 
                                    signed char *av[],
                                    signed char **ppcStringReply);

eExecStatus Shell_exec( signed portCHAR *pcStringCmd,
                        signed portCHAR **ppcStringReply);

#endif // __SHELL_H__
