#include "stm32l0xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "shell.h"
#include "log.h"
#include "shell_call.h"

#define SHELL_MAX_NBTOKEN       8

typedef struct st_cmd_registration {
   const signed portCHAR *const pc_string_cmd_name;
   pfShellCmd      pf_exec_cmd; 
}Cmd_registration;

typedef enum {
   SHELL_CMDSTATUS_FOUND,              
   SHELL_CMDSTATUS_NOTFOUND, 
   SHELL_CMDSTATUS_PERMISSIONDENIED,
} eCmdStatus;

signed portCHAR        *PrevCmdAv[SHELL_MAX_NBTOKEN];

const Cmd_registration a_cmd_registration[] = {
  {"log", shell_log_process},
  {"set", shell_set_process},
  {"show", shell_show_process},
  {"ctrl", shell_ctrl_process},
  {NULL, NULL}
};

static eCmdStatus prvCmdIdentify_Tokenize(signed portCHAR *pcStringCmd, 
                                          portBASE_TYPE *ac, 
                                          signed portCHAR **av,
                                          portBASE_TYPE *pCmdIdx );

eExecStatus Shell_exec( signed portCHAR *pcStringCmd,
                        signed portCHAR **ppcStringReply)
{
   eCmdStatus        xCmdStatus;
   portBASE_TYPE     ac;
   signed portCHAR   *av[SHELL_MAX_NBTOKEN];
   portBASE_TYPE     CmdIdx;
   eExecStatus       xRet;

   xCmdStatus = prvCmdIdentify_Tokenize(pcStringCmd, &ac, av, &CmdIdx);
   if(SHELL_CMDSTATUS_NOTFOUND == xCmdStatus)
   {  
      if(ppcStringReply != NULL)
      {
         *ppcStringReply = (signed portCHAR *)SHELL_MSG_CMDNOTFOUND;
      }
      return(SHELL_EXECSTATUS_KO);
   }
   else if(SHELL_CMDSTATUS_PERMISSIONDENIED == xCmdStatus)
   {  
      if(ppcStringReply != NULL)
      {
         *ppcStringReply = (signed portCHAR *)SHELL_MSG_PERMISSIONDENIED;
      }
      return(SHELL_EXECSTATUS_KO);
   }

   if(ppcStringReply != NULL)
      *ppcStringReply = NULL;
   xRet = a_cmd_registration[CmdIdx].pf_exec_cmd(ac, 
                                                 av, 
                                                 ppcStringReply);

   return( xRet );
}

static eCmdStatus prvCmdIdentify_Tokenize(signed portCHAR *pcStringCmd, 
                                          portBASE_TYPE *ac, 
                                          signed portCHAR **av,
                                          portBASE_TYPE *pCmdIdx)
{
   signed portCHAR   *pcStringPtr = pcStringCmd;
   size_t            token_len, parsed_len, tempo_len;
   portBASE_TYPE     cmd_len = strlen( (char *)pcStringCmd );

   pcStringPtr += parsed_len = strspn((char *)pcStringCmd, " ");

   parsed_len += token_len = strcspn((char *)pcStringPtr," ");

   pcStringPtr[token_len] = '\0';

   for(*pCmdIdx = 0; 
       a_cmd_registration[*pCmdIdx].pc_string_cmd_name != NULL; 
       (*pCmdIdx)++)
   {  
      if(0 == strcmp( (char *)pcStringPtr,
                      (char *)a_cmd_registration[*pCmdIdx].pc_string_cmd_name))
         break;
   }
   if(a_cmd_registration[*pCmdIdx].pc_string_cmd_name == NULL)
      return(SHELL_CMDSTATUS_NOTFOUND);

   pcStringPtr += token_len+1;
   parsed_len++;

   *ac = 0; 
   while( ( parsed_len < cmd_len ) 
         && (*pcStringPtr != '\0') 
         && (*ac <= SHELL_MAX_NBTOKEN) )
   {
      parsed_len += tempo_len = strspn((char *)pcStringPtr, " =");
      pcStringPtr += tempo_len;

      if(*pcStringPtr == '\0')
         break;

      if(*pcStringPtr == '"')
      {
         pcStringPtr++;
         parsed_len++;

         token_len = strcspn((char *)pcStringPtr,"\"");
      }
      else
      {
         token_len = strcspn((char *)pcStringPtr," =");
      }

      av[*ac] = pcStringPtr;
      (*ac)++;
      pcStringPtr += token_len;
      parsed_len += token_len;
      if(*pcStringPtr == '\0')
         break;
      else
         *pcStringPtr++ = '\0';
   }

   return(SHELL_CMDSTATUS_FOUND);
}