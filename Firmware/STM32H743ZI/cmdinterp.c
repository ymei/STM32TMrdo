#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "cmdinterp.h"

static int is_delimiter(int c)
{
    return (c==' ' || c=='\t' || c=='\r' || c=='\n' || c=='_' || c==',');
}

int cmdinterp_init(cmdinterp_t *hdl, void *s, cmdinterp_nextc_func_t nextc,
                   const cmdinterp_funcmap_t *cmdfuncmap)
{
    if(hdl) {
        hdl->s = s;
        hdl->nextc = nextc;
        hdl->cmdfuncmap = cmdfuncmap;
        hdl->nparam = 0;
        hdl->state = CMDINTERP_STATE_WAIT;
        return 1;
    }
    return 0;
}

int cmdinterp_run(cmdinterp_t *hdl)
{
    int c=' ', cp=' ', i=0, j=0, k=0;
    const cmdinterp_funcmap_t *fm;

    hdl->nparam = 0;
    while((c=(*hdl->nextc)(hdl->s))>0) {
        if(c=='!') { /* Kill interpreter and prepare for re-entry. */
            hdl->nparam = 0;
            hdl->state = CMDINTERP_STATE_WAIT;
            return 1;
        }
        if(c==':') {
            hdl->state = CMDINTERP_STATE_GETCMD;
            i = 0;
            continue;
        }
        switch(hdl->state) {
        case CMDINTERP_STATE_WAIT:
            i=0; j=0; hdl->nparam = 0;
            break;
        case CMDINTERP_STATE_GETCMD:
            if(i<CMDINTERP_CMDLEN_MAX-1) {
                if(!is_delimiter(c)) {
                    hdl->cmd[i++] = c;
                    if(c==';') {
                        hdl->cmd[i-1] = '\0';
                        hdl->state = CMDINTERP_STATE_RUNCMD;
                        goto runcmd;
                    }
                } else {
                    hdl->cmd[i] = '\0';
                    hdl->state = CMDINTERP_STATE_GETPARAM;
                    i=0; j=0; cp=c;
                }
            } else { /* CMD too long */
                strcpy(hdl->msg, "Err: CMD too long!");
                hdl->state = CMDINTERP_STATE_ERROR;
                goto errcmd;
            }
            break;
        case CMDINTERP_STATE_GETPARAM:
            if(is_delimiter(c) || c==';') {
                if(is_delimiter(cp)) {
                    continue;
                } else { /* One complete parameter found. */
                    hdl->tmp[i] = '\0';
                    /* Decode param */
                    if(i==1 && ((hdl->tmp[0]>='A' && hdl->tmp[0]<= 'Z')
                                || (hdl->tmp[0]>='a' && hdl->tmp[0]<= 'z'))) {
                        hdl->param[j].c = hdl->tmp[0];
                        hdl->paramdt[j] = CMDINTERP_DTYPE_CHAR;
                    } else if(hdl->tmp[0]=='#') {
                        hdl->paramdt[j] = CMDINTERP_DTYPE_INT;
                        switch(hdl->tmp[1]) {
                        case 'H':
                        case 'h':
                            hdl->param[j].i = strtol(hdl->tmp+2, NULL, 16);
                            break;
                        case 'O':
                        case 'o':
                            hdl->param[j].i = strtol(hdl->tmp+2, NULL, 8);
                            break;
                        case 'B':
                        case 'b':
                            hdl->param[j].i = strtol(hdl->tmp+2, NULL, 2);
                            break;
                        default:
                            hdl->param[j].i = strtol(hdl->tmp+2, NULL, 16);
                            break;
                        }
                    } else {
                        for(k=0; k<i; k++) {
                            if(hdl->tmp[k] == '.') break;
                        }
                        if(k==i) {
                            hdl->paramdt[j] = CMDINTERP_DTYPE_INT;
                            hdl->param[j].i = strtol(hdl->tmp, NULL, 10);
                        } else {
                            hdl->paramdt[j] = CMDINTERP_DTYPE_FLOAT;
                            hdl->param[j].f = strtof(hdl->tmp, NULL);
                        }
                    }
                    /* Advance or run. */
                    i = 0; j++; hdl->nparam = j;
                    if(c==';') {
                        hdl->state = CMDINTERP_STATE_RUNCMD;
                        goto runcmd;
                    } else if(hdl->nparam == CMDINTERP_NPARAM_MAX) {
                        strcpy(hdl->msg, "Err: Two many parameters!");
                        hdl->state = CMDINTERP_STATE_ERROR;
                        goto errcmd;
                    }
                }
            } else {
                if(i<CMDINTERP_CMDLEN_MAX-1) {
                    hdl->tmp[i++] = c;
                } else { /* Parameter too long. */
                    strcpy(hdl->msg, "Err: Param too long!");
                    hdl->state = CMDINTERP_STATE_ERROR;
                    goto errcmd;
                }
            }
            cp=c;
            break;
        case CMDINTERP_STATE_ERROR:
        errcmd:
            hdl->cmd[0] = 'e';
            hdl->cmd[1] = 'r';
            hdl->cmd[2] = 'r';
            hdl->cmd[3] = '\0';
        case CMDINTERP_STATE_RUNCMD:
        runcmd:
            fm = hdl->cmdfuncmap;
            while(fm->s) {
                if(strcmp(hdl->cmd, fm->s) == 0) {
                    (*fm->f)(hdl);
                    break;
                }
                fm++;
            }
            hdl->state = CMDINTERP_STATE_WAIT;
            break;
        default:
            break;
        }
    }
    return 1;
}

#ifdef CMDINTERP_DEBUG_ENABLEMAIN
#include <stdio.h>
#include <stdlib.h>
static int cmdinterp_nextc(void *s)
{
    return fgetc((FILE*)s);
}
static cmdinterp_data_t f0(cmdinterp_t *hdl)
{
    cmdinterp_data_t ret;
    ret.f = hdl->param[0].f + hdl->param[1].f;
    return ret;
}
static cmdinterp_data_t cmdinterp_cmderr(cmdinterp_t *hdl)
{
    int i;
    cmdinterp_data_t ret = {1};
    printf("cmd: %s\n", hdl->cmd);
    for(i=0; i<hdl->nparam; i++) {
        switch(hdl->paramdt[i]) {
        case CMDINTERP_DTYPE_CHAR:
            printf("Param %2d: %c\n", i, hdl->param[i].c);
            break;
        case CMDINTERP_DTYPE_INT:
            printf("Param %2d: %d\n", i, hdl->param[i].i);
            break;
        case CMDINTERP_DTYPE_FLOAT:
            printf("Param %2d: %f\n", i, hdl->param[i].f);
            break;
        }
    }
    printf("msg: %s\n", hdl->msg);
    return ret;
}
static const cmdinterp_funcmap_t cmdfuncmap[] PROGMEM = {
    {"IO", (cmdinterp_cmdfunc_t)f0, 2},
    {"DD", (cmdinterp_cmdfunc_t)f0, 2},
    {"err", (cmdinterp_cmdfunc_t)cmdinterp_cmderr, 1},
    {NULL, NULL, 0}
};
int main(int argc, char **argv)
{
    cmdinterp_t interp;
    cmdinterp_init(&interp, stdin, cmdinterp_nextc, cmdfuncmap);
    cmdinterp_run(&interp);
    return EXIT_SUCCESS;
}
#endif
