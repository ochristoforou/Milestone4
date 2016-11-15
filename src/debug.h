/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _DEBUG_H    /* Guard against multiple inclusion */
#define _DEBUG_H

#define DLOC_TASKS_START  ((unsigned char)0x01)
#define DLOC_START_WHILE  ((unsigned char)0x02)
#define DLOC_START_QUEUE  ((unsigned char)0x03)
#define DLOC_END_QUEUE  ((unsigned char)0x04)
#define DLOC_START_ISR  ((unsigned char)0x05)
#define DLOC_END_ISR  ((unsigned char)0x06)
#define DLOC_START_QUEUE_ISR  ((unsigned char)0x07)
#define DLOC_END_QUEUE_ISR  ((unsigned char)0x08)

void dbgOutputVal(unsigned char outVal);
void dbgOutputLoc(unsigned char outLoc);
void dbgOutputErr();

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _DEBUG_H */

/* *****************************************************************************
 End of File
 */
