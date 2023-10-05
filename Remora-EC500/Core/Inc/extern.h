#ifndef EXTERN_H
#define EXTERN_H

#include "configuration.h"
#include "remora.h"

#include "../Src/lib/ArduinoJson6/ArduinoJson.h"
#include "../Src/thread/pruThread.h"


extern uint32_t base_freq;
extern uint32_t servo_freq;

extern JsonObject module;

extern volatile bool PRUreset;

extern int32_t portA_DMAbuffer_0[DMA_BUFFER_SIZE];		// double buffers for port DMA transfers
extern int32_t portA_DMAbuffer_1[DMA_BUFFER_SIZE];
extern int32_t portE_DMAbuffer_0[DMA_BUFFER_SIZE];
extern int32_t portE_DMAbuffer_1[DMA_BUFFER_SIZE];

extern bool portA_DMAbuffer;							// indicates which buffer to use 0 or 1 (false or true)
extern bool portE_DMAbuffer;

extern vector<Module*> vDMAthread;

// pointers to objects with global scope
extern pruThread* baseThread;
extern pruThread* servoThread;
extern pruThread* commsThread;
extern Module* MPG;

// unions for RX and TX data
extern volatile rxData_t rxData;
extern volatile txData_t txData;
extern mpgData_t mpgData;

// pointers to data
extern volatile rxData_t*  ptrRxData;
extern volatile txData_t*  ptrTxData;
extern volatile int32_t*   ptrTxHeader;  
extern volatile bool*      ptrPRUreset;
extern volatile int32_t*   ptrJointFreqCmd[JOINTS];
extern volatile int32_t*   ptrJointFeedback[JOINTS];
extern volatile uint8_t*   ptrJointEnable;
extern volatile float*     ptrSetPoint[VARIABLES];
extern volatile float*     ptrProcessVariable[VARIABLES];
extern volatile uint32_t*  ptrInputs;
extern volatile uint32_t*  ptrOutputs;
extern volatile uint16_t*  ptrNVMPGInputs;
extern volatile mpgData_t* ptrMpgData;


#endif
