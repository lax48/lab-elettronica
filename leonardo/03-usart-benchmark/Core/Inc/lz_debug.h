#pragma once

void Error_Handler(void);

#ifdef DEBUG
#	define LZ_DBG(code) do code while(0)
#	define LZ_ASSERT(cond, msg) do { if (!(cond)) Error_Handler(); } while(0)
#else
#	define LZ_DBG(code) do code while(0)
#	define LZ_ASSERT(cond, msg)
#endif
