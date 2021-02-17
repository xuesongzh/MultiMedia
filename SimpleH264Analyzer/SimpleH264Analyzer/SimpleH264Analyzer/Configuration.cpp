#include "Configuration.h"

#include <fstream>

#include "stdafx.h"

#if TRACE_CONFIG_LOGOUT
std::ofstream g_traceFile;
#endif

#if TRACE_CONFIG_TEMP_LOG
std::ofstream g_tempFile;
#endif