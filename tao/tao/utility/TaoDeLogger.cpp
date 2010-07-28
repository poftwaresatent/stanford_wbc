/* Copyright (c) 2005 Arachi, Inc. and Stanford University. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdio.h> // NULL
#include <stdarg.h>
#ifdef WIN32
#include <windows.h> // OutputDebugString
#endif
#include "TaoDeLogger.h"

#ifdef WIN32
#define vsnprintf _vsnprintf
#endif

//
// defines
//

#define	PRINTF_BUFSIZE	4096

//
// statics
//

deLogger* deLogger::logger = NULL;

//
// deLoggerOutputStd
//

void deLoggerOutputStd::Log(deChar *msg)
{
        printf("%s", msg);
	fflush(stdout);
}

deLoggerOutputFile::deLoggerOutputFile(deChar* filename)
{
	logfile = fopen(filename, "w");
}

deLoggerOutputFile::~deLoggerOutputFile()
{
	if (logfile) fclose(logfile);
}

void deLoggerOutputFile::Log(deChar *msg)
{
	if (!logfile) return;
	fprintf(logfile, "%s", msg);
}

//
// deLogger
//

deLogger::deLogger()
{
	output = NULL;
}

deLogger::~deLogger()
{
	if (output) delete output;
}

void deLogger::Initialize()
{
	if (!logger) 
	{
		logger = new deLogger;
	}
}

void deLogger::Shutdown()
{
	if (logger)
		delete logger;
}

void deLogger::Format(deChar *buf, deInt type, deChar* category, deChar *msg)
{
	deChar const *prefix = NULL;
	deChar str[12];

	if (type == deLogger::_DFL_ERROR) prefix = "Error";
	else if (type == deLogger::_DFL_WARN)  prefix = "Warning";
	else if (type == deLogger::_DFL_INFO)  prefix = "Info";
	else if (type == deLogger::_DFL_DEBUG) prefix = "Debug";
	else 
	{
		// XXZ: add a way to let strings be specified for each type?
		sprintf(str, "%d", type);
		prefix = str;
	}

	if (category)
		sprintf(buf, "%s(%s): %s\n", prefix, category, msg);
	else
		sprintf(buf, "%s: %s\n", prefix, msg);
}

void deLogger::Log(deInt type, deChar* category, deChar* format, ...)
{
	va_list argptr;
	deChar buf[PRINTF_BUFSIZE], msg[PRINTF_BUFSIZE];
	deLoggerOutput *pout;

	if (!logger || !logger->output) return;

	// filter here

	va_start(argptr, format);
#ifdef PS2
	vsprintf(msg, format, argptr);
#else
	vsnprintf(msg, PRINTF_BUFSIZE, format, argptr);
#endif
	va_end(argptr);

	Format(buf, type, category, msg);

	pout = logger->output;
	while (pout)
	{
		pout->Log(buf);
		pout = pout->next;
	}
}

void deLogger::Printf(deChar* format, ...)
{
	va_list argptr;
	deChar buf[PRINTF_BUFSIZE];
	deLoggerOutput *pout;

	if (!logger || !logger->output) return;

	va_start(argptr, format);
#ifdef PS2
	vsprintf(buf, format, argptr);
#else
	vsnprintf(buf, PRINTF_BUFSIZE, format, argptr);
#endif
	va_end(argptr);

	pout = logger->output;
	while (pout)
	{
		pout->Log(buf);
		pout = pout->next;
	}
}

void deLogger::AddOutput(deLoggerOutput *out)
{
	Initialize();

	out->next = logger->output;
	logger->output = out;
}
