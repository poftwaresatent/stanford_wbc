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

#ifndef _DELOGGER_H
#define _DELOGGER_H

#include <stdio.h>

#include <tao/matrix/TaoDeTypes.h>

/*!
 *	\ingroup	deUtility
 *	\name		Log types
 *	arbitrary but you can use these if you want
 */
//	@{
#define DFL_ERROR deLogger::_DFL_ERROR
#define DFL_WARN  deLogger::_DFL_WARN
#define DFL_INFO  deLogger::_DFL_INFO
#define DFL_DEBUG deLogger::_DFL_DEBUG
//	@}

/*!
 *	\ingroup	deUtility
 *	\name	calls to static functions
 */
//	@{
#define dePrintf deLogger::Printf
#define deLog deLogger::Log
//	@}

class deLogger;
/*!
 *	\brief		Setup the output
 *	\ingroup	deUtility
 */
class deLoggerOutput
{
	friend class deLogger;
public:
	deLoggerOutput() 
	{
		next = NULL;
	}

	virtual ~deLoggerOutput()
	{
		if (next) delete next;
	}

	virtual void Log(deChar *msg) = 0;

protected:
	deLoggerOutput *next;
};

/*!
 *	\brief		Output to the standard output
 *	\ingroup	deUtility
 */
class deLoggerOutputStd : public deLoggerOutput
{
public:
	virtual void Log(deChar *msg);
};

#ifdef WIN32
/*!
 *	\brief		Output to the debug window
 *	\ingroup	deUtility
 */
class deLoggerOutputWinDebug : public deLoggerOutput
{
public:
	virtual void Log(deChar *msg);
};
#endif

/*!
 *	\brief		Output to a file
 *	\ingroup	deUtility
 */
class deLoggerOutputFile : public deLoggerOutput
{
public:
	deLoggerOutputFile(deChar* filename);
	~deLoggerOutputFile();

	virtual void Log(deChar *msg);

private:
	FILE *logfile;
};

/*!
 *	\brief		Logger class - printing to various outputs
 *	\ingroup	deUtility
 *
 *	This provides print methods for log.
 *	\sa	deLoggerOutput
 */
class deLogger
{
public:
	//! log types, arbitrary but you can use these if you want
	enum 
	{
		_DFL_ERROR = 1,
		_DFL_WARN,
		_DFL_INFO,
		_DFL_DEBUG
	};

	deLogger();
	virtual ~deLogger();

	static void Initialize();
	static void Shutdown();

	static void Format(deChar *buf, deInt type, deChar* category, deChar *msg);
	static void Log(deInt type, deChar* category, deChar* format, ...);
	static void Printf(deChar* format, ...);

	static void AddOutput(deLoggerOutput *out);

private:
	static deLogger* logger;

	deLoggerOutput *output;
};

#endif // _DELOGGER_H
