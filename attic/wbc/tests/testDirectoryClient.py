#!/usr/bin/env python

# Copyright (c) 2009 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
#
# BSD license:
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of
#    contributors to this software may be used to endorse or promote
#    products derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
# TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import xmlrpclib

addr = 'localhost'
port = 8080
print 'creating XMLRPC server proxy for %s:%d' % (addr, port)
proxy = xmlrpclib.ServerProxy('http://%s:%d/' % (addr, port))

print 'ListBehaviors():'
print proxy.ListBehaviors()
print
for ii in xrange(4):
    print 'ListBehaviorCmds(%d):' % ii
    print proxy.ListBehaviorCmds(ii)
    print 
    print 'ListTasks(%d):' % ii
    print proxy.ListTasks(ii)
    print
    for jj in xrange(4):
        print 'ListTaskCmds(%d, %d):' % (ii, jj)
        print proxy.ListTaskCmds(ii, jj)
        print 
