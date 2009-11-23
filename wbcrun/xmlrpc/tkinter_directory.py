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

import Tkinter
import tkMessageBox
import xmlrpclib
import sys
import Numeric
import traceback
import functools
import time
from threading import Thread

##################################################
# utils

def exception_box(culprit):
    traceback.print_exc()
    exc = sys.exc_info()
    tkMessageBox.showwarning('EXCEPTION',
                             '%s\n%s\n%s' % (culprit, repr(exc[0]), repr(exc[1])))

##################################################
# XMLRPC wrapper

class DirectoryProtocolError(EnvironmentError):
    'Problem with the XMLRPC communication between the GUI and the directory server'

class DirectoryFailure(StandardError):
    'XMLRPC succeeded, but something went wrong on the direcotry server'

class DirectoryValueError(StandardError):
   'Problem with creating or handling parameters or results within the XMLRPC protocol'

def pack_code_and_data(code_in, data_in):
    pack = dict()
    if not code_in:
        pack['code_in'] = []
    else:
        if len(code_in.shape) > 1:
            raise DirectoryValueError, 'code_in must be a vector of integers'
        pack['code_in'] = code_in.tolist()
    data = dict()
    if not data_in:
        data['nrows'] = 0
        data['ncols'] = 0
        data['data'] = []
    else:
        rank = len(data_in.shape)
        if rank == 0:
            data['nrows'] = 1
            data['ncols'] = 1
            data['data'] = data_in.tolist()
        elif rank == 1:
            # automatically transpose... not sure if this is a good idea
            data['nrows'] = data_in.shape[0]
            data['ncols'] = 1
            data['data'] = data_in.tolist()
        elif rank == 2:
            data['nrows'] = data_in.shape[0]
            data['ncols'] = data_in.shape[1]
            data['data'] = data_in.flat.tolist()
        else:
            raise DirectoryValueError, 'data_in must be a matrix of doubles'
    pack['data_in'] = data
    return pack

def unpack_code_and_data(code_out, data_out):
    unpack = {'code_out':None, 'data_out':None}
    if code_out and len(code_out) > 0:
        unpack['code_out'] = Numeric.array(code_out, 'i')
    if data_out:
        if not data_out.has_key('nrows'):
            raise DirectoryValueError, 'data_out has no nrows'
        if not data_out.has_key('ncols'):
            raise DirectoryValueError, 'data_out has no ncols'
        if not data_out.has_key('data'):
            raise DirectoryValueError, 'data_out has no data'
        nrows = data_out['nrows']
        ncols = data_out['ncols']
        data = data_out['data']
        arr = Numeric.zeros((nrows, ncols), 'd')
        for ir in xrange(nrows):
            for ic in xrange(ncols):
                arr[ir][ic] = data[ir * ncols + ic]
        unpack['data_out'] = arr
    return unpack

class Directory:
    def __init__(self, addr = 'localhost', port = 8080):
        self.proxy = xmlrpclib.ServerProxy('http://%s:%d/' % (addr, port))
        
    def ListBehaviors(self):
        result = self.proxy.ListBehaviors()
        if not result.has_key('retval'):
            raise DirectoryProtocolError, 'no retval in result'
        if result['retval'] != 'SUCCESS':
            raise DirectoryFailure, 'ListBehaviors(): %s' % result['retval']
        if not result.has_key('behaviors'):
            raise DirectoryProtocolError, 'no behaviors in result'
        return result['behaviors']
        
    def ListTasks(self, behaviorID):
        result = self.proxy.ListTasks(behaviorID)
        if not result.has_key('retval'):
            raise DirectoryProtocolError, 'no retval in result'
        if result['retval'] != 'SUCCESS':
            raise DirectoryFailure, 'ListTasks(%d): %s' % (behaviorID, result['retval'])
        if not result.has_key('tasks'):
            raise DirectoryProtocolError, 'no tasks in result'
        return result['tasks']
        
    def ListTaskCmds(self, behaviorID, taskID):
        result = self.proxy.ListTaskCmds(behaviorID, taskID)
        if not result.has_key('retval'):
            raise DirectoryProtocolError, 'no retval in result'
        if result['retval'] != 'SUCCESS':
            raise DirectoryFailure, 'ListTaskCmds(%d, %d): %s' % (behaviorID, taskID, result['retval'])
        if not result.has_key('requests'):
            raise DirectoryProtocolError, 'no requests in result'
        return result['requests']
        
    def HandleTaskCmd(self, behaviorID, taskID, request, code_in, data_in):
        result = self.proxy.HandleTaskCmd(behaviorID, taskID, request,
                                          pack_code_and_data(code_in, data_in))
        if not result.has_key('retval'):
            raise DirectoryProtocolError, 'no retval in result'
        if result['retval'] != 'SUCCESS':
            raise DirectoryFailure, 'HandleTaskCmd(%d, %d, %s, ...): %s' % (behaviorID, taskID, request, result['retval'])
        return unpack_code_and_data(result['code_out'], result['data_out'])

    def HandleServoCmd(self, request, code_in, data_in):
        result = self.proxy.HandleServoCmd(request,
                                          pack_code_and_data(code_in, data_in))
        if not result.has_key('retval'):
            raise DirectoryProtocolError, 'no retval in result'
        if result['retval'] != 'SUCCESS':
            raise DirectoryFailure, 'HandleServoCmd(%d, %d, %s, ...): %s' % ( request, result['retval'])
        return unpack_code_and_data(result['code_out'], result['data_out'])
##################################################
# behavior selection

class BehaviorSelection(Tkinter.Frame):
    def __init__(self, parent, directory):
        # Tkinter uses classic style classes, so super() does not work
        #   super(BehaviorSelection, self).__init__(master, cnf, kw)
        Tkinter.Frame.__init__(self, parent, bd = 2, relief = Tkinter.GROOVE)
        self.directory = directory
        self.listbox = Tkinter.Listbox(self)
        self.listbox.grid(column = 0, row = 1)
        self.id = -1
        self.labeltext = Tkinter.StringVar(self)
        self.labeltext.set('no behavior selected')
        self.label = Tkinter.Label(self, textvariable = self.labeltext)
        self.label.grid(column = 0, row = 2)
        self.button = Tkinter.Button(self, text = 'list behaviors', command = self.List)
        self.button.grid(column = 0, row = 0)

        self.button = Tkinter.Button(self, text = 'SET', command = self.Set)
        self.button.grid(column = 0, row = 3)

    def List(self):
        self.listbox.delete(0, Tkinter.END)
        try:
            for behavior in self.directory.ListBehaviors():
                self.listbox.insert(Tkinter.END, behavior)
        except:
            exception_box('ListBehaviors()')
    
    def Get(self, caller):
        ids = self.listbox.curselection()
        if len(ids) == 0:
            if self.id < 0:
                tkMessageBox.showwarning(caller, 'please select exactly one behavior')
                return -1
            return self.id
        if len(ids) > 1:
            tkMessageBox.showwarning(caller, 'please select exactly one behavior')
            return -1
        id = int(ids[0])
        if id != self.id:
            self.labeltext.set('%d: %s' %(id, self.listbox.get(ids[0])))
            self.id = id
        return self.id

    
    def Set(self):
        behaviorID = self.Get('list tasks')
        self.directory.HandleServoCmd('SET_BEHAVIOR',Numeric.array([behaviorID]) ,None)

##################################################
# to do: behavior cmd selection

##################################################
# task selection

class TaskSelection(Tkinter.Frame):
    def __init__(self, parent, directory, behavior_selection):
        Tkinter.Frame.__init__(self, parent, bd = 2, relief = Tkinter.GROOVE)
        self.directory = directory
        self.behavior = behavior_selection
        self.listbox = Tkinter.Listbox(self)
        self.listbox.grid(column = 0, row = 1)
        self.id = -1
        self.labeltext = Tkinter.StringVar(self)
        self.labeltext.set('no task selected')
        self.label = Tkinter.Label(self, textvariable = self.labeltext)
        self.label.grid(column = 0, row = 2)
        self.button = Tkinter.Button(self, text = 'list tasks', command = self.List)
        self.button.grid(column = 0, row = 0)
        self.label = Tkinter.Label(self, textvariable = '')
        self.label.grid(column = 0, row = 3)
    def List(self):
        behaviorID = self.behavior.Get('list tasks')
        if behaviorID < 0:
            return
        self.listbox.delete(0, Tkinter.END)
        try:
            for task in self.directory.ListTasks(behaviorID):
                self.listbox.insert(Tkinter.END, task)
        except:
            exception_box('ListTasks(%d)' % behaviorID)

    def Get(self, caller):
        ids = self.listbox.curselection()
        if len(ids) == 0:
            if self.id < 0:
                tkMessageBox.showwarning(caller, 'please select exactly one task')
                return -1
            return self.id
        if len(ids) > 1:
            tkMessageBox.showwarning(caller, 'please select exactly one task')
            return -1
        id = int(ids[0])
        if id != self.id:
            self.labeltext.set('%d: %s' %(id, self.listbox.get(ids[0])))
            self.id = id
        return self.id

##################################################
# task command selection

class TaskCmdSelection(Tkinter.Frame):
    def __init__(self, parent, directory, behavior_selection, task_selection):
        Tkinter.Frame.__init__(self, parent, bd = 2, relief = Tkinter.GROOVE)
        self.directory = directory
        self.behavior = behavior_selection
        self.task = task_selection
        self.listbox = Tkinter.Listbox(self)
        self.listbox.grid(column = 0, row = 1)
        self.id = -1
        self.labeltext = Tkinter.StringVar(self)
        self.labeltext.set('no task command selected')
        self.label = Tkinter.Label(self, textvariable = self.labeltext)
        self.label.grid(column = 0, row = 2)
        self.button = Tkinter.Button(self, text = 'list task commands', command = self.List)
        self.button.grid(column = 0, row = 0)
        self.label = Tkinter.Label(self, textvariable = '')
        self.label.grid(column = 0, row = 3)

    def List(self):
        behaviorID = self.behavior.Get('list task commands')
        if behaviorID < 0:
            return
        taskID = self.task.Get('list task commands')
        if taskID < 0:
            return
        self.listbox.delete(0, Tkinter.END)
        try:
            for cmd in self.directory.ListTaskCmds(behaviorID, taskID):
                self.listbox.insert(Tkinter.END, cmd)
        except:
            exception_box('ListTaskCmds(%d, %d)' % (behaviorID, taskID))

    def Get(self, caller):
        ids = self.listbox.curselection()
        if len(ids) == 0:
            if self.id < 0:
                tkMessageBox.showwarning(caller, 'please select exactly one task command')
                return -1
            return self.id
        if len(ids) > 1:
            tkMessageBox.showwarning(caller, 'please select exactly one task command')
            return -1
        id = int(ids[0])
        if id != self.id:
            self.labeltext.set('%d: %s' %(id, self.listbox.get(ids[0])))
            self.id = id
        return self.id

##################################################
# control slider

class ControlSlider(Tkinter.Frame):
    def __init__(self, parent, directory, dimension, behavior_selection, task_selection):
        Tkinter.Frame.__init__(self, parent, bd = 2, relief = Tkinter.GROOVE)
        self.directory = directory
        self.behavior = behavior_selection
        self.task = task_selection
        self.get = Tkinter.Button(self, text = 'get', command = self.Get)
        self.get.grid(column = 0, row = 0)
        self.set = Tkinter.Button(self, text = 'set', command = self.Set)
        self.set.grid(column = 1, row = 0)
        self.optionvar = Tkinter.StringVar(self)
        self.optionvar.set('PROP_GAIN')
        self.option = Tkinter.OptionMenu(self, self.optionvar,
                                         'PROP_GAIN', 'DIFF_GAIN', 'MAX_VEL', 'MAX_ACCEL')
        self.option.grid(column = 0, columnspan = 2, row = 1)
        self.dimension = 0
        self.value = []
        self.value_label = []
        self.scale = []
        for ii in xrange(dimension):
            self.CreateSlider(ii, 0, 50, 400) # XXXX magic values

    def CreateSlider(self, id, minval, val, maxval):
        for igrow in xrange(len(self.value), id + 1):
            self.value.append(None)
            self.value_label.append(None)
            self.scale.append(None)
        if self.value[id]:
            self.value[id].set(val)
            self.scale[id]['from_'] = minval
            self.scale[id]['to_'] = maxval
        else:
            self.value[id] = Tkinter.DoubleVar(self)
            self.value[id].set(val)
            self.value_label[id] = Tkinter.Label(self, textvariable = self.value[id])
            self.value_label[id].grid(column = 2, row = id)
            self.scale[id] = Tkinter.Scale(self, orient = Tkinter.HORIZONTAL,
                                           variable = self.value[id], showvalue = 0,
                                           from_ = minval, to = maxval)
            self.scale[id].grid(column = 3, row = id)
        if id + 1 > self.dimension:
            self.dimension = id + 1
    
    def DestroyExtraSliders(self, dimension):
        for ii in xrange(dimension, len(self.value)):
            if self.value[ii]:
                self.value[ii].destroy()
                self.value[ii] = None
                self.value_label[ii].destroy()
                self.value_label[ii] = None
                self.scale[ii].destroy()
                self.scale[ii] = None
        self.dimension = dimension
        
    def Get(self):
        behaviorID = self.behavior.Get('ControlSlider.Get()')
        if behaviorID < 0:
            return
        taskID = self.task.Get('ControlSlider.Get()')
        if taskID < 0:
            return
        try:
            pack = self.directory.HandleTaskCmd(behaviorID, taskID,
                                                'GET_%s' % self.optionvar.get(),
                                                None, None)
            data = pack['data_out']
            if not data:
                raise DirectoryProtocolError, 'expected non-empty data_out'
            # XXXX should make sure we get an N*1 matrix...
            if len(data.flat) < self.dimension:
                self.DestroyExtraSliders(len(data.flat))
            for ii in xrange(len(data.flat)):
                self.CreateSlider(ii, 0, data.flat[ii], 400) # XXXX magic bounds
        except:
            exception_box('HandleTaskCmd(%d, %d, GET_%s, ...)' % (behaviorID, taskID,
                                                                  self.optionvar.get()))
            
    def Set(self):
        behaviorID = self.behavior.Get('ControlSlider.Get()')
        if behaviorID < 0:
            return
        taskID = self.task.Get('ControlSlider.Get()')
        if taskID < 0:
            return
        try:
            data = Numeric.zeros((self.dimension, 1))
            for ii in xrange(self.dimension):
                data.flat[ii] = self.value[ii].get()
            self.directory.HandleTaskCmd(behaviorID, taskID,
                                         'SET_%s' % self.optionvar.get(),
                                         None, data)
        except:
            exception_box('HandleTaskCmd(%d, %d, SET_%s, ...)' % (behaviorID, taskID,
                                                                  self.optionvar.get()))


##################################################
# scalar control slider

class ScalarControlSlider(Tkinter.Frame):
    def __init__(self, parent, directory, behavior_selection, task_selection):
        Tkinter.Frame.__init__(self, parent, bd = 2, relief = Tkinter.GROOVE)
        self.directory = directory
        self.behavior = behavior_selection
        self.task = task_selection
        self.get = {}
        self.set = {}
        self.value = {}
        self.scale = {}
        ii = 0
        for param in ['PROP_GAIN', 'DIFF_GAIN', 'MAX_VEL', 'MAX_ACCEL']:
            Tkinter.Label(self, text = param).grid(column = 0, row = ii)
            self.get[param] = Tkinter.Button(self, text = 'GET',
                                             command = functools.partial(self.Get, param))
            self.get[param].grid(column = 1, row = ii)
            self.set[param] = Tkinter.Button(self, text = 'SET',
                                             command = functools.partial(self.Set, param))
            self.set[param].grid(column = 2, row = ii)
            self.value[param] = Tkinter.DoubleVar(self)
            self.value[param].set(0)
            Tkinter.Label(self, textvariable = self.value[param]).grid(column = 3, row = ii)
            self.scale[param] = Tkinter.Scale(self, orient = Tkinter.HORIZONTAL,
                                              variable = self.value[param], showvalue = 0,
                                              from_ = 0, to = 400)
            self.scale[param].grid(column = 4, row = ii)
            ii = ii + 1
        
    def Get(self, param):
        behaviorID = self.behavior.Get('ControlSlider.Get(%s)' % param)
        if behaviorID < 0:
            return
        taskID = self.task.Get('ControlSlider.Get()')
        if taskID < 0:
            return
        try:
            pack = self.directory.HandleTaskCmd(behaviorID, taskID, 'GET_%s' % param,
                                                None, None)
            data = pack['data_out']
            if not data:
                raise DirectoryProtocolError, 'expected non-empty data_out'
            # XXXX should make sure we get an N*1 matrix...
            self.value[param].set(data.flat[0])
        except:
            exception_box('HandleTaskCmd(%d, %d, GET_%s, ...)' % (behaviorID, taskID, param))
            
    def Set(self, param):
        behaviorID = self.behavior.Get('ControlSlider.Set(%s)' % param)
        if behaviorID < 0:
            return
        taskID = self.task.Get('ControlSlider.Get()')
        if taskID < 0:
            return
        try:
            self.directory.HandleTaskCmd(behaviorID, taskID, 'SET_%s' % param,
                                         None, Numeric.array([[self.value[param].get()]]))
        except:
            exception_box('HandleTaskCmd(%d, %d, SET_%s, ...)' % (behaviorID, taskID, param))


##################################################
# main

class GetPosition(Tkinter.Frame):
    def __init__(self, parent, directory):
        Tkinter.Frame.__init__(self, parent, bd = 2, relief = Tkinter.GROOVE)
        self.directory = directory
        self.button = Tkinter.Button(self, text = 'Get Position', command = self.Get)
        self.button.grid(column = 0, row = 0)

    def Get(self):        
        jointpos.update_position()
class Robot():
    def __init__(self, ndof):
        self.ndof =ndof
        self.q = Numeric.array(list([0 for i in range(ndof)]))

class JointPositions(Tkinter.Frame):
    def __init__(self, parent, directory,robot):
        Tkinter.Frame.__init__(self, parent, bd = 2, relief = Tkinter.GROOVE)
        self.directory = directory
        self.robot = robot
        Tkinter.Label(self, text = 'Joint Positions').grid(column = 0,columnspan = self.robot.ndof, row = 0)        
        self.jlabel = list(Tkinter.DoubleVar() for i in range(self.robot.ndof))
        for i in range(self.robot.ndof):            
            Tkinter.Label(self, text = 'J%s' %i  ).grid(column = 0, row = i+1)
            Tkinter.Label(self, textvariable =  self.jlabel[i]).grid(column = 1, row = i+1)
        
    def update_position(self):
        pack =  self.directory.HandleServoCmd('GET_LINK_ANGLE',Numeric.array([0,1,2,3,4,5,6]) ,None)
        self.robot.q = pack['data_out']
        for i in range(self.robot.ndof):                      
            self.jlabel[i].set(self.robot.q[i])


root = Tkinter.Tk()

dir = Directory()

behavior_selection = BehaviorSelection(root, dir)
behavior_selection.grid(column = 0, row = 0)

task_selection = TaskSelection(root, dir, behavior_selection)
task_selection.grid(column = 1, row = 0)

#get_position = GetPosition(root, dir)
#get_position.grid(column = 4, row = 0)

task_cmd_selection = TaskCmdSelection(root, dir, behavior_selection, task_selection)
task_cmd_selection.grid(column = 2, row = 0)

#control_slider = ControlSlider(root, dir, 3, behavior_selection, task_selection)
control_slider = ScalarControlSlider(root, dir, behavior_selection, task_selection)
control_slider.grid(column = 0, columnspan = 3, row = 1)

wam = Robot(7)

jointpos = JointPositions(root, dir,wam)
jointpos.grid(column = 4, columnspan = 1,  row = 0)


class updatemain(Thread):
    def run(self):
        while  1:
            jointpos.update_position()
            time.sleep(0.01)
th1 = updatemain()
#th1.start()

Tkinter.Button(root, text = 'quit', command = root.quit).grid(column = 0, row = 2)
root.mainloop()
