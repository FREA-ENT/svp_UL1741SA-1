####################################################################################################
# This script was tuned specifically for the AIST FREA environment (Fixed in 2018)
#     AIST:National Institute of Advanced Industrial Science and Technology 
#     FREA:Fukushima Renewable Energy Institute
#
# What is the AIST FREA environment
#   Communication with SunSpecSVP is middleware called ExCon, and ExCon is
#   a mechanism to communicate with inverters and simulators.
#
# Libray Name    : device_dl850e
# Libray Version : 1.0.0
# Update Date    : 2018/11/20
# Used Script    : SA09 - SA15
###################################################################################################
"""
Copyright (c) 2017, Sandia National Labs and SunSpec Alliance
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the names of the Sandia National Labs and SunSpec Alliance nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Questions can be directed to support@sunspec.org
"""

import time

import vxi11

class DeviceError(Exception):
    """
    Exception to wrap all das generated exceptions.
    """
    pass


class Device(object):

    def __init__(self, params):
        self.vx = None
        self.params = params
        self.channels = params.get('channels')
        #self.data_points = ['TIME']
        self.data_points = []
        #self.pf_points = []

        # create query string for configured channels
        query_chan_str = ''
        item = 0
        for i in range(1,5):
            chan = self.channels[i]
            if chan is not None:
                chan_type = chan.get('type')
                points = chan.get('points')
                if points is not None:
                    chan_label = chan.get('label')
                    if chan_type is None:
                        raise DeviceError('No channel type specified')
                    if points is None:
                        raise DeviceError('No points specified')
        self.query_str = 'FILE?'

        self.vx = vxi11.Instrument(self.params['ip_addr'])

        # clear any error conditions
        self.cmd('*CLS')

    def open(self):
        pass

    def close(self):
        if self.vx is not None:
            self.vx.close()
            self.vx = None

    def cmd(self, cmd_str):
        try:
            self.vx.write(cmd_str)
            resp = self.query('STAT:ERRor?')

            if len(resp) > 0:
                if resp[0] != '0':
                    raise DeviceError(resp)
        except Exception, e:
            raise DeviceError('DL850E communication error: %s' % str(e))

    def query(self, cmd_str):
        try:
            print '[dl850e]cmd_str = %s' % (cmd_str)
            resp = self.vx.ask(cmd_str)
        except Exception, e:
            raise DeviceError('DL850E communication error: %s' % str(e))

        return resp

    def info(self):
        return self.query('*IDN?')

    def data_capture(self, enable=True):
        self.capture(enable)

    def data_read(self):
        """
        q = self.query(self.query_str)
#        data = [float(i) for i in q.split(',')]
        data = [str(i) for i in q.split(',')]
        data.insert(0, time.time())
#        for p in self.pf_points:
#            data[p[0]] = pf_adjust_sign(data, *p)
        return data
        """

        pass

    def capture(self, enable=None):
        """
        Enable/disable capture.
        """
        if enable is not None:
            if enable is True:
                self.cmd('STAR')
            else:
                self.cmd('STOP')

    def trigger(self, value=None):
        """
        Create trigger event with provided value.
        """
        pass

    COND_RUN = 0x1000
    COND_TRG = 0x0004
    COND_CAP = 0x0001

    def status(self):
        """
        Returns dict with following entries:
            'trigger_wait' - waiting for trigger - True/False
            'capturing' - waveform capture is active - True/False
        """
        cond = int(d.query('STAT:COND?'))
        result = {'trigger_wait': (cond & COND_TRG),
                  'capturing': (cond & COND_CAP),
                  'cond': cond}
        return result

    def waveform(self):
        """
        Return waveform (Waveform) created from last waveform capture.
        """
        pass

    def trigger_config(self, params):
        """
        slope - (rise, fall, both)
        level - (V, I, P)
        chan - (chan num)
        action - (memory save)
        position - (trigger % in capture)
        """

        """
        samples/sec
        secs pre/post

        rise/fall
        level (V, A)
        """

        pass

    def data_save(self):
        self.cmd('FILE:SAVE:ASC:EXECUTE')
        pass

    def data_capture_dataset(self):
        pass

    def data_sample(self):
        pass

if __name__ == "__main__":

    pass
