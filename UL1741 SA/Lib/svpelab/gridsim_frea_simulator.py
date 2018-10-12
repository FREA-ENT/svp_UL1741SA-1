####################################################################################################
# This script was tuned specifically for the AIST FREA environment (Fixed in 2018)
#     AIST:National Institute of Advanced Industrial Science and Technology 
#     FREA:Fukushima Renewable Energy Institute
#
# What is the AIST FREA environment
#   Communication with SunSpecSVP is middleware called ExCon, and ExCon is
#   a mechanism to communicate with inverters and simulators.
#
# Libray Name    : gridsim_frea_simulator
# Libray Version : 4.1.0
# Update Date    : 2018/10/05
# Used Script    : SA09 - SA15
###################################################################################################

import os
import time
import serial
import socket
import gridsim
import grid_profiles

FREA_info = {
    'name': os.path.splitext(os.path.basename(__file__))[0],
    'mode': 'FREA_Simulator'
}

def gridsim_info():
    return FREA_info

def params(info, group_name):
    gname = lambda name: group_name + '.' + name
    pname = lambda name: group_name + '.' + GROUP_NAME + '.' + name
    mode = FREA_info['mode']
    info.param_add_value(gname('mode'), mode)
    info.param_group(gname(GROUP_NAME), label='%s Parameters' % mode,
                     active=gname('mode'),  active_value=mode, glob=True)
    info.param(pname('phases'), label='Phases', default=1, values=[1, 2, 3])
    info.param(pname('comm'), label='Communications Interface', default='TCP/IP', values=['Serial', 'TCP/IP'])
    info.param(pname('ip_addr'), label='IP Address',
               active=pname('comm'),  active_value=['TCP/IP'], default='127.0.0.1')
    info.param(pname('ip_port'), label='IP Port',
               active=pname('comm'),  active_value=['TCP/IP'], default=2001)

GROUP_NAME = 'frea'

class GridSim(gridsim.GridSim):
    """
    FREA Simulator info implementation.
    """

###################################################################################################
# SetOpe/Excon command group
###################################################################################################

    #############################################
    # __init__
    #############################################
    def __init__(self, ts, group_name):
        self.buffer_size = 1024
        self.conn = None
        gridsim.GridSim.__init__(self, ts, group_name)

        self.phases_param = ts.param_value('gridsim.frea.phases')
        self.auto_config = ts.param_value('gridsim.auto_config')
        self.freq_param = ts.param_value('gridsim.frea.freq')
        self.comm = ts.param_value('gridsim.frea.comm')
        self.ipaddr = ts.param_value('gridsim.frea.ip_addr')
        self.ipport = ts.param_value('gridsim.frea.ip_port')
        self.relay_state = gridsim.RELAY_OPEN
        self.regen_state = gridsim.REGEN_OFF
        self.timeout = 100
        self.cmd_str = ''
        self._cmd = None
        self._query = None
        self.profile_name = ts.param_value('profile.profile_name')

        if self.comm == 'TCP/IP':
            self._cmd = self.cmd_tcp
            self._query = self.query_tcp
        if self.auto_config == 'Enabled':
           ts.log('Configuring the FREA AC Simulator.')
           # self.config()

###        state = self.relay()
###        if state != gridsim.RELAY_CLOSED:
###            if self.ts.confirm('Would you like to close the grid simulator relay and ENERGIZE the system?') is False:
###                raise gridsim.GridSimError('Aborted grid simulation')
###            else:
###                self.ts.log('Turning on FREA AC Simulator.')
###                self.relay(state=gridsim.RELAY_CLOSED)


    #############################################
    # cmd_tcp
    #############################################
    def cmd_tcp(self, cmd_str):
        try:
            if self.conn is None:
                self.ts.log('ipaddr = %s  ipport = %s' % (self.ipaddr, self.ipport))
                self.conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.conn.settimeout(self.timeout)
                self.conn.connect((self.ipaddr, self.ipport))

            # print 'cmd> %s' % (cmd_str)
            self.conn.send(cmd_str)
        except Exception, e:
            raise gridsim.GridSimError(str(e))


    #############################################
    # query_tcp
    #############################################
    def query_tcp(self, cmd_str):
        resp = ''
        more_data = True

        self._cmd(cmd_str)

        while more_data:
            try:
                data = self.conn.recv(self.buffer_size)
                if len(data) > 0:
                    for d in data:
                        resp += d
                        if d == '\n': #\r
                            more_data = False
                            break
            except Exception, e:
                raise gridsim.GridSimError('Timeout waiting for response')

        return resp


    #############################################
    # cmd
    #############################################
    def cmd(self, cmd_str):
        self.cmd_str = cmd_str
        try:
            self._cmd(cmd_str)
        except Exception, e:
            raise gridsim.GridSimError(str(e))


    #############################################
    # query
    #############################################
    def query(self, cmd_str):
        try:
            self.ts.log('cmd_str = %s' % (cmd_str))
            resp = self._query(cmd_str).strip()
        except Exception, e:
            raise gridsim.GridSimError(str(e))
        return resp


###################################################################################################
# AC simulator command group
###################################################################################################

    #############################################
    # freq
    #############################################
    def freq(self, freq=None):
        """
        Set the value for frequency if provided. If none provided, obtains
        the value for frequency.
        """
        if freq is not None:
            self.cmd(':AC:SETB:FREQ %0.2f\n' % freq)
            self.freq_param = freq

        return freq


    #############################################
    # relay
    #############################################
    def relay(self, state=None):
        """
        Set the state of the relay if provided. Valid states are: RELAY_OPEN,
        RELAY_CLOSED. If none is provided, obtains the state of the relay.
        """
        if state is not None:
            if state == gridsim.RELAY_OPEN:
                self.cmd('abort;:outp off\n')
            elif state == gridsim.RELAY_CLOSED:
                self.cmd('abort;:outp on\n')
            else:
                raise gridsim.GridSimError('Invalid relay state. State = "%s"', state)
        else:
            relay = self.query(':AC:STAT:READ?\n').strip()
            #self.ts.log(relay)
            #if relay == '0':
            if relay == ':AC:STAT:READ 0':
                state = gridsim.RELAY_OPEN
            #elif relay == '1':
            elif relay == ':AC:STAT:READ 1':
                state = gridsim.RELAY_CLOSED

            else:
                state = gridsim.RELAY_UNKNOWN
        return state


    #############################################
    # cmd_run
    #############################################
    def cmd_run(self):
        relay = self.query(':AC:STAT:READ?\n').strip()
        if relay == ':AC:STAT:READ 1':
            self.cmd(':AC:CONT:RUN 1')


    #############################################
    # cmd_stop
    #############################################
    def cmd_stop(self):
        self.cmd(':AC:CONT:RUN 0')


    #############################################
    # voltage
    #############################################
    def voltage(self, voltage=None):
        """
        Set the value for voltage 1, 2, 3 if provided. If none provided, obtains
        the value for voltage. Voltage is a tuple containing a voltage value for
        each phase.
        """
        if voltage is not None:
            # set output voltage on all phases
            # self.ts.log_debug('voltage: %s, type: %s' % (voltage, type(voltage)))
            if type(voltage) is not list and type(voltage) is not tuple:
                self.cmd(':AC:SETB:VOLT PERC,%0.1f,%0.1f,%0.1f\n' % (voltage, voltage, voltage))
                v1 = voltage
                v2 = voltage
                v3 = voltage
            else:
                self.cmd(':AC:SETB:VOLT PERC,%0.1f,%0.1f,%0.1f\n' % (voltage[0], voltage[0], voltage[0])) # use the first value in the 3 phase list
                v1 = voltage[0]
                v2 = voltage[0]
                v3 = voltage[0]


    #############################################
    # voltageRR
    #############################################
    def voltageRR(self, voltage1=None, voltage2=None, voltage3=None):
        """
        Set the value for voltage 1, 2, 3 if provided. If none provided, obtains
        the value for voltage. Voltage is a tuple containing a voltage value for
        each phase.
        """
        if voltage1 is not None:
            # set output voltage on all phases
            # self.ts.log_debug('voltage: %s, type: %s' % (voltage, type(voltage)))
###            if type(voltage1) is not list and type(voltage1) is not tuple:
###                self.cmd(':AC:SETB:VOLT PERC,%0.1f,%0.1f,%0.1f\n' % (voltage1, voltage2, voltage3))
###                v1 = voltage1
###                v2 = voltage2
###                v3 = voltage3
###            else:
###                self.cmd(':AC:SETB:VOLT PERC,%0.1f,%0.1f,%0.1f\n' % (voltage1[0], voltage1[0], voltage1[0])) # use the first value in the 3 phase list
###                v1 = voltage1[0]
###                v2 = voltage1[0]
###                v3 = voltage1[0]
            self.ts.log('--------------AC Simulator command Start----------------')
            self.ts.log(':AC:SETB:VOLT PERC,%s,%s,%s\n' % (voltage1, voltage2, voltage3))
            self.cmd(':AC:SETB:VOLT PERC,%s,%s,%s\n' % (voltage1, voltage2, voltage3))
###            self.ts.log('--------------AC Simulator command End------------------')
            v1 = voltage1
            v2 = voltage2
            v3 = voltage3

        else:
            self.ts.log('--------------AC Simulator command Start----------------')
            self.ts.log(':AC:SETB:VOLT? PERC\n')
            rtn = self.query(':AC:SETB:VOLT? PERC\n')
###            self.ts.log('--------------AC Simulator command End------------------')
###            rtn = self.query(':AC:SETB:VOLT PERC,240,240,240\n').strip()
            self.ts.log('rtn = %s' % (rtn))
            tmp = rtn.split(",")
            if tmp[0] == ':AC:SETB:VOLT PERC':
###                tmp = rtn.split(",")
                v1 = tmp[1]
                v2 = tmp[2]
                v3 = tmp[3]
            else:
                v1 = 100.0
                v2 = 100.0
                v3 = 100.0

        return v1,v2,v3


    #############################################
    # voltageRH
    #############################################
    def voltageRH(self, voltage1=None, voltage2=None, voltage3=None):
        """
        Set the value for voltage 1, 2, 3 if provided. If none provided, obtains
        the value for voltage. Voltage is a tuple containing a voltage value for
        each phase.
        """
        if voltage1 is not None:
            # set output voltage on all phases
            # self.ts.log_debug('voltage: %s, type: %s' % (voltage, type(voltage)))
            self.ts.log('--------------AC Simulator command Start----------------')
###            self.ts.log(':AC:SETB:VOLT PERC,%s,%s,%s\n' % (voltage1, voltage2, voltage3))
            self.ts.log(':AC:SETB:VOLT VALU,%0.2f,%0.2f,%0.2f\n' % (voltage1, voltage2, voltage3))
            self.cmd(':AC:SETB:VOLT VALU,%0.2f,%0.2f,%0.2f\n' % (voltage1, voltage2, voltage3))
###            self.ts.log('--------------AC Simulator command End------------------')
            v1 = voltage1
            v2 = voltage2
            v3 = voltage3

        else:
            self.ts.log('--------------AC Simulator command Start----------------')
###            self.ts.log(':AC:SETB:VOLT? PERC\n')
            self.ts.log(':AC:SETB:VOLT? VALU\n')
            rtn = self.query(':AC:SETB:VALU? PERC\n')
###            self.ts.log('--------------AC Simulator command End------------------')
###            rtn = self.query(':AC:SETB:VOLT PERC,240,240,240\n').strip()
            self.ts.log('rtn = %s' % (rtn))
            tmp = rtn.split(",")
            if tmp[0] == ':AC:SETB:VOLT VALU':
###                tmp = rtn.split(",")
                v1 = tmp[1]
                v2 = tmp[2]
                v3 = tmp[3]
            else:
                v1 = 100.0
                v2 = 100.0
                v3 = 100.0

        return v1,v2,v3


###################################################################################################
# SanRex command group
###################################################################################################

    #############################################
    # close
    #############################################
    def close(self):
        try:
            # I/F Setting EUTs (Added)
            essX0=0                        ### Active Power
            essX1=0                        ### Power Ramp rate
            essX2=0                        ### Power Time Delay
            essX3=0                        ### Power Timeout period
            essX4=1                        ### Strage Charge from Grid Setting
            ess_setting = [essX0, essX1, essX2, essX3, essX4]

            bcc = 0
            cmd_str = 'ES '

            for ess in ess_setting:
                if ess < 0:
                   ess = ess + 65536
            
                cmd_str = cmd_str + format((int(ess)),'04X') +','

            cmd_str_chars = list(cmd_str)

            for cmd_chars in cmd_str_chars:
                v_xor = ord(cmd_chars)
                bcc = bcc ^ v_xor

            self.ts.log('--------------ESS1 command Start----------------')
            self.ts.log(':PCS:SABT ' + cmd_str + format((int(bcc)),'x'))
            cmd_str = ':PCS:SABT ' + cmd_str + format((int(bcc)),'x') + '\n'
            self.query(cmd_str)


            # Initial Setting EUTs (Added)
            X0 = 0      ### Time Delay
            X1 = 0      ### TimeOut period

            self.ts.log('--------------VV14 SanRex Param----------------')
            self.ts.log('X0 = %d, X1 = %d' % (X0, X1))

            v4_setting = [X0, X1]

            bcc = 0
            cmd_str = 'V4 '

            for v4 in v4_setting:
                if v4 < 0:
                   v4 = v4 + 65536

                cmd_str = cmd_str + format((int(v4)),'04X') +','

            cmd_str_chars = list(cmd_str)

            for cmd_chars in cmd_str_chars:
                v_xor = ord(cmd_chars)
                bcc = bcc ^ v_xor

            self.ts.log('--------------V14 command Start----------------')
            self.ts.log(':PCS:SABT ' + cmd_str + format((int(bcc)),'x'))
            cmd_str = ':PCS:SABT ' + cmd_str + format((int(bcc)),'x') + '\n'
            self.query(cmd_str)


            self.ts.log('--------------AC Simulator command Start----------------')
            self.ts.log(':AC:SETB:VOLT PERC,%s,%s,%s\n' % (100, 100, 100))
            self.cmd(':AC:SETB:VOLT PERC,%s,%s,%s\n' % (100, 100, 100))

        except Exception, e:
            #raise gridsim.GridSimError(str(e))
            pass


    #############################################
    # power_set
    #############################################
#    def power_set(self, params=None):
    def power_set(self, params=None, s_rated=None):
        """ Get/set volt/var control

        Params:
            Ena - Enabled (True/False)
            ActCrv - Active curve number (0 - no active curve)
            NCrv - Number of curves supported
            NPt - Number of points supported per curve
            WinTms - Randomized start time delay in seconds
            RmpTms - Ramp time in seconds to updated output level
            RvrtTms - Reversion time in seconds

        :param params: Dictionary of parameters to be updated.
        :return: Dictionary of active settings for volt/var control.
        """
#        if self.inv is None:
#            raise BattSimError('BATT not initialized')

        try:
            if params is not None:
                # I/F Setting EUTs (Added)
#                essX0= params*10000/50000   ### Active Power
                essX0= params*10000/s_rated ### Active Power
                essX1=0                     ### Power Ramp rate
                essX2=0                     ### Power Time Delay
                essX3=0                     ### Power Timeout period
                essX4=1                     ### Strage Charge from Grid Setting
                ess_setting = [essX0, essX1, essX2, essX3, essX4]

                bcc = 0
                cmd_str = 'ES '

                for ess in ess_setting:
                    if ess < 0:
                       ess = ess + 65536

                    cmd_str = cmd_str + format((int(ess)),'04X') +','

                cmd_str_chars = list(cmd_str)

                for cmd_chars in cmd_str_chars:
                    v_xor = ord(cmd_chars)
                    bcc = bcc ^ v_xor

                self.ts.log('--------------ESS1 command Start----------------')
                self.ts.log(':PCS:SABT ' + cmd_str + format((int(bcc)),'x'))
                cmd_str = ':PCS:SABT ' + cmd_str + format((int(bcc)),'x') + '\n'
###                self.cmd(cmd_str)
                self.query(cmd_str)

        except Exception, e:
            #raise battsim.battsimError(str(e))
            pass


    #############################################
    # power_setRR
    #############################################
    def power_setRR(self, voltage=None):
        """
        Set the value for voltage 1, 2, 3 if provided. If none provided, obtains
        the value for voltage. Voltage is a tuple containing a voltage value for
        each phase.
        """
        try:
            if voltage is not None:

                # I/F Setting EUTs (Added)
                essX0= voltage*100     ### Active Power
                essX1=0                ### Power Ramp rate
                essX2=0                ### Power Time Delay
                essX3=0                ### Power Timeout period
                essX4=1                ### Strage Charge from Grid Setting
                ess_setting = [essX0, essX1, essX2, essX3, essX4]

                bcc = 0
                cmd_str = 'ES '

                for ess in ess_setting:
                    if ess < 0:
                       ess = ess + 65536

                    cmd_str = cmd_str + format((int(ess)),'04X') +','

                cmd_str_chars = list(cmd_str)

                for cmd_chars in cmd_str_chars:
                    v_xor = ord(cmd_chars)
                    bcc = bcc ^ v_xor

                self.ts.log('--------------ESS1 command Start----------------')
                self.ts.log(':PCS:SABT ' + cmd_str + format((int(bcc)),'x'))
                cmd_str = ':PCS:SABT ' + cmd_str + format((int(bcc)),'x') + '\n'
###                self.cmd(cmd_str)
                self.query(cmd_str)

        except Exception, e:
            #raise battsim.battsimError(str(e))
            pass


    #############################################
    # power_setVV
    #############################################
    def power_setVV(self, params=None, s_rated=None, ramp_rate=None):
        """
        Set the value for voltage 1, 2, 3 if provided. If none provided, obtains
        the value for voltage. Voltage is a tuple containing a voltage value for
        each phase.
        """
        try:
            if params is not None:

                # I/F Setting EUTs (Added)
#                essX0= params*10000/50000   ### Active Power
                essX0= params*10000/s_rated ### Active Power
                essX1= ramp_rate            ### Power Ramp rate
                essX2=0                     ### Power Time Delay
                essX3=0                     ### Power Timeout period
                essX4=1                     ### Strage Charge from Grid Setting
                ess_setting = [essX0, essX1, essX2, essX3, essX4]

                bcc = 0
                cmd_str = 'ES '

                for ess in ess_setting:
                    if ess < 0:
                       ess = ess + 65536

                    cmd_str = cmd_str + format((int(ess)),'04X') +','

                cmd_str_chars = list(cmd_str)

                for cmd_chars in cmd_str_chars:
                    v_xor = ord(cmd_chars)
                    bcc = bcc ^ v_xor

                self.ts.log('--------------ESS1 command Start----------------')
                self.ts.log(':PCS:SABT ' + cmd_str + format((int(bcc)),'x'))
                cmd_str = ':PCS:SABT ' + cmd_str + format((int(bcc)),'x') + '\n'
###                self.cmd(cmd_str)
                self.query(cmd_str)

        except Exception, e:
            #raise battsim.battsimError(str(e))
            pass


    #############################################
    # ramp_rates
    #############################################
    def ramp_rates(self, params=None):
        """ Get/set ramp_rates control

        Params:
            power      - power
            ramp_rates - Ramp time in seconds to updated output level

        :param params: Dictionary of parameters to be updated.
        :return: Dictionary of active settings for ramp_rate control.
        """

        try:
            if params is not None:

                # I/F Setting EUTs (Added)
                arg_power = params['power']
                arg_ramp_rate = params['ramp_rate']
                arg_s_rate = params['s_rate']
#                essX0= arg_power*10000/50000        ### Active Power
                essX0= arg_power*10000/arg_s_rate   ### Active Power
                essX1= arg_ramp_rate                ### Power Ramp rate
                essX2=0                             ### Power Time Delay
                essX3=0                             ### Power Timeout period
                essX4=1                             ### Strage Charge from Grid Setting
                ess_setting = [essX0, essX1, essX2, essX3, essX4]

                bcc = 0
                cmd_str = 'ES '

                for ess in ess_setting:
                    if ess < 0:
                       ess = ess + 65536
                
                    cmd_str = cmd_str + format((int(ess)),'04X') +','

                cmd_str_chars = list(cmd_str)

                for cmd_chars in cmd_str_chars:
                    v_xor = ord(cmd_chars)
                    bcc = bcc ^ v_xor

                self.ts.log('--------------ESS1 command Start----------------')
                self.ts.log(':PCS:SABT ' + cmd_str + format((int(bcc)),'x'))
                cmd_str = ':PCS:SABT ' + cmd_str + format((int(bcc)),'x') + '\n'
###                self.cmd(cmd_str)
                self.query(cmd_str)

        except Exception, e:
            #raise battsim.battsimError(str(e))
            pass


    #############################################
    # fixed_pf
    #############################################
    def fixed_pf(self, params=None):
        """ Get/set fixed power factor control settings.

        Params:
            Ena - Enabled (True/False)
            PF - Power Factor set point
            WinTms - Randomized start time delay in seconds
            RmpTms - Ramp time in seconds to updated output level
            RvrtTms - Reversion time in seconds

        :param params: Dictionary of parameters to be updated.
        :return: Dictionary of active settings for fixed factor.
        """
###        if self.inv is None:
###            raise der.DERError('DER not initialized')

        try:
            if params is not None:
###                self.ts.log('DER SanRex-fixed_pf = %s' % (params))
                if params['Ena'] == True:
                   arg_pf = params['PF']
###                   self.ts.log('DER SanRex-params = %d' % (arg_pf))

                   #Send INV3 (V, Q) pairs according to required test as in Table 19,20.
                   n3X0=arg_pf*10000        ### Power Factor (PF)
                   n3X1=0                   ### PF Ramp Rate
                   n3X2=0                   ### PF Time Delay
                   n3X3=0                   ### PF Timeout period
                   pf_setting = [n3X0, n3X1, n3X2, n3X3]

                   bcc = 0
                   cmd_str = 'N3 '

                   for pf in pf_setting:
                       if pf < 0:
                          pf = pf + 65536
                   
                       cmd_str = cmd_str + format((int(pf)),'04X') +','

                   cmd_str_chars = list(cmd_str)

                   for cmd_chars in cmd_str_chars:
                       v_xor = ord(cmd_chars)
                       bcc = bcc ^ v_xor

                   self.ts.log('--------------N3 command Start----------------')
                   self.ts.log(':PCS:SABT ' + cmd_str + format((int(bcc)),'x'))
                   cmd_str = ':PCS:SABT ' + cmd_str + format((int(bcc)),'x') + '\n'
###                   self.cmd(cmd_str)
                   self.query(cmd_str)
            else:
                #params = {'Ena': True, 'PF': 100, 'WinTms': 0, 'RmpTms': 0, 'RvrtTms': 0}
                #params['Ena'] = True
                #params['PF'] = 1
                #params['WinTms'] = 0
                #params['RmpTms'] = 0
                #params['RvrtTms'] = 0

                params = {}

        except Exception, e:
            #raise battsim.battsimError(str(e))
            pass


    #############################################
    # volt_var
    #############################################
    def volt_var(self, params=None):
        """ Get/set volt/var control

        Params:
            Ena - Enabled (True/False)
            ActCrv - Active curve number (0 - no active curve)
            NCrv - Number of curves supported
            NPt - Number of points supported per curve
            WinTms - Randomized start time delay in seconds
            RmpTms - Ramp time in seconds to updated output level
            RvrtTms - Reversion time in seconds

        :param params: Dictionary of parameters to be updated.
        :return: Dictionary of active settings for volt/var control.
        """
###        if self.inv is None:
###            raise der.DERError('DER not initialized')

        try:
            if params is not None:
                pass

            else:
                params = {}
                params['Ena'] = True
                params['ActCrv'] = 1
                params['NCrv'] = 1
                params['NPt'] = 1
                params['WinTms'] = 0
                params['RmpTms'] = 0
                params['RvrtTms'] = 0

                params['curve'] = 1

        except Exception, e:
            #raise der.DERError(str(e))
            pass

        return params


    #############################################
    # volt_var_curve
    #############################################
    def volt_var_curve(self, id, params=None):
        """ Get/set volt/var curve
            v [] - List of voltage curve points
            var [] - List of var curve points based on DeptRef
            DeptRef - Dependent reference type: 'VAR_MAX_PCT', 'VAR_AVAL_PCT', 'VA_MAX_PCT', 'W_MAX_PCT'
            RmpTms - Ramp timer
            RmpDecTmm - Ramp decrement timer
            RmpIncTmm - Ramp increment timer

        :param params: Dictionary of parameters to be updated.
        :return: Dictionary of active settings for volt/var curve control.
        """
###        if self.inv is None:
###            raise der.DERError('DER not initialized')

        try:
            self.ts.log('volt_var_curve params = %s' % (params))
            arg_v = params['v']
            arg_q = params['var']

            X0 = arg_v[0]*100         ### V1
            X1 = arg_v[1]*100         ### V2
            X2 = arg_v[2]*100         ### V3
            X3 = arg_v[3]*100         ### V4
            X4 = arg_q[0]*100*-1      ### Q1
            X5 = arg_q[1]*100*-1      ### Q2
            X6 = arg_q[2]*100*-1      ### Q3
            X7 = arg_q[3]*100*-1      ### Q4
            X8 = 0                    ### Var Ramp Rate 
            X9 = 0                    ### V1 Time Delay
            X10 = 0                   ### V1 Timeout period

            self.ts.log('--------------VV11 SanRex Param----------------')
            self.ts.log('X0 = %d, X1 = %d, X2 = %d, X3 = %d, X4 = %d, X5 = %d, X6 = %d, X7 = %d, X8 = %d, X9 = %d, X10 = %d' % (X0, X1, X2, X3, X4, X5, X6, X7, X8, X9, X10))
            v11_setting = [X0, X1, X2, X3, X4, X5, X6, X7, X8, X9, X10]

            bcc = 0
            cmd_str = 'V1 '

            for vv in v11_setting:
                if vv < 0:
                   vv = vv + 65536
            
                cmd_str = cmd_str + format((int(vv)),'04X') +','

            cmd_str_chars = list(cmd_str)

            for cmd_chars in cmd_str_chars:
                v_xor = ord(cmd_chars)
                bcc = bcc ^ v_xor

            self.ts.log('--------------V1 command Start----------------')
            self.ts.log(':PCS:SABT ' + cmd_str + format((int(bcc)),'x'))
            cmd_str = ':PCS:SABT ' + cmd_str + format((int(bcc)),'x') + '\n'
###            self.cmd(cmd_str)
            self.query(cmd_str)

        except Exception, e:
            #raise battsim.battsimError(str(e))
            pass


###################################################################################################
# WT3000 command group
###################################################################################################

    #############################################
    # NUM
    #############################################
    def wt3000_data_capture_read(self):

        rtn_up = {}

        self.ts.log('--------------WT3000 command Start----------------')

        ##########
        # Segment3
        ##########
        # Voltage(U)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 23')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        #rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 23\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 23\n')
###        self.ts.sleep(3)
###        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 23\n')
        #test start
###        rtn = ':MEAS:WT3000 200.222E+00'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['S3_AC_VRMS_1'] = float(tmp[1])
            rtn_up['S3_AC_VRMS_2'] = float(tmp[1])
            rtn_up['S3_AC_VRMS_3'] = float(tmp[1])

        # Current(I)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 24')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 24\n')
###        self.ts.sleep(3)
        #test start
###        rtn = ':MEAS:WT3000 0.848E+00'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['S3_AC_IRMS_1'] = float(tmp[1])
            rtn_up['S3_AC_IRMS_2'] = float(tmp[1])
            rtn_up['S3_AC_IRMS_3'] = float(tmp[1])

        # Active power(P)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 25')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 25\n')
###        self.ts.sleep(3)
        #test start
###        rtn = ':MEAS:WT3000 -0.073E+03'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['S3_AC_P_1'] = float(tmp[1])
            rtn_up['S3_AC_P_2'] = float(tmp[1])
            rtn_up['S3_AC_P_3'] = float(tmp[1])

        # Apparent power(S)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 26')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 26\n')
###        self.ts.sleep(3)
        #test start
###        rtn = ':MEAS:WT3000 -0.073E+03'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['S3_AC_S_1'] = float(tmp[1])
            rtn_up['S3_AC_S_2'] = float(tmp[1])
            rtn_up['S3_AC_S_3'] = float(tmp[1])

        # Reactive power(Q)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 27')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 27\n')
###        self.ts.sleep(3)
        #test start
###        rtn = ':MEAS:WT3000 -0.10001E+05'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['S3_AC_Q_1'] = float(tmp[1])
            rtn_up['S3_AC_Q_2'] = float(tmp[1])
            rtn_up['S3_AC_Q_3'] = float(tmp[1])

        # Frequency voltage(fu)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 30')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 30\n')
###        self.ts.sleep(3)
        #test start
###        rtn = ':MEAS:WT3000 47.500'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['S3_AC_FREQ_1'] = float(tmp[1])
            rtn_up['S3_AC_FREQ_2'] = float(tmp[1])
            rtn_up['S3_AC_FREQ_3'] = float(tmp[1])

        ##########
        # Segment4
        ##########
        # Voltage(U)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 34')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        #rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 34\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 34\n')
###        self.ts.sleep(3)
###        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 34\n')
        #test start
###        rtn = ':MEAS:WT3000 202.222E+00'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['S4_AC_VRMS_1'] = float(tmp[1])
            rtn_up['S4_AC_VRMS_2'] = float(tmp[1])
            rtn_up['S4_AC_VRMS_3'] = float(tmp[1])

        # Current(I)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 35')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 35\n')
###        self.ts.sleep(3)
        #test start
###        rtn = ':MEAS:WT3000 0.851E+00'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['S4_AC_IRMS_1'] = float(tmp[1])
            rtn_up['S4_AC_IRMS_2'] = float(tmp[1])
            rtn_up['S4_AC_IRMS_3'] = float(tmp[1])

        # Active power(P)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 36')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 36\n')
###        self.ts.sleep(3)
        #test start
###        rtn = ':MEAS:WT3000 -0.076E+03'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['S4_AC_P_1'] = float(tmp[1])
            rtn_up['S4_AC_P_2'] = float(tmp[1])
            rtn_up['S4_AC_P_3'] = float(tmp[1])

        # Apparent power(S)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 37')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 37\n')
###        self.ts.sleep(3)
        #test start
###        rtn = ':MEAS:WT3000 -0.073E+03'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['S4_AC_S_1'] = float(tmp[1])
            rtn_up['S4_AC_S_2'] = float(tmp[1])
            rtn_up['S4_AC_S_3'] = float(tmp[1])

        # Reactive power(Q)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 38')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 38\n')
###        self.ts.sleep(3)
        #test start
###        rtn = ':MEAS:WT3000 -0.10021E+05'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['S4_AC_Q_1'] = float(tmp[1])
            rtn_up['S4_AC_Q_2'] = float(tmp[1])
            rtn_up['S4_AC_Q_3'] = float(tmp[1])

        # Frequency voltage(fu)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 41')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 41\n')
###        self.ts.sleep(3)
        #test start
###        rtn = ':MEAS:WT3000 47.500'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['S4_AC_FREQ_1'] = float(tmp[1])
            rtn_up['S4_AC_FREQ_2'] = float(tmp[1])
            rtn_up['S4_AC_FREQ_3'] = float(tmp[1])

        #####################
        # Segment3 + Segment4
        #####################
        # Voltage(U)
        rtn_up['AC_VRMS_1'] = (rtn_up['S3_AC_VRMS_1'] + rtn_up['S4_AC_VRMS_1'])/2
        rtn_up['AC_VRMS_2'] = (rtn_up['S3_AC_VRMS_2'] + rtn_up['S4_AC_VRMS_2'])/2
        rtn_up['AC_VRMS_3'] = (rtn_up['S3_AC_VRMS_3'] + rtn_up['S4_AC_VRMS_3'])/2

        # Current(I)
        rtn_up['AC_IRMS_1'] = rtn_up['S3_AC_IRMS_1'] + rtn_up['S4_AC_IRMS_1']
        rtn_up['AC_IRMS_2'] = rtn_up['S3_AC_IRMS_2'] + rtn_up['S4_AC_IRMS_2']
        rtn_up['AC_IRMS_3'] = rtn_up['S3_AC_IRMS_3'] + rtn_up['S4_AC_IRMS_3']

        # Active power(P)
        rtn_up['AC_P_1'] = rtn_up['S3_AC_P_1'] + rtn_up['S4_AC_P_1']
        rtn_up['AC_P_2'] = rtn_up['S3_AC_P_2'] + rtn_up['S4_AC_P_2']
        rtn_up['AC_P_3'] = rtn_up['S3_AC_P_3'] + rtn_up['S4_AC_P_3']

        # Apparent power(S)
        rtn_up['AC_S_1'] = rtn_up['S3_AC_S_1'] + rtn_up['S4_AC_S_1']
        rtn_up['AC_S_2'] = rtn_up['S3_AC_S_2'] + rtn_up['S4_AC_S_2']
        rtn_up['AC_S_3'] = rtn_up['S3_AC_S_3'] + rtn_up['S4_AC_S_3']

        # Reactive power(Q)
        rtn_up['AC_Q_1'] = rtn_up['S3_AC_Q_1'] + rtn_up['S4_AC_Q_1']
        rtn_up['AC_Q_2'] = rtn_up['S3_AC_Q_2'] + rtn_up['S4_AC_Q_2']
        rtn_up['AC_Q_3'] = rtn_up['S3_AC_Q_3'] + rtn_up['S4_AC_Q_3']

        # Frequency voltage(fu)
        rtn_up['AC_FREQ_1'] = (rtn_up['S3_AC_FREQ_1'] + rtn_up['S4_AC_FREQ_1'])/2
        rtn_up['AC_FREQ_2'] = (rtn_up['S3_AC_FREQ_2'] + rtn_up['S4_AC_FREQ_2'])/2
        rtn_up['AC_FREQ_3'] = (rtn_up['S3_AC_FREQ_3'] + rtn_up['S4_AC_FREQ_3'])/2


        self.ts.log('--------------WT3000 command End-----------------')

        return rtn_up


###################################################################################################
# __main__ pass
###################################################################################################
if __name__ == "__main__":
    pass
