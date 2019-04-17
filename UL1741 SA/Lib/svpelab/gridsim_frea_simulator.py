####################################################################################################
# This script was tuned specifically for the AIST FREA environment (Fixed in 2019)
#     AIST:National Institute of Advanced Industrial Science and Technology 
#     FREA:Fukushima Renewable Energy Institute
#
# What is the AIST FREA environment
#   Communication with SunSpecSVP is middleware called ExCon, and ExCon is
#   a mechanism to communicate with inverters and simulators.
#
# Libray Name    : gridsim_frea_simulator
# Libray Version : 4.5.0
# Update Date    : 2019/02/22
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
                #self.cmd(':AC:SETB:VOLT PERC,%0.1f,%0.1f,%0.1f\n' % (voltage, voltage, voltage))
                #self.cmd(':AC:SETB:VOLT VALU,%d,%d,%d\n' % (round(voltage), round(voltage), round(voltage)))
                self.cmd(':AC:SETB:VOLT VALU,%0.2f,%0.2f,%0.2f\n' % (round(voltage,2), round(voltage,2), round(voltage,2)))
                self.ts.log('--------------voltage command Start----------------')
                #self.ts.log(':AC:SETB:VOLT VALU,%d,%d,%d\n' % (round(voltage), round(voltage), round(voltage)))
                self.ts.log(':AC:SETB:VOLT VALU,%0.2f,%0.2f,%0.2f\n' % (round(voltage,2), round(voltage,2), round(voltage,2)))
            else:
                #self.cmd(':AC:SETB:VOLT PERC,%0.1f,%0.1f,%0.1f\n' % (voltage[0], voltage[0], voltage[0]))                                        # use the first value in the 3 phase list
                #self.cmd(':AC:SETB:VOLT VALU,%d,%d,%d\n' % (round(voltage[0]), round(voltage[0]), round(voltage[0])))                            # use the first value in the 3 phase list
                self.cmd(':AC:SETB:VOLT VALU,%0.2f,%0.2f,%0.2f\n' % (round(voltage[0],2), round(voltage[0],2), round(voltage[0],2)))              # use the first value in the 3 phase list
                self.ts.log('--------------voltage command Start----------------')
                #self.ts.log(':AC:SETB:VOLT PERC,%0.1f,%0.1f,%0.1f\n' % (voltage[0], voltage[0], voltage[0]))
                #self.ts.log(':AC:SETB:VOLT VALU,%d,%d,%d\n' % (round(voltage[0]), round(voltage[0]), round(voltage[0])))
                self.ts.log(':AC:SETB:VOLT VALU,%0.2f,%0.2f,%0.2f\n' % (round(voltage[0],2), round(voltage[0],2),round(voltage[0],2)))


    #############################################
    # voltageV
    #############################################
    def voltageV(self, voltage1=None, voltage2=None, voltage3=None):
        """
        Set the value for voltage 1, 2, 3 if provided. If none provided, obtains
        the value for voltage. Voltage is a tuple containing a voltage value for
        each phase.
        """
        if voltage1 is not None:
            # set output voltage on all phases
            # self.ts.log_debug('voltage: %s, type: %s' % (voltage, type(voltage)))
            self.ts.log('--------------AC Simulator command Start----------------')
            self.ts.log(':AC:SETB:VOLT VALU,%d,%d,%d\n' % (voltage1, voltage2, voltage3))
            self.cmd(':AC:SETB:VOLT VALU,%d,%d,%d\n' % (voltage1, voltage2, voltage3))
###            self.ts.log('--------------AC Simulator command End------------------')
            v1 = voltage1
            v2 = voltage2
            v3 = voltage3

        else:
            self.ts.log('--------------AC Simulator command Start----------------')
            self.ts.log(':AC:SETB:VOLT? VALU\n')
            rtn = self.query(':AC:SETB:VOLT? VALU\n')
            self.ts.log('rtn = %s' % (rtn))
            tmp = rtn.split(",")
            if tmp[0] == ':AC:SETB:VOLT VALU':
                v1 = tmp[1]
                v2 = tmp[2]
                v3 = tmp[3]
            else:
                v1 = 200.0
                v2 = 200.0
                v3 = 200.0

        return v1,v2,v3


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
# SanRex PCS command group
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

                self.ts.log('params = %d, s_rated = %d, ramp_rate = %d' % (params, s_rated, ramp_rate))

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
                arg_s_rate = params['s_rated']
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

    #############################################
    # FW51  SA15 Volt-Watt(VW) SanRex
    #############################################
    #############################################
    def volt_w(self, v_start, p_rated, k_power_volt, v_max, s_rated, v_nom):

        try:
            if params is not None:

                ww1x0 = float(v_start)*10000/100                       ### V1
###                ww1x1 = p_rated*10000/50000                            ### W1
                ww1x1 = p_rated*10000/s_rated                          ### W1
###                ww1x2 = (p_rated*100/50000)/k_power_volt*100 + ww1x0   ### V2
                ww1x2 = (p_rated*100/s_rated)/k_power_volt*100 + ww1x0 ### V2
                ww1x3 = 0                                              ### W2
                ww1x4 = (ww1x2 + (float(v_max)*10000/100))/2           ### V3
                ww1x5 = 0                                              ### W3
                ww1x6 = float(v_max)*10000/100                         ### V4
                ww1x7 = 0                                         ### W4
                ww1x8 = 0                                         ### Ramp rate
                ww1x9 = 0                                         ### Recover R rate
                ww1x10 = 0                                        ### Time Delay
                ww1x11 = 0                                        ### Timeout period

                self.ts.log('--------------VW51 SanRex Param (temp)---------')
                self.ts.log('X0 = %d, X1 = %d, X2 = %d, X3 = %d, X4 = %d, X5 = %d '\
                            'X6 = %d, X7 = %d, X8 = %d, X9 = %d, X10 = %d, X11 = %d '\
                             % (ww1x0, ww1x1, ww1x2, ww1x3, ww1x4, ww1x5, ww1x6, 
                                     ww1x7, ww1x8, ww1x9, ww1x10, ww1x11))

                w1x0 = float(ww1x0)/float(v_nom)*100                   ### V1
                w1x1 = ww1x1                                           ### W1
                w1x2 = float(ww1x2)/float(v_nom)*100                   ### V2
                w1x3 = ww1x3                                           ### W2
                w1x4 = float(ww1x4)/float(v_nom)*100                   ### V3
                w1x5 = ww1x5                                           ### W3
                w1x6 = float(ww1x6)/float(v_nom)*100                   ### V4
                w1x7 = ww1x7                                           ### W4
                w1x8 = ww1x8                                           ### Ramp rate
                w1x9 = ww1x9                                           ### Recover R rate
                w1x10 = ww1x10                                         ### Time Delay
                w1x11 = ww1x11                                         ### Timeout period

                self.ts.log('--------------VW51 SanRex Param----------------')
                self.ts.log('X0 = %d, X1 = %d, X2 = %d, X3 = %d, X4 = %d, X5 = %d '\
                            'X6 = %d, X7 = %d, X8 = %d, X9 = %d, X10 = %d, X11 = %d '\
                             % (w1x0, w1x1, w1x2, w1x3, w1x4, w1x5, w1x6, 
                                     w1x7, w1x8, w1x9, w1x10, w1x11))

                w1_setting = [w1x0, w1x1, w1x2, w1x3, w1x4, w1x5, 
                              w1x6, w1x7, w1x8, w1x9, w1x10, w1x11]

                bcc = 0
                cmd_str = 'W1 '

                for w1 in w1_setting:
                    if w1 < 0:
                       w1 = w1 + 65536

                    cmd_str = cmd_str + format((int(w1)),'04X') +','

                cmd_str_chars = list(cmd_str)

                for cmd_chars in cmd_str_chars:
                    v_xor = ord(cmd_chars)
                    bcc = bcc ^ v_xor

                self.ts.log('--------------W1 command Start----------------')
                self.ts.log(':PCS:SABT ' + cmd_str + format((int(bcc)),'x'))
                cmd_str = ':PCS:SABT ' + cmd_str + format((int(bcc)),'x') + '\n'
                self.query(cmd_str)

        except Exception, e:
            #raise battsim.battsimError(str(e))
            pass

    #############################################
    # FW21  SA14 Frequency-Watt(FW) SanRex
    #############################################
    def fw_sanrex(self, k_pf, fstart, f_nom):

        try:
            if params is not None:

                F1X0 = float(k_pf)*10               ### WGrahi
                F1X1 = float(fstart - f_nom)*100    ### HzStrhi
                F1X2 = 0                                ### HzHyshi
                F1X3 = 0                                ### HysEnahi
                F1X4 = 0                             ### ReqRamphi
                F1X5 = 0                             ### RecoverRamphi
                F1X6 = 0                             ### TwPFlowEna
                F1X7 = 0                             ### HzlowEna
                F1X8 = 0                             ### WGralow
                F1X9 = 0                             ### HzStlow
                F1X10 = 0                            ### HzHyslow
                F1X11 = 0                            ### HysEnalow
                F1X12 = 0                            ### ReqRamplow
                F1X13 = 0                            ### RecoverRamplow
                F1X14 = 0                            ### Time Delay
                F1X15 = 0                            ### Tmout period

                self.ts.log('--------------FW21 SanRex Param----------------')
                self.ts.log('X0 = %d, X1 = %d, X2 = %d, X3 = %d, X4 = %d, X5 = %d '\
                            'X6 = %d, X7 = %d, X8 = %d, X9 = %d, X10 = %d '\
                            'X11 = %d, X12 = %d, X13 = %d, X14 = %d, X15 = %d '\
                             % (F1X0, F1X1, F1X2, F1X3, F1X4, F1X5, F1X6, F1X7, 
                                F1X8, F1X9, F1X10, F1X11, F1X12, F1X13, F1X14, F1X15))

                f1_setting = [F1X0, F1X1, F1X2, F1X3, F1X4, 
                              F1X5, F1X6, F1X7, F1X8, F1X9, 
                              F1X10, F1X11, F1X12, F1X13, F1X14, F1X15]

                bcc = 0
                cmd_str = 'F1 '

                for f1 in f1_setting:
                    if f1 < 0:
                       f1 = f1 + 65536

                    cmd_str = cmd_str + format((int(f1)),'04X') +','

                cmd_str_chars = list(cmd_str)

                for cmd_chars in cmd_str_chars:
                    v_xor = ord(cmd_chars)
                    bcc = bcc ^ v_xor

                self.ts.log('--------------F1 command Start----------------')
                self.ts.log(':PCS:SABT ' + cmd_str + format((int(bcc)),'x'))
                cmd_str = ':PCS:SABT ' + cmd_str + format((int(bcc)),'x') + '\n'
                self.query(cmd_str)

        except Exception, e:
            #raise battsim.battsimError(str(e))
            pass

    #############################################
    # close4fq
    #############################################
    def close4fq(self):
        
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

            self.ts.log('--------------Close ESS1 command----------------')
            self.ts.log(':PCS:SABT ' + cmd_str + format((int(bcc)),'x'))
            cmd_str = ':PCS:SABT ' + cmd_str + format((int(bcc)),'x') + '\n'
            self.query(cmd_str)

            F1X0 = 0                             ### WGrahi
            F1X1 = 0                             ### HzStrhi
            F1X2 = 0                             ### HzHyshi
            F1X3 = 0                             ### HysEnahi
            F1X4 = 0                             ### ReqRamphi
            F1X5 = 0                             ### RecoverRamphi
            F1X6 = 0                             ### TwPFlowEna
            F1X7 = 0                             ### HzlowEna
            F1X8 = 0                             ### WGralow
            F1X9 = 0                             ### HzStlow
            F1X10 = 0                            ### HzHyslow
            F1X11 = 0                            ### HysEnalow
            F1X12 = 0                            ### ReqRamplow
            F1X13 = 0                            ### RecoverRamplow
            F1X14 = 0                            ### Time Delay
            F1X15 = 1                            ### Tmout period

            f1_setting = [F1X0, F1X1, F1X2, F1X3, F1X4, 
                          F1X5, F1X6, F1X7, F1X8, F1X9, 
                          F1X10, F1X11, F1X12, F1X13, F1X14, F1X15]

            bcc = 0
            cmd_str = 'F1 '

            for f1 in f1_setting:
                if f1 < 0:
                   f1 = f1 + 65536
            
                cmd_str = cmd_str + format((int(f1)),'04X') +','

            cmd_str_chars = list(cmd_str)

            for cmd_chars in cmd_str_chars:
                v_xor = ord(cmd_chars)
                bcc = bcc ^ v_xor

            self.ts.log('--------------Close F1 command----------------')
            self.ts.log(':PCS:SABT ' + cmd_str + format((int(bcc)),'x'))
            cmd_str = ':PCS:SABT ' + cmd_str + format((int(bcc)),'x') + '\n'
            self.query(cmd_str)

        except Exception as e:
            #raise gridsim.GridSimError(str(e))
            pass

    #############################################
    # close4vw
    #############################################
    def close4vw(self, v_start, p_rated, k_power_volt, v_max, s_rated, v_nom):

        try:
            ww1x0 = float(v_start)*10000/100                       ### V1
###            ww1x1 = p_rated*10000/50000                            ### W1
            ww1x1 = p_rated*10000/s_rated                          ### W1
###            ww1x2 = (p_rated*100/50000)/k_power_volt*100 + ww1x0   ### V2
            ww1x2 = (p_rated*100/s_rated)/k_power_volt*100 + ww1x0 ### V2
            ww1x3 = 0                                              ### W2
            ww1x4 = (ww1x2 + (float(v_max)*10000/100))/2           ### V3
            ww1x5 = 0                                              ### W3
            ww1x6 = float(v_max)*10000/100                         ### V4
            ww1x7 = 0                                         ### W4
            ww1x8 = 0                                         ### Ramp rate
            ww1x9 = 0                                         ### Recover R rate
            ww1x10 = 0                                        ### Time Delay
            ww1x11 = 1                                        ### Timeout period

            self.ts.log('--------------VW51 SanRex Param (temp)---------')
            self.ts.log('X0 = %d, X1 = %d, X2 = %d, X3 = %d, X4 = %d, X5 = %d '\
                        'X6 = %d, X7 = %d, X8 = %d, X9 = %d, X10 = %d, X11 = %d '\
                         % (ww1x0, ww1x1, ww1x2, ww1x3, ww1x4, ww1x5, ww1x6, 
                                 ww1x7, ww1x8, ww1x9, ww1x10, ww1x11))

            w1x0 = float(ww1x0)/float(v_nom)*100                   ### V1
            w1x1 = ww1x1                                           ### W1
            w1x2 = float(ww1x2)/float(v_nom)*100                   ### V2
            w1x3 = ww1x3                                           ### W2
            w1x4 = float(ww1x4)/float(v_nom)*100                   ### V3
            w1x5 = ww1x5                                           ### W3
            w1x6 = float(ww1x6)/float(v_nom)*100                   ### V4
            w1x7 = ww1x7                                           ### W4
            w1x8 = ww1x8                                           ### Ramp rate
            w1x9 = ww1x9                                           ### Recover R rate
            w1x10 = ww1x10                                         ### Time Delay
            w1x11 = ww1x11                                         ### Timeout period

            self.ts.log('--------------VW51 SanRex Param (close)--------')
            self.ts.log('X0 = %d, X1 = %d, X2 = %d, X3 = %d, X4 = %d, X5 = %d '\
                        'X6 = %d, X7 = %d, X8 = %d, X9 = %d, X10 = %d, X11 = %d '\
                         % (w1x0, w1x1, w1x2, w1x3, w1x4, w1x5, w1x6, 
                            w1x7, w1x8, w1x9, w1x10, w1x11))

            w1_setting = [w1x0, w1x1, w1x2, w1x3, w1x4, w1x5, 
                          w1x6, w1x7, w1x8, w1x9, w1x10, w1x11]

            bcc = 0
            cmd_str = 'W1 '

            for w1 in w1_setting:
                if w1 < 0:
                   w1 = w1 + 65536
            
                cmd_str = cmd_str + format((int(w1)),'04X') +','

            cmd_str_chars = list(cmd_str)

            for cmd_chars in cmd_str_chars:
                v_xor = ord(cmd_chars)
                bcc = bcc ^ v_xor

            self.ts.log('--------------Close W1 command----------------')
            self.ts.log(':PCS:SABT ' + cmd_str + format((int(bcc)),'x'))
            cmd_str = ':PCS:SABT ' + cmd_str + format((int(bcc)),'x') + '\n'
            self.query(cmd_str)

            self.ts.sleep(3)

            # I/F Setting EUTs (Added)
            essX0 = 0                        ### Active Power
            essX1 = 0                        ### Power Ramp rate
            essX2 = 0                        ### Power Time Delay
            essX3 = 0                        ### Power Timeout period
            essX4 = 1                        ### Strage Charge from Grid Setting
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

            self.ts.log('--------------Close ESS1 command----------------')
            self.ts.log(':PCS:SABT ' + cmd_str + format((int(bcc)),'x'))
            cmd_str = ':PCS:SABT ' + cmd_str + format((int(bcc)),'x') + '\n'
            self.query(cmd_str)

        except Exception as e:
            #raise gridsim.GridSimError(str(e))
            pass

    #############################################
    # volt_watt
    #############################################
    def volt_watt(self, params=None):
        pass

    #############################################
    # freq_watt_param
    #############################################
    def freq_watt_param(self, params=None):

        """ Get/set frequency-watt with parameters

        Params:
            Ena - Enabled (True/False)
            HysEna - Enable hysterisis (True/False)
            WGra - The slope of the reduction in the maximum allowed watts output as a function of frequency.
            HzStr - The frequency deviation from nominal frequency (ECPNomHz) at which a snapshot of the instantaneous
                    power output is taken to act as the CAPPED power level (PM) and above which reduction in power
                    output occurs.
            HzStop - The frequency deviation from nominal frequency (ECPNomHz) at which curtailed power output may
                    return to normal and the cap on the power level value is removed.
            HzStopWGra - The maximum time-based rate of change at which power output returns to normal after having
                         been capped by an over frequency event.

        :param params: Dictionary of parameters to be updated.
        :return: Dictionary of active settings for frequency-watt with parameters control.
        """

        try:
            if params is not None:
                pass
                # self.ts.confirm('Set the following parameters %s' % params)
                
            else:
                params = {}
                self.inv.hfrtc.read()
                if self.inv.freq_watt_param.ModEna == 0:
                    params['Ena'] = False
                else:
                    params['Ena'] = True
                if self.inv.freq_watt_param.HysEna == 0:
                    params['HysEna'] = False
                else:
                    params['HysEna'] = True
                params['WGra'] = self.inv.freq_watt_param.WGra
                params['HzStr'] = self.inv.freq_watt_param.HzStr
                params['HzStop'] = self.inv.freq_watt_param.HzStop
                params['HzStopWGra'] = self.inv.freq_watt_param.HzStopWGra

        except Exception as e:
            raise der.DERError(str(e))

        return params

    #############################################
    # freq_watt
    #############################################
    def freq_watt(self, params=None):

        """ Get/set freq/watt control

        Params:
            Ena - Enabled (True/False)
            ActCrv - Active curve number (0 - no active curve)
            NCrv - Number of curves supported
            NPt - Number of points supported per curve
            WinTms - Randomized start time delay in seconds
            RmpTms - Ramp time in seconds to updated output level
            RvrtTms - Reversion time in seconds

        :param params: Dictionary of parameters to be updated.
        :return: Dictionary of active settings for freq/watt control.
        """

        try:
            if params is not None:
                pass
                # self.ts.confirm('Set the following parameters %s' % params)

            else:
                params = {}
                self.inv.freq_watt.read()
                if self.inv.freq_watt.ModEna == 0:
                    params['Ena'] = False
                else:
                    params['Ena'] = True
                params['ActCrv'] = self.inv.freq_watt.ActCrv
                params['NCrv'] = self.inv.freq_watt.NCrv
                params['NPt'] = self.inv.freq_watt.NPt
                params['WinTms'] = self.inv.freq_watt.WinTms
                params['RmpTms'] = self.inv.freq_watt.RmpTms
                params['RvrtTms'] = self.inv.freq_watt.RvrtTms
                if self.inv.freq_watt.ActCrv != 0:
                    params['curve'] = self.freq_watt_curve(id=self.inv.freq_watt.ActCrv)

        except Exception as e:
            raise der.DERError(str(e))

        return params

    #############################################
    # freq_watt_curve
    #############################################
    def freq_watt_curve(self, id, params=None):
        
        """ Get/set volt/var curve
            hz [] - List of frequency curve points
            w [] - List of power curve points
            CrvNam - Optional description for curve. (Max 16 chars)
            RmpPT1Tms - The time of the PT1 in seconds (time to accomplish a change of 95%).
            RmpDecTmm - Ramp decrement timer
            RmpIncTmm - Ramp increment timer
            RmpRsUp - The maximum rate at which the power may be increased after releasing the frozen value of
                      snap shot function.
            SnptW - 1=enable snapshot/capture mode
            WRef - Reference active power (default = WMax).
            WRefStrHz - Frequency deviation from nominal frequency at the time of the snapshot to start constraining
                        power output.
            WRefStopHz - Frequency deviation from nominal frequency at which to release the power output.
            ReadOnly - 0 = READWRITE, 1 = READONLY

        :param params: Dictionary of parameters to be updated.
        :return: Dictionary of active settings for freq/watt curve.
        """

        try:
            if params is not None:
                pass
                # self.ts.confirm('Set the following parameters %s' % params)

            else:
                params = {}
                act_pt = curve.ActPt
                params['CrvNam'] = curve.CrvNam
                params['RmpPT1Tms'] = curve.RmpPT1Tms
                params['RmpDecTmm'] = curve.RmpDecTmm
                params['RmpIncTmm'] = curve.RmpIncTmm
                params['RmpRsUp'] = curve.RmpRsUp
                params['SnptW'] = curve.SnptW
                params['WRef'] = curve.WRef
                params['WRefStrHz'] = curve.WRefStrHz
                params['WRefStopHz'] = curve.WRefStopHz
                params['ReadOnly'] = curve.ReadOnly
                params['id'] = id  #also store the curve number
                hz = []
                w = []
                for i in xrange(1, act_pt + 1):  # SunSpec point index starts at 1
                    hz_point = 'Hz%d' % i
                    w_point = 'VAr%d' % i
                    hz.append(getattr(curve, hz_point))
                    w.append(getattr(curve, w_point))
                params['hz'] = hz
                params['w'] = w

        except Exception as e:
            raise der.DERError(str(e))

        return params


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
        rtn = ':MEAS:WT3000 200.222E+00'
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
        rtn = ':MEAS:WT3000 0.848E+00'
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
        rtn = ':MEAS:WT3000 +8.573E+03'
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
        rtn = ':MEAS:WT3000 -8.593E+03'
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
        rtn = ':MEAS:WT3000 -0.083E+03'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['S3_AC_Q_1'] = float(tmp[1])
            rtn_up['S3_AC_Q_2'] = float(tmp[1])
            rtn_up['S3_AC_Q_3'] = float(tmp[1])

        # Power factor(lambda)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 28')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 28\n')
###        self.ts.sleep(3)
        #test start
        rtn = ':MEAS:WT3000 0.950E+00'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['S3_AC_LAMB_1'] = float(tmp[1])
            rtn_up['S3_AC_LAMB_2'] = float(tmp[1])
            rtn_up['S3_AC_LAMB_3'] = float(tmp[1])

        # Frequency voltage(fu)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 30')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 30\n')
###        self.ts.sleep(3)
        #test start
        rtn = ':MEAS:WT3000 47.500'
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
        rtn = ':MEAS:WT3000 202.222E+00'
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
        rtn = ':MEAS:WT3000 0.851E+00'
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
        rtn = ':MEAS:WT3000 +0.576E+03'
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
        rtn = ':MEAS:WT3000 -0.593E+03'
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
        rtn = ':MEAS:WT3000 -0.083E+03'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['S4_AC_Q_1'] = float(tmp[1])
            rtn_up['S4_AC_Q_2'] = float(tmp[1])
            rtn_up['S4_AC_Q_3'] = float(tmp[1])

        # Power factor(lambda)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 29')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 29\n')
###        self.ts.sleep(3)
        #test start
        rtn = ':MEAS:WT3000 0.900E+00'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['S4_AC_LAMB_1'] = float(tmp[1])
            rtn_up['S4_AC_LAMB_2'] = float(tmp[1])
            rtn_up['S4_AC_LAMB_3'] = float(tmp[1])

        # Frequency voltage(fu)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 41')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 41\n')
###        self.ts.sleep(3)
        #test start
        rtn = ':MEAS:WT3000 47.500'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['S4_AC_FREQ_1'] = float(tmp[1])
            rtn_up['S4_AC_FREQ_2'] = float(tmp[1])
            rtn_up['S4_AC_FREQ_3'] = float(tmp[1])

        ##########
        # SIGMA
        ##########

        # Voltage(U)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 45')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        #rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 45\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 45\n')
###        self.ts.sleep(3)
###        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 45\n')
        #test start
        rtn = ':MEAS:WT3000 202.222E+00'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['SIGMA_AC_VRMS_1'] = float(tmp[1])
            rtn_up['SIGMA_AC_VRMS_2'] = float(tmp[1])
            rtn_up['SIGMA_AC_VRMS_3'] = float(tmp[1])

        # Current(I)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 46')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 46\n')
###        self.ts.sleep(3)
        #test start
        rtn = ':MEAS:WT3000 0.851E+00'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['SIGMA_AC_IRMS_1'] = float(tmp[1])
            rtn_up['SIGMA_AC_IRMS_2'] = float(tmp[1])
            rtn_up['SIGMA_AC_IRMS_3'] = float(tmp[1])

        # Active power(P)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 47')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 47\n')
###        self.ts.sleep(3)
        #test start
        rtn = ':MEAS:WT3000 +0.576E+03'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['SIGMA_AC_P_1'] = float(tmp[1])
            rtn_up['SIGMA_AC_P_2'] = float(tmp[1])
            rtn_up['SIGMA_AC_P_3'] = float(tmp[1])

        # Apparent power(S)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 48')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 48\n')
###        self.ts.sleep(3)
        #test start
        rtn = ':MEAS:WT3000 -0.593E+03'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['SIGMA_AC_S_1'] = float(tmp[1])
            rtn_up['SIGMA_AC_S_2'] = float(tmp[1])
            rtn_up['SIGMA_AC_S_3'] = float(tmp[1])

        # Reactive power(Q)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 49')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 49\n')
###        self.ts.sleep(3)
        #test start
        rtn = ':MEAS:WT3000 -0.083E+03'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['SIGMA_AC_Q_1'] = float(tmp[1])
            rtn_up['SIGMA_AC_Q_2'] = float(tmp[1])
            rtn_up['SIGMA_AC_Q_3'] = float(tmp[1])

        # Power factor(lambda)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 50')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 50\n')
###        self.ts.sleep(3)
        #test start
        rtn = ':MEAS:WT3000 0.900E+00'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['SIGMA_AC_LAMB_1'] = float(tmp[1])
            rtn_up['SIGMA_AC_LAMB_2'] = float(tmp[1])
            rtn_up['SIGMA_AC_LAMB_3'] = float(tmp[1])

        # Frequency voltage(fu)
        self.ts.log(':MEAS:WT3000 :NUM:NORM:VAL? 51')
        ###rtn = self.query(':MEAS:WT3000 :NUM:VAL?\n')
        rtn = self.query(':MEAS:WT3000 :NUM:NORM:VAL? 51\n')
###        self.ts.sleep(3)
        #test start
        rtn = ':MEAS:WT3000 47.500'
        #test end
        self.ts.log('rtn = %s' % (rtn))
        tmp = rtn.split(" ")
        ###if tmp[0] == ':MEAS:WT3000':
        if tmp[0] == ':MEAS:WT3000':
            rtn_up['SIGMA_AC_FREQ_1'] = float(tmp[1])
            rtn_up['SIGMA_AC_FREQ_2'] = float(tmp[1])
            rtn_up['SIGMA_AC_FREQ_3'] = float(tmp[1])


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

        # Power factor(lambda)
        rtn_up['AC_LAMB_1'] = (rtn_up['S3_AC_LAMB_1'] + rtn_up['S4_AC_LAMB_1'])/2
        rtn_up['AC_LAMB_2'] = (rtn_up['S3_AC_LAMB_2'] + rtn_up['S4_AC_LAMB_2'])/2
        rtn_up['AC_LAMB_3'] = (rtn_up['S3_AC_LAMB_3'] + rtn_up['S4_AC_LAMB_3'])/2

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
