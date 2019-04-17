# coding: shift-jis
####################################################################################################
# This script was tuned specifically for the AIST FREA environment (Fixed in 2019)
#     AIST:National Institute of Advanced Industrial Science and Technology 
#     FREA:Fukushima Renewable Energy Institute
#
# What is the AIST FREA environment
#   Communication with SunSpecSVP is middleware called ExCon, and ExCon is
#   a mechanism to communicate with inverters and simulators.
####################################################################################################
"""
Copyright (c) 2018, Sandia National Labs and SunSpec Alliance
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

import sys
import os
import traceback
from svpelab import gridsim
###from svpelab import loadsim     <- Commented out because middleware is communicated using gridsim
###from svpelab import pvsim       <- Commented out because middleware is communicated using gridsim
from svpelab import das
from svpelab import der
from svpelab import hil
import script
import result as rslt
import time
import datetime

import subprocess
from subprocess import PIPE
import re
import csv

import ctypes                      # WT3000 compatible
from ctypes import *               # WT3000 compatible
import threading                   # WT3000 compatible

### WT3000 compatible
### <START>
def wt3000_format_set(dll, device_id):

    ts.log('--------------WT3000 FORMAT SET command Start----------------')

    rtn_up = {}
    msg = ""

    #------------------------
    #SET Format
    #------------------------
    msg = ":NUMERIC:FORMAT ASCII"
    rtn = dll.TmSend(device_id, msg)
    if rtn == 0:
        ts.log('@@@(WT3000) SET Format OK: %s' % (rtn))
    else:
        ts.log('@@@(WT3000) SET Format NG: %s' % (rtn))

    #------------------------
    #SET Number of Items
    #------------------------
    msg = ":NUMERIC:NORMAL:NUMBER 2"
    rtn = dll.TmSend(device_id, msg)
    if rtn == 0:
        ts.log('@@@(WT3000) SET Number of Items OK: %s' % (rtn))
    else:
        ts.log('@@@(WT3000) SET Number of Items NG: %s' % (rtn))

    #------------------------
    #SET Items
    #------------------------
#    msg = ":NUMERIC:NORMAL:"
#    msg = msg + "ITEM1 P,SIGMA,Total;"
#    msg = msg + "ITEM2 Q,SIGMA,Total;"
#    msg = msg + "ITEM3 S,SIGMA,Total;"
#    msg = msg + "ITEM4 LAMBDA,SIGMA,Total;"
#    msg = msg + "ITEM5 I,SIGMA,Total;"

    msg = ":NUMERIC:NORMAL:"
    msg = msg + "ITEM1 I,SIGMA,Total;"
    msg = msg + "ITEM1 FU,SIGMA,Total;"
    rtn = dll.TmSend(device_id, msg)
    if rtn == 0:
        ts.log('@@@(WT3000) SET Items OK: %s' % (rtn))
    else:
        ts.log('@@@(WT3000) SET Items NG: %s' % (rtn))


    ts.log('--------------WT3000 FORMATSET command End-----------------')

    pass

def wt3000_data_read(dll, device_id):

    ts.log('--------------WT3000 command Start----------------')

    rtn_up = {}
    msg = ""

    #------------------------
    #SET VALUE?
    #------------------------
    msg = ":NUMERIC:NORMAL:VALUE?"
    rtn = dll.TmSend(device_id, msg)
    if rtn == 0:
        ts.log('@@@(WT3000) SET VALUE? OK: %s' % (rtn))
    else:
        ts.log('@@@(WT3000) SET VALUE? NG: %s' % (rtn))


    buf = ctypes.create_string_buffer(1024,1024)
    bufsize = ctypes.c_int(1024)
    length = ctypes.c_int()
    rtn = dll.TmReceive(device_id, buf, bufsize, ctypes.pointer(length))
    if rtn == 0:
        ts.log('@@@(WT3000) RECEIVE OK: %s' % (rtn))
    else:
        ts.log('@@@(WT3000) RECEIVE NG: %s' % (rtn))

    ts.log('buf.value: %s' % (buf.value))
    ts.log('length: %s' % (length))


    #------------------------
    #Return Data Set
    #------------------------
    tmp_data = buf.value.split(",")
    ts.log('@@@(WT3000) split: %s' % (tmp_data))

    # Current(I)
    if tmp_data[0] == "NAN" or tmp_data[0] == "INF":
        rtn_up['SIGMA_AC_IRMS_1'] = 0
        rtn_up['SIGMA_AC_IRMS_2'] = 0
        rtn_up['SIGMA_AC_IRMS_3'] = 0
    else:
        rtn_up['SIGMA_AC_IRMS_1'] = float(tmp_data[0])
        rtn_up['SIGMA_AC_IRMS_2'] = float(tmp_data[0])
        rtn_up['SIGMA_AC_IRMS_3'] = float(tmp_data[0])
    ts.log('@@@(WT3000) Current(I): %s' % (rtn_up['SIGMA_AC_IRMS_1']))

    # Frequency of Voltage(FU)
    if tmp_data[1] == "NAN" or tmp_data[1] == "INF":
        rtn_up['SIGMA_AC_FREQ_1'] = 0
        rtn_up['SIGMA_AC_FREQ_2'] = 0
        rtn_up['SIGMA_AC_FREQ_3'] = 0
    else:
        rtn_up['SIGMA_AC_FREQ_1'] = float(tmp_data[1])
        rtn_up['SIGMA_AC_FREQ_2'] = float(tmp_data[1])
        rtn_up['SIGMA_AC_FREQ_3'] = float(tmp_data[1])
    ts.log('@@@(WT3000) Frequency(FU): %s' % (rtn_up['SIGMA_AC_FREQ_1']))



    '''
    #------------------------
    #Local Test Code start
    #------------------------
    # Current(I)
    rtn_up['SIGMA_AC_IRMS_1'] = 205
    rtn_up['SIGMA_AC_IRMS_2'] = 205
    rtn_up['SIGMA_AC_IRMS_3'] = 205

    # Frequency of Voltage(FU)
    rtn_up['SIGMA_AC_FREQ_1'] = 50.1
    rtn_up['SIGMA_AC_FREQ_2'] = 50.1
    rtn_up['SIGMA_AC_FREQ_3'] = 50.1
    '''

    ts.log('--------------WT3000 command End-----------------')

    return rtn_up

### <END>


### Add for Thread control
### <START>
def MeasureThread(e ,MeasurMachine ,writer ,m_time ,dll ,device_id):

    ts.log('--------------MeasureThread Start----------------')

    sv_time = time.time()

    MeasureData = wt3000_format_set(dll, device_id)
    ts.log('@@@wt3000_format_set()')

    MeasureData = wt3000_data_read(dll, device_id)
    ts.log('@@@wt3000_data_read()')

    MeasureTime = time.time() - sv_time

    #------------------------
    #  Frequency
    #------------------------
    grf_rec_frequency = [MeasureTime, MeasureData.get('SIGMA_AC_FREQ_1')]
    writer.writerow(grf_rec_frequency)
    ts.log('grf_rec_frequency: %s ' % (grf_rec_frequency))

#    for i in range(99999):
    while not e.stop_event.is_set():
        time.sleep(m_time)
#        ts.sleep(m_time)
        #event_is_set = e.wait()
        #ts.log('@@@ Starting MeasureThread cnt = %s' % (i))

        MeasureData = wt3000_data_read(dll, device_id)
        ts.log('@@@wt3000_data_read()')

        MeasureTime = time.time() - sv_time

        #------------------------
        #  Frequency
        #------------------------
        grf_rec_frequency = [MeasureTime, MeasureData.get('SIGMA_AC_FREQ_1')]
        writer.writerow(grf_rec_frequency)
        ts.log('grf_rec_frequency: %s ' % (grf_rec_frequency))

    ts.log('--------------MeasureThread End------------------')
    pass
### <END>



def freq_rt_profile(v_nom=100.0, freq_nom=100.0, freq_t=100.0, t_fall=0, t_hold=1, t_rise=0, t_dwell=5, n=5):
    """
    :param: v_nom - starting voltage value
    :param: freq_nom - starting frequency value
    :param: freq_t - test frequency value
    :param: t_fall - fall time
    :param: t_hold is hold time (s)
    :param: t_rise - rise time
    :param: t_dwell - dwell time
    :param: n - number of iterations

    :returns: profile - grid sim profile, in format of grid_profiles
    """
    profile = []
    t = 0
    profile.append((t, v_nom, v_nom, v_nom, freq_nom))       # (time offset, starting voltage (%), freq (100%))
    for i in range(1, n+1):
        t += t_dwell                                         # hold for dwell time
        profile.append((t, v_nom, v_nom, v_nom, freq_nom))   # (time offset, starting voltage (%), freq (100%))
        t += t_fall                                          # ramp over fall time
        profile.append((t, v_nom, v_nom, v_nom, freq_t))     # (time offset, test voltage (%), freq (100%))
        t += t_hold                                          # hold for hold time
        profile.append((t, v_nom, v_nom, v_nom, freq_t))     # (time offset, test voltage (%), freq (100%))
        t += t_rise                                          # ramp over rise time
        profile.append((t, v_nom, v_nom, v_nom, freq_nom))   # (time offset, starting voltage (%), freq (100%))
    t += t_dwell                                             # hold for dwell time
    profile.append((t, v_nom, v_nom, v_nom, freq_nom))       # (time offset, starting voltage (%), freq (100%))

    return profile

def test_run():

    result = script.RESULT_FAIL
    eut = grid = load = pv = daq_rms = daq_wf = chil = None


    dll = None                       # WT3000 compatible
    tcp_control = ctypes.c_int(4)    # WT3000 compatible
    device_id = ctypes.c_int()       # WT3000 compatible
    e = None                         # WT3000 compatible

    grf_dat = None

    grf_dat_file = ""

### Correction as graph is not displayed
### <START>
###    sc_points = ['AC_IRMS_MIN']
    sc_points = ['TIME', 'AC_FREQ_1', 'AC_IRMS_1', 'AC_IRMS_MIN']
### <END>

    # result params
    result_params = {
        'plot.title': ts.name,
        'plot.x.title': 'Time (secs)',
        'plot.x.points': 'TIME',
        'plot.y.points': 'AC_FREQ_1',
        'plot.y.title': 'Frequency (Hz)',
        'plot.y2.points': 'AC_IRMS_1, AC_IRMS_MIN',
        'plot.y2.title': 'Current (A)'
    }

    writer = None

    try:
        test_label = ts.param_value('frt.test_label')
        # get test parameters
        freq_msa = ts.param_value('eut.freq_msa')
        s_rated = ts.param_value('eut.s_rated')
        p_rated = ts.param_value('eut.p_rated')
        p_ramp_rate = ts.param_value('eut.ramp_rate')
        v_nom = ts.param_value('eut.v_nom')
        t_msa = ts.param_value('eut.t_msa')
        t_dwell = ts.param_value('eut.frt_t_dwell')
        freq_nom = ts.param_value('eut.freq_nom')
        freq_grid_min = ts.param_value('frt.freq_grid_min')
        freq_grid_max = ts.param_value('frt.freq_grid_max')
        freq_test = ts.param_value('frt.freq_test')
        t_hold = ts.param_value('frt.t_hold')
        n_r = ts.param_value('frt.n_r')

        ip_addr = ts.param_value('frt.ip_addr')                 # WT3000 compatible
        m_time = ts.param_value('frt.m_time')                   # Add for Thread control

        # calculate voltage adjustment based on msa
        freq_msa_adj = freq_msa * 1.5
        if freq_test > freq_nom:
            # apply HFRT msa adjustments
            freq_n = freq_grid_min + freq_msa_adj
            freq_t = freq_test - freq_msa_adj
        else:
            # apply LFRT msa adjustments
            freq_n = freq_grid_max - freq_msa_adj
            freq_t = freq_test + freq_msa_adj

        # set power levels that are enabled
        power_levels = []
        if ts.param_value('frt.p_100') == 'Enabled':
            power_levels.append((100, '100'))
        if ts.param_value('frt.p_20') == 'Enabled':
            power_levels.append((20, '20'))



### WT3000 compatible
### <START>
        dll = ctypes.WinDLL(r"C:\\Python27\\DLLs\\tmctl")
        ts.log('@@@(WT3000) DLL OK')

        tcp_address = ip_addr + ",anonymous,"
        rtn = dll.TmcInitialize(tcp_control, tcp_address, ctypes.pointer(device_id))
        if rtn == 0:
            ts.log('@@@(WT3000) CONNECT OK: %s' % (rtn))
        else:
            ts.log('@@@(WT3000) CONNECT NG: %s' % (tcp_address))
### <END>



        # initialize HIL environment, if necessary
###        chil = hil.hil_init(ts)
###        if chil is not None:
###            chil.config()

        # grid simulator is initialized with test parameters and enabled
        grid = gridsim.gridsim_init(ts)
        profile_supported = False

        # In cases where the grid simulator has voltage rise/loss on the line to the EUT or operates through a
        # transformer, the nominal voltage of the grid simulator won't be the same as the EUT and a correction
        # factor is applied.
        try:
            v_nom_grid = grid.v_nom_param
        except Exception, e:
            v_nom_grid = v_nom


###        grid.voltage((v_nom_grid, v_nom_grid, v_nom_grid))      <- Commented out because middleware is communicated using gridsim
###        grid.voltageRH(v_nom_grid, v_nom_grid, v_nom_grid)         # <- Change to control from grid
        grid.voltageV(v_nom_grid, v_nom_grid, v_nom_grid)          # <- Change to control from grid

        # load simulator initialization
###        load = loadsim.loadsim_init(ts)                         <- Commented out because middleware is communicated using gridsim
###        if load is not None:                                    <- Commented out because middleware is communicated using gridsim
###            ts.log('Load device: %s' % load.info())             <- Commented out because middleware is communicated using gridsim

### Commented out because middleware is communicated using gridsim
### <START>
###        # pv simulator is initialized with test parameters and enabled
###        pv = pvsim.pvsim_init(ts)
###        pv.power_set(p_rated)
###        pv.power_on()
### <END>

        # initialize rms data acquisition
        daq_rms = das.das_init(ts, 'das_rms', sc_points=sc_points)
        if daq_rms is not None:
            ts.log('DAS RMS device: %s' % (daq_rms.info()))
            daq_rms.sc['SC_TRIG'] = 0
            daq_rms.sc['AC_IRMS_MIN'] = ''


        # initialize waveform data acquisition
        daq_wf = das.das_init(ts, 'das_wf')
        if daq_wf is not None:
            ts.log('DAS Waveform device: %s' % (daq_wf.info()))

        # it is assumed the EUT is on
        eut = der.der_init(ts)
        if eut is not None:
            eut.config()

### Graph drawing for FREA original gnuplot
### <START>
        e = threading.Event()
        e.stop_event = threading.Event()

        grf_dat_file = ts.results_dir() + "\SA10_freq_ride_through.csv"
        grf_dat_file = re.sub(r'\\', "/", grf_dat_file)
        ts.log('grf_dat_file = %s' % (grf_dat_file))
        grf_dat = open(grf_dat_file, mode='w')
        writer = csv.writer(grf_dat, lineterminator='\n')

        thread = threading.Thread(target=MeasureThread, args=(e ,grid ,writer ,m_time ,dll, device_id,))
        thread.start()
#        time.sleep(1)
        ts.sleep(1)

        sv_time = time.time()

### <END>

        # perform all power levels
        for power_level in power_levels:

            if daq_wf is not None:             # DL850E compatible
                daq_wf.data_capture(True)      # DL850E compatible

            # set test power level
            power = float(power_level[0])/100 * p_rated
###            pv.power_set(power)                                                   <- Commented out because middleware is communicated using gridsim
###            grid.power_set(power)                                                    # <- Change to control from grid
            grid.power_setVV(power, s_rated, p_ramp_rate)   # <- Change to control from grid
            ts.log('Setting power level to %s%% of rated' % (power_level[0]))

            if daq_rms is not None:
###                daq_rms.sc['AC_IRMS_MIN'] = ''
###                data = grid.wt3000_data_capture_read()
                e.clear()                                              # Add for Thread control
                data = wt3000_data_read(dll, device_id)                # <- Since the graph is not displayed, it is added
                e.set()                                                # Add for Thread control

###                daq_rms.sc['TIME'] = time.time()                       # <- Since the graph is not displayed, it is added
                daq_rms.sc['TIME'] = round(time.time(),3) - sv_time    # <- Since the graph is not displayed, it is added
                daq_rms.sc['AC_FREQ_1'] = data.get('SIGMA_AC_FREQ_1')  # <- Since the graph is not displayed, it is added
                daq_rms.sc['AC_IRMS_1'] = data.get('SIGMA_AC_IRMS_1')  # <- Since the graph is not displayed, it is added
                irms = data.get('SIGMA_AC_IRMS_1')                     # <- Since the graph is not displayed, it is added
                daq_rms.sc['AC_IRMS_MIN'] = round(irms * .8, 2)        # <- Since the graph is not displayed, it is added
                ts.log('Starting RMS data capture')
                daq_rms.data_capture(True)
                ts.log('Waiting 5 seconds to start test')
###                ts.sleep(5)
                time.sleep(5)                                          # DL850E compatible

            if profile_supported:
                # create and execute test profile
                profile = freq_rt_profile(v_nom=v_nom, freq_nom=freq_n/freq_nom, freq_t=freq_t/freq_nom, t_hold=t_hold,
                                          t_dwell=t_dwell, n=n_r)
                grid.profile_load(profile=profile)
                grid.profile_start()
                # create countdown timer
                start_time = time.time()
                profile_time = profile[-1][0]
                ts.log('Profile duration is %s seconds' % profile_time)
                while (time.time() - start_time) < profile_time:
                    remaining_time = profile_time - (time.time()-start_time)
                    ts.log('Sleeping for another %0.1f seconds' % remaining_time)
                    sleep_time = min(remaining_time, 10)
###                    ts.sleep(sleep_time)
                    time.sleep(sleep_time)                             # DL850E compatible
                grid.profile_stop()
            else:
                # execute test sequence
                ts.log('Test duration is %s seconds' % ((float(t_dwell) + float(t_hold)) * float(n_r) +
                                                            float(t_dwell)))

                # get initial current level to determine threshold
                if daq_rms is not None:
                    daq_rms.data_sample()
###                    data = daq_rms.data_capture_read()                  <- Commented out because middleware is communicated using gridsim
                    data = grid.wt3000_data_capture_read()                 # <- Change to control from grid
###                    daq_rms.sc['TIME'] = time.time()                       # <- Since the graph is not displayed, it is added
                    daq_rms.sc['TIME'] = round(time.time(),3) - sv_time    # <- Since the graph is not displayed, it is added
                    daq_rms.sc['AC_FREQ_1'] = data.get('SIGMA_AC_FREQ_1')  # <- Since the graph is not displayed, it is added
                    daq_rms.sc['AC_IRMS_1'] = data.get('SIGMA_AC_IRMS_1')  # <- Since the graph is not displayed, it is added
                    irms = data.get('SIGMA_AC_IRMS_1')
                    if irms is not None:
                        daq_rms.sc['AC_IRMS_MIN'] = round(irms * .8, 2)

                for i in range(n_r):
                    grid.freq(freq=freq_n)
                    ts.log('Setting frequency: freq = %s for %s seconds' % (freq_n, t_dwell))
###                    ts.sleep(t_dwell)
                    time.sleep(t_dwell)     # DL850E compatible
                    grid.freq(freq=freq_t)
                    ts.log('Setting frequency: freq = %s for %s seconds' % (freq_t, t_hold))
###                    ts.sleep(t_hold)
                    time.sleep(t_hold)     # DL850E compatible
                grid.freq(freq=freq_n)
                ts.log('Setting frequency: freq = %s for %s seconds' % (freq_n, t_dwell))
###                ts.sleep(t_dwell)
                time.sleep(t_dwell)     # DL850E compatible
            if daq_rms is not None:
                daq_rms.data_capture(False)
                ds = daq_rms.data_capture_dataset()
                test_name = '%s_rms_%s' % (test_label, power_level[1])
                filename = '%s.csv' % (test_name)
                ds.to_csv(ts.result_file_path(filename))
                result_params['plot.title'] = test_name
                ts.result_file(filename, params=result_params)
                ts.log('Saving data capture %s' % (filename))

            if daq_wf is not None:             # DL850E compatible
                daq_wf.data_capture(False)     # DL850E compatible
                time.sleep(3)                  # DL850E compatible
                daq_wf.data_save()             # DL850E compatible
                time.sleep(5)                  # DL850E compatible

### Graph drawing for FREA original gnuplot
### <START>
        e.stop_event.set()               # WT3000 compatible
        thread.join()                    # WT3000 compatible

        grf_dat.close()

        rtn = dll.TmFinish(device_id)                                  # WT3000 compatible
        if rtn == 0:                                                   # WT3000 compatible
            ts.log('@@@(WT3000) DISCONNECT OK: %s' % (rtn))            # WT3000 compatible
        else:                                                          # WT3000 compatible
            ts.log('@@@(WT3000) DISCONNECT NG: %s' % (rtn))            # WT3000 compatible
### <END>

        result = script.RESULT_COMPLETE

    except script.ScriptFail, e:
        reason = str(e)
        if reason:
            ts.log_error(reason)
    finally:
        ts.log('--------------Finally START----------------')

### Commented out because middleware is communicated using gridsim
### <START>
###        # reset to nominal frequency and full power
###        grid.freq(freq=freq_nom)
###        pv.power_set(p_rated)
### <END>

        if eut is not None:
            eut.close()
        if grid is not None:
            grid.freq(freq=freq_nom)                                           # <- Change to control from grid
            grid.close()
### Commented out because middleware is communicated using gridsim
### <START>
###        if load is not None:
###            load.close()
###        if pv is not None:
###            pv.close()
### <END>
        if daq_rms is not None:
            daq_rms.close()
        if daq_wf is not None:
            daq_wf.data_capture(False)     # DL850E compatible
            time.sleep(3)                  # DL850E compatible
            daq_wf.close()
###        if chil is not None:
###            chil.close()
        if grf_dat is not None:
            grf_dat.close()

        # create result workbook
        file = ts.config_name() + '.xlsx'
        rslt.result_workbook(file, ts.results_dir(), ts.result_dir())
        ts.result_file(file)





### Graph drawing for FREA original gnuplot
### <START>

        gnuplot =  subprocess.Popen('gnuplot', shell=True, stdin=PIPE, stdout=PIPE, stderr=PIPE, universal_newlines=True)

        ### SA10_freq_ride_through.png
        graph_out = ts.results_dir() + "\SA10_freq_ride_through.png"
        ts.log('graph_out = %s' % (graph_out))
        graph_cmd = "set output " + "'" + graph_out + "'\n"
        ts.log('graph_cmd = %s' % (graph_cmd))
        graph_cmd = "set output " + "'" + graph_out + "'\n"

        gnuplot.stdin.write('set ylabel "Frequency (Hz)"\n')
###        gnuplot.stdin.write('set xdata time"\n')
        gnuplot.stdin.write('set xlabel "Time (seconds)"\n')
###        gnuplot.stdin.write('set timefmt "%Y/%m/%d %H:%M:%S"\n')
###        gnuplot.stdin.write('set format x "%M:%S"\n')
        gnuplot.stdin.write('set term png size 1000, 1000\n')
        gnuplot.stdin.write('set grid lw 1\n')

        set_cmd = "set autoscale y \n"
        gnuplot.stdin.write(set_cmd)
        set_cmd = "set autoscale x \n"
        gnuplot.stdin.write(set_cmd)

###        gnuplot.stdin.write('set key box\n')
        gnuplot.stdin.write(graph_cmd)
        graph_cmd = "set datafile separator ','\n"
        gnuplot.stdin.write(graph_cmd)
###        graph_cmd = "plot " + "'" + grf_dat_file + "'" + " using 1:2 with lines ti 'FRT Line', " + "'" + grf_dat_file + "' using 1:2 ti 'FRT Point' pt 7\n"
###        graph_cmd = "plot " + "'" + grf_dat_file + "'" + " ti 'Ideal Point' with points pt 7 lc rgb 'blue', " + "'" + grf_dat_file + "' ti 'Measurement Point' with points pt 7 lc rgb 'magenta'\n"
###        graph_cmd = "plot " + "'" + grf_dat_file + "' ti 'FRT Point' with points pt 7 lc rgb 'magenta'\n"
        graph_cmd = "plot " + "'" + grf_dat_file + "' using 1:2 ti 'FRT Point' with linespoints pt 7 lc rgb 'magenta'\n"
        ts.log('graph_cmd = %s' % (graph_cmd))
        gnuplot.stdin.write(graph_cmd)

        ### Return setting
        gnuplot.stdin.write('set terminal windows\n')
        gnuplot.stdin.write('set output\n')
### <END>






    return result

def run(test_script):

    try:
        global ts
        ts = test_script
        rc = 0
        result = script.RESULT_COMPLETE

        ts.log_debug('')
        ts.log_debug('**************  Starting %s  **************' % (ts.config_name()))
        ts.log_debug('Script: %s %s' % (ts.name, ts.info.version))
        ts.log_active_params()

        ts.svp_version(required='1.5.9')

        result = test_run()

        ts.result(result)
        if result == script.RESULT_FAIL:
            rc = 1

    except Exception, e:
        ts.log_error('Test script exception: %s' % traceback.format_exc())
        rc = 1

    sys.exit(rc)

info = script.ScriptInfo(name=os.path.basename(__file__), run=run, version='1.0.0')

### Add for version control
### <START>
info.param_group('aist', label='AIST Parameters', glob=True)
info.param('aist.script_version', label='Script Version', default='5.0.b')
info.param('aist.library1_version', label='Library Version (gridsim_frea_simulator)', default='4.5.0')
info.param('aist.library2_version', label='Library Version (das_dl850e/device_dl850e)', default='1.0.0')
### <END>

'''
    eut
        p_rated
        v_nom
        freq_nom
        freq_msa (%)
        t_msa
        frt_t_dwell
    frt
        freq_test
        t_hold
        freq_grid_min
        freq_grid_max
        Power Level
            100%
            20%
'''

info.param_group('frt', label='Test Parameters')
info.param('frt.test_label', label='Test Label', default='frt')
info.param('frt.freq_test', label='Ride-Through Frequency (Hz)', default=60.0)
info.param('frt.t_hold', label='Ride-Through Duration (secs)', default=10.0)
info.param('frt.freq_grid_min', label='Minimum grid frequency (Hz)', default=60.0)
info.param('frt.freq_grid_max', label='Maximum grid frequency (Hz)', default=60.0)
info.param('frt.p_100', label='Power Level 100% Tests', default='Enabled', values=['Disabled', 'Enabled'])
info.param('frt.p_20', label='Power Level 20% Tests', default='Enabled', values=['Disabled', 'Enabled'])
info.param('frt.n_r', label='Number of test repetitions', default=5)

info.param('frt.ip_addr', label='WT3000 IP Address for NonExCon', default='192.168.127.200')
info.param('frt.m_time', label='Measurement time interval (secs)', default=0.5)

info.param_group('eut', label='EUT Parameters', glob=True)
info.param('eut.s_rated', label='Apparent power rating (VA)', default=0.0)
###info.param('eut.p_rated', label='P_rated', default=3000)
info.param('eut.p_rated', label='Output power rating (W)', default=0.0)
info.param('eut.v_nom', label='V_nom', default=240)
info.param('eut.freq_nom', label='Freq_nom', default=60.0)
info.param('eut.freq_msa', label='Freq_msa', default=2.0)
info.param('eut.t_msa', label='T_msa', default=1.0)
info.param('eut.frt_t_dwell', label='FRT T_dwell', default=5)
info.param('eut.ramp_rate', label='Power Ramp Rate (0.01%/s)', default=0)

der.params(info)
das.params(info, 'das_rms', 'Data Acquisition (RMS)')
das.params(info, 'das_wf', 'Data Acquisition (Waveform)')
gridsim.params(info)
###loadsim.params(info)                                     <- Commented out because middleware is communicated using gridsim
###pvsim.params(info)                                       <- Commented out because middleware is communicated using gridsim
###hil.params(info)

def script_info():
    
    return info


if __name__ == "__main__":

    # stand alone invocation
    config_file = None
    if len(sys.argv) > 1:
        config_file = sys.argv[1]

    params = None

    test_script = script.Script(info=script_info(), config_file=config_file, params=params)
    test_script.log('log it')

    run(test_script)


