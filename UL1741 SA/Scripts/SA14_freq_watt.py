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
# from svpelab import pvsim
from svpelab import das
# from svpelab import der
# from svpelab import hil
import script
import openpyxl
import numpy as np
import result as rslt

import time    # <- add

import subprocess
from subprocess import PIPE
import re
import csv


def fw_params_interp(f, f_nom, hz_start, hz_stop):
    """
    Interpolation function to find the target power (using the FW parameter definition)

    :param value: freq point for the interpolation
    :param f: FW freq points
    :param p: FW power points
    :return: target power
    """
    f_pct = 100.*(f/f_nom)
    hz_start_pct = 100.*(hz_start/f_nom)
    hz_stop_pct = 100.*(hz_stop/f_nom)
    if f_pct < hz_start_pct:
        p_targ = 100.
    elif f_pct > hz_stop_pct:
        p_targ = 0.
    else:
        p_targ = 100. - 100.*((f_pct-hz_start_pct)/(hz_stop_pct-hz_start_pct))
    return float(p_targ)


def fw_point_interp(value, f, p):
    """
    Interpolation function to find the target power (using the FW pointwise definition)

    :param value: freq point for the interpolation
    :param f: FW freq points
    :param p: FW power points
    :return: target power
    """
    if value <= f[0]:  # if freq is less than f[0]
        return float(p[0])
    elif value >= f[-1]:  # if freq is greater than f[end]
        return float(p[-1])
    else:
        for i in range(len(f[1:-1])):
            # if freq_point[i+1] <= value <= freq_point[i+2]:   # <- coding miss?
            if f[i+1] <= value <= f[i+2]:
                p_value = p[i+1] - ((p[i+1] - p[i+2])/(f[i+2] - f[i+1]) * (value - f[i+1]))
                return float(p_value)
            else:
                ts.log_warning('Unable to Interpolate FW function. f=%s, p=%s, value=%s' % (f, p, value))
                ts.log_warning('Returning nominal power...')
                return 100.


def p_msa_range(f_value, f_msa, p_msa, f_nom, fw_mode, f=None, p=None, f_slope_start=None, f_slope_stop=None):
    """
    Determine power target and the min/max p values for pass/fail acceptance based on manufacturer's specified
    accuracies (MSAs).

    :param f_value: measured freq value
    :param f_msa: manufacturer's specified accuracy of freq
    :param p_msa: manufacturer's specified accuracy of power
    :param f_nom: EUT nominal freq
    :param fw_mode: FW F1
    :param f: FW freq points (list)
    :param p: FW power points (list)
    :param f_slope_start: FW F1
    :param f_slope_stop: FW F2
    :return: points for p_target, p_target_min, p_target_max
    """
    if fw_mode == 'Pointwise':
        p_targ = fw_point_interp(f_value, f, p)      # target power for the voltage measurement
        p1 = fw_point_interp(f_value - f_msa, f, p)  # power target from the lower voltage limit
        p2 = fw_point_interp(f_value + f_msa, f, p)  # power target from the upper voltage limit
    else:  # Parameters
        p_targ = fw_params_interp(f_value, f_nom, f_slope_start, f_slope_stop)      # target power for freq
        p1 = fw_params_interp(f_value - f_msa, f_nom, f_slope_start, f_slope_stop)  # power target from the lower freq
        p2 = fw_params_interp(f_value + f_msa, f_nom, f_slope_start, f_slope_stop)  # power target from the upper freq
    if p1 >= p_targ:
        # if the FW curve has a negative slope
        # add the power MSA to the high side (left point, p1)
        # subtract the power MSA from the low side (right point, p2)
        #
        #                          \ * (f_value - f_msa, p_upper)
        #                           \
        #                            . (f_value - f_msa, p1)
        #                             \
        #                              x (f_value, p_target)
        #                               \
        #                                . (f_value + f_msa, p2)
        #                                 \
        #     (f_value + f_msa, p_lower) * \

        p_upper = round(p1 + p_msa, 1)
        p_lower = round(p2 - p_msa, 1)
        return p_targ, p_lower, p_upper
    else:
        p_lower = round(p1 - p_msa, 1)
        p_upper = round(p2 + p_msa, 1)
        return p_targ, p_lower, p_upper


def test_run():

    result = script.RESULT_FAIL
    daq = None
    daq_wf = None
    grid = None
    # pv = None
    # eut = None
    # chil = None
    # rs = None

    filename_up = ""
    filename_down = ""

    result_params = {
        'plot.title': 'title_name',
        'plot.x.title': 'Time (secs)',
        'plot.x.points': 'TIME',
        'plot.y.points': 'AC_P_1, AC_FREQ_1, P_TARGET',
        'plot.y.title': 'Power (W), Freq (Hz)',
        'plot.P_TARGET.point': 'True',
        'plot.P_TARGET.min_error': 'P_TARGET_MIN',
        'plot.P_TARGET.max_error': 'P_TARGET_MAX',
        'plot.P_MIN.point': 'True',
        'plot.P_MAX.point': 'True',
        'plot.F_ACT.point': 'True',
    }

    try:
        # initialize hardware-in-the-loop environment (if applicable)
        # ts.log('Configuring HIL system...')
        # chil = hil.hil_init(ts)
        # if chil is not None:
        #     chil.config()

        # initialize grid simulator
        grid = gridsim.gridsim_init(ts)

        # initialize pv simulator
        # pv = pvsim.pvsim_init(ts)
        p_rated = ts.param_value('fw.p_rated')
        # pv.power_set(p_rated)
        # pv.power_on()  # power on at p_rated

        # DAS soft channels
###        das_points = {'sc': ('P_TARGET', 'P_TARGET_MIN', 'P_TARGET_MAX', 'eval_flag', 'freq_set')}
        das_points = {'sc': ('TIME', 'P_TARGET', 'P_TARGET_MIN', 'P_TARGET_MAX', 'eval_flag', 'freq_set')}    # <- add 'Time'

        # initialize data acquisition system
        daq = das.das_init(ts, sc_points=das_points['sc'])
        daq.sc['TIME'] = 0
        daq.sc['P_TARGET'] = 100
        daq.sc['P_TARGET_MIN'] = 100
        daq.sc['P_TARGET_MAX'] = 100
        daq.sc['eval_flag'] = 0

### Graph drawing for FREA original gnuplot
### <START>
        ### SA15_ramp_rate.png
        gnuplot =  subprocess.Popen('gnuplot', shell=True, stdin=PIPE, stdout=PIPE, stderr=PIPE, universal_newlines=True)
### <END>

### DL850E compatible
### <START>
        # initialize waveform data acquisition
        daq_wf = das.das_init(ts, 'das_wf')
        if daq_wf is not None:
            ts.log('DAS Waveform device: %s' % (daq_wf.info()))
### <END>

        # Configure the EUT communications
        # eut = der.der_init(ts)
        # eut.config()
        # ts.log_debug(eut.measurements())
        # ts.log_debug('Set L/HFRT and trip parameters to the widest range of adjustability possible.')

        fw_mode = ts.param_value('fw.fw_mode')
        f_nom = ts.param_value('fw.f_nom')
        f_min = ts.param_value('fw.f_min')
        f_max = ts.param_value('fw.f_max')
        MSAHz = ts.param_value('fw.MSAHz')
        MSAP = ts.param_value('fw.MSAP')
        t_settling = ts.param_value('fw.ts')
        fstart_min = ts.param_value('fw.fstart_min')
        fstart_max = ts.param_value('fw.fstart_max')
        k_pf_min = ts.param_value('fw.k_pf_min')
        k_pf_max = ts.param_value('fw.k_pf_max')
        k_pf = ts.param_value('fw.kpf')  # %Prated/Hz
        phases = ts.param_value('fw.phases')
        s_rated = ts.param_value('fw.s_rated')

        n_points = ts.param_value('test.n_points')
        irr = ts.param_value('test.irr')
        n_iterations = ts.param_value('test.n_iter')
        curves = ts.param_value('test.curves')

        if curves == 'Both':
            fw_curves = [1, 2]
        elif curves == 'Characteristic Curve 1':
            fw_curves = [1]
        else:  # Characteristic Curve 2
            fw_curves = [2]

        if irr == 'All':
            pv_powers = [1., 0.66, 0.33]
        elif irr == '100%':
            pv_powers = [1.]
        elif irr == '66%':
            pv_powers = [0.66]
        else:
            pv_powers = [0.33]

        # open result summary file
        result_summary_filename = 'result_summary.csv'
        result_summary = open(ts.result_file_path(result_summary_filename), 'a+')
        ts.result_file(result_summary_filename)
        result_summary.write('Result, Test Name, Power Level, Iteration, direction, '
                             'Freq, Power, P_min, P_max, Dataset File\n')

### Graph drawing for FREA original gnuplot
### <START>
#        grf_dat_file = ts.results_dir() + "\SA14_freq_watt.csv"
#        grf_dat_file = re.sub(r'\\', "/", grf_dat_file)
#        ts.log('grf_dat_file = %s' % (grf_dat_file))
#        grf_dat = open(grf_dat_file, mode='w')
#        writer = csv.writer(grf_dat, lineterminator='\n')
### <END>

        for fw_curve in fw_curves:
            # if fw_curve == 1:  # characteristic curve 1
            #     hz_stop = fstart_max + 100./k_pf_max
            #     hz_start = fstart_min
            # else:  # characteristic curve 2
            #     hz_stop = fstart_min + 100./k_pf_min
            #     hz_start = fstart_max

            if fw_curve == 1:  # characteristic curve 1
                ts.log('-------------- characteristic curve 1 ----------------')
###                grid.fw_sanrex(k_pf_max, fstart_max, f_nom)
                grid.fw_sanrex(k_pf_max, fstart_min, f_nom)
###                ts.sleep(5)
                time.sleep(5)                  # DL850E compatible
###                hz_stop = fstart_max + 100./k_pf_max
                hz_stop = fstart_min + 100./k_pf_max
                hz_start = fstart_min

            else:  # characteristic curve 2
                ts.log('-------------- characteristic curve 2 ----------------')
###                grid.fw_sanrex(k_pf_min, fstart_min, f_nom)
                grid.fw_sanrex(k_pf_min, fstart_max, f_nom)
###                ts.sleep(5)
                time.sleep(5)                  # DL850E compatible
###                hz_stop = fstart_min + 100./k_pf_min
                hz_stop = fstart_max + 100./k_pf_min
                hz_start = fstart_max

            f_points = [fstart_min, hz_stop]
            p_points = [100, 0]

            if fw_mode == 'Parameters':
                # eut.freq_watt_param(params={'HysEna': False, 'HzStr': hz_start,
                                            # 'HzStop': hz_stop, 'WGra': k_pf_min})
                grid.freq_watt_param(params={'HysEna': False, 'HzStr': hz_start,
                                            'HzStop': hz_stop, 'WGra': k_pf_min})

            else:  # Pointwise
                # eut.freq_watt(params={'ActCrv': 1})
                # f_points = [fstart_min, fstart_min, hz_stop, hz_stop]
                # p_points = [100, 100, 0, 0]
                # parameters = {'hz': f_points, 'w': p_points}
                # ts.log_debug(parameters)
                # eut.freq_watt_curve(id=1, params=parameters)
                # eut.freq_watt(params={'Ena': True})
                # ts.log_debug(eut.freq_watt())

                grid.freq_watt(params={'ActCrv': 1})
                f_points = [fstart_min, fstart_min, hz_stop, hz_stop]
                p_points = [100, 100, 0, 0]
                parameters = {'hz': f_points, 'w': p_points}
                ts.log_debug(parameters)
                grid.freq_watt_curve(id=1, params=parameters)
                grid.freq_watt(params={'Ena': True})

            # start and stop frequencies for the grid simulator steps
            f_start = fstart_min
            f_end = f_max - MSAHz

            for power in pv_powers:
                # pv.power_set(p_rated*power)
                grid.power_set(p_rated*power, s_rated)

                for n_iter in range(n_iterations):

### Graph drawing for FREA original gnuplot
### <START>
                    grf_str = '\SA14_cv=%s_pw=%0.1f_i=%s' % (fw_curve, power, n_iter+1)

                    grf_dat_file = ts.results_dir() + grf_str + ".csv"
                    grf_dat_file = re.sub(r'\\', "/", grf_dat_file)
                    ts.log('grf_dat_file = %s' % (grf_dat_file))
                    grf_dat = open(grf_dat_file, mode='w')
                    writer = csv.writer(grf_dat, lineterminator='\n')

                    grf_dat_file_trg = ts.results_dir() + grf_str + "_trg.csv"
                    grf_dat_file_trg = re.sub(r'\\', "/", grf_dat_file_trg)
                    ts.log('grf_dat_file_trg = %s' % (grf_dat_file_trg))
                    grf_dat_trg = open(grf_dat_file_trg, mode='w')
                    writer_trg = csv.writer(grf_dat_trg, lineterminator='\n')
### <END>

                    # SA14.3.2(d) and (e)
                    daq.data_capture(True)
                    if daq_wf is not None:                      # DL850E compatible
                        daq_wf.data_capture(True)               # DL850E compatible
                    f_steps = list(np.linspace(f_start, f_end, n_points)) + \
                              list(np.linspace(f_end, f_start, n_points)) + [f_nom-1.]

###                    test_str = 'FW_curve_%s_pw=%0.2f_iter=%s' % (fw_curve, power, n_iter+1)
                    test_str = 'FW_cv_%s_pw=%0.1f_i=%s' % (fw_curve, power, n_iter+1)
                    filename = test_str + '.csv'
                    step_count = 0

                    for f_step in f_steps:
                        step_count += 1
                        grid.freq(f_step)
                        daq.sc['TIME'] = time.time()            #<- add
                        daq.sc['freq_set'] = f_step
                        ts.log('        Recording power at frequency %0.3f Hz for 2*t_settling = %0.1f sec.' %
                               (f_step, 2*t_settling))
                        p_targ, p_min, p_max = p_msa_range(f_value=f_step, f_msa=MSAHz, p_msa=MSAP, f_nom=f_nom,
                                                           fw_mode=fw_mode, f=f_points, p=p_points,
                                                           f_slope_start=hz_start, f_slope_stop=hz_stop)
                        daq.sc['P_TARGET'] = p_targ
                        daq.sc['P_TARGET_MIN'] = p_min
                        daq.sc['P_TARGET_MAX'] = p_max
###                        ts.sleep(t_settling)
                        time.sleep(t_settling)                  # DL850E compatible
                        daq.sc['eval_flag'] = 1  # flag the time in which the power will be analyzed, see Figure SA14.3
                        daq.data_capture()
###                        ts.sleep(t_settling*1.5)  # This time period will be analyzed for pass/fail criteria
                        time.sleep(t_settling*1.5)              # DL850E compatible

                        daq.data_sample()                       # <- add
                        data = grid.wt3000_data_capture_read()
                        #data = daq.data_capture_read()
                        ts.log_debug('Powers targ, min, max: %s, %s, %s' % (p_targ, p_min, p_max))
                        if phases == 'Single Phase':
                            ts.log_debug('EUT Power: %s' % data.get('AC_P_1'))
                            AC_W = data.get('AC_P_1')
                        else:
                            ts.log_debug('EUT Powers are: %s, %s, %s' %
                                         (data.get('AC_P_1'), data.get('AC_P_2'), data.get('AC_P_3')))
                            AC_W = data.get('AC_P_1') + data.get('AC_P_2') + data.get('AC_P_3')
                        ts.log('AC_W = %s' % (AC_W))
                        ts.log('p_rated = %s' % (p_rated))
                        AC_W_pct = (AC_W/p_rated)*100.
                        if daq.sc['P_TARGET_MIN'] <= AC_W_pct <= daq.sc['P_TARGET_MAX']:
                            passfail = 'Pass'
                        else:
                            passfail = 'Fail'
                        if step_count <= n_points:
                            direction = 'up'
                        else:
                            direction = 'down'

###                        result_summary.write('%s, %s, %s, %s, %s, %s, %s, %s, %s, %s \n' %
###                                             (passfail, ts.config_name(), power*100., n_iter+1, direction,
###                                              f_step, AC_W_pct, daq.sc['P_TARGET_MIN'], daq.sc['P_TARGET_MAX'],
###                                              filename))
                        daq.sc['eval_flag'] = 0
                        daq.data_capture()

### Graph drawing for FREA original gnuplot
### <START>
                        grf_rec = [(data.get('AC_FREQ_1')), (data.get('AC_P_1')/1000)]
                        writer.writerow(grf_rec)

                        grf_rec_trg = [f_step, ((p_rated*(p_targ/100))*power/1000)]
                        writer_trg.writerow(grf_rec_trg)
### <END>
                    result_summary.write('%s, %s, %s, %s, %s, %s, %s, %s, %s, %s \n' %
                                        (passfail, ts.config_name(), power*100., n_iter+1, direction,
                                         f_step, AC_W_pct, daq.sc['P_TARGET_MIN'], daq.sc['P_TARGET_MAX'],
                                         filename))

                    daq.data_capture(False)
                    ds = daq.data_capture_dataset()
                    ts.log('Saving file: %s' % filename)
                    ds.to_csv(ts.result_file_path(filename))
                    result_params['plot.title'] = test_str
                    ts.result_file(filename, params=result_params)

                    grf_dat.close()             # <-add
                    grf_dat_trg.close()         # <-add

                    gnuplot.stdin.write('set xlabel "Frequency (Hz)"\n')
                    gnuplot.stdin.write('set ylabel "Active Power (kW)"\n')

###                    std = round(float(p_rated)/1000)
###                    set_over = std + std*0.1
###                    set_under = 0
###                    set_cmd = "set yrange [" + str(set_under) + ":" + str(set_over) + "]\n"
                    set_cmd = "set autoscale y\n"
                    ts.log('set_cmd = %s' % (set_cmd))
                    gnuplot.stdin.write(set_cmd)

###                    set_cmd = 'set xrange [80:120]\n'
                    set_cmd = "set autoscale x\n"
                    ts.log('set_cmd = %s' % (set_cmd))
                    gnuplot.stdin.write(set_cmd)

                    gnuplot.stdin.write('set term png size 1000, 1000\n')
                    gnuplot.stdin.write('set grid lw 1\n')
###                    gnuplot.stdin.write('set key box\n')

                    graph_cmd = "set datafile separator ','\n"
                    gnuplot.stdin.write(graph_cmd)

                    # Up Graph Start ---------------------------------------------------
                    graph_str = '\SA14_cv=%s_pw=%0.1f_i=%s' % (fw_curve, power, n_iter+1)
                    graph_out_file = ts.results_dir() + graph_str + ".png"
                    graph_out = re.sub(r'\\', "/", graph_out_file)
                    ts.log('graph_out = %s' % (graph_out))
                    graph_cmd = "set output " + "'" + graph_out + "'\n"
                    ts.log('graph_cmd = %s' % (graph_cmd))
                    gnuplot.stdin.write(graph_cmd)

                    # Active power
                    graph_cmd = "plot " + "'" + grf_dat_file_trg + "'" + " ti 'Ideal Point' with linespoints pt 7 lc rgb 'navy', " + "'" + grf_dat_file + "' ti 'Measurement Point' with points pt 7 lc rgb 'red'\n"
                    ts.log('graph_cmd = %s' % (graph_cmd))
                    gnuplot.stdin.write(graph_cmd)
                    # Up Graph End -----------------------------------------------------

###     Return setting
        gnuplot.stdin.write('set terminal windows\n')
        gnuplot.stdin.write('set output\n')

###        ts.sleep(5)
        time.sleep(5)
### <END>

        if daq_wf is not None:             # DL850E compatible
           daq_wf.data_capture(False)      # DL850E compatible
        time.sleep(3)                      # DL850E compatible
###        ts.sleep(3)                        # DL850E compatible
        if daq_wf is not None:             # DL850E compatible
           daq_wf.data_save()              # DL850E compatible
        time.sleep(5)                      # DL850E compatible
###        ts.sleep(5)                        # DL850E compatible

        result = script.RESULT_COMPLETE

    # except script.ScriptFail, e:
    except script.ScriptFail as e:
        reason = str(e)
        if reason:
            ts.log_error(reason)
    finally:
        if daq is not None:
            daq.close()
        if daq_wf is not None:             # DL850E compatible
            daq_wf.data_capture(False)     # DL850E compatible
            time.sleep(3)                  # DL850E compatible
###            ts.sleep(3)                    # DL850E compatible
            daq_wf.close()                 # DL850E compatible
        # if eut is not None:
        #     eut.close()
        # if pv is not None:
        #     pv.close()
        # if grid is not None:
        #     grid.close()
        # if rs is not None:
        #     rs.close()
        # if chil is not None:
        #     chil.close()
        if grid is not None:
            grid.close4fq()

        # create result workbook
        xlsxfile = ts.config_name() + '.xlsx'
        rslt.result_workbook(xlsxfile, ts.results_dir(), ts.result_dir())
        ts.result_file(xlsxfile)

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

        result = test_run()

        ts.result(result)
        if result == script.RESULT_FAIL:
            rc = 1

    # except Exception, e:
    except Exception as e:
        ts.log_error('Test script exception: %s' % traceback.format_exc())
        rc = 1

    sys.exit(rc)

info = script.ScriptInfo(name=os.path.basename(__file__), run=run, version='1.0.0')

### Add for version control
### <START>
info.param_group('aist', label='AIST Parameters', glob=True)
info.param('aist.script_version', label='Script Version', default='5.0.b')
info.param('aist.library1_version', label='Library Version (gridsim_frea_simulator)', default='4.5.0')
info.param('aist.library2_version', label='Library Version (das_dl850e)', default='1.0.0')
### <END>

# der.params(info)
# EUT FW parameters
info.param_group('fw', label='FW Configuration')
info.param('fw.fw_mode', label='Freq-Watt Mode', default='Parameters',
           values=['Parameters', 'Pointwise'],
           desc='Parameterized FW curve or pointwise linear FW curve?')
###info.param('fw.p_rated', label='Output Power Rating (W)', default=34500.)
info.param('fw.s_rated', label='Apparent power rating (VA)', default=50000)
info.param('fw.p_rated', label='Output Power Rating (W)', default=50000)
info.param('fw.f_nom', label='Nominal AC frequency (Hz)', default=50.)
info.param('fw.f_min', label='Min AC frequency (Hz)', default=49.)
info.param('fw.f_max', label='Max AC frequency (Hz)', default=52.)
info.param('fw.MSAHz', label='Manufacturer\'s stated AC frequency accuracy (Hz)', default=0.1)
info.param('fw.MSAP', label='Manufacturer\'s stated power accuracy (W)', default=10.)
info.param('fw.ts', label='Settling time (s)', default=1.)
info.param('fw.fstart_min', label='Min start of frequency droop (Hz)', default=50.1)
info.param('fw.fstart_max', label='Max start of frequency droop (Hz)', default=51.)
info.param('fw.k_pf_min', label='Min slope of frequency droop (%Prated/Hz)', default=0.1)
info.param('fw.k_pf_max', label='Max slope of frequency droop (%Prated/Hz)', default=1.0)
info.param('fw.k_pf', label='Slope of frequency droop (%Prated/Hz)', default=0.4)
info.param('fw.phases', label='Phases', default='Single Phase', values=['Single Phase', '3-Phase 3-Wire',
                                                                         '3-Phase 4-Wire'])
info.param_group('test', label='Test Parameters')
info.param('test.curves', label='Curves to Evaluate', default='Both',
           values=['Characteristic Curve 1', 'Characteristic Curve 2', 'Both'])
info.param('test.irr', label='Power Levels', default='All',
           values=['100%', '66%', '33%', 'All'])
info.param('test.n_iter', label='Number of iteration for each test', default=3)
info.param('test.n_points', label='Number of points tested above f_start', default=3)

gridsim.params(info)
# pvsim.params(info)
das.params(info)
das.params(info, 'das_wf', 'Data Acquisition (Waveform)')     # DL850E compatible
# hil.params(info)

# info.logo('sunspec.gif')

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
