# coding: shift-jis
####################################################################################################
# This script was tuned specifically for the AIST FREA environment (Fixed in 2018)
#     AIST:National Institute of Advanced Industrial Science and Technology 
#     FREA:Fukushima Renewable Energy Institute
#
# What is the AIST FREA environment
#   Communication with SunSpecSVP is middleware called ExCon, and ExCon is
#   a mechanism to communicate with inverters and simulators.
####################################################################################################
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

import sys
import os
import traceback
from svpelab import gridsim
###from svpelab import pvsim       <- Commented out because middleware is communicated using gridsim
###from svpelab import battsim     <- Commented out because middleware is communicated using gridsim, Since it is storage battery control, it added
from svpelab import das
###from svpelab import der         <- Commented out because middleware is communicated using gridsim
from svpelab import loadsim
from svpelab import hil
import result as rslt

import script
import datetime
import time

import subprocess
from subprocess import PIPE
import re
import csv


TRIP_WAIT_DELAY = 5
POWER_WAIT_DELAY = 5

'''
def test_pass_fail(i_target=None, ds=None):

    i_10 = i_target * .1
    i_90 = i_target * .9

    passfail = 'Fail'

    point = None
    trigger_data = []
    try:
        point = 'AC_IRMS_1'
        idx = ds.points.index(point)
        i_data = ds.data[idx]
        point = 'TRIGGER'
        idx = ds.points.index(point)
        trigger_data = ds.data[idx]
    except ValueError, e:
        ts.fail('Data point %s not in dataset' % (point))
    if len(trigger_data) <= 0:
        ts.fail('No data in dataset')

    for t in range(len(trigger_data)):
        if t
        if v_target[i] != 0:
            act = var[i]
            target = var_target[i]
            min = target - var_msa
            max = target + var_msa
            var_act.append(var[i])
            var_target.append(target)
            var_min.append(min)
            var_max.append(max)
            if act < min or act > max:
                passfail = 'Fail'

    return (passfail)
'''

def test_run():

    result = script.RESULT_FAIL
    grid = None
    load = None
    pv = None
    daq_rms = None
    daq_wf = None
    eut = None
    chil = None

### Correction as graph is not displayed
### <START>
    sc_points = ['TIME', 'AC_IRMS_1']
### <END>

    # result params
###        'plot.title': 'title_name',
    result_params = {
        'plot.title': ts.name,
        'plot.x.title': 'Time (secs)',
        'plot.x.points': 'TIME',
        'plot.y.points': 'AC_IRMS_1',
        'plot.y.title': 'Current (A)'
    }

    try:
        # read aist parameters
        s_version = ts.param_value('aist.script_version')
        l1_version = ts.param_value('aist.library_version_1')
        l2_version = ts.param_value('aist.library_version_2')
        l3_version = ts.param_value('aist.library_version_3')

        s_rated = ts.param_value('eut.s_rated')                 # <- Change to control from grid
        v_nom = ts.param_value('eut.v_nom')
        i_rated = ts.param_value('eut.i_rated')
        i_low = ts.param_value('eut.i_low')
        rr_up_min = ts.param_value('eut.rr_up_min')
        rr_up_max = ts.param_value('eut.rr_up_max')
        rr_msa = ts.param_value('eut.rr_msa')
        t_dwell = ts.param_value('eut.t_dwell')

        ramp_rates = []
        if ts.param_value('rr.rr_max') == 'Enabled':
            ramp_rates.append(rr_up_max)
        if ts.param_value('rr.rr_mid') == 'Enabled':
            ramp_rates.append((rr_up_min + rr_up_max)/2)
        if ts.param_value('rr.rr_min') == 'Enabled':
            ramp_rates.append(rr_up_min)

        soft_start = ts.param_value('rr.soft_start') == 'Enabled'
        n_r = ts.param_value('rr.n_r')
        v_trip = ts.param_value('rr.v_trip')
        t_reconnect = ts.param_value('rr.t_reconnect')

        p_low = i_low * v_nom
        p_rated = i_rated * v_nom

        '''
        Test assumes the following steps have been performed:
            - Connect the EUT according to test requirements.
            - Set all AC source parameters to the nominal operating conditions for the EUT.
            - Turn on the EUT and allow to reach steady state.
        '''

        # initialize HIL environment, if necessary
        chil = hil.hil_init(ts)
        if chil is not None:
            chil.config()

        # grid simulator is initialized with test parameters and enabled
        grid = gridsim.gridsim_init(ts)
        profile_supported = False

        # load simulator initialization
        load = loadsim.loadsim_init(ts)
        if load is not None:
            ts.log('Load device: %s' % load.info())

### Commented out because middleware is communicated using gridsim
### <START>
###        # pv simulator is initialized with test parameters and enabled
###        pv = pvsim.pvsim_init(ts)
###        pv.power_set(p_low)
###        pv.power_on()
### <END>

### Commented out because middleware is communicated using gridsim
### <START>
###        # batt simulator is initialized with test parameters and enabled
###        batt = battsim.battsim_init(ts)
###        batt.power_set(p_max)
###        batt.power_on()
### <END>

        # initialize rms data acquisition
###        daq_rms = das.das_init(ts, 'das_rms')
        daq_rms = das.das_init(ts, 'das_rms', sc_points=sc_points)
        if daq_rms is not None:
            ts.log('DAS RMS device: %s' % (daq_rms.info()))

        # initialize waveform data acquisition
        daq_wf = das.das_init(ts, 'das_wf')
        if daq_wf is not None:
            ts.log('DAS Waveform device: %s' % (daq_wf.info()))

### Commented out because middleware is communicated using gridsim
### <START>
###        # it is assumed the EUT is on
###        eut = der.der_init(ts)
###        if eut is not None:
###            eut.config()
### <END>

### Graph drawing for FREA original gnuplot
### <START>
        grf_dat_file = ts.results_dir() + "\SA11_ramp_rate_3wave.csv"
        grf_dat_file = re.sub(r'\\', "/", grf_dat_file)
        ts.log('grf_dat_file = %s' % (grf_dat_file))
        grf_dat = open(grf_dat_file, mode='w')
        writer = csv.writer(grf_dat, lineterminator='\n')
### <END>

        if soft_start:
            test_label = 'ss'
        else:
            test_label = 'rr'

        # For each ramp rate test level in Table SA11.1
        for rr in ramp_rates:
            duration = 100/rr + (t_dwell * 2)

            if soft_start:
                # set soft start ramp rate
### Commented out because middleware is communicated using gridsim
### <START>
###                if eut is not None:
###                    # eut.soft_start_ramp_rate(rr)
###                batt.ramp_rates(params={'power': 0 ,'ramp_rate': rr * 100})      <- Commented out because middleware is communicated using gridsim
###                grid.ramp_rates(params={'power': 0 ,'ramp_rate': rr * 100})         # <- Change to control from grid
                grid.ramp_rates(params={'power': 0 ,'ramp_rate': rr * 100, 's_rated': s_rated})       # <- Change to control from grid
### <END>
                sample_duration = duration + TRIP_WAIT_DELAY + t_reconnect
            else:
                # set normal ramp rate
### Commented out because middleware is communicated using gridsim
### <START>
###                if eut is not None:
###                    # eut.ramp_rate(rr)
###                batt.ramp_rates(params={'power': 0 ,'ramp_rate': rr * 100})      <- Commented out because middleware is communicated using gridsim
###                grid.ramp_rates(params={'power': 0 ,'ramp_rate': rr * 100})         # <- Change to control from grid
                grid.ramp_rates(params={'power': 0 ,'ramp_rate': rr * 100, 's_rated': s_rated})       # <- Change to control from grid
### <END>
                sample_duration = duration + POWER_WAIT_DELAY

            for count in range(1, n_r + 1):
                if daq_rms is not None:
                    data = grid.wt3000_data_capture_read()                 # <- Since the graph is not displayed, it is added
                    daq_rms.sc['TIME'] = time.time()                       # <- Since the graph is not displayed, it is added
                    daq_rms.sc['AC_IRMS_1'] = data.get('AC_IRMS_1')        # <- Since the graph is not displayed, it is added
                    ts.log('Starting data capture %s' % (rr))
### Graph drawing for FREA original gnuplot
### <START>
                    ###grf_rec = [time.time(), data.get('AC_IRMS_1')]
                    now = datetime.datetime.now()
                    grf_rec = [now.strftime("%Y/%m/%d %H:%M:%S"), data.get('AC_IRMS_1')]
                    writer.writerow(grf_rec)
### <END>
                    daq_rms.data_capture(True)
                    ts.log('Waiting for 3 seconds to start test')
                    #ts.sleep(3)
                    time.sleep(3)
                if soft_start:
                    # set soft start ramp rate
###                    batt.ramp_rates(params={'power': p_rated ,'ramp_rate': rr * 100})       <- Commented out because middleware is communicated using gridsim
###                    grid.ramp_rates(params={'power': p_rated ,'ramp_rate': rr * 100})          # <- Change to control from grid
                    grid.ramp_rates(params={'power': p_rated ,'ramp_rate': rr * 100, 's_rated': s_rated})       # <- Change to control from grid
                    # set to trip voltage
### Commented out because middleware is communicated using gridsim
### <START>
###                    v1, v2, v3 = grid.voltage()
###                    ts.log('Waiting for 5 seconds to start test')
###                    ts.sleep(5)
###                    v1, v2, v3 = grid.voltageRR()
###                    ts.sleep(5)
                    v1, v2, v3 = grid.voltageRR()                                   # <- Change to control from grid
### <END>
                    ts.log('Nominal voltage V1= %s' % (v1))
                    ts.log('Nominal voltage V2= %s' % (v2))
                    ts.log('Nominal voltage V3= %s' % (v3))
### Since it will not trip if only one phase, set the same value for all three phases
### <START>
###                    v_trip_grid = (v1 * v_trip/100)
                    v_trip_grid1 = (float(v1) * float(v_trip)/100)
                    v_trip_grid2 = (float(v2) * float(v_trip)/100)
                    v_trip_grid3 = (float(v3) * float(v_trip)/100)
###                    grid.voltage((v_trip_grid, v2, v3))
                    ts.log('Setting voltage to trip voltage1 (%s V)' % v_trip_grid1)
                    ts.log('Setting voltage to trip voltage2 (%s V)' % v_trip_grid2)
                    ts.log('Setting voltage to trip voltage3 (%s V)' % v_trip_grid3)
### <END>
### for test           ts.log('Waiting for 5 seconds to start test')
### for test           ts.sleep(5)
### for test           ts.log('Nominal voltage V1= %s' % (v_trip_grid))
### for test           ts.log('Nominal voltage V2= %s' % (v2))
### for test           ts.log('Nominal voltage V3= %s' % (v3))
                    grid.voltageRR(str(v_trip_grid1), str(v_trip_grid2), str(v_trip_grid3))
                    ts.log('Waiting %s seconds' % (TRIP_WAIT_DELAY))
                    #ts.sleep(TRIP_WAIT_DELAY)
                    time.sleep(TRIP_WAIT_DELAY)
                    ts.log('Setting voltage to original nominal voltage (%s V)' % v1)
###                    grid.voltage((v1, v2, v3))                                       <- Commented out because middleware is communicated using gridsim
                    grid.voltageRR(str(v1), str(v2), str(v3))                           # <- Change to control from grid
                else:
                    ts.log('Setting to low power threshold (%s W)' % p_low)
###                    pv.power_set(p_low)                                              <- Commented out because middleware is communicated using gridsim
###                    batt.ramp_rates(params={'power': p_low ,'ramp_rate': rr * 100})  <- Commented out because middleware is communicated using gridsim
###                    grid.ramp_rates(params={'power': p_low ,'ramp_rate': rr * 100})     # <- Change to control from grid
                    grid.ramp_rates(params={'power': p_low ,'ramp_rate': rr * 100, 's_rated': s_rated})       # <- Change to control from grid
                    ts.log('Waiting for %s seconds' % (POWER_WAIT_DELAY))
                    #ts.sleep(POWER_WAIT_DELAY)
                    time.sleep(POWER_WAIT_DELAY)

                ts.log('Ramp rate: %s%%/sec - pass %s' % (rr, count))
                ts.log('Setting to I_rated: %s' % (i_rated))
###                pv.power_set(p_rated)                                                <- Commented out because middleware is communicated using gridsim
###                batt.ramp_rates(params={'power': p_rated ,'ramp_rate': rr * 100})    <- Commented out because middleware is communicated using gridsim
###                grid.ramp_rates(params={'power': p_rated ,'ramp_rate': rr * 100})       # <- Change to control from grid
                grid.ramp_rates(params={'power': p_rated ,'ramp_rate': rr * 100, 's_rated': s_rated})       # <- Change to control from grid
                ts.log('Sampling for %s seconds' % (sample_duration))
                #ts.sleep(sample_duration)
                time.sleep(sample_duration)
                if daq_rms is not None:
                    daq_rms.data_sample()                                               # <- Since the graph is not displayed, it is added
                    data = grid.wt3000_data_capture_read()                              # <- Since the graph is not displayed, it is added
                    daq_rms.sc['TIME'] = time.time()                                    # <- Since the graph is not displayed, it is added
                    daq_rms.sc['AC_IRMS_1'] = data.get('AC_IRMS_1')                     # <- Since the graph is not displayed, it is added
### Graph drawing for FREA original gnuplot
### <START>
                    ###grf_rec = [time.time(), data.get('AC_IRMS_1')]
                    now = datetime.datetime.now()
                    grf_rec = [now.strftime("%Y/%m/%d %H:%M:%S"), data.get('AC_IRMS_1')]
                    writer.writerow(grf_rec)
### <END>
                    # Increase available input power to I_rated
                    ts.log('Sampling complete')
                    daq_rms.data_capture(False)
                    ds = daq_rms.data_capture_dataset()

                    test_name = '%s_%s_%s' % (test_label, str(int(rr)), str(count))
                    filename = '%s.csv' % (test_name)
                    ds.to_csv(ts.result_file_path(filename))
                    result_params['plot.title'] = test_name
                    ts.result_file(filename, params=result_params)
                    ts.log('Saving data capture %s' % (filename))

        grf_dat.close()

        result = script.RESULT_COMPLETE

    except script.ScriptFail, e:
        reason = str(e)
        if reason:
            ts.log_error(reason)
    finally:
        ts.log('--------------Finally START----------------')

        if grid is not None:
            grid.close()
### Commented out because middleware is communicated using gridsim
### <START>
###        if eut is not None:
###            eut.close()
###        if pv is not None:
###            pv.close()
###        if batt is not None:
###            batt.close()
### <END>
        if load is not None:
            load.close()
        if daq_rms is not None:
            daq_rms.close()
        if daq_wf is not None:
            daq_wf.close()
        if chil is not None:
            chil.close()

        # create result workbook
        file = ts.config_name() + '.xlsx'
        rslt.result_workbook(file, ts.results_dir(), ts.result_dir())
        ts.result_file(file)





### Graph drawing for FREA original gnuplot
### <START>

        gnuplot =  subprocess.Popen('gnuplot', shell=True, stdin=PIPE, stdout=PIPE, stderr=PIPE, universal_newlines=True)

        ### SA11_ramp_rate_3wave.png
        graph_out = ts.results_dir() + "\SA11_ramp_rate_3wave.png"
        ts.log('graph_out = %s' % (graph_out))
        graph_cmd = "set output " + "'" + graph_out + "'\n"
        ts.log('graph_cmd1 = %s' % (graph_cmd))
        graph_cmd = "set output " + "'" + graph_out + "'\n"
        gnuplot.stdin.write(graph_cmd)
        gnuplot.stdin.write('set term png size 1000, 1000\n')

        gnuplot.stdin.write('set ylabel "Active Power (W)"\n')
        gnuplot.stdin.write('set xdata time"\n')
        gnuplot.stdin.write('set xlabel "Time"\n')
        gnuplot.stdin.write('set timefmt "%Y/%m/%d %H:%M:%S"\n')
        gnuplot.stdin.write('set grid lw 1\n')
        gnuplot.stdin.write('set key box\n')

        graph_cmd = "set datafile separator ','\n"
        gnuplot.stdin.write(graph_cmd)
        graph_cmd = "plot " + "'" + grf_dat_file + "'" + " using 1:2 with lines ti 'RR Line', " + "'" + grf_dat_file + "' using 1:2 ti 'RR Point' pt 7\n"
###        graph_cmd = "plot " + "'" + grf_dat_file + "' using 1:2 ti 'RR Point' pt 7\n"
        ts.log('graph_cmd1 = %s' % (graph_cmd))
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
info.param('aist.script_version', label='Script Version', default='4.0.0')
info.param('aist.library_version', label='Library Version (gridsim_frea_ac_simulator)', default='4.1.0')
### <END>

info.param_group('rr', label='Test Parameters')
info.param('rr.rr_max', label='Maximum Ramp Rate Test', default='Enabled', values=['Disabled', 'Enabled'])
info.param('rr.rr_mid', label='Medium Ramp Rate Test', default='Enabled', values=['Disabled', 'Enabled'])
info.param('rr.rr_min', label='Minimum Ramp Rate Test', default='Enabled', values=['Disabled', 'Enabled'])
info.param('rr.n_r', label='Number of test repetitions', default=3)
info.param('rr.soft_start', label='Perform Soft Start', default='Disabled', values=['Disabled', 'Enabled'])
info.param('rr.v_trip', label='Trip Threshold (% V_nom)', default=140.0,
           active='rr.soft_start', active_value=['Enabled'])
info.param('rr.t_reconnect', label='Reconnect Time (secs)', default=600.0,
           active='rr.soft_start', active_value=['Enabled'])

info.param_group('eut', label='EUT Parameters', glob=True)
info.param('eut.s_rated', label='Apparent power rating (VA)', default=0.0)      # <- Change to control from grid
info.param('eut.v_nom', label='V_nom', default=120.0)
info.param('eut.i_rated', label='I_rated', default=10.0)
info.param('eut.i_low', label='I_low', default=1.0)
info.param('eut.rr_up_min', label='RR_up_min', default=20.0)
info.param('eut.rr_up_max', label='RR_up_max', default=100.0)
info.param('eut.t_dwell', label='T_dwell', default=5.0)
info.param('eut.rr_msa', label='RR_msa', default=5)

###der.params(info)       <- Commented out because middleware is communicated using gridsim
das.params(info, 'das_rms', 'Data Acquisition (RMS)')
das.params(info, 'das_wf', 'Data Acquisition (Waveform)')
gridsim.params(info)
loadsim.params(info)
###pvsim.params(info)     <- Commented out because middleware is communicated using gridsim
###battsim.params(info)   <- Commented out because middleware is communicated using gridsim, Since it is storage battery control, it added
hil.params(info)

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


