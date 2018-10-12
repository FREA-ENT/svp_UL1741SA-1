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
from svpelab import loadsim
###from svpelab import pvsim       <- Commented out because middleware is communicated using gridsim
###from svpelab import battsim     <- Commented out because middleware is communicated using gridsim, Since it is storage battery control, it added
from svpelab import das
###from svpelab import der         <- Commented out because middleware is communicated using gridsim
from svpelab import hil
import script
from svpelab import result as rslt
import numpy as np
import time
import subprocess
from subprocess import PIPE
import re
import csv

def test_pass_fail(pf_act=[], pf_target=None, pf_msa=None):

    # set span=True if pf pass/fail range spans unity
    pf_lower = pf_upper = 1.0
    span = False
    if pf_target < 0:
        pf_msa = -pf_msa
        offset = 1.0 + pf_target
        if offset <= pf_msa:
            pf_upper = 1.0 - (pf_msa - offset)
            span = True
        else:
            pf_upper = pf_target - pf_msa
        pf_lower = pf_target + pf_msa
    elif pf_target < 1.0:
        offset = 1.0 - pf_target
        if offset <= pf_msa:
            pf_lower = -(1.0 - (pf_msa - offset))
            if pf_lower == -1.0:
                pf_lower = 1.0
            span = True
        else:
            pf_lower = pf_target - pf_msa
        pf_upper = pf_target + pf_msa
    elif pf_target == 1.0:
        pf_upper = 1.0 - pf_msa
        pf_lower = -pf_upper
        span = True

    pass_fail = 'Pass'
    # check if pf in range
    for pf in pf_act:
        if span:
            # if the pass/fail span includes unity. If the actual PF is unity, test PASS
            if pf == 1.0:
                pass
            # if span & neg PF, only check lower limit at a less negative value than pf_act
            elif pf < 0 and pf <= pf_lower:
                pass
            # if span & pos PF, check pf_act is between unity and PF upper
            elif 1.0 > pf >= pf_upper:
                pass
            else:
                # one of the PF measurements is not within the pass/fail boundary
                pass_fail = 'Fail'
                break
        else:
            # if PF between upper and lower limits, pass
            if pf_lower <= pf <= pf_upper:
                pass
            else:
                # one of the PF measurements is not within the pass/fail boundary
                pass_fail = 'Fail'
                break

    return pass_fail, pf_lower, pf_upper


def get_last_point_from_dataset(ds=None, point_list=None, list_idx=0):
    """
    Returns the last data point for a given dataset and data point name

    ds: dataset
    point_list: list of data names, e.g., ['AC_PF_1', 'AC_PF_2', 'AC_PF_3']
    list_idx: phase index for the point to get the last data from
    """
    try:
        ts.log('ERR Cheack get_last_point_from_dataset 1')
        idx = ds.points.index(point_list[list_idx])  # get the data index
        ts.log('ERR Cheack get_last_point_from_dataset 2')
        data = ds.data[idx]  # get the data
        ts.log('ERR Cheack get_last_point_from_dataset 3')
        if len(data) <= 0:
            ts.fail('No data for data point %s' % (point_list[list_idx]))
        elif len(data) <= 0:
            ts.fail('No data for data point %s' % (point_list[list_idx]))
        return float(data[-1])  # use the last measurement for the pass/fail check
    except ValueError, e:
        ts.fail('Data point %s not in dataset: %s' % (point_list[list_idx], e))


def test_run():

    result = script.RESULT_FAIL
    grid = None
    pv = p_rated = None
    daq = None
    eut = None
    rs = None
    chil = None
    result_summary = None

### Correction as graph is not displayed
### <START>
###    sc_points = ['PF_TARGET', 'PF_MAX', 'PF_MIN']
###    sc_points = ['TIME', 'AC_Q_1', 'AC_P_1', 'AC_S_1', 'AC_PF_1', 'AC_IRMS_1', 'PF_TARGET', 'PF_MAX', 'PF_MIN']
    sc_points = ['TIME', 'AC_Q_1', 'AC_Q_2', 'AC_Q_3', 'AC_P_1', 'AC_P_2', 'AC_P_3', 'AC_S_1', 'AC_S_2', 'AC_S_3', 'AC_PF_1', 'AC_PF_2', 'AC_PF_3', 'AC_IRMS_1', 'AC_IRMS_2', 'AC_IRMS_3', 'PF_TARGET', 'PF_MAX', 'PF_MIN']
### <END>

    pf_points = {}
    va_points = {}
    p_points = {}
    q_points = {}


    # result params
    result_params = {
        'plot.title': ts.name,
        'plot.x.title': 'Time (sec)',
        'plot.x.points': 'TIME',
        'plot.y.points': 'AC_PF_1, PF_TARGET',
        'plot.y.title': 'Power Factor',
        'plot.y2.points': 'AC_IRMS_1',
        'plot.y2.title': 'Current (A)'
    }

    try:
        # read aist parameters
        s_version = ts.param_value('aist.script_version')
        l1_version = ts.param_value('aist.library_version_1')
        l2_version = ts.param_value('aist.library_version_2')
        l3_version = ts.param_value('aist.library_version_3')

        s_rated = ts.param_value('eut.s_rated')
        p_rated = ts.param_value('eut.p_rated')
        p_ramp_rate = ts.param_value('eut.ramp_rate')
        pf_min_ind = ts.param_value('eut.pf_min_ind')
        pf_min_cap = ts.param_value('eut.pf_min_cap')
        pf_settling_time = ts.param_value('eut.pf_settling_time')
        pf_msa = ts.param_value('eut.pf_msa')
        phases = ts.param_value('eut.phases')

        p_low = p_rated * .2
        pf_mid_ind = (1. + pf_min_ind)/2.
        pf_mid_cap = -(1. - pf_min_cap)/2.

        '''
        2) Set all AC source parameters to the normal operating conditions for the EUT. 
        '''

        # initialize HIL environment, if necessary
        chil = hil.hil_init(ts)
        if chil is not None:
            chil.config()

        # grid simulator is initialized with test parameters and enabled
        grid = gridsim.gridsim_init(ts)

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

        # initialize data acquisition
        daq = das.das_init(ts, sc_points=sc_points)
        ts.log('DAS device: %s' % daq.info())

        '''
        3) Turn on the EUT. It is permitted to set all L/HVRT limits and abnormal voltage trip parameters to the
        widest range of adjustability possible with the SPF enabled in order not to cross the must trip
        magnitude threshold during the test.
        '''
### Commented out because middleware is communicated using gridsim
### <START>
###        # it is assumed the EUT is on
###        eut = der.der_init(ts)
###        if eut is not None:
###            eut.config()
### <END>

        # disable volt/var curve
###        eut.volt_var(params={'Ena': False})    <- Commented out because middleware is communicated using gridsim
        grid.volt_var(params={'Ena': False})      # <- Change to control from grid

        '''
        4) Select 'Fixed Power Factor' operational mode.
        '''
        # set power levels that are enabled
        power_levels = []
        if ts.param_value('spf.p_100') == 'Enabled':
            power_levels.append((1, '100'))
        if ts.param_value('spf.p_50') == 'Enabled':
            power_levels.append((.5, '50'))
        if ts.param_value('spf.p_20') == 'Enabled':
            power_levels.append((.2, '20'))

        # set target power factors
        pf_targets = []
        if ts.param_value('spf.pf_min_ind') == 'Enabled':
            pf_targets.append(pf_min_ind)
        if ts.param_value('spf.pf_mid_ind') == 'Enabled':
            pf_targets.append(pf_mid_ind)
        if ts.param_value('spf.pf_min_cap') == 'Enabled':
            pf_targets.append(pf_min_cap)
        if ts.param_value('spf.pf_mid_cap') == 'Enabled':
            pf_targets.append(pf_mid_cap)
        ts.log('The target power factors for this test are: %s' % pf_targets)

        n_r = ts.param_value('spf.n_r')

        # open result summary file
        result_summary_filename = 'result_summary.csv'
        result_summary = open(ts.result_file_path(result_summary_filename), 'a+')
        ts.result_file(result_summary_filename)
        if phases == 'Single Phase':
            result_summary.write('Result, Test Name, Power Level (%), Iteration, PF Actual, PF Target, '
                                 'PF MSA, PF Min Allowed, PF Max Allowed, Dataset File,'
                                 'Power (pu), Reactive Power (pu), P Target at Rated (pu), Q Target at Rated (pu)\n')
        else:
            result_summary.write('Result, Test Name, Power Level (%), Iteration, PF Actual 1, PF Actual 2, PF Actual 3,'
                                 'PF Target, PF MSA, PF Min Allowed, PF Max Allowed, Dataset File,'
                                 'Power 1 (pu), Power 2 (pu), Power 3 (pu),'
                                 'Reactive Power 1 (pu), Reactive Power 2 (pu), Reactive Power 3 (pu), '
                                 'P Target at Rated (pu), Q Target at Rated (pu) \n')

### Graph drawing for FREA original gnuplot
### <START>
        grf_dat_file = ts.results_dir() + "\SA12_power_factor.csv"
        grf_dat_file = re.sub(r'\\', "/", grf_dat_file)
        ts.log('grf_dat_file = %s' % (grf_dat_file))
        grf_dat = open(grf_dat_file, mode='w')
        writer = csv.writer(grf_dat, lineterminator='\n')
### <END>

        for pf in pf_targets:
            for power_level in power_levels:
                '''
                5) Set the input source to produce Prated for the EUT.
                '''
                power, power_label = power_level
                ###pv.power_set(p_rated * power)             <- Commented out because middleware is communicated using gridsim
                ###batt.power_set(p_rated * power)           <- Commented out because middleware is communicated using gridsim
                ###grid.power_set(p_rated * power)              # <- Change to control from grid
                grid.power_setVV(p_rated * power, s_rated, p_ramp_rate)   # <- Change to control from grid
                ts.log('*** Setting power level to %s W (rated power * %s)' % ((p_rated * power), power))

                for count in range(1, n_r + 1):
                    ts.log('Starting pass %s' % (count))
                    '''
                    6) Set the EUT power factor to unity. Measure the AC source voltage and EUT current to measure the
                    displacement
                    '''
                    # use pass_fail to get lower, upper
                    passfail, pf_lower, pf_upper = test_pass_fail(pf_act=[1.0], pf_target=1.0, pf_msa=pf_msa)
                    daq.data_sample()                                # <- Since the graph is not displayed, it is added
                    data = grid.wt3000_data_capture_read()           # <- Since the graph is not displayed, it is added
                    daq.sc['TIME'] = time.time()                     # <- Since the graph is not displayed, it is added
                    daq.sc['AC_Q_1'] = data.get('AC_Q_1')            # <- Since the graph is not displayed, it is added
                    daq.sc['AC_P_1'] = data.get('AC_P_1')            # <- Since the graph is not displayed, it is added
                    daq.sc['AC_S_1'] = data.get('AC_S_1')            # <- Since the graph is not displayed, it is added
                    daq.sc['AC_PF_1'] = data.get('AC_P_1')           # <- Since the graph is not displayed, it is added
                    daq.sc['AC_IRMS_1'] = data.get('AC_IRMS_1')      # <- Since the graph is not displayed, it is added
                    daq.sc['PF_TARGET'] = 1.0
                    daq.sc['PF_MAX'] = pf_upper
                    daq.sc['PF_MIN'] = pf_lower
                    if phases != 'Single Phase':
                        daq.sc['AC_Q_2'] = data.get('AC_Q_2')            # <- Since the graph is not displayed, it is added
                        daq.sc['AC_Q_3'] = data.get('AC_Q_3')            # <- Since the graph is not displayed, it is added
                        daq.sc['AC_P_2'] = data.get('AC_P_2')            # <- Since the graph is not displayed, it is added
                        daq.sc['AC_P_3'] = data.get('AC_P_3')            # <- Since the graph is not displayed, it is added
                        daq.sc['AC_S_2'] = data.get('AC_S_2')            # <- Since the graph is not displayed, it is added
                        daq.sc['AC_S_3'] = data.get('AC_S_3')            # <- Since the graph is not displayed, it is added
                        daq.sc['AC_PF_2'] = data.get('AC_P_2')           # <- Since the graph is not displayed, it is added
                        daq.sc['AC_PF_3'] = data.get('AC_P_3')           # <- Since the graph is not displayed, it is added
                        daq.sc['AC_IRMS_2'] = data.get('AC_IRMS_2')      # <- Since the graph is not displayed, it is added
                        daq.sc['AC_IRMS_3'] = data.get('AC_IRMS_3')      # <- Since the graph is not displayed, it is added
                    ###if eut is not None:
                    if grid is not None:
                        if chil is not None:
                            ###eut.fixed_pf(params={'Ena': True, 'PF': 100})  # HACK for ASGC - To be fixed in firmware
                            ###eut.fixed_pf(params={'Ena': True, 'PF': 1.0})
                            grid.fixed_pf(params={'Ena': True, 'PF': 1.0})
                        else:
                            ###eut.fixed_pf(params={'Ena': True, 'PF': 1.0})
                            grid.fixed_pf(params={'Ena': True, 'PF': 1.0})
                        ###pf_setting = eut.fixed_pf()
                        pf_setting = grid.fixed_pf()
                        ts.log('PF setting read: %s' % (pf_setting))
                    ts.log('Starting data capture for pf = %s' % (1.0))
                    daq.data_capture(True)
                    ts.log('Sampling for %s seconds' % (pf_settling_time * 3))
                    ts.sleep(pf_settling_time * 3)
                    ts.log('Sampling complete')
                    daq.data_capture(False)
                    ds = daq.data_capture_dataset()
                    filename = 'spf_1000_%s_%s.csv' % (str(power_label), str(count))
                    ds.to_csv(ts.result_file_path(filename))
                    ts.result_file(filename)
                    ts.log('Saving data capture1 %s' % (filename))

                    # create result summary entry
                    pf_points = ['AC_PF_1']
                    va_points = ['AC_S_1']
                    p_points = ['AC_P_1']
                    q_points = ['AC_Q_1']
                    if phases != 'Single Phase':
                        pf_points.append('AC_PF_2')
                        pf_points.append('AC_PF_3')
                        va_points.append('AC_S_2')
                        va_points.append('AC_S_3')
                        p_points.append('AC_P_2')
                        p_points.append('AC_P_3')
                        q_points.append('AC_Q_2')
                        q_points.append('AC_Q_3')
                    pf_act = []
                    va_act = []
                    #p_act = [0]*len(pf_points)  # Used for plotting results on P-Q plane
                    #q_act = [0]*len(pf_points)  # Used for plotting results on P-Q plane
                    p_act = []  # Used for plotting results on P-Q plane
                    q_act = []  # Used for plotting results on P-Q plane
                    va_nameplate_per_phase = p_rated/len(pf_points)  # assume VA_nameplate and P_rated are the same

                    for ph in range(len(pf_points)):  # for each phase...
                        ts.log('ERR Cheack capture 1')
                        pf_act.append(get_last_point_from_dataset(ds=ds, point_list=pf_points, list_idx=ph))
                        ts.log('ERR Cheack capture 2')
                        va_act.append(get_last_point_from_dataset(ds=ds, point_list=va_points, list_idx=ph))
                        ts.log('ERR Cheack capture 3')
                        # p_act[ph] = (va_act[ph]/va_nameplate_per_phase)*pf_act[ph]
                        # q_act[ph] = (va_act[ph]/va_nameplate_per_phase)*-np.sin(np.arccos(pf_act[ph]))
                        # this only produces negative reactive power values because sin(arccos(x)) is positive in [-1,1]
                        # if this method is to be used pulling the sign off the reactive power is necessary.
                        p_act.append(get_last_point_from_dataset(ds=ds, point_list=p_points, list_idx=ph)
                                     /va_nameplate_per_phase)  # in pu
                        q_act.append(get_last_point_from_dataset(ds=ds, point_list=q_points, list_idx=ph)
                                     /va_nameplate_per_phase)  # in pu

                    p_target_at_rated = 1.0
                    q_target_at_rated = 0.0


### Graph drawing for FREA original gnuplot
### <START>
                    grf_rec = [p_act[0], q_act[0]]
                    writer.writerow(grf_rec)
### <END>

                    passfail, pf_lower, pf_upper = test_pass_fail(pf_act=pf_act, pf_target=1.0, pf_msa=pf_msa)
                    if phases == 'Single Phase':
                        result_summary.write('%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n' %
                                             (passfail, ts.config_name(), power * 100, count, pf_act[0], 1.0, pf_msa,
                                              pf_lower, pf_upper, filename, p_act[0], q_act[0],
                                              p_target_at_rated, q_target_at_rated))
                    else:
                        result_summary.write('%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s,'
                                             '%s, %s\n' %
                                             (passfail, ts.config_name(), power * 100, count, pf_act[0],
                                              pf_act[1], pf_act[2], 1.0, pf_msa, pf_lower, pf_upper, filename,
                                              p_act[0], p_act[1], p_act[2], q_act[0], q_act[1], q_act[2],
                                              p_target_at_rated, q_target_at_rated))

                    '''
                    7) Set the EUT power factor to the value in Test 1 of Table SA12.1. Measure the AC source voltage
                    and EUT current to measure the displacement power factor and record all data.
                    '''
                    # use pass_fail to get lower, upper
                    passfail, pf_lower, pf_upper = test_pass_fail(pf_act=[1.0], pf_target=pf, pf_msa=pf_msa)
                    daq.data_sample()                                # <- Since the graph is not displayed, it is added
                    data = grid.wt3000_data_capture_read()           # <- Since the graph is not displayed, it is added
                    daq.sc['TIME'] = time.time()                     # <- Since the graph is not displayed, it is added
                    daq.sc['AC_Q_1'] = data.get('AC_Q_1')            # <- Since the graph is not displayed, it is added
                    daq.sc['AC_P_1'] = data.get('AC_P_1')            # <- Since the graph is not displayed, it is added
                    daq.sc['AC_S_1'] = data.get('AC_S_1')            # <- Since the graph is not displayed, it is added
                    daq.sc['AC_PF_1'] = data.get('AC_P_1')           # <- Since the graph is not displayed, it is added
                    daq.sc['AC_IRMS_1'] = data.get('AC_IRMS_1')      # <- Since the graph is not displayed, it is added
                    daq.sc['PF_TARGET'] = pf
                    daq.sc['PF_MAX'] = pf_upper
                    daq.sc['PF_MIN'] = pf_lower
                    if phases != 'Single Phase':
                        daq.sc['AC_Q_2'] = data.get('AC_Q_2')            # <- Since the graph is not displayed, it is added
                        daq.sc['AC_Q_3'] = data.get('AC_Q_3')            # <- Since the graph is not displayed, it is added
                        daq.sc['AC_P_2'] = data.get('AC_P_2')            # <- Since the graph is not displayed, it is added
                        daq.sc['AC_P_3'] = data.get('AC_P_3')            # <- Since the graph is not displayed, it is added
                        daq.sc['AC_S_2'] = data.get('AC_S_2')            # <- Since the graph is not displayed, it is added
                        daq.sc['AC_S_3'] = data.get('AC_S_3')            # <- Since the graph is not displayed, it is added
                        daq.sc['AC_PF_2'] = data.get('AC_P_2')           # <- Since the graph is not displayed, it is added
                        daq.sc['AC_PF_3'] = data.get('AC_P_3')           # <- Since the graph is not displayed, it is added
                        daq.sc['AC_IRMS_2'] = data.get('AC_IRMS_2')      # <- Since the graph is not displayed, it is added
                        daq.sc['AC_IRMS_3'] = data.get('AC_IRMS_3')      # <- Since the graph is not displayed, it is added
                    ###if eut is not None:
                    if grid is not None:
                        if chil is not None:
### Although it may not be necessary to change it, I tried it other than typhoon
### <START>
###                            parameters = {'Ena': True, 'PF': pf*100}  # HACK for ASGC - To be fixed in their firmware
                            parameters = {'Ena': True, 'PF': pf}
### <END>
                        else:
                            parameters = {'Ena': True, 'PF': pf}
                        ts.log('PF set: %s' % (parameters))
                        ###eut.fixed_pf(params=parameters)
                        grid.fixed_pf(params=parameters)
                        ###pf_setting = eut.fixed_pf()
                        pf_setting = grid.fixed_pf()
                        ts.log('PF setting read: %s' % (pf_setting))
                    ts.log('Starting data capture for pf = %s' % (pf))
                    daq.data_capture(True)
                    ts.log('Sampling for %s seconds' % (pf_settling_time * 3))
                    ts.sleep(pf_settling_time * 3)
                    ts.log('Sampling complete')
                    daq.data_capture(False)
                    ds = daq.data_capture_dataset()
                    testname = 'spf_%s_%s_%s' % (str(pf * 1000), str(power_label), str(count))
                    filename = '%s.csv' % (testname)
                    ds.to_csv(ts.result_file_path(filename))
                    result_params['plot.title'] = testname
                    ts.result_file(filename, params=result_params)
                    ts.log('Saving data capture2 %s' % (filename))

                    # create result summary entry
                    pf_act = []
                    va_act = []
                    #p_act = [0]*len(pf_points)  # Used for plotting results on P-Q plane
                    #q_act = [0]*len(pf_points)  # Used for plotting results on P-Q plane
                    p_act = []  # Used for plotting results on P-Q plane
                    q_act = []  # Used for plotting results on P-Q plane
                    va_nameplate_per_phase = p_rated/len(pf_points)  # assume VA_nameplate and P_rated are the same

                    for ph in range(len(pf_points)):  # for each phase...
                        pf_act.append(get_last_point_from_dataset(ds=ds, point_list=pf_points, list_idx=ph))
                        va_act.append(get_last_point_from_dataset(ds=ds, point_list=va_points, list_idx=ph))
                        #p_act[ph] = (va_act[ph]/va_nameplate_per_phase)*pf_act[ph]
                        #q_act[ph] = (va_act[ph]/va_nameplate_per_phase)*-np.sin(np.arccos(pf_act[ph]))
                        p_act.append(get_last_point_from_dataset(ds=ds, point_list=p_points, list_idx=ph)
                                     /va_nameplate_per_phase)  # in pu
                        q_act.append(get_last_point_from_dataset(ds=ds, point_list=q_points, list_idx=ph)
                                     /va_nameplate_per_phase)  # in pu

                    p_target_at_rated = np.fabs(pf)
                    if pf < 0:
                        q_target_at_rated = np.sin(np.arccos(pf))  # PF < 0, +Q
                    else:
                        q_target_at_rated = -np.sin(np.arccos(pf))  # PF > 0, -Q

                    #p_target_at_rated = pf
                    #q_target_at_rated = -np.sin(np.arccos(pf))

                    passfail, pf_lower, pf_upper = test_pass_fail(pf_act=pf_act, pf_target=pf, pf_msa=pf_msa)
                    if phases == 'Single Phase':
                        result_summary.write('%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n' %
                                             (passfail, ts.config_name(), power * 100, count, pf_act[0], pf, pf_msa,
                                              pf_lower, pf_upper, filename, p_act[0], q_act[0],
                                              p_target_at_rated, q_target_at_rated))
                    else:
                        result_summary.write('%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s,'
                                             '%s, %s\n' %
                                             (passfail, ts.config_name(), power * 100, count, pf_act[0],
                                              pf_act[1], pf_act[2], pf, pf_msa, pf_lower, pf_upper, filename,
                                              p_act[0], p_act[1], p_act[2], q_act[0], q_act[1], q_act[2],
                                              p_target_at_rated, q_target_at_rated))

                    '''
                    8) Repeat steps (6) - (8) for two additional times for a total of three repetitions.
                    '''
                '''
                9) Repeat steps (5) - (7) at two additional power levels. One power level shall be a Pmin or 20% of
                Prated and the second at any power level between 33% and 66% of Prated.
                '''
            '''
            10) Repeat Steps (6) - (9) for Tests 2 - 5 in Table SA12.1
            '''

        '''
        11) In the case of bi-directional inverters, repeat Steps (6) - (10) for the active power flow direction
        '''

        grf_dat.close()

        result = script.RESULT_COMPLETE

    except script.ScriptFail, e:
        reason = str(e)
        if reason:
            ts.log_error(reason)
    finally:
        ts.log('--------------Finally START----------------')

        if grid is not None:
            grid.fixed_pf(params={'Ena': True, 'PF': 1.0})                     # <- Change to control from grid
            grid.close()
### Commented out because middleware is communicated using gridsim
### <START>
###        if pv is not None:
###            if p_rated is not None:
###                pv.power_set(p_rated)
###            pv.close()
###        if batt is not None:
###            batt.close()
###            if p_rated is not None:
###                ts.log('p_rated = %s' % (p_rated))
###                batt.power_set(p_rated)
###        if eut is not None:
###            eut.fixed_pf(params={'Ena': False, 'PF': 1.0})
###            eut.close()
### <END>
        if daq is not None:
            daq.close()
        if rs is not None:
            rs.close()
        if chil is not None:
            chil.close()

        if result_summary is not None:
            result_summary.close()

        # create result workbook
        excelfile = ts.config_name() + '.xlsx'
        rslt.result_workbook(excelfile, ts.results_dir(), ts.result_dir())
        ts.result_file(excelfile)





### Graph drawing for FREA original gnuplot
### <START>

        gnuplot =  subprocess.Popen('gnuplot', shell=True, stdin=PIPE, stdout=PIPE, stderr=PIPE, universal_newlines=True)

        ### SA12_power_factor.png
        graph_out = ts.results_dir() + "\SA12_power_factor.png.png"
        ts.log('graph_out = %s' % (graph_out))
        graph_cmd = "set output " + "'" + graph_out + "'\n"
        ts.log('graph_cmd1 = %s' % (graph_cmd))
        graph_cmd = "set output " + "'" + graph_out + "'\n"

        gnuplot.stdin.write('set xlabel "Active Power (pu)"\n')
        gnuplot.stdin.write('set ylabel "Reactive Active Power (pu)"\n')
        gnuplot.stdin.write('set term png size 1000, 1000\n')
        gnuplot.stdin.write('set grid lw 1\n')
        gnuplot.stdin.write('set key box\n')
        gnuplot.stdin.write(graph_cmd)
        graph_cmd = "set datafile separator ','\n"
        gnuplot.stdin.write(graph_cmd)
###        graph_cmd = "plot " + "'" + grf_dat_file + "'" + " with lines ti 'VV Line', " + "'" + grf_dat_file + "' ti 'VV Point' pt 6\n"
        graph_cmd = "plot " + "'" + grf_dat_file + "' ti 'PF Point' pt 7\n"
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

info = script.ScriptInfo(name=os.path.basename(__file__), run=run, version='1.1.0')

### Add for version control
### <START>
info.param_group('aist', label='AIST Parameters', glob=True)
info.param('aist.script_version', label='Script Version', default='4.0.0')
info.param('aist.library_version', label='Library Version (gridsim_frea_ac_simulator)', default='4.1.0')
### <END>

info.param_group('spf', label='Test Parameters')
info.param('spf.p_100', label='Power Level 100% Tests', default='Enabled', values=['Disabled', 'Enabled'])
info.param('spf.p_50', label='Power Level 50% Tests', default='Enabled', values=['Disabled', 'Enabled'])
info.param('spf.p_20', label='Power Level 20% Tests', default='Enabled', values=['Disabled', 'Enabled'])
info.param('spf.n_r', label='Number of test repetitions', default=3)

info.param('spf.pf_min_ind', label='Minimum inductive', default='Enabled', values=['Disabled', 'Enabled'])
info.param('spf.pf_mid_ind', label='Mid-range inductive', default='Enabled', values=['Disabled', 'Enabled'])
info.param('spf.pf_min_cap', label='Minimum capacitive', default='Enabled', values=['Disabled', 'Enabled'])
info.param('spf.pf_mid_cap', label='Mid-range capacitive', default='Enabled', values=['Disabled', 'Enabled'])

info.param_group('eut', label='EUT Parameters', glob=True)
info.param('eut.s_rated', label='Apparent power rating (VA)', default=0.0)
#info.param('eut.p_rated', label='P_rated', default=3000)
info.param('eut.p_rated', label='Output power rating (W)', default=0.0)
info.param('eut.phases', label='Phases', default='Single Phase', values=['Single Phase', '3-Phase 3-Wire',
                                                                         '3-Phase 4-Wire'])

info.param('eut.pf_min_ind', label='PF_min_ind (Underexcited)', default=0.850)
info.param('eut.pf_min_cap', label='PF_min_cap (Overexcited) (negative value)', default=-0.850)
info.param('eut.pf_settling_time', label='PF Settling Time (secs)', default=1)
info.param('eut.pf_msa', label='PF Manufacturer Stated Accuracy (PF units)', default=5.0)
info.param('eut.ramp_rate', label='Power Ramp Rate (0.01%/s)', default=0)

###der.params(info)       <- Commented out because middleware is communicated using gridsim
das.params(info)
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


