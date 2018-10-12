<scriptConfig name="LFRT_LF1" script="SA10_freq_ride_through">
  <params>
    <param name="gridsim.frea.phases" type="int">1</param>
    <param name="eut.t_msa" type="float">1.0</param>
    <param name="eut.freq_msa" type="float">2.0</param>
    <param name="frt.n_r" type="int">5</param>
    <param name="eut.frt_t_dwell" type="int">5</param>
    <param name="frt.freq_test" type="float">57.0</param>
    <param name="eut.freq_nom" type="float">60.0</param>
    <param name="frt.freq_grid_max" type="float">60.0</param>
    <param name="frt.freq_grid_min" type="float">60.0</param>
    <param name="eut.v_nom" type="int">190</param>
    <param name="frt.t_hold" type="float">299.0</param>
    <param name="gridsim.frea.ip_port" type="int">2001</param>
    <param name="eut.p_rated" type="int">40000</param>
    <param name="gridsim.frea.ip_addr" type="string">127.0.0.1</param>
    <param name="aist.script_version" type="string">2.0.0</param>
    <param name="aist.library_version" type="string">2.1.0</param>
    <param name="der.mode" type="string">Disabled</param>
    <param name="hil.mode" type="string">Disabled</param>
    <param name="loadsim.mode" type="string">Disabled</param>
    <param name="frt.p_100" type="string">Enabled</param>
    <param name="frt.p_20" type="string">Enabled</param>
    <param name="gridsim.auto_config" type="string">Enabled</param>
    <param name="gridsim.mode" type="string">FREA_AC_Simulator</param>
    <param name="das_das_wf.mode" type="string">Manual</param>
    <param name="das_das_rms.mode" type="string">Manual</param>
    <param name="gridsim.frea.comm" type="string">TCP/IP</param>
    <param name="frt.test_label" type="string">lfrt_lf1</param>
  </params>
</scriptConfig>
