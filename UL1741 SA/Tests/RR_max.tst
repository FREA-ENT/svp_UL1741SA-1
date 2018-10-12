<scriptConfig name="RR_max" script="SA11_ramp_rate">
  <params>
    <param name="rr.n_r" type="int">1</param>
    <param name="gridsim.frea.phases" type="int">1</param>
    <param name="eut.t_dwell" type="float">3.0</param>
    <param name="eut.rr_msa" type="int">5</param>
    <param name="eut.rr_up_min" type="float">10.0</param>
    <param name="eut.rr_up_max" type="float">20.0</param>
    <param name="eut.i_low" type="float">20.0</param>
    <param name="eut.i_rated" type="float">115.2</param>
    <param name="eut.v_nom" type="float">346.4</param>
    <param name="gridsim.frea.ip_port" type="int">2001</param>
    <param name="gridsim.frea.ip_addr" type="string">127.0.0.1</param>
    <param name="aist.library_version" type="string">4.0.0</param>
    <param name="aist.script_version" type="string">4.0.0</param>
    <param name="hil.mode" type="string">Disabled</param>
    <param name="loadsim.mode" type="string">Disabled</param>
    <param name="rr.soft_start" type="string">Disabled</param>
    <param name="gridsim.auto_config" type="string">Enabled</param>
    <param name="rr.rr_min" type="string">Enabled</param>
    <param name="rr.rr_mid" type="string">Enabled</param>
    <param name="rr.rr_max" type="string">Enabled</param>
    <param name="gridsim.mode" type="string">FREA_Simulator</param>
    <param name="das_das_wf.mode" type="string">Manual</param>
    <param name="das_das_rms.mode" type="string">Manual</param>
    <param name="gridsim.frea.comm" type="string">TCP/IP</param>
  </params>
</scriptConfig>
