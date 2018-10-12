<scriptConfig name="SPF_mid_ind" script="SA12_power_factor">
  <params>
    <param name="eut.pf_min_cap" type="float">-0.85</param>
    <param name="eut.pf_min_ind" type="float">0.85</param>
    <param name="gridsim.frea.phases" type="int">1</param>
    <param name="eut.pf_settling_time" type="int">1</param>
    <param name="spf.n_r" type="int">3</param>
    <param name="eut.pf_msa" type="float">5.0</param>
    <param name="gridsim.frea.ip_port" type="int">2001</param>
    <param name="eut.p_rated" type="int">40000</param>
    <param name="gridsim.frea.ip_addr" type="string">127.0.0.1</param>
    <param name="aist.library_version" type="string">2.0.0</param>
    <param name="aist.script_version" type="string">2.0.0</param>
    <param name="spf.pf_min_cap" type="string">Disabled</param>
    <param name="spf.pf_min_ind" type="string">Disabled</param>
    <param name="hil.mode" type="string">Disabled</param>
    <param name="loadsim.mode" type="string">Disabled</param>
    <param name="spf.pf_mid_cap" type="string">Disabled</param>
    <param name="spf.p_100" type="string">Enabled</param>
    <param name="spf.p_20" type="string">Enabled</param>
    <param name="spf.p_50" type="string">Enabled</param>
    <param name="spf.pf_mid_ind" type="string">Enabled</param>
    <param name="gridsim.auto_config" type="string">Enabled</param>
    <param name="gridsim.mode" type="string">FREA_AC_Simulator</param>
    <param name="das.mode" type="string">Manual</param>
    <param name="eut.phases" type="string">Single Phase</param>
    <param name="gridsim.frea.comm" type="string">TCP/IP</param>
  </params>
</scriptConfig>
