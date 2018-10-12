<scriptConfig name="LVRT_LV1" script="SA9_volt_trip_time">
  <params>
    <param name="gridsim.frea.phases" type="int">1</param>
    <param name="vrt.n_r" type="int">1</param>
    <param name="vrt.t_hold" type="float">1.0</param>
    <param name="eut.t_msa" type="float">1.0</param>
    <param name="eut.v_msa" type="float">2.0</param>
    <param name="eut.t_trip" type="int">5</param>
    <param name="vrt.v_test" type="float">100.0</param>
    <param name="eut.v_nom" type="float">190.0</param>
    <param name="gridsim.frea.ip_port" type="int">2001</param>
    <param name="eut.p_rated" type="int">4000</param>
    <param name="gridsim.frea.ip_addr" type="string">127.0.0.1</param>
    <param name="eut.phases" type="string">3-Phase 3-Wire</param>
    <param name="aist.library_version" type="string">4.0.0</param>
    <param name="aist.script_version" type="string">4.0.0</param>
    <param name="hil.mode" type="string">Disabled</param>
    <param name="der.mode" type="string">Disabled</param>
    <param name="gridsim.auto_config" type="string">Enabled</param>
    <param name="vrt.phase_1" type="string">Enabled</param>
    <param name="vrt.phase_3" type="string">Enabled</param>
    <param name="vrt.phase_2" type="string">Enabled</param>
    <param name="gridsim.mode" type="string">FREA_Simulator</param>
    <param name="das.mode" type="string">Manual</param>
    <param name="gridsim.frea.comm" type="string">TCP/IP</param>
  </params>
</scriptConfig>
