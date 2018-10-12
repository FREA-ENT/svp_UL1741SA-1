<scriptConfig name="VV_1" script="SA13_volt_var">
  <params>
    <param name="eut_vv.q_max_under" type="float">-10000.0</param>
    <param name="eut_vv.v_msa" type="float">0.5</param>
    <param name="eut_vv.vv_t_settling" type="float">1.0</param>
    <param name="vv.n_r_min" type="int">1</param>
    <param name="eut_vv.vv_deadband_min" type="float">1.0</param>
    <param name="gridsim.frea.phases" type="int">1</param>
    <param name="vv.n_r_66" type="int">1</param>
    <param name="vv.n_r_100" type="int">1</param>
    <param name="srd.vv_segment_point_count" type="int">3</param>
    <param name="eut_vv.vv_deadband_max" type="float">5.0</param>
    <param name="srd.vv_p_min_pct" type="float">20.0</param>
    <param name="srd.vv_p_max_pct" type="float">100.0</param>
    <param name="eut_vv.v_min" type="float">190.0</param>
    <param name="eut_vv.v_nom" type="float">200.0</param>
    <param name="eut_vv.v_max" type="float">210.0</param>
    <param name="srd.vv_k_var_min" type="float">1000.0</param>
    <param name="gridsim.frea.ip_port" type="int">2001</param>
    <param name="eut_vv.var_msa" type="float">3000.0</param>
    <param name="eut_vv.ramp_rate" type="int">5000</param>
    <param name="eut_vv.k_var_max" type="float">10000.0</param>
    <param name="eut_vv.q_max_over" type="float">10000.0</param>
    <param name="eut_vv.var_ramp_max" type="float">12500.0</param>
    <param name="eut_vv.var_rated" type="float">45000.0</param>
    <param name="eut_vv.s_rated" type="float">50000.0</param>
    <param name="eut_vv.p_rated" type="float">50000.0</param>
    <param name="gridsim.frea.ip_addr" type="string">127.0.0.1</param>
    <param name="eut_vv.phases" type="string">3-Phase 3-Wire</param>
    <param name="aist.script_version" type="string">4.0.0</param>
    <param name="aist.library_version" type="string">4.1.0</param>
    <param name="vv.spec_curve" type="string">Disabled</param>
    <param name="hil.mode" type="string">Disabled</param>
    <param name="vv.pp_active" type="string">Disabled</param>
    <param name="vv.test_1" type="string">Enabled</param>
    <param name="vv.pp_reactive" type="string">Enabled</param>
    <param name="vv.test_2" type="string">Enabled</param>
    <param name="vv.test_3" type="string">Enabled</param>
    <param name="gridsim.auto_config" type="string">Enabled</param>
    <param name="gridsim.mode" type="string">FREA_Simulator</param>
    <param name="das.mode" type="string">Manual</param>
    <param name="gridsim.frea.comm" type="string">TCP/IP</param>
  </params>
</scriptConfig>
