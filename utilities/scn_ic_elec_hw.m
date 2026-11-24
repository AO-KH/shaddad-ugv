function ic = scn_ic_elec_hw(P, implId)
%SCN_IC_ELEC_HW  Initial condition of Shaddad_System/Plant/ElecHw_Delay (Bus_ElecHwIn at power-up, T16).
%
%   ic = scn_ic_elec_hw(P, VC_IMPL)
%   The DI word the MCU reads at t = 0 is the electrical state of the body at key-on: both E-stop loops
%   closed (ESTOP_CH1_OK, ESTOP_CH2_OK), lid closed, key on, the implement coding plug (IMPL_ID0/1 =
%   VC_IMPL), safety relay still open (SAFETY_OK / SAFETY_HV_OK = 0, ICD 6 power-up sequence). An all-zero
%   word would read as "E-stop pressed" and latch HW_ESTOP in the first Task_10 step.
%   Field order = Bus_ElecHwIn (di_word, reset_flag, mcu_reset).

d = P.electronics.di;
w = 2^d.estop_ch1_ok + 2^d.estop_ch2_ok + 2^d.lid_closed + 2^d.key_on + ...
    bitand(implId, 1)*2^d.impl_id0 + (bitand(implId, 2) > 0)*2^d.impl_id1;
ic = P.ic.Bus_ElecHwIn;
ic.di_word = uint16(w);
ic.reset_flag = false;
ic.mcu_reset = false;
end
