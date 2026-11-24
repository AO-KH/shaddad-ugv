function rosmsgOut = UInt8MultiArray(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.layout = bus_conv_fcns.ros2.busToMsg.std_msgs.MultiArrayLayout(slBusIn.layout,rosmsgOut.layout(1));
    rosmsgOut.data = uint8(slBusIn.data(1:slBusIn.data_SL_Info.CurrentLength));
end
