#ifndef DYNAMIXEL_ACTUATOR_HPP_
#define DYNAMIXEL_ACTUATOR_HPP_

#include "stdint.h"
#include "utility/config.h"

#define DXL_TORQUE_ON  1
#define DXL_TORQUE_OFF  0

// The reason for checking #ifndef here is to avoid conflict with Dynamixel SDK.
#ifndef AX12A
#define AX12A               (uint16_t)12
#endif
#ifndef AX12W
#define AX12W               (uint16_t)300
#endif
#ifndef AX18A
#define AX18A               (uint16_t)18
#endif

#ifndef RX10
#define RX10                (uint16_t)10
#endif
#ifndef RX24F
#define RX24F               (uint16_t)24
#endif
#ifndef RX28
#define RX28                (uint16_t)28
#endif
#ifndef RX64
#define RX64                (uint16_t)64
#endif

#ifndef DX113
#define DX113               (uint16_t)113
#endif
#ifndef DX116
#define DX116               (uint16_t)116
#endif
#ifndef DX117
#define DX117               (uint16_t)117
#endif

#ifndef EX106
#define EX106               (uint16_t)107
#endif

#ifndef MX12W
#define MX12W               (uint16_t)360
#endif
#ifndef MX28
#define MX28                (uint16_t)29
#endif
#ifndef MX64
#define MX64                (uint16_t)310
#endif
#ifndef MX106
#define MX106               (uint16_t)320
#endif

#ifndef MX28_2
#define MX28_2              (uint16_t)30
#endif
#ifndef MX64_2
#define MX64_2              (uint16_t)311
#endif
#ifndef MX106_2
#define MX106_2             (uint16_t)321
#endif

#ifndef XL320
#define XL320               (uint16_t)350
#endif

#ifndef XL330_M077
#define XL330_M077          (uint16_t)1190
#endif
#ifndef XL330_M288
#define XL330_M288          (uint16_t)1200
#endif

#ifndef XC330_M181
#define XC330_M181          (uint16_t)1230
#endif
#ifndef XC330_M288
#define XC330_M288          (uint16_t)1240
#endif

#ifndef XC330_T181
#define XC330_T181          (uint16_t)1210
#endif
#ifndef XC330_T288
#define XC330_T288          (uint16_t)1220
#endif

#ifndef XC430_W150
#define XC430_W150          (uint16_t)1070
#endif
#ifndef XC430_W240
#define XC430_W240          (uint16_t)1080
#endif
#ifndef XXC430_W250
#define XXC430_W250         (uint16_t)1160
#endif

#ifndef XL430_W250
#define XL430_W250          (uint16_t)1060
#endif
#ifndef XXL430_W250
#define XXL430_W250         (uint16_t)1090
#endif

#ifndef XM430_W210
#define XM430_W210          (uint16_t)1030
#endif
#ifndef XM430_W350
#define XM430_W350          (uint16_t)1020
#endif

#ifndef XM540_W150
#define XM540_W150          (uint16_t)1130
#endif
#ifndef XM540_W270
#define XM540_W270          (uint16_t)1120
#endif

#ifndef XH430_V210
#define XH430_V210          (uint16_t)1050
#endif
#ifndef XH430_V350
#define XH430_V350          (uint16_t)1040
#endif
#ifndef XH430_W210
#define XH430_W210          (uint16_t)1010
#endif
#ifndef XH430_W350
#define XH430_W350          (uint16_t)1000
#endif

#ifndef XH540_W150
#define XH540_W150          (uint16_t)1110
#endif
#ifndef XH540_W270
#define XH540_W270          (uint16_t)1100
#endif
#ifndef XH540_V150
#define XH540_V150          (uint16_t)1150
#endif
#ifndef XH540_V270
#define XH540_V270          (uint16_t)1140
#endif

#ifndef XD430_T210
#define XD430_T210          (uint16_t)1011
#endif
#ifndef XD430_T350
#define XD430_T350          (uint16_t)1001
#endif

#ifndef XD540_T150
#define XD540_T150          (uint16_t)1111
#endif
#ifndef XD540_T270
#define XD540_T270          (uint16_t)1101
#endif

#ifndef XW430_T200
#define XW430_T200          (uint16_t)1280
#endif
#ifndef XW430_T333
#define XW430_T333          (uint16_t)1270
#endif
#ifndef XW540_T140
#define XW540_T140          (uint16_t)1180
#endif
#ifndef XW540_T260
#define XW540_T260          (uint16_t)1170
#endif

#ifndef PRO_L42_10_S300_R
#define PRO_L42_10_S300_R   (uint16_t)35072
#endif
#ifndef PRO_L54_30_S400_R
#define PRO_L54_30_S400_R   (uint16_t)37928
#endif
#ifndef PRO_L54_30_S500_R
#define PRO_L54_30_S500_R   (uint16_t)37896
#endif
#ifndef PRO_L54_50_S290_R
#define PRO_L54_50_S290_R   (uint16_t)38176
#endif
#ifndef PRO_L54_50_S500_R
#define PRO_L54_50_S500_R   (uint16_t)38152
#endif

#ifndef PRO_M42_10_S260_R
#define PRO_M42_10_S260_R   (uint16_t)43288
#endif
#ifndef PRO_M54_40_S250_R
#define PRO_M54_40_S250_R   (uint16_t)46096
#endif
#ifndef PRO_M54_60_S250_R
#define PRO_M54_60_S250_R   (uint16_t)46352
#endif

#ifndef PRO_H42_20_S300_R
#define PRO_H42_20_S300_R   (uint16_t)51200
#endif
#ifndef PRO_H54_100_S500_R
#define PRO_H54_100_S500_R  (uint16_t)53768
#endif
#ifndef PRO_H54_200_S500_R
#define PRO_H54_200_S500_R  (uint16_t)54024
#endif

#ifndef PRO_M42_10_S260_RA
#define PRO_M42_10_S260_RA  (uint16_t)43289
#endif
#ifndef PRO_M54_40_S250_RA
#define PRO_M54_40_S250_RA  (uint16_t)46097
#endif
#ifndef PRO_M54_60_S250_RA
#define PRO_M54_60_S250_RA  (uint16_t)46353
#endif

#ifndef PRO_H42_20_S300_RA
#define PRO_H42_20_S300_RA  (uint16_t)51201
#endif
#ifndef PRO_H54_100_S500_RA
#define PRO_H54_100_S500_RA (uint16_t)53761
#endif
#ifndef PRO_H54_200_S500_RA
#define PRO_H54_200_S500_RA (uint16_t)54025
#endif

#ifndef PRO_H42P_020_S300_R
#define PRO_H42P_020_S300_R (uint16_t)2000
#endif
#ifndef PRO_H54P_100_S500_R
#define PRO_H54P_100_S500_R (uint16_t)2010
#endif
#ifndef PRO_H54P_200_S500_R
#define PRO_H54P_200_S500_R (uint16_t)2020
#endif

#ifndef PRO_M42P_010_S260_R
#define PRO_M42P_010_S260_R (uint16_t)2100
#endif
#ifndef PRO_M54P_040_S250_R
#define PRO_M54P_040_S250_R (uint16_t)2110
#endif
#ifndef PRO_M54P_060_S250_R
#define PRO_M54P_060_S250_R (uint16_t)2120
#endif

#ifndef YM070_210_M001_RH
#define YM070_210_M001_RH (uint16_t)4000
#endif
#ifndef YM070_210_B001_RH
#define YM070_210_B001_RH (uint16_t)4010
#endif
#ifndef YM070_210_R051_RH
#define YM070_210_R051_RH (uint16_t)4020
#endif
#ifndef YM070_210_R099_RH
#define YM070_210_R099_RH (uint16_t)4030
#endif
#ifndef YM070_210_A051_RH
#define YM070_210_A051_RH (uint16_t)4040
#endif
#ifndef YM070_210_A099_RH
#define YM070_210_A099_RH (uint16_t)4050
#endif

#ifndef YM080_230_M001_RH
#define YM080_230_M001_RH (uint16_t)4120
#endif
#ifndef YM080_230_B001_RH
#define YM080_230_B001_RH (uint16_t)4130
#endif
#ifndef YM080_230_R051_RH
#define YM080_230_R051_RH (uint16_t)4140
#endif
#ifndef YM080_230_R099_RH
#define YM080_230_R099_RH (uint16_t)4150
#endif
#ifndef YM080_230_A051_RH
#define YM080_230_A051_RH (uint16_t)4160
#endif
#ifndef YM080_230_A099_RH
#define YM080_230_A099_RH (uint16_t)4170
#endif



namespace ControlTableItem{
  enum ControlTableItemIndex{
    MODEL_NUMBER = 0,
    MODEL_INFORMATION,
    FIRMWARE_VERSION,
    PROTOCOL_VERSION,
    ID,
    SECONDARY_ID,
    BAUD_RATE,
    DRIVE_MODE,
    CONTROL_MODE,
    OPERATING_MODE,
    CW_ANGLE_LIMIT,
    CCW_ANGLE_LIMIT,
    TEMPERATURE_LIMIT,
    MIN_VOLTAGE_LIMIT,
    MAX_VOLTAGE_LIMIT,
    PWM_LIMIT,
    CURRENT_LIMIT,
    VELOCITY_LIMIT,
    MAX_POSITION_LIMIT,
    MIN_POSITION_LIMIT,
    ACCELERATION_LIMIT,
    MAX_TORQUE,
    HOMING_OFFSET,
    MOVING_THRESHOLD,
    MULTI_TURN_OFFSET,
    RESOLUTION_DIVIDER,
    EXTERNAL_PORT_MODE_1,
    EXTERNAL_PORT_MODE_2,
    EXTERNAL_PORT_MODE_3,
    EXTERNAL_PORT_MODE_4,
    STATUS_RETURN_LEVEL,
    RETURN_DELAY_TIME,
    ALARM_LED,
    SHUTDOWN,

    TORQUE_ENABLE,
    DXL_LED,
    DXL_LED_RED,
    DXL_LED_GREEN,
    DXL_LED_BLUE,
    REGISTERED_INSTRUCTION,
    HARDWARE_ERROR_STATUS,
    VELOCITY_P_GAIN,
    VELOCITY_I_GAIN,
    POSITION_P_GAIN,
    POSITION_I_GAIN,
    POSITION_D_GAIN,
    FEEDFORWARD_1ST_GAIN,
    FEEDFORWARD_2ND_GAIN,
    P_GAIN,
    I_GAIN,
    D_GAIN,
    CW_COMPLIANCE_MARGIN,
    CCW_COMPLIANCE_MARGIN,
    CW_COMPLIANCE_SLOPE,
    CCW_COMPLIANCE_SLOPE,
    GOAL_PWM,
    GOAL_TORQUE,
    GOAL_CURRENT,
    GOAL_POSITION,
    GOAL_VELOCITY,
    GOAL_ACCELERATION,
    MOVING_SPEED,
    PRESENT_PWM,
    PRESENT_LOAD,
    PRESENT_SPEED,
    PRESENT_CURRENT,
    PRESENT_POSITION,
    PRESENT_VELOCITY,
    PRESENT_VOLTAGE,
    PRESENT_TEMPERATURE,
    TORQUE_LIMIT,
    REGISTERED,
    MOVING,
    LOCK,
    PUNCH,
    CURRENT,
    SENSED_CURRENT,
    REALTIME_TICK,
    TORQUE_CTRL_MODE_ENABLE,
    BUS_WATCHDOG,
    PROFILE_ACCELERATION,
    PROFILE_VELOCITY,
    MOVING_STATUS,
    VELOCITY_TRAJECTORY,
    POSITION_TRAJECTORY,
    PRESENT_INPUT_VOLTAGE,
    EXTERNAL_PORT_DATA_1,
    EXTERNAL_PORT_DATA_2,
    EXTERNAL_PORT_DATA_3,
    EXTERNAL_PORT_DATA_4,

    //for DY
    STARTUP_CONFIGURATION,
    POSITION_LIMIT_THRESHOLD,
    IN_POSITION_THRESHOLD,
    FOLLOWING_ERROR_THRESHOLD,
    INVERTER_TEMPERATURE_LIMIT,
    MOTOR_TEMPERATURE_LIMIT,
    ELECTRONIC_GEAR_RATIO_NUMERATOR,
    ELECTRONIC_GEAR_RATIO_DENOMINATOR,
    Safe_STOP_TIME,
    BRAKE_DELAY,
    GOAL_UPDATE_DELAY,
    OVEREXCITATION_VOLTAGE,
    NORMAL_EXCITATION_VOLTAGE,
    OVEREXCITATION_TIME,
    PRESENT_VELOCITY_LPF_FREQUENCY,
    GOAL_CURRENT_LPF_FREQUENCY,
    POSITION_FF_LPF_TIME,
    VELOCITY_FF_LPF_TIME,
    CONTROLLER_STATE,
    ERROR_CODE,
    ERROR_CODE_HISTORY1,
    HYBRID_SAVE,
    PROFILE_ACCELERATION_TIME,
    PROFIIE_TIME,
    PWM_OFFSET,
    CURRENT_OFFSET,
    VELOCITY_OFFSET,
    PRESENT_MOTOR_TEMPERATURE,


    LAST_DUMMY_ITEM = 0xFF
  };
}

namespace DYNAMIXEL{

typedef struct ControlTableItemInfo{
  uint16_t addr;
  uint8_t addr_length;
} ControlTableItemInfo_t;

ControlTableItemInfo_t getControlTableItemInfo(uint16_t model_num, uint8_t control_item);

} // namespace DYNAMIXEL

#endif /* DYNAMIXEL_ACTUATOR_HPP_ */