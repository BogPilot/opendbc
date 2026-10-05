#pragma once

#include "opendbc/safety/declarations.h"

// Tesla AP1 Model S: Mobileye autopilot on the chassis bus (panda bus 0, autopilot side bus 2).
// A separate car port. Nothing here is shared with, or derived from, the Model 3/Y/X code in tesla.h.
//
// Reference: BogGyver/Tinkla (BogGyver/panda f7751e4 board/safety/safety_tesla.h, has_ap_hardware path),
// via BogPilot/openpilot tags ap1-driving-milestone-1 (013f1ffa) and ap1-driving-milestone-2 (92e84996)
// panda/board/safety/safety_tesla.h (milestone 2 adds the AP1 instrument-cluster substitution).
// Message layouts are opendbc tesla_can.dbc (same frames as BogGyver/opendbc 9c0b6fe tesla_can.dbc).
//
// Safety param: BogGyver/Tinkla's numbering (safety_tesla.h FLAG_TESLA_*), not the Model 3/Y layout.
//   FLAG_TESLA_LONG_CONTROL = 2   chassis 0x2b9 longitudinal (ALLOW_DEBUG builds only, see init)
//   FLAG_TESLA_HAS_AP       = 16  AP hardware on the chassis bus; selects this file from tesla_init()
// BogGyver/Tinkla's other bits (POWERTRAIN 1, RADAR_BEHIND_NOSECONE 4, HAS_IC_INTEGRATION 8,
// NEED_RADAR_EMULATION 32, ENABLE_HAO 64, HAS_IBOOSTER 128) are not implemented and are ignored.
//
// Differences from BogGyver/Tinkla and BogPilot, all stricter:
//  - 0x488 control type must be NONE (0) or ANGLE_CONTROL (1). BogPilot also let 2 through.
//  - 0x2b9 may not have both accel limits below inactive (could reverse the car after a stop).
//  - 0x2b9 min accel limit is -3.52 m/s^2 (BogPilot). BogGyver/Tinkla allowed -4.51.
//  - 0x349 (all-zero Hold clear) is only allowed with longitudinal control.
//  - longitudinal is only honored in ALLOW_DEBUG builds (BogGyver/Tinkla honored it in all builds).
//
// Instrument-cluster frames (BogPilot milestone 2, Tinkla TESLA_AP_FWD_MODDED idea): openpilot rebuilds each new
// stock 0x399 AutopilotStatus / 0x389 DAS_status2 / 0x239 DAS_lanes from bus 2 and sends it on bus 0 with stock
// counter + 1. The stock copy from bus 2 is dropped only while openpilot sent that address within 1.5x its stock
// period (750 ms for the 2 Hz 0x399 / 0x389, 150 ms for the 10 Hz 0x239). A 0x399 with an active autopilot state
// (3, 4, 5) is rejected unless controls are allowed. No 0x309 / 0x3a9 / 0x3e9.

#define TESLA_AP1_FLAG_LONG_CONTROL 2U   // BogGyver/Tinkla FLAG_TESLA_LONG_CONTROL
#define TESLA_AP1_FLAG_HAS_AP 16U        // BogGyver/Tinkla FLAG_TESLA_HAS_AP

#define TESLA_AP1_STEER_SUBSTITUTE_TIMEOUT_US 100000U  // 100 ms, BogPilot value (openpilot sends 0x488 at 50 Hz)
#define TESLA_AP1_LONG_SUBSTITUTE_TIMEOUT_US 50000U    // 50 ms, BogPilot value (0x2b9 follows the stock ~40 Hz counter)

static bool tesla_ap1_longitudinal = false;
static bool tesla_ap1_stock_aeb = false;

// Interceptor: a stock DAS frame from bus 2 is dropped only while openpilot has recently
// transmitted its own copy on bus 0. Otherwise stock 0x488 / 0x2b9 keep flowing.
static uint32_t tesla_ap1_last_steer_tx_ts = 0U;
static uint32_t tesla_ap1_last_long_tx_ts = 0U;
static bool tesla_ap1_steer_tx_seen = false;
static bool tesla_ap1_long_tx_seen = false;

// AP1 cluster substitution, per address. BogPilot TESLA_AP1_CLUSTER_* values.
#define TESLA_AP1_CLUSTER_LEN 3
static uint32_t tesla_ap1_cluster_tx_ts[TESLA_AP1_CLUSTER_LEN] = {0U, 0U, 0U};
static bool tesla_ap1_cluster_tx_seen[TESLA_AP1_CLUSTER_LEN] = {false, false, false};

static int tesla_ap1_cluster_index(int addr) {
  // 0x399 AutopilotStatus, 0x389 DAS_status2, 0x239 DAS_lanes
  static const int TESLA_AP1_CLUSTER_ADDRS[TESLA_AP1_CLUSTER_LEN] = {0x399, 0x389, 0x239};
  int idx = -1;
  for (int i = 0; i < TESLA_AP1_CLUSTER_LEN; i++) {
    if (TESLA_AP1_CLUSTER_ADDRS[i] == addr) {
      idx = i;
    }
  }
  return idx;
}

static bool tesla_ap1_op_recently_sent(uint32_t last_ts, bool seen, uint32_t timeout_us) {
  bool recent = false;
  if (seen) {
    recent = safety_get_ts_elapsed(microsecond_timer_get(), last_ts) < timeout_us;
  }
  return recent;
}

static void tesla_ap1_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    // EPAS_sysStatus: EPAS_internalSAS, (0.1 * val) - 819.2 deg. Stored in 1/10 deg like the steering request.
    if (msg->addr == 0x370U) {
      const int angle_meas_new = (((msg->data[4] & 0x3FU) << 8) | msg->data[5]) - 8192U;
      update_sample(&angle_meas, angle_meas_new);
    }

    // DI_torque2: DI_vehicleSpeed, ((0.05 * val) - 25) mph
    if (msg->addr == 0x118U) {
      float speed = (((((msg->data[3] & 0x0FU) << 8) | msg->data[2]) * 0.05) - 25.) * 0.447;
      vehicle_moving = SAFETY_ABS(speed) > 0.1;
      UPDATE_VEHICLE_SPEED(speed);
    }

    // DI_torque1: DI_pedalPos
    if (msg->addr == 0x108U) {
      gas_pressed = msg->data[6] != 0U;
    }

    // BrakeMessage: driverBrakeStatus, 1 is NOT_APPLIED
    if (msg->addr == 0x20aU) {
      brake_pressed = ((msg->data[0] & 0x0CU) >> 2) != 1U;
    }

    // DI_state: DI_cruiseState
    if (msg->addr == 0x368U) {
      int cruise_state = msg->data[1] >> 4;

      acc_main_on = (cruise_state == 1) ||  // STANDBY
                    (cruise_state == 2) ||  // ENABLED
                    (cruise_state == 3) ||  // STANDSTILL
                    (cruise_state == 4) ||  // OVERRIDE
                    (cruise_state == 6) ||  // PRE_FAULT
                    (cruise_state == 7);    // PRE_CANCEL

      bool cruise_engaged = (cruise_state == 2) ||  // ENABLED
                            (cruise_state == 3) ||  // STANDSTILL
                            (cruise_state == 4) ||  // OVERRIDE
                            (cruise_state == 6) ||  // PRE_FAULT
                            (cruise_state == 7);    // PRE_CANCEL
      pcm_cruise_check(cruise_engaged);
    }
  }

  if (msg->bus == 2U) {
    // DAS_control: DAS_aebEvent == AEB_ACTIVE
    if (msg->addr == 0x2b9U) {
      tesla_ap1_stock_aeb = (msg->data[2] & 0x03U) == 1U;
    }
  }
}

static bool tesla_ap1_tx_hook(const CANPacket_t *msg) {
  // Tinkla earlytesla-panda TESLA_LOOKUP_ANGLE_RATE_UP / _DOWN, TESLA_DEG_TO_CAN 10.
  // max_angle is the EPAS_internalSAS range so the inactive check does not clip a NONE frame.
  const AngleSteeringLimits TESLA_AP1_STEERING_LIMITS = {
    .max_angle = 8192,  // 819.2 deg
    .angle_deg_to_can = 10,
    .angle_rate_up_lookup = {
      {2., 7., 17.},
      {8., 4., 2.5}
    },
    .angle_rate_down_lookup = {
      {2., 7., 17.},
      {9., 5., 4.5}
    },
    .frequency = 50U,
  };

  const LongitudinalLimits TESLA_AP1_LONG_LIMITS = {
    .max_accel = 425,       // 2 m/s^2
    .min_accel = 287,       // -3.52 m/s^2, BogPilot value
    .inactive_accel = 375,  // 0. m/s^2
  };

  bool tx = true;
  bool violation = false;

  // DAS_steeringControl: (0.1 * val) - 1638.35 deg, 1/10 deg units here
  if (msg->addr == 0x488U) {
    int raw_angle_can = ((msg->data[0] & 0x7FU) << 8) | msg->data[1];
    int desired_angle = raw_angle_can - 16384;
    int steer_control_type = msg->data[2] >> 6;
    bool steer_control_enabled = steer_control_type == 1;  // ANGLE_CONTROL

    if (steer_angle_cmd_checks(desired_angle, steer_control_enabled, TESLA_AP1_STEERING_LIMITS)) {
      violation = true;
    }

    // NONE (0) or ANGLE_CONTROL (1) only
    if ((steer_control_type != 0) && (steer_control_type != 1)) {
      violation = true;
    }
  }

  // STW_ACTN_RQ: only the cancel lever position may be sent
  if (msg->addr == 0x45U) {
    int control_lever_status = msg->data[0] & 0x3FU;
    if (control_lever_status != 1) {
      violation = true;
    }
  }

  // DAS_control (chassis): longitudinal only
  if (msg->addr == 0x2b9U) {
    if (tesla_ap1_longitudinal) {
      // No AEB events may be sent by openpilot
      int aeb_event = msg->data[2] & 0x03U;
      if (aeb_event != 0) {
        violation = true;
      }

      // Don't send while the stock AEB system is active
      if (tesla_ap1_stock_aeb) {
        violation = true;
      }

      int raw_accel_max = ((msg->data[6] & 0x1FU) << 4) | (msg->data[5] >> 4);
      int raw_accel_min = ((msg->data[5] & 0x0FU) << 5) | (msg->data[4] >> 3);

      // Both limits negative could reverse the car after a stop
      if ((raw_accel_max < TESLA_AP1_LONG_LIMITS.inactive_accel) && (raw_accel_min < TESLA_AP1_LONG_LIMITS.inactive_accel)) {
        violation = true;
      }

      violation |= longitudinal_accel_checks(raw_accel_max, TESLA_AP1_LONG_LIMITS);
      violation |= longitudinal_accel_checks(raw_accel_min, TESLA_AP1_LONG_LIMITS);
    } else {
      violation = true;
    }
  }

  // 0x349 Tinkla DAS warning matrix 3. Only the all-zero Hold clear (DAS_gas_to_resume = 0).
  if (msg->addr == 0x349U) {
    if (!tesla_ap1_longitudinal) {
      violation = true;
    }
    for (int i = 0; i < 8; i++) {
      if (msg->data[i] != 0U) {
        violation = true;
      }
    }
  }

  // 0x399 AutopilotStatus (cluster): an active autopilot state (3, 4, 5) needs controls allowed.
  // autopilotStatus is bits 0-3. 0x389 / 0x239 carry no state openpilot could fake.
  if (msg->addr == 0x399U) {
    int autopilot_state = msg->data[0] & 0x0FU;
    if ((autopilot_state >= 3) && (autopilot_state <= 5) && !controls_allowed) {
      violation = true;
    }
  }

  if (violation) {
    tx = false;
  }

  // Remember allowed openpilot copies so the forward hook drops the matching stock frame
  if (tx) {
    if (msg->addr == 0x488U) {
      tesla_ap1_last_steer_tx_ts = microsecond_timer_get();
      tesla_ap1_steer_tx_seen = true;
    }
    if (msg->addr == 0x2b9U) {
      tesla_ap1_last_long_tx_ts = microsecond_timer_get();
      tesla_ap1_long_tx_seen = true;
    }
    int idx = tesla_ap1_cluster_index((int)msg->addr);
    if (idx >= 0) {
      tesla_ap1_cluster_tx_ts[idx] = microsecond_timer_get();
      tesla_ap1_cluster_tx_seen[idx] = true;
    }
  }

  return tx;
}

static bool tesla_ap1_fwd_hook(int bus_num, int addr) {
  // Stock-drop window per cluster address: 1.5x the stock period (2 Hz, 2 Hz, 10 Hz in AP1 rlogs)
  static const uint32_t TESLA_AP1_CLUSTER_TIMEOUT_US[TESLA_AP1_CLUSTER_LEN] = {750000U, 750000U, 150000U};
  bool block_msg = false;

  // Autopilot to chassis. Bus 0 to 2 is always forwarded.
  if (bus_num == 2) {
    if (addr == 0x488) {
      block_msg = tesla_ap1_op_recently_sent(tesla_ap1_last_steer_tx_ts, tesla_ap1_steer_tx_seen,
                                             TESLA_AP1_STEER_SUBSTITUTE_TIMEOUT_US);
    }

    if (tesla_ap1_longitudinal && (addr == 0x2b9) && !tesla_ap1_stock_aeb) {
      block_msg = tesla_ap1_op_recently_sent(tesla_ap1_last_long_tx_ts, tesla_ap1_long_tx_seen,
                                             TESLA_AP1_LONG_SUBSTITUTE_TIMEOUT_US);
    }

    // Cluster: same rule, per address. Stock frames flow unless openpilot sent its copy recently.
    int idx = tesla_ap1_cluster_index(addr);
    if (idx >= 0) {
      block_msg = tesla_ap1_op_recently_sent(tesla_ap1_cluster_tx_ts[idx], tesla_ap1_cluster_tx_seen[idx],
                                             TESLA_AP1_CLUSTER_TIMEOUT_US[idx]);
    }
  }

  return block_msg;
}

static safety_config tesla_ap1_init(uint16_t param) {
  // 0x488 is checked for relay malfunction on bus 0 (BogPilot generic_rx_checks), but not statically
  // blocked: the forward hook substitutes it. 0x45 comes from the stalk on bus 0, so no relay check.
  static const CanMsg TESLA_AP1_TX_MSGS[] = {
    {0x488, 0, 4, .check_relay = true, .disable_static_blocking = true},  // DAS_steeringControl
    {0x45, 0, 8, .check_relay = false},                                   // STW_ACTN_RQ (cancel)
    {0x45, 2, 8, .check_relay = false},                                   // STW_ACTN_RQ (cancel)
    {0x2b9, 0, 8, .check_relay = false},                                  // DAS_control (long only)
    {0x349, 0, 8, .check_relay = false},                                  // Hold clear (long only, all zero)
    {0x399, 0, 8, .check_relay = false},                                  // AutopilotStatus (cluster)
    {0x389, 0, 8, .check_relay = false},                                  // DAS_status2 (cluster)
    {0x239, 0, 8, .check_relay = false},                                  // DAS_lanes (cluster)
  };

  // BogPilot tesla_rx_checks. tesla_can.dbc frames carry no counter/checksum the old panda checked.
  static RxCheck tesla_ap1_rx_checks[] = {
    {.msg = {{0x2b9, 2, 8, 25U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // DAS_control
    {.msg = {{0x370, 0, 8, 25U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // EPAS_sysStatus
    {.msg = {{0x108, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // DI_torque1
    {.msg = {{0x118, 0, 6, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // DI_torque2
    {.msg = {{0x20a, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // BrakeMessage
    {.msg = {{0x368, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // DI_state
    {.msg = {{0x318, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // GTW_carState
  };

  // BogGyver/Tinkla honors FLAG_TESLA_LONG_CONTROL in every build. That panda predates comma's
  // ALLOW_DEBUG gate on Tesla longitudinal; it is not an AP1-specific safety argument, so the gate stays.
  // A release-signed panda therefore keeps stock ACC (0x2b9 TX rejected, stock 0x2b9 forwarded).
  tesla_ap1_longitudinal = false;
#ifdef ALLOW_DEBUG
  tesla_ap1_longitudinal = GET_FLAG(param, TESLA_AP1_FLAG_LONG_CONTROL);
#else
  SAFETY_UNUSED(param);
#endif

  tesla_ap1_stock_aeb = false;
  tesla_ap1_steer_tx_seen = false;
  tesla_ap1_long_tx_seen = false;
  tesla_ap1_last_steer_tx_ts = 0U;
  tesla_ap1_last_long_tx_ts = 0U;
  for (int i = 0; i < TESLA_AP1_CLUSTER_LEN; i++) {
    tesla_ap1_cluster_tx_ts[i] = 0U;
    tesla_ap1_cluster_tx_seen[i] = false;
  }

  safety_config ret;
  SET_TX_MSGS(TESLA_AP1_TX_MSGS, ret);
  SET_RX_CHECKS(tesla_ap1_rx_checks, ret);
  return ret;
}
