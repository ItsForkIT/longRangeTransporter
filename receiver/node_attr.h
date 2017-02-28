/*
 * File Name : node_attr.h
 * Author : ram krishnan
 * Created : april/22/2015
 */

#ifndef __NODE_ATTR_H__
#define __NODE_ATTR_H__

typedef enum
{
    MAC_ACK_TMO_DELTA_ATTR_ID = 1,
    MAC_CCA_FLR_CNT_ATTR_ID = 2,
    MAC_CSMA_FLR_CNT_ATTR_ID = 3,
    MAC_ACK_TMO_CNT_ATTR_ID = 4,
    MAC_CCA_OK_CNT_ATTR_ID = 5,
    MAC_VALID_ACK_RCVD_CNT_ATTR_ID = 6,
    MAC_LAST_CSMA_BACK_OFF_ATTR_ID = 7,
    MAC_TX_ON_CCA_ENA_FLAG_ATTR_ID = 8,     // Enable / Disable TX_ON_CCA on a node
    MAC_LPWMN_ID_ATTR_ID = 9,
    PHY_RF_CHANN_ID_ATTR_ID = 10,
    APP_SENSOR_DATA_PUSH_INTERVAL_SECS_ATTR_ID = 11,
    FW_BUILD_DATE_ATTR_ID = 12,
    FW_BUILD_TIME_ATTR_ID = 13,
    SYS_FREE_MEM_DESC_CNT_ATTR_ID = 14,
    MAC_ACK_FRAME_RAW_TRX_TIME_ATTR_ID = 15,

    FU_IMAGE_STORE_IMAGE_FLAGS_ATTR_ID = 16,
    FU_IMAGE_STORE_IMAGE_INFO_ATTR_ID = 17,

    FU_IMAGE_STORE_IMAGE_1_LEN_ATTR_ID = 20,
    FU_IMAGE_STORE_IMAGE_1_SEG_CNT_ATTR_ID = 21,
    FU_IMAGE_STORE_IMAGE_1_TYPE_ATTR_ID = 22,
    FU_IMAGE_STORE_IMAGE_1_IMAGE_RCVD_TIME_STAMP_ATTR_ID = 23,

    FU_IMAGE_STORE_IMAGE_2_LEN_ATTR_ID = 40,
    FU_IMAGE_STORE_IMAGE_2_SEG_CNT_ATTR_ID = 41,
    FU_IMAGE_STORE_IMAGE_2_TYPE_ATTR_ID = 42,
    FU_IMAGE_STORE_IMAGE_2_IMAGE_RCVD_TIME_STAMP_ATTR_ID = 43,

    SYS_UP_TIME_IN_SECONDS_ATTR_ID = 50,

    MAC_BAD_UC_SA_RCVD_CNT_ATTR_ID = 55,
    MAC_GOOD_UC_SA_RCVD_CNT_ATTR_ID = 56,

    MAC_NWK_TYPE_ATTR_ID = 58,

    NWK_ROUTING_TABLE_MAX_ENTRY_CNT_ATTR_ID = 60,
    NWK_ROUTING_TABLE_VALID_ENTRY_CNT_ATTR_ID = 61,

    AODVL_RT_DISCOVERY_TABLE_MAX_ENTRY_CNT_ATTR_ID = 62,
    AODVL_RT_DISCOVERY_TABLE_VALID_ENTRY_CNT_ATTR_ID = 63,
    AODVL_RT_REQ_MSG_RCVD_CNT_ATTR_ID = 64,
    AODVL_RT_REP_MSG_RCVD_CNT_ATTR_ID = 65,
    AODVL_RT_REQ_MSG_TRX_CNT_ATTR_ID = 66,
    AODVL_RT_REP_MSG_TRX_CNT_ATTR_ID = 67,
    AODVL_RT_COST_TO_LPWMN_COORD_ATTR_ID = 68,
    AODVL_WEAK_LINK_COUNT_TO_LPWMN_COORD_ATTR_ID = 69,
    MAC_RT_PURGE_REQUEST_ON_NO_ACK_ATTR_ID  = 70,

    NWK_ROUTING_TABLE_ENTRY_ATTR_ID = 100,
    AODVL_RT_DISCOVERY_TABLE_ENTRY_ATTR_ID = 101,

    AODVL_RT_DISCOVERY_TABLE_FULL_CNT_ATTR_ID = 102,

    NWK_RT_DISC_STARTED_CNT_ATTR_ID = 120,
    NWK_RT_DISC_SUCCESS_CNT_ATTR_ID = 121,
    NWK_RT_DISC_FLR_CNT_ATTR_ID = 122,
    NWK_RT_DISC_ERR_CNT_ATTR_ID = 123,

    MAC_BCN_REQ_RCVD_CNT_ATTR_ID = 124,
    MAC_TIME_SINCE_LAST_BCN_REQ_RCVD_ATTR_ID = 125,
    MAC_LAST_BCN_REQ_SRC_EXT_ADDR_ATTR_ID = 126,
    MAC_BCN_TX_ATTEMPT_CNT_ATTR_ID = 127,

    MAC_ACK_TX_CNT_ATTR_ID = 200,

    SYS_MEM_ALLOC_SUCCESS_CNT_ATTR_ID = 300,
    SYS_BD_ALLOC_FLR_CNT_ATTR_ID = 301,
    SYS_BUFF_ALLOC_FLR_CNT_ATTR_ID = 302,
    SYS_MEM_FREE_CNT_ATTR_ID = 303,
    SYS_REBOOT_DELAY_ATTR_ID = 304,

    ADP_MESH_BC_PKT_RCVD_CNT_ATTR_ID = 400,
    ADP_MESH_DUP_BC_PKT_DROPPED_CNT_ATTR_ID = 401,
    NWK_MESH_LOOP_DETECTED_CNT_ATTR_ID = 402,
    ADP_APP_PYLD_TX_PROC_FLR_CNT_ATTR_ID = 403,
    ADP_RELAYED_ASSOC_RESP_RCVD_CNT_ATTR_ID =  404,

    MAC_ASSOC_REQ_RECEIVED_CNT_ATTR_ID = 410,
    MAC_ASSOC_RESP_TRX_CNT_ATTR_ID = 411,
    MAC_ASSOC_REQ_RELAYED_CNT_ATTR_ID = 412,
    MAC_MAX_RFD_SUPPORT_CNT_PER_FFD_ATTR_ID = 415,
    MAC_FFD_ASSOC_RFD_CNT_ATTR_ID = 416,

    MAC_FFD_ASSOC_RFD_TABLE_ENTRY_ATTR_ID = 417,

    MAC_DUPLICATE_PKT_RECEIVED_CNT_ATTR_ID  = 420,
    MAC_PKT_TRANSMIT_ABORT_CNT_ATTR_ID = 421,

    MAC_ACK_PDU_COORD_ADDR_COMPRESS_CFG_ATTR_ID = 425,
    MAC_LPWMN_ID_ENA_DIS_CFG_ATTR_ID = 426,

    APP_NO_OP_CMD_RCVD_CNT_ATTR_ID = 500,

    APP_DATA_CONFIRM_SUCCESS_CNT_ATTR_ID = 510,
    APP_DATA_CONFIRM_FAILURE_CNT_ATTR_ID = 511,

    APP_ADP_DATA_REQ_CNT_ATTR_ID = 512,
    APP_ADP_DATA_REQ_FLR_CNT_ATTR_ID =  513,

    APP_SENSOR_DATA_TX_INTERVAL_ATTR_ID = 514,

    APP_MAX_SENSOR_DATA_TX_INTERVAL_ATTR_ID =  515,

    FFD_LAST_APP_TX_TO_RBT_INTERVAL_SECS_ATTR_ID  = 550,
    FFD_LAST_APP_TX_TO_RBT_INTERVAL_MULTIPLE_ATTR_ID = 551,

    MESH_TRIGGER_PATH_DISC_ATTR_ID = 600,

    NWK_TRIGGER_RT_TABLE_PURGE_ATTR_ID = 601,

    APP_PIR_SENSOR_ENA_DIS_CTRL_ATTR_ID = 700,  // read/write

    APP_VEH_DET_MFS_HPF_DETECTION_THRESHOLD = 750,  // read/write
    APP_VEH_DET_MFS_HPF_SETTLING_THRESHOLD = 751,
    APP_VEH_DET_MFS_HPF_DETECTION_ALPHA = 752,

    DEV_PULSE_CNTR_RESET_CNTR  =  755,
    DEV_PULSE_CNTR_SET_CNTR  =  756,

    AD7797_SYSTEM_ZERO_SCALE_CAL_REQ_ATTR_ID = 760,  // action
    AD7797_SYSTEM_FULL_SCALE_CAL_REQ_ATTR_ID = 761,  // action
    AD7797_SYSTEM_ZERO_SCALE_CAL_REQ_CNT_ATTR_ID = 762,
    AD7797_SYSTEM_ZERO_SCALE_CAL_DONE_CNT_ATTR_ID = 763,
    AD7797_OFFSET_REG_VAL_ATTR_ID = 764,

    GW_LPWMN_MSGS_RCVD_CNT_ATTR_ID = 800,   // Coord  only

    APP_MAG3110_X_AXIS_BASE_VAL_MFS_ATTR_ID = 900,
    APP_MAG3110_Y_AXIS_BASE_VAL_MFS_ATTR_ID = 901,
    APP_MAG3110_Z_AXIS_BASE_VAL_MFS_ATTR_ID = 902,


    APP_SNSR_DATA_REPORT_MODE_ATTR_ID = 1000,
    APP_MAX_SNSR_DATA_REPORT_INTERVAL_SECS_ATTR_ID  = 1001,
    APP_SNSR_DATA_REPORT_DELTA_THRESHOLD_ATTR_ID  = 1002,
    FFD_APP_MAX_SENSOR_RPT_SKIP_COUNT = 1003,

    APP_RG11_RS_MAX_REPORT_INTERVAL_RAINING_ATTR_ID = 1010,
    APP_RG11_RS_MAX_REPORT_INTERVAL_NOT_RAINING_ATTR_ID = 1011,

    NM_CHANGE_RF_CHANN_FOR_FU_ATTR_ID = 2000,

    // rkris@wisense.in / nov/18/2016
    NM_INTER_SCAN_INTERVAL_SECS_ATTR_ID = 2010,
    NM_MAX_SCAN_FLR_CNT_TO_RESET_ATTR_ID = 2011,

    RADIO_RX_OVFL_CNT_ATTR_ID  = 3000,
    RADIO_CC1101_FSM_STATE_ATTR_ID = 3001,
    MAC_TX_MOD_STATE_ATTR_ID = 3002,
    RADIO_TX_INTERRUPT_FLAG_ATTR_ID = 3003,
    RADIO_RX_INTERRUPT_FLAG_ATTR_ID = 3004,
    PLTFRM_GLOBAL_INT_STATE_ATTR_ID = 3005,
    RADIO_RX_INT_CNT_ATTR_ID = 3006,
    RADIO_TX_INT_CNT_ATTR_ID = 3007,
    RADIO_RX_FIFO_BYTE_CNT_ATTR_ID = 3008,
    RADIO_RESTART_RX_ON_OVFL_CNT_ATTR_ID =  3009,
    RADIO_OVFL_RX_FIFO_MIN_BYTE_CNT_ATTR_ID = 3010,
    RADIO_OVFL_RX_FIFO_MAX_BYTE_CNT_ATTR_ID = 3011,
    CC1101_RX_OVFL_BUG_CNT_ATTR_ID = 3012,
    RADIO_BAD_CRC_PKT_RX_CNT_ATTR_ID = 3013,
    RADIO_GOOD_CRC_PKT_RX_CNT_ATTR_ID = 3014,
    RADIO_TX_TO_RX_TURN_AROUND_DELAY_ATTR_ID = 3015,
    RADIO_FREQ_OFFSET_ATTR_ID = 3016,

    RADIO_RF_TX_TEST_CHANN_ID_ATTR_ID = 3020,
    RADIO_RF_TX_TEST_PA_CFG_ATTR_ID = 3021,
    RADIO_START_CW_UNMOD_TX_TEST_ATTR_ID = 3022,
    RADIO_START_CW_MOD_TX_TEST_ATTR_ID = 3023,

    PHY_RAW_BAUD_RATE_ATTR_ID = 3100,

} NM_attrId_t;

#endif
