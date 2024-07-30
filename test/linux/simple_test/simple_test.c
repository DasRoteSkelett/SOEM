/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "ethercat.h"

#define EC_TIMEOUTMON 500

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
boolean forceByteAlignment = FALSE;

typedef struct __attribute__((packed,aligned(1))) fsoe_safety_master_message_14_bytes_out_T {
  uint8_t fsoe_master_command; /* FSoE SafetyMasterMessage 14 Bytes Out *Start* */
  uint8_t fsoe_data1_0 : 1;
  uint8_t fsoe_data1_1 : 1;
  uint8_t fsoe_data1_2 : 1;
  uint8_t fsoe_data1_3 : 1;
  uint8_t fsoe_data1_4 : 1;
  uint8_t fsoe_data1_5 : 1;
  uint8_t fsoe_data1_6 : 1;
  uint8_t fsoe_data1_7 : 1;
  uint8_t fsoe_data1_8 : 1;
  uint8_t fsoe_data1_9 : 1;
  uint8_t fsoe_data1_10 : 1;
  uint8_t fsoe_data1_11 : 1;
  uint8_t fsoe_data1_12 : 1;
  uint8_t fsoe_data1_13 : 1;
  uint8_t fsoe_data1_14 : 1;
  uint8_t fsoe_data1_15 : 1;
  uint16_t fsoe_master_crc_0;
  uint8_t fsoe_data1_16 : 1;
  uint8_t fsoe_data1_17 : 1;
  uint8_t fsoe_data1_18 : 1;
  uint8_t fsoe_data1_19 : 1;
  uint8_t fsoe_data1_20 : 1;
  uint8_t fsoe_data1_21 : 1;
  uint8_t fsoe_data1_22 : 1;
  uint8_t fsoe_data1_23 : 1;
  uint8_t fsoe_data1_24 : 1;
  uint8_t fsoe_data1_25 : 1;
  uint8_t fsoe_data1_26 : 1;
  uint8_t fsoe_data1_27 : 1;
  uint8_t fsoe_data1_28 : 1;
  uint8_t fsoe_data1_29 : 1;
  uint8_t fsoe_data1_30 : 1;
  uint8_t fsoe_data1_31 : 1;
  uint16_t fsoe_master_crc_1;
  uint8_t fsoe_data1_32 : 1;
  uint8_t fsoe_data1_33 : 1;
  uint8_t fsoe_data1_34 : 1;
  uint8_t fsoe_data1_35 : 1;
  uint8_t fsoe_data1_36 : 1;
  uint8_t fsoe_data1_37 : 1;
  uint8_t fsoe_data1_38 : 1;
  uint8_t fsoe_data1_39 : 1;
  uint8_t fsoe_data1_40 : 1;
  uint8_t fsoe_data1_41 : 1;
  uint8_t fsoe_data1_42 : 1;
  uint8_t fsoe_data1_43 : 1;
  uint8_t fsoe_data1_44 : 1;
  uint8_t fsoe_data1_45 : 1;
  uint8_t fsoe_data1_46 : 1;
  uint8_t fsoe_data1_47 : 1;
  uint16_t fsoe_master_crc_2;
  uint8_t fsoe_data1_48 : 1;
  uint8_t fsoe_data1_49 : 1;
  uint8_t fsoe_data1_50 : 1;
  uint8_t fsoe_data1_51 : 1;
  uint8_t fsoe_data1_52 : 1;
  uint8_t fsoe_data1_53 : 1;
  uint8_t fsoe_data1_54 : 1;
  uint8_t fsoe_data1_55 : 1;
  uint8_t fsoe_data1_56 : 1;
  uint8_t fsoe_data1_57 : 1;
  uint8_t fsoe_data1_58 : 1;
  uint8_t fsoe_data1_59 : 1;
  uint8_t fsoe_data1_60 : 1;
  uint8_t fsoe_data1_61 : 1;
  uint8_t fsoe_data1_62 : 1;
  uint8_t fsoe_data1_63 : 1;
  uint16_t fsoe_master_crc_3;
  uint8_t fsoe_data1_64 : 1;
  uint8_t fsoe_data1_65 : 1;
  uint8_t fsoe_data1_66 : 1;
  uint8_t fsoe_data1_67 : 1;
  uint8_t fsoe_data1_68 : 1;
  uint8_t fsoe_data1_69 : 1;
  uint8_t fsoe_data1_70 : 1;
  uint8_t fsoe_data1_71 : 1;
  uint8_t fsoe_data1_72 : 1;
  uint8_t fsoe_data1_73 : 1;
  uint8_t fsoe_data1_74 : 1;
  uint8_t fsoe_data1_75 : 1;
  uint8_t fsoe_data1_76 : 1;
  uint8_t fsoe_data1_77 : 1;
  uint8_t fsoe_data1_78 : 1;
  uint8_t fsoe_data1_79 : 1;
  uint16_t fsoe_master_crc_4;
  uint8_t fsoe_data1_80 : 1;
  uint8_t fsoe_data1_81 : 1;
  uint8_t fsoe_data1_82 : 1;
  uint8_t fsoe_data1_83 : 1;
  uint8_t fsoe_data1_84 : 1;
  uint8_t fsoe_data1_85 : 1;
  uint8_t fsoe_data1_86 : 1;
  uint8_t fsoe_data1_87 : 1;
  uint8_t fsoe_data1_88 : 1;
  uint8_t fsoe_data1_89 : 1;
  uint8_t fsoe_data1_90 : 1;
  uint8_t fsoe_data1_91 : 1;
  uint8_t fsoe_data1_92 : 1;
  uint8_t fsoe_data1_93 : 1;
  uint8_t fsoe_data1_94 : 1;
  uint8_t fsoe_data1_95 : 1;
  uint16_t fsoe_master_crc_5;
  uint8_t fsoe_data1_96 : 1;
  uint8_t fsoe_data1_97 : 1;
  uint8_t fsoe_data1_98 : 1;
  uint8_t fsoe_data1_99 : 1;
  uint8_t fsoe_data1_100 : 1;
  uint8_t fsoe_data1_101 : 1;
  uint8_t fsoe_data1_102 : 1;
  uint8_t fsoe_data1_103 : 1;
  uint8_t fsoe_data1_104 : 1;
  uint8_t fsoe_data1_105 : 1;
  uint8_t fsoe_data1_106 : 1;
  uint8_t fsoe_data1_107 : 1;
  uint8_t fsoe_data1_108 : 1;
  uint8_t fsoe_data1_109 : 1;
  uint8_t fsoe_data1_110 : 1;
  uint8_t fsoe_data1_111 : 1;
  uint16_t fsoe_master_crc_6;
  uint16_t fsoe_master_connection_id;  /* FSoE SafetyMasterMessage 14 Bytes Out *End* */
} fsoe_safety_master_message_14_bytes_out_t;


typedef struct __attribute__((packed,aligned(1)))  master_to_master_communication_inputs_T {
  uint64_t smmc_inputs1;
  uint64_t smmc_inputs2;
  uint64_t smmc_inputs3;
  uint64_t smmc_inputs4;
} master_to_master_communication_inputs_t;

typedef struct __attribute__((packed,aligned(1))) functional_inputs_T {
  uint8_t byte_in_0;  /* Functional Inputs Start */
  uint8_t byte_in_1;
  uint8_t byte_in_2;
  uint8_t byte_in_3;
  uint8_t byte_in_4;
  uint8_t byte_in_5;
  uint8_t byte_in_6;
  uint8_t byte_in_7;
  uint8_t byte_in_8;
  uint8_t byte_in_9;
  uint8_t byte_in_10;
  uint8_t byte_in_11;
  uint8_t byte_in_12;
  uint8_t byte_in_13;
  uint8_t byte_in_14;
  uint8_t byte_in_15;
  uint8_t byte_in_16;
  uint8_t byte_in_17;  /* Functional Inputs End */
} functional_inputs_t;

typedef struct __attribute__((packed,aligned(1))) fsoe_safety_slave_message_4_bytes_in_T {
  uint8_t fsoe_slave_command; /* FSoE SafetySlaveMessage 04 Bytes In *Start* */
  uint8_t fsoe_data1_0 : 1;
  uint8_t fsoe_data1_1 : 1;
  uint8_t fsoe_data1_2 : 1;
  uint8_t fsoe_data1_3 : 1;
  uint8_t fsoe_data1_4 : 1;
  uint8_t fsoe_data1_5 : 1;
  uint8_t fsoe_data1_6 : 1;
  uint8_t fsoe_data1_7 : 1;
  uint8_t fsoe_data1_8 : 1;
  uint8_t fsoe_data1_9 : 1;
  uint8_t fsoe_data1_10 : 1;
  uint8_t fsoe_data1_11 : 1;
  uint8_t fsoe_data1_12 : 1;
  uint8_t fsoe_data1_13 : 1;
  uint8_t fsoe_data1_14 : 1;
  uint8_t fsoe_data1_15 : 1;
  uint16_t fsoe_slave_crc_0;
  uint8_t fsoe_data1_16 : 1;
  uint8_t fsoe_data1_17 : 1;
  uint8_t fsoe_data1_18 : 1;
  uint8_t fsoe_data1_19 : 1;
  uint8_t fsoe_data1_20 : 1;
  uint8_t fsoe_data1_21 : 1;
  uint8_t fsoe_data1_22 : 1;
  uint8_t fsoe_data1_23 : 1;
  uint8_t fsoe_data1_24 : 1;
  uint8_t fsoe_data1_25 : 1;
  uint8_t fsoe_data1_26 : 1;
  uint8_t fsoe_data1_27 : 1;
  uint8_t fsoe_data1_28 : 1;
  uint8_t fsoe_data1_29 : 1;
  uint8_t fsoe_data1_30 : 1;
  uint8_t fsoe_data1_31 : 1;
  uint16_t fsoe_slave_crc_1;
  uint16_t fsoe_connection_id; /* FSoE SafetySlaveMessage 04 Bytes In *End* */
} fsoe_safety_slave_message_4_bytes_in_t;

typedef struct __attribute__((packed,aligned(1))) master_to_master_communication_outputs_T {
  uint64_t smmc_outputs;
} master_to_master_communication_outputs_t;

typedef struct __attribute__((packed,aligned(1))) functional_outputs_T {
  uint16_t status_word; /* Functional Outputs Start */
  uint8_t debug_0;
  uint8_t debug_1;
  uint8_t debug_2;
  uint8_t debug_3;
  uint8_t debug_4;
  uint8_t byte_out_0;
  uint8_t byte_out_1;
  uint8_t byte_out_2;
  uint8_t byte_out_3;
  uint8_t byte_out_4;
  uint8_t byte_out_5;
  uint8_t byte_out_6;
  uint8_t byte_out_7;
  uint8_t byte_out_8;
  uint8_t byte_out_9;
  uint8_t byte_out_10;
  uint8_t byte_out_11;
  uint8_t byte_out_12;
  uint8_t byte_out_13;
  uint8_t byte_out_14;
  uint8_t byte_out_15;
  uint8_t byte_out_16;
  uint8_t byte_out_17;
  uint8_t byte_out_18;
  uint8_t byte_out_19;
  uint8_t byte_out_20;
  uint8_t byte_out_21;
  uint8_t byte_out_22;
  uint8_t byte_out_23;
  uint8_t byte_out_24;
  uint8_t byte_out_25;
  uint8_t byte_out_26;
} functional_outputs_t;


typedef struct __attribute__((packed,aligned(1))) slave_0_pdo_mapping_t {
  fsoe_safety_slave_message_4_bytes_in_t fsoe_safety_slave_message_4_bytes_in;
  uint8_t alignment_byte_conn1;
  master_to_master_communication_outputs_t master_to_master_communication_outputs;
  functional_outputs_t functional_outputs;
  fsoe_safety_master_message_14_bytes_out_t fsoe_safety_master_message_14_bytes_out;
  uint8_t alignment_byte_conn1_1;
  master_to_master_communication_inputs_t master_to_master_communications_inputs;
  functional_inputs_t functional_inputs;
} bbh_slave_mapping;

typedef struct __attribute__((packed,aligned(1))) bbh_write_mapping_T {
  fsoe_safety_master_message_14_bytes_out_t fsoe_safety_master_message_14_bytes_out;
  uint8_t alignment_byte_conn1_1;
  master_to_master_communication_inputs_t master_to_master_communications_inputs;
  functional_inputs_t functional_inputs;
} bbh_write_mapping_t;

typedef struct __attribute__((packed,aligned(1))) bbh_read_mapping_T {
  fsoe_safety_slave_message_4_bytes_in_t fsoe_safety_slave_message_4_bytes_in;
  uint8_t alignment_byte_conn1;
  master_to_master_communication_outputs_t master_to_master_communication_outputs;
  functional_outputs_t functional_outputs;
} bbh_read_mapping_t;

typedef struct __attribute__((packed,aligned(1))) cia402_rx_T {
  uint16_t control_word;
  uint8_t modes_of_operation;
  int16_t target_torque;
  uint32_t target_position;
  int32_t target_velocity;
  uint16_t torque_offset;
  uint32_t tuning_command; /* RxPDO Mapping 1 End */
} cia_402_rx_t;

typedef struct __attribute__((packed,aligned(1))) cia_402_rx2_T {
  uint32_t pysical_output;
  uint32_t bit_mask;
} cia_402_rx2_t;

typedef struct __attribute__((packed,aligned(1))) cia_402_rx3_T {
  uint32_t user_mosi;
  uint32_t velocity_offsets;
} cia_402_rx3_t;

typedef struct __attribute__((packed,aligned(1))) plc_to_drive_T {
  uint8_t fsoe_command;
  uint8_t fsoe_sto :1;
  uint8_t fsoe_ss1 :1;
  uint8_t fsoe_ss2 :1;
  uint8_t fsoe_sos :1;
  uint8_t fsoe_pad1 :3;
  uint8_t fsoe_error_acknowledge :1;
  uint8_t fsoe_sls_instance_1 :1;
  uint8_t fsoe_sls_instance_2 :1;
  uint8_t fsoe_sls_instance_3 :1;
  uint8_t fsoe_sls_instance_4 :1;
  uint8_t fsoe_restart_acknowledge :1;
  uint8_t fsoe_sbc_command :1;
  uint8_t fsoe_reset_position :1;
  uint8_t fsoe_pad2 :1;
  uint16_t fsoe_crc_0;
  uint8_t fsoe_pad3;
  uint8_t fsoe_pad4 :4;
  uint8_t fsoe_safe_output_1 :1;
  uint8_t fsoe_safe_output_2 :1;
  uint8_t fsoe_pad5 : 2;
  uint16_t fsoe_crc_1;
  uint16_t fsoe_connection_id;
} plc_to_drive_t;

typedef struct __attribute__((packed,aligned(1))) cia_402_tx_T {
  uint16_t status_word;
  uint8_t modes_of_operation_display;
  uint32_t position_actual_value;
  int32_t velocity_actual_value;
  int16_t torque_actual_value;
} cia_402_tx_t;

typedef struct __attribute__((packed,aligned(1))) cia_402_tx2_T {
  int16_t analog_input_1;
  int16_t analog_input_2;
  int16_t analog_input_3;
  int16_t analog_input_4;
  uint32_t tuning_status;
} cia_402_tx2_t;

typedef struct __attribute__((packed,aligned(1))) cia_402_tx3_T {
  uint32_t digital_inputs;
} cia_402_tx3_t;

typedef struct __attribute__((packed,aligned(1))) cia_402_tx4_T {
  uint32_t user_miso;
  uint32_t timestamp;
  uint32_t position_demand_internal_value;
  uint32_t velocity_demand_value;
  uint16_t torque_demand;
} cia_402_tx4_t;

typedef struct __attribute__((packed,aligned(1))) drive_to_plc_T {
  uint8_t fsoe_command;
  uint8_t fsoe_sto_state :1;
  uint8_t fsoe_pad1 :2;
  uint8_t fsoe_sos_state :1;
  uint8_t fsoe_pad2 :3;
  uint8_t fsoe_error_state :1;
  uint8_t fsoe_ss1_state :1;
  uint8_t fsoe_ss2_state :1;
  uint8_t fsoe_pad3 :2;
  uint8_t fsoe_sls_instance_1 :1;
  uint8_t fsoe_sls_instance_2 :1;
  uint8_t fsoe_sls_instance_3 :1;
  uint8_t fsoe_sls_instance_4 :1;
  uint16_t fsoe_crc_0;
  uint8_t fsoe_restart_acknowledge :1;
  uint8_t fsoe_sbs_state :1;
  uint8_t fsoe_temperature_warning :1;
  uint8_t fsoe_safe_position_valid :1;
  uint8_t fsoe_safe_speed_valid :1;
  uint8_t fsoe_pad4 :3;
  uint8_t fsoe_safe_input_1 :1;
  uint8_t fsoe_safe_input_2 :1;
  uint8_t fsoe_safe_input_3 :1;
  uint8_t fsoe_safe_input_4 :1;
  uint8_t fsoe_safe_output_monitor_1 :1;
  uint8_t fsoe_safe_output_monitor_2 :1;
  uint8_t fsoe_analog_diagnostic_active :1;
  uint8_t fsoe_analog_value_valid :1;
  uint16_t fsoe_crc_1;
  uint16_t fsoe_safe_position_actual_value;
  uint16_t fsoe_crc_2;
  uint16_t fsoe_pad5;
  uint16_t fsoe_crc_3;
  int16_t fsoe_save_velocity_actual_value;
  uint16_t fsoe_crc_4;
  uint16_t fsoe_pad6;
  uint16_t fsoe_crc_5;
  int16_t fsoe_safe_analog_value_scaled;
  uint16_t fsoe_crc_6;
  uint16_t fsoe_connection_id;
} drive_to_plc_t;



typedef struct __attribute__((packed,aligned(1))) slave_1_pdo_mapping_T {
  cia_402_rx_t cia_402_rx;
  cia_402_rx2_t cia_402_rx2;
  cia_402_rx3_t cia_402_rx3;
  plc_to_drive_t plc_to_drive;
  uint8_t padding_modulePdoAling1Out;
  /* RxPDO Mapping stop */
  cia_402_tx_t cia_402_tx;
  cia_402_tx2_t cia_402_tx2;
  cia_402_tx3_t cia_402_tx3;
  cia_402_tx4_t cia_402_tx4;
  drive_to_plc_t drive_to_plc;
} slave_1_pdo_mapping_t;

typedef struct __attribute__((packed,aligned(1))) synapticon_read_mapping_T {
  cia_402_tx_t cia_402_tx;
  cia_402_tx2_t cia_402_tx2;
  cia_402_tx3_t cia_402_tx3;
  cia_402_tx4_t cia_402_tx4;
  drive_to_plc_t drive_to_plc;
} synapticon_read_mapping_t;

typedef struct __attribute__((packed,aligned(1))) synapticon_write_mapping_T {
  cia_402_rx_t cia_402_rx;
  cia_402_rx2_t cia_402_rx2;
  cia_402_rx3_t cia_402_rx3;
  plc_to_drive_t plc_to_drive;
} synapticon_write_mapping_t;



static const uint32_t initModulesBBH[] = {0x6212, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                          0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                          0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                          0x0, 0x0, 0x0, 0x6194, 0x6195};

static const uint32_t initModulesSynapticon[] = {0x04020001, 0x22D20001};

int BBHsetup(uint16 slave) {
    printf("Executing BBHSetup\n");
    uint8_t initModulesBBHSize = (uint8_t) sizeof(initModulesBBH) / sizeof(initModulesBBH[0]);
    ec_slave[slave].SM[2].SMflags = htoel(0x64);
    ec_slave[slave].SM[2].SMlength = htoes(82);
    ec_slave[slave].SM[2].StartAddr = htoes(0x1200);
    ec_slave[slave].SM[3].SMflags = htoel(0x20);
    ec_slave[slave].SM[3].SMlength = htoes(54);
    ec_slave[slave].SM[3].StartAddr = htoes(0x1800);

    ec_slave[slave].SMtype[2] = 3;
    ec_slave[slave].SMtype[3] = 4;

    for (size_t i = 0; i < initModulesBBHSize; ++i) {
      ec_SDOwrite(slave, 0xf030, (uint8_t) i + 1, FALSE, sizeof(uint32_t), &initModulesBBH[i],
                             EC_TIMEOUTSAFE);
    }
    ec_SDOwrite(slave, 0xf030, 0, FALSE, sizeof(uint8_t), &initModulesBBHSize, EC_TIMEOUTSAFE);

    ec_slave[slave].configindex = 1;
    ec_slave[slave].Ibits = ec_slave[slave].SM[3].SMlength*8;
    ec_slave[slave].Obits = ec_slave[slave].SM[2].SMlength*8;
    return 1;
}

int SynapticonSetup(uint16_t slave) {
    uint8_t initModulesSynapticonSize = (uint8_t) sizeof(initModulesSynapticon) / sizeof(initModulesSynapticon[0]);
    for (size_t i = 0; i < initModulesSynapticonSize; ++i) {
        ec_SDOwrite(slave, 0xf030, (uint8_t) i + 1, FALSE, sizeof(uint32_t), &initModulesSynapticon[i],
                             EC_TIMEOUTSAFE);
    }
    ec_SDOwrite(slave, 0xf030, 0, FALSE, sizeof(uint8_t), &initModulesSynapticonSize, EC_TIMEOUTSAFE);
    return 1;
}


void simpletest(char *ifname)
{
    int i, j, oloop, iloop, chk;
    needlf = FALSE;
    inOP = FALSE;

   printf("Starting simple test\n");

   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n",ifname);

      /* find and auto-config slaves */


       if ( ec_config_init(FALSE) > 0 )
      {
         /* Set special functions for BBH and Synapticon */
         ec_slave[1].PO2SOconfig = BBHsetup;
         ec_slave[2].PO2SOconfig = SynapticonSetup;

         printf("%d slaves found and configured.\n",ec_slavecount);


         if (forceByteAlignment)
         {
            ec_config_map_aligned(&IOmap);
         }
         else
         {
            ec_config_map(&IOmap);
         }

         ec_configdc();

         printf("Slaves mapped, state to SAFE_OP.\n");
         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

         oloop = ec_slave[0].Obytes;
         if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
         if (oloop > 8) oloop = 8;
         iloop = ec_slave[0].Ibytes;
         if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
         if (iloop > 8) iloop = 8;

         printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

         printf("Request operational state for all slaves\n");
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         /* send one valid process data to make outputs in slaves happy*/
         ec_send_processdata();
         ec_receive_processdata(EC_TIMEOUTRET);
         /* request OP state for all slaves */
         ec_writestate(0);
         chk = 200;
         /* wait for all slaves to reach OP state */
         do
         {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
         }
         while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
         if (ec_slave[0].state == EC_STATE_OPERATIONAL )
         {
            printf("Operational state reached for all slaves.\n");
            inOP = TRUE;
                /* cyclic loop */
            for(i = 1; i <= 10000; i++)
            {
               memcpy(&((bbh_write_mapping_t*) ec_slave[1].outputs)->fsoe_safety_master_message_14_bytes_out,&(((synapticon_read_mapping_t*) ec_slave[2].inputs)->drive_to_plc),sizeof(drive_to_plc_t));
               memcpy(&((synapticon_write_mapping_t*) ec_slave[2].outputs)->plc_to_drive,&((bbh_read_mapping_t*) ec_slave[1].inputs)->fsoe_safety_slave_message_4_bytes_in,sizeof(plc_to_drive_t));
               ec_send_processdata();
               wkc = ec_receive_processdata(EC_TIMEOUTRET);

                    if(wkc >= expectedWKC)
                    {
                        printf("Processdata cycle %4d, WKC %d , O:", i, wkc);

                        for(j = 0 ; j < oloop; j++)
                        {
                            printf(" %2.2x", *(ec_slave[0].outputs + j));
                        }

                        printf(" I:");
                        for(j = 0 ; j < iloop; j++)
                        {
                            printf(" %2.2x", *(ec_slave[0].inputs + j));
                        }
                        printf(" T:%"PRId64"\r",ec_DCtime);
                        needlf = TRUE;
                    }
                    osal_usleep(2000);

                }
                inOP = FALSE;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
            printf("\nRequest init state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End simple test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExecute as root\n",ifname);
    }
}

OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    int slave;
    (void)ptr;                  /* Not used */

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
               needlf = FALSE;
               printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > EC_STATE_NONE)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (ec_slave[slave].state == EC_STATE_NONE)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(ec_slave[slave].state == EC_STATE_NONE)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

int main(int argc, char *argv[])
{
   printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

   if (argc > 1)
   {
      /* create thread to handle slave error handling in OP */
      osal_thread_create(&thread1, 128000, &ecatcheck, NULL);
      /* start cyclic part */
      simpletest(argv[1]);
   }
   else
   {
      ec_adaptert * adapter = NULL;
      ec_adaptert * head = NULL;
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");

      printf ("\nAvailable adapters:\n");
      head = adapter = ec_find_adapters ();
      while (adapter != NULL)
      {
         printf ("    - %s  (%s)\n", adapter->name, adapter->desc);
         adapter = adapter->next;
      }
      ec_free_adapters(head);
   }

   printf("End program\n");
   return (0);
}
