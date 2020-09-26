-------------------------------------------------------------------------------
-- File       : AppReg.vhd
-- Company    : SLAC National Accelerator Laboratory
-------------------------------------------------------------------------------
-- Description:  Application Registers and Modules
-------------------------------------------------------------------------------
-- This file is part of 'Example Project Firmware'.
-- It is subject to the license terms in the LICENSE.txt file found in the 
-- top-level directory of this distribution and at: 
--    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. 
-- No part of 'Example Project Firmware', including this file, 
-- may be copied, modified, propagated, or distributed except according to 
-- the terms contained in the LICENSE.txt file.
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

library surf;
use surf.StdRtlPkg.all;
use surf.AxiLitePkg.all;
use surf.AxiStreamPkg.all;
use surf.SsiPkg.all;
use surf.SsiCmdMasterPkg.all;
use surf.Pgp2bPkg.all;

use work.AppPkg.all;

library epix_hr_core;
use epix_hr_core.EpixHrCorePkg.all;

use work.HrAdcPkg.all;

library unisim;
use unisim.vcomponents.all;

entity AppReg is
   generic (
      TPD_G             : time    := 1 ns;
      BUILD_INFO_G      : BuildInfoType;
      SIMULATION_G      : boolean := false;
      PRBS_TX_BATCHER_G : boolean := false;
      CLK_FREQUENCY_G   : real    := 156.25E+6;
      XIL_DEVICE_G      : string  := "7SERIES");
   port (
      -- Clock and Reset
      clk             : in  sl;
      rst             : in  sl;
      -- AXI-Lite Register Interface (sysClk domain)
      -- Register Address Range = [0x80000000:0xFFFFFFFF]
      sAxilReadMaster  : in    AxiLiteReadMasterType;
      sAxilReadSlave   : out   AxiLiteReadSlaveType;
      sAxilWriteMaster : in    AxiLiteWriteMasterType;
      sAxilWriteSlave  : out   AxiLiteWriteSlaveType;
      -- Communication AXI-Lite Interface
      commWriteMaster : out AxiLiteWriteMasterType;
      commWriteSlave  : in  AxiLiteWriteSlaveType;
      commReadMaster  : out AxiLiteReadMasterType;
      commReadSlave   : in  AxiLiteReadSlaveType;
      -- PBRS Interface
      pbrsTxMaster    : out AxiStreamMasterType;
      pbrsTxSlave     : in  AxiStreamSlaveType;
      pbrsRxMaster    : in  AxiStreamMasterType;
      pbrsRxSlave     : out AxiStreamSlaveType;
      -- HLS Interface
      hlsTxMaster     : out AxiStreamMasterType;
      hlsTxSlave      : in  AxiStreamSlaveType;
      hlsRxMaster     : in  AxiStreamMasterType;
      hlsRxSlave      : out AxiStreamSlaveType;
      -- MB Interface
      mbTxMaster      : out AxiStreamMasterType;
      mbTxSlave       : in  AxiStreamSlaveType;
      -- Add CRYO pins for FEMB
      asicGlblRst   : out sl;
      asicPulse     : out sl;
      asicSaciClk_p : out sl;
      asicSaciClk_n : out sl;
      asicSaciCmd_p : out sl;
      asicSaciCmd_n : out sl;
      asicSaciRsp_p : in  sl;
      asicSaciRsp_n : in  sl;      
      asicSmpClk_p  : out sl;
      asicSmpClk_n  : out sl;
      asicSaciSel   : out slv(1 downto 0);
      asicR0_p      : out sl;
      asicR0_n      : out sl;
      asicD0out_p   : in  slv(1 downto 0);
      asicD0out_n   : in  slv(1 downto 0);
      asicD1out_p   : in  slv(1 downto 0);
      asicD1out_n   : in  slv(1 downto 0);
      -- Add jitter cleaner pins
      pllSck        : out sl;
      pllSdo        : in  sl;
      pllSdi        : out sl;
      pllCsL        : out sl;
      pllInClk      : out sl;
      -- SSI commands 
      ssiCmd        : in  SsiCmdMasterType := SSI_CMD_MASTER_INIT_C;
      -- ADC Ports
      vPIn          : in  sl;
      vNIn          : in  sl);
end AppReg;

architecture mapping of AppReg is

   constant SHARED_MEM_WIDTH_C : positive                           := 13;
   constant IRQ_ADDR_C         : slv(SHARED_MEM_WIDTH_C-1 downto 0) := (others => '1');

   constant NUM_AXI_MASTERS_C : natural := 19;

--   constant NUMBER_OF_ASICS_C : natural := 2; -- Moved to AppPkg.vhd

   constant VERSION_INDEX_C  : natural := 0;
   constant XADC_INDEX_C     : natural := 1;
   constant SYS_MON_INDEX_C  : natural := 2;
   constant MEM_INDEX_C      : natural := 3;
   constant PRBS_TX_INDEX_C  : natural := 4;
   constant PRBS_RX_INDEX_C  : natural := 5;
   constant HLS_INDEX_C      : natural := 6;
   constant COMM_INDEX_C     : natural := 7;
   constant AXIS_MON_INDEX_C : natural := 8;
--   constant TEST_INDEX_C     : natural := 9;
   constant PLLREGS_AXI_INDEX_C           : natural := 9;
   constant TRIG_REG_AXI_INDEX_C          : natural := 10;
   constant SACIREGS_INDEX_C              : natural := 11;
   constant CLK_JIT_CLR_REG_AXI_INDEX_C   : natural := 12;
   constant APP_REG_AXI_INDEX_C           : natural := 13;
   constant PLL2REGS_AXI_INDEX_C          : natural := 14;
   constant CRYO_ASIC0_READOUT_AXI_INDEX_C : natural := 15;
   constant DIG_ASIC0_STREAM_AXI_INDEX_C  : natural := 16;
   constant CRYO_ASIC1_READOUT_AXI_INDEX_C : natural := 17;
   constant DIG_ASIC1_STREAM_AXI_INDEX_C  : natural := 18;

   constant PLLREGS_AXI_BASE_ADDR_C         : slv(31 downto 0) := X"8000_0000";--9
   constant TRIG_REG_AXI_BASE_ADDR_C        : slv(31 downto 0) := X"8100_0000";--10
   constant SACIREGS_BASE_ADDR_C            : slv(31 downto 0) := X"8800_0000";--11
   constant CLK_JIT_CLR_REG_AXI_ADDR_C      : slv(31 downto 0) := X"9300_0000";--12
   constant APP_REG_AXI_ADDR_C              : slv(31 downto 0) := X"9600_0000";--13
   constant PLL2REGS_AXI_BASE_ADDR_C        : slv(31 downto 0) := X"9700_0000";--14
   constant CRYO_ASIC0_READOUT_AXI_ADDR_C   : slv(31 downto 0) := X"9400_0000";--15
   constant DIG_ASIC0_STREAM_AXI_ADDR_C    : slv(31 downto 0) := X"9500_0000";--16
   constant CRYO_ASIC1_READOUT_AXI_ADDR_C   : slv(31 downto 0) := X"9800_0000";--17
   constant DIG_ASIC1_STREAM_AXI_ADDR_C    : slv(31 downto 0) := X"9900_0000";--18

   -- constant AXI_CONFIG_C : AxiLiteCrossbarMasterConfigArray(NUM_AXI_MASTERS_C-1 downto 0) := genAxiLiteConfig(NUM_AXI_MASTERS_C, x"0000_0000", 20, 16);
   constant AXI_CONFIG_C : AxiLiteCrossbarMasterConfigArray(NUM_AXI_MASTERS_C-1 downto 0) := (
      VERSION_INDEX_C  => (
         baseAddr      => x"0000_0000",
         addrBits      => 16,
         connectivity  => x"FFFF"),
      XADC_INDEX_C     => (
         baseAddr      => x"0001_0000",
         addrBits      => 16,
         connectivity  => x"FFFF"),
      SYS_MON_INDEX_C  => (
         baseAddr      => x"0002_0000",
         addrBits      => 16,
         connectivity  => x"FFFF"),
      MEM_INDEX_C      => (
         baseAddr      => x"0003_0000",
         addrBits      => 16,
         connectivity  => x"FFFF"),
      PRBS_TX_INDEX_C  => (
         baseAddr      => x"0004_0000",
         addrBits      => 16,
         connectivity  => x"FFFF"),
      PRBS_RX_INDEX_C  => (
         baseAddr      => x"0005_0000",
         addrBits      => 16,
         connectivity  => x"FFFF"),
      HLS_INDEX_C      => (
         baseAddr      => x"0006_0000",
         addrBits      => 16,
         connectivity  => x"FFFF"),
      COMM_INDEX_C     => (
         baseAddr      => x"0007_0000",
         addrBits      => 16,
         connectivity  => x"FFFF"),
      AXIS_MON_INDEX_C => (
         baseAddr      => x"0008_0000",
         addrBits      => 16,
         connectivity  => x"FFFF"),
--      TEST_INDEX_C     => (
--         baseAddr      => x"8000_0000",
--         addrBits      => 31,
--         connectivity  => x"FFFF"),
      PLLREGS_AXI_INDEX_C     => (
         baseAddr             => PLLREGS_AXI_BASE_ADDR_C,
         addrBits             => 24,
         connectivity         => x"FFFF"),
      TRIG_REG_AXI_INDEX_C => (
         baseAddr          => TRIG_REG_AXI_BASE_ADDR_C,
         addrBits          => 24,
         connectivity      => x"FFFF"),
      SACIREGS_INDEX_C => (
         baseAddr      => SACIREGS_BASE_ADDR_C,
         addrBits      => 24,
         connectivity  => x"FFFF"),
      CLK_JIT_CLR_REG_AXI_INDEX_C => (
         baseAddr                 => CLK_JIT_CLR_REG_AXI_ADDR_C,
         addrBits                 => 24,
         connectivity             => x"FFFF"),       
      APP_REG_AXI_INDEX_C                => (
         baseAddr             => APP_REG_AXI_ADDR_C,
         addrBits             => 24,
         connectivity         => x"FFFF"),    
      PLL2REGS_AXI_INDEX_C => (
         baseAddr                 => PLL2REGS_AXI_BASE_ADDR_C,
         addrBits                 => 24,
         connectivity             => x"FFFF"),
      CRYO_ASIC0_READOUT_AXI_INDEX_C => (
         baseAddr                 => CRYO_ASIC0_READOUT_AXI_ADDR_C,
         addrBits                 => 24,
         connectivity             => x"FFFF"),
      DIG_ASIC0_STREAM_AXI_INDEX_C => (
         baseAddr                 => DIG_ASIC0_STREAM_AXI_ADDR_C,
         addrBits                 => 24,
         connectivity             => x"FFFF"),
      CRYO_ASIC1_READOUT_AXI_INDEX_C => (
         baseAddr                 => CRYO_ASIC1_READOUT_AXI_ADDR_C,
         addrBits                 => 24,
         connectivity             => x"FFFF"),
      DIG_ASIC1_STREAM_AXI_INDEX_C => (
         baseAddr                 => DIG_ASIC1_STREAM_AXI_ADDR_C,
         addrBits                 => 24,
         connectivity             => x"FFFF")
       );         

--   type AppConfigType is record
--      AppVersion           : slv(31 downto 0);
--      powerEnable          : slv(3 downto 0);
--      asicMask             : slv(NUMBER_OF_ASICS_C-1 downto 0);
--      acqCnt               : slv(31 downto 0);
--      requestStartupCal    : sl;
--      startupAck           : sl;
--      startupFail          : sl;
--      SR0Delay             : slv(7 downto 0);
--      SR0Period            : slv(7 downto 0);
--      epixhrDbgSel1        : slv(4 downto 0);
--      epixhrDbgSel2        : slv(4 downto 0);
--   end record;
--
--
--   constant APP_CONFIG_INIT_C : AppConfigType := (
--      AppVersion           => (others => '0'),
--      powerEnable          => (others => '0'),
--      asicMask             => (others => '0'),
--      acqCnt               => (others => '0'),
--      requestStartupCal    => '1',
--      startupAck           => '0',
--      startupFail          => '0',
--      SR0Delay             => (others => '0'),
--      SR0Period            => (others => '0'),
--      epixhrDbgSel1        => "01101",
--      epixhrDbgSel2        => "01101"
--   );

   constant HR_FD_NUM_AXI_SLAVE_SLOTS_C   : natural := 1;

   -- AXI-Lite Signals
   signal sAxiReadMaster  : AxiLiteReadMasterArray(HR_FD_NUM_AXI_SLAVE_SLOTS_C-1 downto 0);
   signal sAxiReadSlave   : AxiLiteReadSlaveArray(HR_FD_NUM_AXI_SLAVE_SLOTS_C-1 downto 0);
   signal sAxiWriteMaster : AxiLiteWriteMasterArray(HR_FD_NUM_AXI_SLAVE_SLOTS_C-1 downto 0);
   signal sAxiWriteSlave  : AxiLiteWriteSlaveArray(HR_FD_NUM_AXI_SLAVE_SLOTS_C-1 downto 0);
   -- AXI-Lite Signals
   signal mAxiWriteMasters : AxiLiteWriteMasterArray(NUM_AXI_MASTERS_C-1 downto 0);
   signal mAxiWriteSlaves  : AxiLiteWriteSlaveArray(NUM_AXI_MASTERS_C-1 downto 0);
   signal mAxiReadMasters  : AxiLiteReadMasterArray(NUM_AXI_MASTERS_C-1 downto 0);
   signal mAxiReadSlaves   : AxiLiteReadSlaveArray(NUM_AXI_MASTERS_C-1 downto 0);

   signal txMaster : AxiStreamMasterType;
   signal txSlave  : AxiStreamSlaveType;
   signal rxMaster : AxiStreamMasterType;
   signal rxSlave  : AxiStreamSlaveType;

   signal axiWrValid : sl;
   signal axiWrAddr  : slv(SHARED_MEM_WIDTH_C-1 downto 0);

   signal irqReq   : slv(7 downto 0);
   signal irqCount : slv(27 downto 0);
   
   signal iSaciClk  : sl;
   signal iSaciCmd  : sl;
   signal iAsicSaciRsp : sl;

   -- clock signals
   signal appClk         : sl;
   signal bitClk         : sl;
   signal byteClk        : sl;
   signal deserClk       : sl;
   signal asicRdClk      : sl;
   signal idelayCtrlClk  : sl;
   signal appRst         : sl;
   signal axiRst         : sl;
   signal bitRst         : sl;
   signal byteClkRst     : sl;
   signal asicRdClkRst   : sl;
   signal idelayCtrlRst  : sl;
   signal idelayCtrlRst_i: sl;
   signal clkLocked      : sl;

   -- ASIC signals (placeholders)
   signal iAsicEnA             : sl;
   signal iAsicEnB             : sl;
   signal iAsicVid             : sl;
   signal iAsicSR0             : sl;
   signal iAsicSR0Raw          : sl;
   signal iAsicSR0RefClk       : sl;
   signal iAsic01DM1           : sl;
   signal iAsic01DM2           : sl;
   signal iAsicPPbe            : sl;
   signal iAsicPpmat           : sl;
   signal iAsicR0              : sl;
   signal iAsicSync            : sl;
   signal iAsicAcq             : sl;
   signal iAsicGrst            : sl;
   signal boardConfig          : AppConfigType;
   signal monitoringSig        : slv(1 downto 0);

   signal adcClk         : sl;
   signal errInhibit     : sl;

   signal dummy : slv(3 downto 0);

   -- Triggers and associated signals
   signal iDaqTrigger        : sl := '0';
   signal iRunTrigger        : sl := '0';
   signal opCode             : slv(7 downto 0);
   signal pgpOpCodeOneShot   : sl;
   signal acqStart           : sl;
   signal dataSend           : sl;
   signal saciPrepReadoutReq : sl;
   signal saciPrepReadoutAck : sl;
   signal pgpRxOut           : Pgp2bRxOutType;

   signal serialIdIo         : slv(1 downto 0) := "00";
   signal idelayRdy          : sl;
   signal serdesReset        : sl;

   -- 
   signal adcSerial   : HrAdcSerialGroupArray(NUMBER_OF_ASICS_C-1 downto 0);
   signal asicStreams : AxiStreamMasterArray(((NUMBER_OF_ASICS_C-1)*STREAMS_PER_ASIC_C)+STREAMS_PER_ASIC_C-1 downto 0) := (others=>AXI_STREAM_MASTER_INIT_C);
--   signal adcStreamsEn_n : slv(STREAMS_PER_ASIC_C-1 downto 0) := (others => '0');
   signal adcStreamsEn_n : Slv2Array(NUMBER_OF_ASICS_C-1 downto 0) := (others => (others => '0'));

   signal mAxisMastersASIC : AxiStreamMasterArray(3 downto 0);
   signal mAxisSlavesASIC  : AxiStreamSlaveArray(3 downto 0);

begin

   ---------------------------------------------
   -- AXI Lite Async - cross clock domain     --
   ---------------------------------------------
   U_AxiLiteAsync : entity surf.AxiLiteAsync
   generic map(
      TPD_G            => 1 ns,
      AXI_ERROR_RESP_G => AXI_RESP_SLVERR_C,
      COMMON_CLK_G     => false,
      NUM_ADDR_BITS_G  => 32,
      PIPE_STAGES_G    => 0)
   port map(
      -- Slave Port
      sAxiClk         => clk,
      sAxiClkRst      => rst,
      sAxiReadMaster  => sAxilReadMaster,
      sAxiReadSlave   => sAxilReadSlave,
      sAxiWriteMaster => sAxilWriteMaster,
      sAxiWriteSlave  => sAxilWriteSlave,
      -- Master Port
      mAxiClk         => appClk,
      mAxiClkRst      => appRst,
      mAxiReadMaster  => sAxiReadMaster(0),
      mAxiReadSlave   => sAxiReadSlave(0),
      mAxiWriteMaster => sAxiWriteMaster(0),
      mAxiWriteSlave  => sAxiWriteSlave(0)
    );
   ---------------------------------------------
   -- AXI Lite Crossbar for register control  --
   -- Check AppPkg.vhd for addresses          --
   ---------------------------------------------
   U_AxiLiteCrossbar : entity surf.AxiLiteCrossbar
   generic map (
      NUM_SLAVE_SLOTS_G  => 1,
      NUM_MASTER_SLOTS_G => NUM_AXI_MASTERS_C,
      MASTERS_CONFIG_G   => AXI_CONFIG_C
   )
   port map (
      sAxiWriteMasters    => sAxiWriteMaster,
      sAxiWriteSlaves     => sAxiWriteSlave,
      sAxiReadMasters     => sAxiReadMaster,
      sAxiReadSlaves      => sAxiReadSlave,
      mAxiWriteMasters    => mAxiWriteMasters,
      mAxiWriteSlaves     => mAxiWriteSlaves,
      mAxiReadMasters     => mAxiReadMasters,
      mAxiReadSlaves      => mAxiReadSlaves,
      axiClk              => appClk,
      axiClkRst           => appRst
   );


   ---------------------------
   -- AXI-Lite: Version Module
   ---------------------------            
   U_AxiVersion : entity surf.AxiVersion
      generic map (
         TPD_G           => TPD_G,
         CLK_PERIOD_G    => (1.0/CLK_FREQUENCY_G),
         BUILD_INFO_G    => BUILD_INFO_G,
         XIL_DEVICE_G    => XIL_DEVICE_G,
         EN_DEVICE_DNA_G => true)
      port map (
         axiReadMaster  => mAxiReadMasters(VERSION_INDEX_C),
         axiReadSlave   => mAxiReadSlaves(VERSION_INDEX_C),
         axiWriteMaster => mAxiWriteMasters(VERSION_INDEX_C),
         axiWriteSlave  => mAxiWriteSlaves(VERSION_INDEX_C),
         axiClk         => appClk,
         axiRst         => appRst);

   GEN_7SERIES : if (XIL_DEVICE_G = "7SERIES") generate
      ------------------------
      -- AXI-Lite: XADC Module
      ------------------------
      U_XADC : entity work.AxiXadcWrapper
         generic map (
            TPD_G => TPD_G)
         port map (
            axiReadMaster  => mAxiReadMasters(XADC_INDEX_C),
            axiReadSlave   => mAxiReadSlaves(XADC_INDEX_C),
            axiWriteMaster => mAxiWriteMasters(XADC_INDEX_C),
            axiWriteSlave  => mAxiWriteSlaves(XADC_INDEX_C),
            axiClk         => appClk,
            axiRst         => appRst,
            vPIn           => vPIn,
            vNIn           => vNIn);

   end generate;

   GEN_ULTRA_SCALE : if (XIL_DEVICE_G = "ULTRASCALE") generate
      --------------------------
      -- AXI-Lite: SYSMON Module
      --------------------------
      U_SysMon : entity work.SystemManagementWrapper
         generic map (
            TPD_G => TPD_G)
         port map (
            axiReadMaster  => mAxiReadMasters(SYS_MON_INDEX_C),
            axiReadSlave   => mAxiReadSlaves(SYS_MON_INDEX_C),
            axiWriteMaster => mAxiWriteMasters(SYS_MON_INDEX_C),
            axiWriteSlave  => mAxiWriteSlaves(SYS_MON_INDEX_C),
            axiClk         => appClk,
            axiRst         => appRst,
            vPIn           => vPIn,
            vNIn           => vNIn);
   end generate;

   --------------------------------          
   -- AXI-Lite Shared Memory Module
   --------------------------------          
   U_Mem : entity surf.AxiDualPortRam
      generic map (
         TPD_G        => TPD_G,
         AXI_WR_EN_G  => true,
         SYS_WR_EN_G  => false,
         COMMON_CLK_G => true,
         ADDR_WIDTH_G => SHARED_MEM_WIDTH_C,
         DATA_WIDTH_G => 32)
      port map (
         -- Clock and Reset
         clk            => appClk,
         rst            => appRst,
         -- AXI-Lite Write Monitor
         axiWrValid     => axiWrValid,
         axiWrAddr      => axiWrAddr,
         -- AXI-Lite Interface
         axiClk         => appClk,
         axiRst         => appRst,
         axiReadMaster  => mAxiReadMasters(MEM_INDEX_C),
         axiReadSlave   => mAxiReadSlaves(MEM_INDEX_C),
         axiWriteMaster => mAxiWriteMasters(MEM_INDEX_C),
         axiWriteSlave  => mAxiWriteSlaves(MEM_INDEX_C));

   -------------------
   -- AXI-Lite PRBS RX
   -------------------
   U_SsiPrbsTx : entity surf.SsiPrbsTx
      generic map (
         TPD_G                      => TPD_G,
         MASTER_AXI_PIPE_STAGES_G   => 1,
         PRBS_SEED_SIZE_G           => 128,
         MASTER_AXI_STREAM_CONFIG_G => ssiAxiStreamConfig(16))
      port map (
         mAxisClk        => appClk,
         mAxisRst        => appRst,
         mAxisMaster     => txMaster,
         mAxisSlave      => txSlave,
         locClk          => appClk,
         locRst          => appRst,
         trig            => '0',
         packetLength    => X"000000ff",
         tDest           => X"00",
         tId             => X"00",
         axilReadMaster  => mAxiReadMasters(PRBS_TX_INDEX_C),
         axilReadSlave   => mAxiReadSlaves(PRBS_TX_INDEX_C),
         axilWriteMaster => mAxiWriteMasters(PRBS_TX_INDEX_C),
         axilWriteSlave  => mAxiWriteSlaves(PRBS_TX_INDEX_C));

   GEN_BATCHER : if (PRBS_TX_BATCHER_G = true) generate
      U_AxiStreamBatcher : entity surf.AxiStreamBatcher
         generic map (
            TPD_G                        => TPD_G,
            MAX_NUMBER_SUB_FRAMES_G      => 8,
            SUPER_FRAME_BYTE_THRESHOLD_G => 0,  -- 0 = bypass super threshold check
            MAX_CLK_GAP_G                => 0,  -- 0 = bypass MAX clock GAP 
            AXIS_CONFIG_G                => ssiAxiStreamConfig(16),
            INPUT_PIPE_STAGES_G          => 0,
            OUTPUT_PIPE_STAGES_G         => 0)
         port map (
            -- Clock and Reset
            axisClk     => appClk,
            axisRst     => appRst,
            -- AXIS Interfaces
            sAxisMaster => txMaster,
            sAxisSlave  => txSlave,
            mAxisMaster => pbrsTxMaster,
            mAxisSlave  => pbrsTxSlave);
   end generate;

   BYP_BATCHER : if (PRBS_TX_BATCHER_G = false) generate
      pbrsTxMaster <= txMaster;
      txSlave      <= pbrsTxSlave;
   end generate;

   -------------------
   -- AXI-Lite PRBS RX
   -------------------
   U_SsiPrbsRx : entity surf.SsiPrbsRx
      generic map (
         TPD_G                     => TPD_G,
         SLAVE_AXI_PIPE_STAGES_G   => 1,
         PRBS_SEED_SIZE_G          => 128,
         SLAVE_AXI_STREAM_CONFIG_G => ssiAxiStreamConfig(16))
      port map (
         sAxisClk       => appClk,
         sAxisRst       => appRst,
         sAxisMaster    => rxMaster,
         sAxisSlave     => rxSlave,
         axiClk         => appClk,
         axiRst         => appRst,
         axiReadMaster  => mAxiReadMasters(PRBS_RX_INDEX_C),
         axiReadSlave   => mAxiReadSlaves(PRBS_RX_INDEX_C),
         axiWriteMaster => mAxiWriteMasters(PRBS_RX_INDEX_C),
         axiWriteSlave  => mAxiWriteSlaves(PRBS_RX_INDEX_C));

   rxMaster    <= pbrsRxMaster;
   pbrsRxSlave <= rxSlave;

   --------------------------------------
   -- AXI-Lite PRBS AXI Stream Monitoring
   --------------------------------------
   U_AXIS_MON : entity surf.AxiStreamMonAxiL
      generic map(
         TPD_G            => TPD_G,
         COMMON_CLK_G     => true,
         AXIS_CLK_FREQ_G  => CLK_FREQUENCY_G,
         AXIS_NUM_SLOTS_G => 2,
         AXIS_CONFIG_G    => ssiAxiStreamConfig(16))
      port map(
         -- AXIS Stream Interface
         axisClk          => appClk,
         axisRst          => appRst,
         axisMasters(0)   => txMaster,
         axisMasters(1)   => rxMaster,
         axisSlaves(0)    => txSlave,
         axisSlaves(1)    => rxSlave,
         -- AXI lite slave port for register access
         axilClk          => appClk,
         axilRst          => appRst,
         sAxilWriteMaster => mAxiWriteMasters(AXIS_MON_INDEX_C),
         sAxilWriteSlave  => mAxiWriteSlaves(AXIS_MON_INDEX_C),
         sAxilReadMaster  => mAxiReadMasters(AXIS_MON_INDEX_C),
         sAxilReadSlave   => mAxiReadSlaves(AXIS_MON_INDEX_C));

   ------------------------------
   -- AXI-Lite HLS Example Module
   ------------------------------            
   U_AxiLiteExample : entity work.AxiLiteExample
      port map (
         axiClk         => appClk,
         axiRst         => appRst,
         axiReadMaster  => mAxiReadMasters(HLS_INDEX_C),
         axiReadSlave   => mAxiReadSlaves(HLS_INDEX_C),
         axiWriteMaster => mAxiWriteMasters(HLS_INDEX_C),
         axiWriteSlave  => mAxiWriteSlaves(HLS_INDEX_C));

   ------------------------------------
   -- AXI Streaming: HLS Example Module
   ------------------------------------
   U_AxiStreamExample : entity work.AxiStreamExample
      port map (
         axisClk     => appClk,
         axisRst     => appRst,
         -- Slave Port
         sAxisMaster => hlsRxMaster,
         sAxisSlave  => hlsRxSlave,
         -- Master Port
         mAxisMaster => hlsTxMaster,
         mAxisSlave  => hlsTxSlave);

   -----------------------------------------------
   -- Map the AXI-Lite to Communication Monitoring
   -----------------------------------------------
   commReadMaster                 <= mAxiReadMasters(COMM_INDEX_C);
   mAxiReadSlaves(COMM_INDEX_C)  <= commReadSlave;
   commWriteMaster                <= mAxiWriteMasters(COMM_INDEX_C);
   mAxiWriteSlaves(COMM_INDEX_C) <= commWriteSlave;

--   -------------------------------------------------------------
--   -- Map the AXI-Lite to Test bus (never respond with an error)
--   -------------------------------------------------------------
--   mAxilReadSlaves(TEST_INDEX_C)  <= AXI_LITE_READ_SLAVE_EMPTY_OK_C;
--   mAxilWriteSlaves(TEST_INDEX_C) <= AXI_LITE_WRITE_SLAVE_EMPTY_OK_C;


   ------------------------------------------
   -- Generate clocks from 156.25 MHz PGP  --
   ------------------------------------------
   -- clkIn     : 156.25 MHz PGP
   -- clkOut(0) : 100.00 MHz app clock
   -- clkOut(1) : 100.00 MHz asic clock
   -- clkOut(2) : 100.00 MHz asic clock
   -- clkOut(3) : 300.00 MHz idelay control clock
   -- clkOut(4) :  50.00 MHz monitoring adc
   U_CoreClockGen : entity surf.ClockManagerUltraScale
   generic map(
      TPD_G                  => 1 ns,
      TYPE_G                 => "MMCM",  -- or "PLL"
      INPUT_BUFG_G           => true,
      FB_BUFG_G              => true,
      RST_IN_POLARITY_G      => '1',     -- '0' for active low
      NUM_CLOCKS_G           => 3,
      -- MMCM attributes
      BANDWIDTH_G            => "OPTIMIZED",
--      CLKIN_PERIOD_G         => 6.4,    -- Input period in ns );
      CLKIN_PERIOD_G         => 8.0,
      DIVCLK_DIVIDE_G        => 5,
      CLKFBOUT_MULT_F_G      => 36.0,
      CLKFBOUT_MULT_G        => 5,
      CLKOUT0_DIVIDE_F_G     => 9.0,
      CLKOUT0_DIVIDE_G       => 9,
      CLKOUT1_DIVIDE_G       => 9,
      CLKOUT2_DIVIDE_G       => 3,
      CLKOUT0_PHASE_G        => 0.0,
      CLKOUT1_PHASE_G        => 0.0,
      CLKOUT2_PHASE_G        => 0.0,
      CLKOUT0_DUTY_CYCLE_G   => 0.5,
      CLKOUT1_DUTY_CYCLE_G   => 0.5,
      CLKOUT2_DUTY_CYCLE_G   => 0.5,
      CLKOUT0_RST_HOLD_G     => 3,
      CLKOUT1_RST_HOLD_G     => 3,
      CLKOUT2_RST_HOLD_G     => 3,
      CLKOUT0_RST_POLARITY_G => '1',
      CLKOUT1_RST_POLARITY_G => '1',
      CLKOUT2_RST_POLARITY_G => '1')
   port map(
      clkIn           => clk,
      rstIn           => rst,
      clkOut(0)       => appClk,
      clkOut(1)       => idelayCtrlClk,
      clkOut(2)       => adcClk,
      rstOut(0)       => appRst,
      rstOut(1)       => dummy(0),
      rstOut(2)       => dummy(1),
      locked          => clkLocked,
      -- AXI-Lite Interface
      axilClk         => appClk,
      axilRst         => appRst,
      axilReadMaster  => mAxiReadMasters(PLLREGS_AXI_INDEX_C),
      axilReadSlave   => mAxiReadSlaves(PLLREGS_AXI_INDEX_C),
      axilWriteMaster => mAxiWriteMasters(PLLREGS_AXI_INDEX_C),
      axilWriteSlave  => mAxiWriteSlaves(PLLREGS_AXI_INDEX_C)
   );


   ---------------------
   -- Trig control    --
   ---------------------
   U_TrigControl : entity work.TrigControlAxi
   port map (
      -- Trigger outputs
      appClk         => appClk,
      appRst         => appRst,
      acqStart       => acqStart,
      dataSend       => dataSend,

      -- External trigger inputs
      runTrigger     => iRunTrigger,
      daqTrigger     => iDaqTrigger,

      -- PGP clocks and reset
      sysClk         => clk,
      sysRst         => rst,
      -- SW trigger in (from VC)
      ssiCmd         => ssiCmd,
      -- PGP RxOutType (to trigger from sideband)
      pgpRxOut       => pgpRxOut,
      -- Opcode associated with this trigger
      opCodeOut      => opCode,

      -- AXI lite slave port for register access
      axilClk           => appClk,
      axilRst           => appRst,
      sAxilWriteMaster  => mAxiWriteMasters(TRIG_REG_AXI_INDEX_C),
      sAxilWriteSlave   => mAxiWriteSlaves(TRIG_REG_AXI_INDEX_C),
      sAxilReadMaster   => mAxiReadMasters(TRIG_REG_AXI_INDEX_C),
      sAxilReadSlave    => mAxiReadSlaves(TRIG_REG_AXI_INDEX_C)
   );


   
   U_SaciClkObuf : OBUFDS port map (I => iSaciClk, O => asicSaciClk_p, OB => asicSaciClk_n);
   U_SaciCmdObuf : OBUFDS port map (I => iSaciCmd, O => asicSaciCmd_p, OB => asicSaciCmd_n);
   U_SaciRspIbuf : IBUFDS port map (I => asicSaciRsp_p, IB => asicSaciRsp_n, O => iAsicSaciRsp);
   
   --------------------------------------------
   -- SACI interface controller              --
   -------------------------------------------- 
   U_AxiLiteSaciMaster : entity surf.AxiLiteSaciMaster
   generic map (
      AXIL_CLK_PERIOD_G  => 10.0E-9, -- In units of seconds
      AXIL_TIMEOUT_G     => 1.0E-3,  -- In units of seconds
      SACI_CLK_PERIOD_G  => 2.5E-6,  -- In units of seconds
      SACI_CLK_FREERUN_G => false,
      SACI_RSP_BUSSED_G  => true,
      SACI_NUM_CHIPS_G   => 2)
   port map (
      -- SACI interface
      saciClk           => iSaciClk,
      saciCmd           => iSaciCmd,
      saciSelL          => asicSaciSel,
      saciRsp(0)        => iAsicSaciRsp,
      -- AXI-Lite Register Interface
      axilClk           => appClk,
      axilRst           => appRst,
      axilReadMaster    => mAxiReadMasters(SACIREGS_INDEX_C),
      axilReadSlave     => mAxiReadSlaves(SACIREGS_INDEX_C),
      axilWriteMaster   => mAxiWriteMasters(SACIREGS_INDEX_C),
      axilWriteSlave    => mAxiWriteSlaves(SACIREGS_INDEX_C)
   );


   ----------------------------------------
   -- SI5345 Jitter Cleaner              --
   ---------------------------------------- 
   pllInClk <= asicRdClk;
   --
   U_PLL : entity surf.Si5345
   generic map(
      TPD_G              => TPD_G,
      MEMORY_INIT_FILE_G => "Si5345-RevD-Regmap-56MHz.mem",  -- Used to initialization boot ROM
      CLK_PERIOD_G       => (1.0/100.0E+6),
      SPI_SCLK_PERIOD_G  => (1.0/1.0E+6))
   port map(
      -- Clock and Reset
      axiClk         => appClk,
      axiRst         => appRst,
      -- AXI-Lite Interface
      axiReadMaster  =>  mAxiReadMasters(CLK_JIT_CLR_REG_AXI_INDEX_C),
      axiReadSlave   => mAxiReadSlaves(CLK_JIT_CLR_REG_AXI_INDEX_C),
      axiWriteMaster => mAxiWriteMasters(CLK_JIT_CLR_REG_AXI_INDEX_C),
      axiWriteSlave  => mAxiWriteSlaves(CLK_JIT_CLR_REG_AXI_INDEX_C),
      -- SPI Interface
      coreSclk       => pllSck,
      coreSDin       => pllSdo,
      coreSDout      => pllSdi,
      coreCsb        => pllCsL
   );

  ----------------------------------------
  -- Register Control from cryo project --
  ----------------------------------------
  asicGlblRst <= iAsicGrst;
  --asicGlblRst   <= not(rst); --: out sl;
  asicPulse   <= iAsicAcq;   --: out sl;
  U_AsicR0    : OBUFDS port map (I => iAsicSR0Raw, O => asicR0_p, OB => asicR0_n);
  U_AsicSmpClk: OBUFDS port map (I => iAsicPpmat, O => asicSmpClk_p, OB => asicSmpClk_n);
  --
  U_RegControl : entity work.RegisterControl
   generic map (
      TPD_G            => TPD_G,
      EN_DEVICE_DNA_G  => false,        -- this is causing placement errors,
                                        -- needs fixing.
      BUILD_INFO_G     => BUILD_INFO_G
   )
   port map (
      axiClk         => appClk,
      axiRst         => axiRst,
      sysRst         => appRst,
      -- AXI-Lite Register Interface (axiClk domain)
      axiReadMaster  => mAxiReadMasters(APP_REG_AXI_INDEX_C),
      axiReadSlave   => mAxiReadSlaves(APP_REG_AXI_INDEX_C),
      axiWriteMaster => mAxiWriteMasters(APP_REG_AXI_INDEX_C),
      axiWriteSlave  => mAxiWriteSlaves(APP_REG_AXI_INDEX_C),
      -- Register Inputs/Outputs (axiClk domain)
      boardConfig    => boardConfig,
      -- 1-wire board ID interfaces
      serialIdIo     => serialIdIo,
      -- fast ADC clock
      adcClk         => open,
      -- ASICs acquisition signals
      acqStart       => acqStart,
      saciReadoutReq => saciPrepReadoutReq,
      saciReadoutAck => saciPrepReadoutAck,
      asicPPbe       => iAsicPpbe,
      asicPpmat      => iAsicPpmat,
      asicTpulse     => open,
      asicStart      => open,
      asicSR0        => iAsicSR0Raw,
      asicGlblRst    => iAsicGrst,
      asicSync       => iAsicSync,
      asicAcq        => iAsicAcq,
      asicVid        => open,
      errInhibit     => errInhibit
   );

   -----------------------
	 -- Clock Generator 2 --
   -----------------------
   -- clkIn     : 156.25 MHz PGP
   -- clkOut(0) : 448.00 MHz -- 8x cryo clock (default  56MHz)
   -- clkOut(1) : 112.00 MHz -- 448 clock div 4
   -- clkOut(2) : 64.00 MHz  -- 448 clock div 7
   -- clkOut(3) : 56.00 MHz  -- cryo input clock default is 56MHz
   U_iserdesClockGen : entity surf.ClockManagerUltraScale
   generic map(
      TPD_G                  => 1 ns,
      TYPE_G                 => "MMCM",  -- or "PLL"
      INPUT_BUFG_G           => true,
      FB_BUFG_G              => true,
      RST_IN_POLARITY_G      => '1',     -- '0' for active low
      NUM_CLOCKS_G           => 4,
      -- MMCM attributes
      BANDWIDTH_G            => "OPTIMIZED",
      --CLKIN_PERIOD_G         => 6.4,    -- Input period in ns );
      CLKIN_PERIOD_G         => 8.0,      -- Input period in ns );
      DIVCLK_DIVIDE_G        => 1,
      CLKFBOUT_MULT_F_G      => 10.75,
      CLKFBOUT_MULT_G        => 5,
      CLKOUT0_DIVIDE_F_G     => 3.0,
      CLKOUT0_DIVIDE_G       => 1,
      CLKOUT0_PHASE_G        => 0.0,
      CLKOUT0_DUTY_CYCLE_G   => 0.5,
      CLKOUT0_RST_HOLD_G     => 3,
      CLKOUT0_RST_POLARITY_G => '1',
      CLKOUT1_DIVIDE_G       => 12,
      CLKOUT1_PHASE_G        => 0.0,
      CLKOUT1_DUTY_CYCLE_G   => 0.5,
      CLKOUT1_RST_HOLD_G     => 3,
      CLKOUT1_RST_POLARITY_G => '1',
      CLKOUT2_DIVIDE_G       => 21,
      CLKOUT2_PHASE_G        => 0.0,
      CLKOUT2_DUTY_CYCLE_G   => 0.5,
      CLKOUT2_RST_HOLD_G     => 3,
      CLKOUT2_RST_POLARITY_G => '1',
      CLKOUT3_DIVIDE_G       => 24,
      CLKOUT3_PHASE_G        => 0.0,
      CLKOUT3_DUTY_CYCLE_G   => 0.5,
      CLKOUT3_RST_HOLD_G     => 3,
      CLKOUT3_RST_POLARITY_G => '1')
   port map(
      clkIn           => clk,
      rstIn           => rst,
      clkOut(0)       => bitClk,       --bit clk
      clkOut(1)       => deserClk,
      clkOut(2)       => byteClk,
      clkOut(3)       => asicRdClk,
      rstOut(0)       => bitRst,
      rstOut(1)       => dummy(2),
      rstOut(2)       => dummy(3),
      rstOut(3)       => asicRdClkRst,
      locked          => open,
      -- AXI-Lite Interface
      axilClk         => appClk,
      axilRst         => appRst,
      axilReadMaster  => mAxiReadMasters(PLL2REGS_AXI_INDEX_C),
      axilReadSlave   => mAxiReadSlaves(PLL2REGS_AXI_INDEX_C),
      axilWriteMaster => mAxiWriteMasters(PLL2REGS_AXI_INDEX_C),
      axilWriteSlave  => mAxiWriteSlaves(PLL2REGS_AXI_INDEX_C)
      );

   -----------------------
   -- ASICS Loop        --
   -----------------------
   adcSerial(0).chP(0) <= asicD0out_p(0);
   adcSerial(0).chN(0) <= asicD0out_n(0);
   adcSerial(0).chP(1) <= asicD1out_p(0);
   adcSerial(0).chN(1) <= asicD1out_n(0);
   adcSerial(1).chP(0) <= asicD0out_p(1);
   adcSerial(1).chN(0) <= asicD0out_n(1);
   adcSerial(1).chP(1) <= asicD1out_p(1);
   adcSerial(1).chN(1) <= asicD1out_n(1);
   --
   G_ASICS : for i in 0 to NUMBER_OF_ASICS_C-1 generate
     -------------------------------------------------------
     -- ASIC AXI stream framers
     -------------------------------------------------------
     U_AXI_ASIC : entity work.HrAdcReadoutGroup
      generic map (
        TPD_G             => TPD_G,
        NUM_CHANNELS_G    => STREAMS_PER_ASIC_C,
        SIMULATION_G      => SIMULATION_G,
        IODELAY_GROUP_G   => "DEFAULT_GROUP",
        XIL_DEVICE_G      => "ULTRASCALE",
        DEFAULT_DELAY_G   => (others => '0'),
        ADC_INVERT_CH_G   => "00000000")
      port map (
        -- Master system clock, 125Mhz
        axilClk         => appClk,
        axilRst         => appRst,
        axilWriteMaster => mAxiWriteMasters(CRYO_ASIC0_READOUT_AXI_INDEX_C+i*2),
        axilWriteSlave  => mAxiWriteSlaves(CRYO_ASIC0_READOUT_AXI_INDEX_C+i*2),
        axilReadMaster  => mAxiReadMasters(CRYO_ASIC0_READOUT_AXI_INDEX_C+i*2),
        axilReadSlave   => mAxiReadSlaves(CRYO_ASIC0_READOUT_AXI_INDEX_C+i*2),
        bitClk          => bitClk,
        byteClk         => byteClk,
        deserClk        => deserClk,
        adcClkRst       => rst,
        adcSerial       => adcSerial(i),
        adcStreamClk    => byteClk,--fClkP,--sysClk,
        adcStreams      => asicStreams(((i)*STREAMS_PER_ASIC_C)+STREAMS_PER_ASIC_C-1 downto (i*STREAMS_PER_ASIC_C)),
        adcStreamsEn_n  => adcStreamsEn_n(i),
        --monitoringSig   => monitoringSig
        monitoringSig   => open
        );

     -------------------------------------------------------------------------------
     -- generate stream frames
     -------------------------------------------------------------------------------
     U_Framers : entity work.DigitalAsicStreamAxi
       generic map(
         TPD_G               => TPD_G,
         VC_NO_G             => "0000",
         LANE_NO_G           => toSlv(i, 4),
         ASIC_NO_G           => toSlv(i, 3),
         STREAMS_PER_ASIC_G  => STREAMS_PER_ASIC_C,
         ASIC_DATA_G         => (64*16),
         ASIC_WIDTH_G        => 64,
         AXIL_ERR_RESP_G     => AXI_RESP_DECERR_C
         )
       port map(
         -- Deserialized data port
         rxClk             => byteClk, --asicRdClk, --fClkP,    --use frame clock
         rxRst             => byteClkRst,--asicRdClkRst,
         adcStreams        => asicStreams(((i)*STREAMS_PER_ASIC_C)+STREAMS_PER_ASIC_C-1 downto (i*STREAMS_PER_ASIC_C)),
         adcStreamsEn_n    => adcStreamsEn_n(i),

         -- AXI lite slave port for register access
         axilClk           => appClk,
         axilRst           => appRst,
         sAxilWriteMaster  => mAxiWriteMasters(DIG_ASIC0_STREAM_AXI_INDEX_C+i*2),
         sAxilWriteSlave   => mAxiWriteSlaves(DIG_ASIC0_STREAM_AXI_INDEX_C+i*2),
         sAxilReadMaster   => mAxiReadMasters(DIG_ASIC0_STREAM_AXI_INDEX_C+i*2),
         sAxilReadSlave    => mAxiReadSlaves(DIG_ASIC0_STREAM_AXI_INDEX_C+i*2),

         -- AXI data stream output
         axisClk           => clk,
         axisRst           => rst,
         mAxisMaster       => mAxisMastersASIC(i),
         mAxisSlave        => mAxisSlavesASIC(i),

         -- acquisition number input to the header
         acqNo             => boardConfig.acqCnt,

         -- optional readout trigger for test mode
         testTrig          => acqStart,
         errInhibit        => errInhibit
         );
   end generate;

	 -----------------------------
   U_RdPwrUpRst : entity surf.PwrUpRst
   generic map (
     SIM_SPEEDUP_G  => SIMULATION_G,
     DURATION_G     => 200000000
   )
   port map (
      clk      => byteClk,
      rstOut   => byteClkRst
   );

   idelayCtrlRst_i <= idelayCtrlRst;    --cmt_locked or
   U_IDELAYCTRL_0 : IDELAYCTRL
   generic map (
      SIM_DEVICE => "ULTRASCALE"  -- Must be set to "ULTRASCALE"
   )
   port map (
      RDY => idelayRdy,        -- 1-bit output: Ready output
      REFCLK => idelayCtrlClk, -- 1-bit input: Reference clock input
      RST => idelayCtrlRst_i   -- 1-bit input: Active high reset input. Asynchronous assert, synchronous deassert to
                               -- REFCLK.
   );

   U_IdelayCtrlReset : entity surf.RstSync
   generic map (
      TPD_G           => TPD_G,
      RELEASE_DELAY_G => 250
   )
   port map (
      clk      => idelayCtrlClk,
      asyncRst => serdesReset,
      syncRst  => idelayCtrlRst
   );

  U_SR0Synch : entity work.SR0Synchronizer
    generic map (
      TPD_G => TPD_G
      )
    port map(
      -- Master system clock
      sysClk       => bitClk,
      sysClkRst    => bitRst,
      -- DAC Data
      delay        => boardConfig.SR0Delay,
      period       => boardConfig.SR0Period,
      SR0          => iAsicSR0Raw,
      -- DAC Control Signals
      refClk       => iAsicSR0RefClk,
      SR0Out       => iAsicSR0
   );
   -----------------------------


end mapping;
