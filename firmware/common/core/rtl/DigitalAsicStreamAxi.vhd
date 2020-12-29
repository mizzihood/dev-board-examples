-------------------------------------------------------------------------------
-- Title         : DigitalAsicStreamAxi
-- Project       : Hr Detectors
-------------------------------------------------------------------------------
-- File          : DigitalAsicStreamAxi.vhd
-- Created       : 7/12/2018
-------------------------------------------------------------------------------
-- Description:
-------------------------------------------------------------------------------
-- This file is part of 'Hr ADC Development Firmware'.
-- It is subject to the license terms in the LICENSE.txt file found in the 
-- top-level directory of this distribution and at: 
--    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. 
-- No part of 'EPIX Development Firmware', including this file, 
-- may be copied, modified, propagated, or distributed except according to 
-- the terms contained in the LICENSE.txt file.
-------------------------------------------------------------------------------
-- Modification history:
-- 4/27/2017: created.
-------------------------------------------------------------------------------

LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

library surf;
use surf.StdRtlPkg.all;
use surf.AxiStreamPkg.all;
use surf.AxiLitePkg.all;
use surf.SsiPkg.all;

use work.all;

entity DigitalAsicStreamAxi is 
   generic (
      TPD_G           	: time := 1 ns;
      VC_NO_G           : slv(3 downto 0)  := "0000";
      LANE_NO_G         : slv(3 downto 0)  := "0000";
      ASIC_NO_G         : slv(2 downto 0)  := "000";
      STREAMS_PER_ASIC_G  : natural := 2;  -- 1, 2 or 6. Number of screams per ASIC HR
                                         -- prototype has 2 final version 6
      ASIC_DATA_G         : natural := (32*32); --workds
      ASIC_WIDTH_G        : natural := 32; --workds    
      AXIL_ERR_RESP_G     : slv(1 downto 0)  := AXI_RESP_DECERR_C
   );
   port ( 
      -- Deserialized data port
      rxClk             : in  sl;
      rxRst             : in  sl;
      adcStreams        : in AxiStreamMasterArray(STREAMS_PER_ASIC_G-1 downto 0);
      adcStreamsEn_n    : in slv(STREAMS_PER_ASIC_G-1 downto 0) := (others => '0');
      
      -- AXI lite slave port for register access
      axilClk           : in  sl;
      axilRst           : in  sl;
      sAxilWriteMaster  : in  AxiLiteWriteMasterType;
      sAxilWriteSlave   : out AxiLiteWriteSlaveType;
      sAxilReadMaster   : in  AxiLiteReadMasterType;
      sAxilReadSlave    : out AxiLiteReadSlaveType;
      
      -- AXI data stream output
      axisClk           : in  sl;
      axisRst           : in  sl;
      mAxisMaster       : out AxiStreamMasterType;
      mAxisSlave        : in  AxiStreamSlaveType;
      
      -- acquisition number input to the header
      acqNo             : in  slv(31 downto 0);
      
      -- optional readout trigger for test mode
      testTrig          : in  sl := '0';
      -- optional inhibit counting errors 
      -- workaround to tixel bug dropping link after R0
      -- affects only SOF error counter
      errInhibit        : in  sl := '0'
      
   );
end DigitalAsicStreamAxi;


-- Define architecture
architecture RTL of DigitalAsicStreamAxi is

   -- makes the fifo input with 2B per stream
   constant AXI_STREAM_CONFIG_I_C : AxiStreamConfigType   := ssiAxiStreamConfig(2*STREAMS_PER_ASIC_G, TKEEP_COMP_C);
   constant AXI_STREAM_CONFIG_O_C : AxiStreamConfigType   := ssiAxiStreamConfig(16, TKEEP_COMP_C);--
   constant VECTOR_OF_ONES_C  : slv(15 downto 0) := (others => '1');
   constant VECTOR_OF_ZEROS_C : slv(15 downto 0) := (others => '0');
   -- PGP3 protocol is using 128bit (check for global constant for this configuration)
   
   type StateType is (IDLE_S, HDR_S, WAIT_SOF_S, WAIT_SAMPLE_ZERO, DATA_S);
   
   type StrType is record
      state            : StateType;
      stCnt            : natural;
      testColCnt       : natural;
      testRowCnt       : natural;
      testTrig         : slv(2 downto 0);
      testMode         : slv(STREAMS_PER_ASIC_G-1 downto 0);
      forceAdcData     : sl;
      decDataBitOrder  : sl;
      decBypass        : sl;
      streamDataMode   : sl;
      sampleCounter    : slv(4 downto 0);
      sampleCntOffset  : slv(4 downto 0);
      stopDataTx       : sl;
      testBitFlip      : sl;
      frmSize          : slv(31 downto 0);
      frmMax           : slv(31 downto 0);
      frmMin           : slv(31 downto 0);
      acqNo            : Slv32Array(1 downto 0);
      frmCnt           : slv(31 downto 0);
      sofError         : slv(15 downto 0);
      eofError         : slv(15 downto 0);
      ovError          : slv(15 downto 0);
      asicDataReq      : slv(31 downto 0);
      rstCnt           : sl;
      errInhibit       : sl;
      dFifoRd          : slv(STREAMS_PER_ASIC_G-1 downto 0);
      tReady           : sl;
      axisMaster       : AxiStreamMasterType;
   end record;

   constant STR_INIT_C : StrType := (
      state           => IDLE_S,
      stCnt           => 0,
      testColCnt      => 0,
      testRowCnt      => 0,
      testTrig        => "000",
      testMode        => (others=>'0'),
      forceAdcData    => '0',
      decDataBitOrder => '0',
      decBypass       => '0',
      streamDataMode  => '0',
      sampleCounter   => (others=>'0'),
      sampleCntOffset => (others=>'0'),
      stopDataTx      => '0',
      testBitFlip     => '0',
      frmSize         => (others=>'0'),
      frmMax          => (others=>'0'),
      frmMin          => (others=>'1'),
      acqNo           => (others=>(others=>'0')),
      frmCnt          => (others=>'0'),
      sofError        => (others=>'0'),
      eofError        => (others=>'0'),
      ovError         => (others=>'0'),
      asicDataReq     => (others=>'0'),
      rstCnt          => '0',
      errInhibit      => '0',
      dFifoRd         => (others=>'0'),
      tReady          => '0',
      axisMaster      => AXI_STREAM_MASTER_INIT_C
   );
   
   type RegType is record
      testMode          : slv(STREAMS_PER_ASIC_G-1 downto 0);
      forceAdcData      : sl;
      streamDataMode    : sl;
      sampleCntOffset   : slv(4 downto 0);
      stopDataTx        : sl;
      decDataBitOrder   : sl;
      decBypass         : sl;
      frmSize           : slv(31 downto 0);
      frmMax            : slv(31 downto 0);
      frmMin            : slv(31 downto 0);
      frmCnt            : slv(31 downto 0);
      sofError          : slv(15 downto 0);
      eofError          : slv(15 downto 0);
      ovError           : slv(15 downto 0);
      asicDataReq       : slv(31 downto 0);
      rstCnt            : slv(2 downto 0);
      rstSt             : sl;
      sAxilWriteSlave   : AxiLiteWriteSlaveType;
      sAxilReadSlave    : AxiLiteReadSlaveType;
   end record RegType;

   constant REG_INIT_C : RegType := (
      testMode          => (others=>'0'),
      forceAdcData      => '0',
      streamDataMode    => '0',
      sampleCntOffset   => (others=>'0'),
      stopDataTx        => '0',
      decDataBitOrder   => '0',
      decBypass         => '0',
      frmSize           => (others=>'0'),
      frmMax            => (others=>'0'),
      frmMin            => (others=>'0'),
      frmCnt            => (others=>'0'),
      sofError          => (others=>'0'),
      eofError          => (others=>'0'),
      ovError           => (others=>'0'),
      asicDataReq       => toSlv(ASIC_DATA_G, 32),
      rstCnt            => (others=>'0'),
      rstSt             => '0',
      sAxilWriteSlave   => AXI_LITE_WRITE_SLAVE_INIT_C,
      sAxilReadSlave    => AXI_LITE_READ_SLAVE_INIT_C
   );
   
   signal r   : RegType := REG_INIT_C;
   signal rin : RegType;
   signal s   : StrType := STR_INIT_C;
   signal sin : StrType;

   signal decDataOut    : slv12Array(STREAMS_PER_ASIC_G-1 downto 0);
   signal decDataInt    : slv12Array(STREAMS_PER_ASIC_G-1 downto 0);
   signal decValidOut   : slv(STREAMS_PER_ASIC_G-1 downto 0);
   signal decSof        : slv(STREAMS_PER_ASIC_G-1 downto 0);
   signal decEof        : slv(STREAMS_PER_ASIC_G-1 downto 0);
   signal decEofe       : slv(STREAMS_PER_ASIC_G-1 downto 0);
   signal decCodeError  : slv(STREAMS_PER_ASIC_G-1 downto 0);
   signal decDispError  : slv(STREAMS_PER_ASIC_G-1 downto 0);
   
   signal dFifoRd           : slv(STREAMS_PER_ASIC_G-1 downto 0);
   signal dFifoEofe         : slv(STREAMS_PER_ASIC_G-1 downto 0);
   signal dFifoEof          : slv(STREAMS_PER_ASIC_G-1 downto 0);
   signal dFifoSof          : slv(STREAMS_PER_ASIC_G-1 downto 0);
   signal dFifoValid        : slv(STREAMS_PER_ASIC_G-1 downto 0);
   signal dFifoOut          : slv14Array(STREAMS_PER_ASIC_G-1 downto 0);
   signal dFifoEofeDec      : slv(STREAMS_PER_ASIC_G-1 downto 0);
   signal dFifoEofDec       : slv(STREAMS_PER_ASIC_G-1 downto 0);
   signal dFifoSofDec       : slv(STREAMS_PER_ASIC_G-1 downto 0);
   signal dFifoValidDec     : slv(STREAMS_PER_ASIC_G-1 downto 0);
   signal dFifoOutDec       : slv12Array(STREAMS_PER_ASIC_G-1 downto 0);
   signal dFifoEofeBypass   : slv(STREAMS_PER_ASIC_G-1 downto 0);
   signal dFifoEofBypass    : slv(STREAMS_PER_ASIC_G-1 downto 0);
   signal dFifoSofBypass    : slv(STREAMS_PER_ASIC_G-1 downto 0);
   signal dFifoValidBypass  : slv(STREAMS_PER_ASIC_G-1 downto 0);
   signal dFifoOutBypass    : slv14Array(STREAMS_PER_ASIC_G-1 downto 0);

   signal dFifoExtData  : slv(16*STREAMS_PER_ASIC_G-1 downto 0) := (others => '0');
   
   signal sAxisMaster  : AxiStreamMasterType;
   signal sAxisSlave   : AxiStreamSlaveType;
   signal imAxisMaster : AxiStreamMasterType;
   signal imAxisSlave  : AxiStreamSlaveType;
   
   signal testModeSync  : slv(STREAMS_PER_ASIC_G-1 downto 0);
   signal iRxValid      : slv(STREAMS_PER_ASIC_G-1 downto 0);
   signal acqNoSync     : slv(31 downto 0);
   
   signal rxDataCs   : slv(13 downto 0);                 -- for chipscope
   signal rxValidCs  : sl;                               -- for chipscope
   attribute keep : string;                              -- for chipscope
   attribute keep of s            : signal is "true";    -- for chipscope
   attribute keep of dFifoOut     : signal is "true";    -- for chipscope
   attribute keep of dFifoSof     : signal is "true";    -- for chipscope
   attribute keep of dFifoEof     : signal is "true";    -- for chipscope
   attribute keep of dFifoEofe    : signal is "true";    -- for chipscope
   attribute keep of dFifoValid   : signal is "true";    -- for chipscope
   attribute keep of rxDataCs     : signal is "true";    -- for chipscope
   attribute keep of rxValidCs    : signal is "true";    -- for chipscope
   attribute keep of sAxisMaster  : signal is "true";    -- for chipscope
   attribute keep of imAxisMaster : signal is "true";    -- for chipscope 
   attribute keep of imAxisSlave  : signal is "true";    -- for chipscope
   attribute keep of decSof       : signal is "true";    -- for chipscope                                                          -- 
   attribute keep of decEof       : signal is "true";    -- for chipscope 
   attribute keep of decValidOut  : signal is "true";    -- for chipscope 
   attribute keep of decDataOut   : signal is "true";    -- for chipscope 
   



begin
   
   rxDataCs <= adcStreams(0).tData(13 downto 0);     -- for chipscope
   rxValidCs <= adcStreams(0).tValid;   -- for chipscope

   mAxisMaster <= imAxisMaster;
   imAxisSlave <= mAxisSlave;

   fifoExtData_GEN : for i in 0 to STREAMS_PER_ASIC_G-1 generate
     dataExt : process(dFifoOut)
       begin
         dFifoExtData(16*i+15 downto 16*i) <= "00"&dFifoOut(i);
       end process;
   end generate;
   
   -- synchronizers
   Sync1_U : entity surf.SynchronizerVector
     generic map (
       WIDTH_G => 2)
   port map (
      clk     => rxClk,
      rst     => rxRst,
      dataIn  => s.testMode,
      dataOut => testModeSync
   );

  AcqNoSync_U : entity surf.SynchronizerVector
     generic map (
       WIDTH_G => 32)
   port map (
      clk     => axisClk,
      rst     => axisRst,
      dataIn  => acqNo,
      dataOut => acqNoSync
   );

   ----------------------------------------------------------------------------
   -- Instatiate one decoder per data stream.
   ----------------------------------------------------------------------------
   U_DECODERS : for i in 0 to STREAMS_PER_ASIC_G-1 generate
     --------------------------------------------------------------------------
     -- 12b14b decoder with SSP output
     --------------------------------------------------------------------------
     Dec12b14b_U : entity surf.SspDecoder12b14b 
       generic map (
         TPD_G          => TPD_G,
         RST_POLARITY_G => '1',
         RST_ASYNC_G    => false,
         BRK_FRAME_ON_ERROR_G => false)
       port map (
         clk         => rxClk,
         rst         => rxRst,
         dataIn      => adcStreams(i).tData(13 downto 0),
         validIn     => iRxValid(i),
         dataOut     => decDataOut(i),
         validOut    => decValidOut(i),
         sof         => decSof(i),
         eof         => decEof(i),
         eofe        => decEofe(i),
         codeError   => decCodeError(i),
         dispError   => decDispError(i)
         );

     decDataBitReorder : process(s, decDataOut)
     begin
       if s.decDataBitOrder = '0' then
         decDataInt(i) <= decDataOut(i);
       else
         decDataInt(i) <= bitReverse(decDataOut(i));
       end if;
     end process;
   
     -- disable decoder in test mode (fake ASIC data)
     iRxValid(i) <= adcStreams(i).tValid and not testModeSync(i);
   
     -- async fifo for data
     -- for synchronization and small data pipeline
     -- not to store the whole frame
     DataFifo_U : entity surf.FifoCascade
       generic map (
         GEN_SYNC_FIFO_G   => false,
         FWFT_EN_G         => true,
         ADDR_WIDTH_G      => 5,
         DATA_WIDTH_G      => 15
         )
       port map (
         -- Resets
         rst               => rxRst,
         wr_clk            => rxClk,
         wr_en             => decValidOut(i),
         din(11 downto 0)  => decDataInt(i),
         din(12)           => '0',--decEofe(i),
         din(13)           => '0',--decEof(i),
         din(14)           => decSof(i),
         --Read Ports (rd_clk domain)
         rd_clk            => axisClk,
         rd_en             => dFifoRd(i),
         dout(11 downto 0) => dFifoOutDec(i),
         dout(12)          => dFifoEofeDec(i),
         dout(13)          => dFifoEofDec(i),
         dout(14)          => dFifoSofDec(i),
         valid             => dFifoValidDec(i)
         );


     DataFifoDecBypass_U : entity surf.FifoCascade
       generic map (
         GEN_SYNC_FIFO_G   => false,
         FWFT_EN_G         => true,
         ADDR_WIDTH_G      => 5,
         DATA_WIDTH_G      => 17
         )
       port map (
         -- Resets
         rst               => rxRst,
         wr_clk            => rxClk,
         wr_en             => iRxValid(i),
         din(13 downto 0)  => adcStreams(i).tData(13 downto 0),
         din(14)           => '0', --decEofe(i),
         din(15)           => '0', --decEof(i),
         din(16)           => '0', --decSof(i),
         --Read Ports (rd_clk domain)
         rd_clk            => axisClk,
         rd_en             => dFifoRd(i),
         dout(13 downto 0) => dFifoOutBypass(i),
         dout(14)          => dFifoEofeBypass(i),
         dout(15)          => dFifoEofBypass(i),
         dout(16)          => dFifoSofBypass(i),
         valid             => dFifoValidBypass(i)
         );

     decByPassMux : process(s,dFifoOutDec,    dFifoEofeDec,    dFifoEofDec,    dFifoSofDec,    dFifoValidDec,
                              dFifoOutBypass, dFifoEofeBypass, dFifoEofBypass, dFifoSofBypass, dFifoValidBypass)
     begin
       if s.decBypass = '0' then
         dFifoOut(i)    <= "00"&dFifoOutDec(i);
         dFifoEofe(i)   <= dFifoEofeDec(i);
         dFifoEof(i)    <= dFifoEofDec(i);
         dFifoSof(i)    <= dFifoSofDec(i);
         dFifoValid(i)  <= dFifoValidDec(i);
       else
         dFifoOut(i)    <= dFifoOutBypass(i);
         dFifoEofe(i)   <= dFifoEofeBypass(i);
         dFifoEof(i)    <= dFifoEofBypass(i);
         dFifoSof(i)    <= dFifoSofBypass(i);
         dFifoValid(i)  <= dFifoValidBypass(i);
       end if;
     end process;
   end generate;

   ----------------------------------------------------------------------------
   -- axi stream fifo
   ----------------------------------------------------------------------------
   -- must be able to store whole frame if AXIS is muxed
   ----------------------------------------------------------------------------
   AxisFifo_U: entity surf.AxiStreamFifo
   generic map(
      GEN_SYNC_FIFO_G      => false,
      FIFO_ADDR_WIDTH_G    => 16,
      SLAVE_AXI_CONFIG_G   => AXI_STREAM_CONFIG_I_C,
      MASTER_AXI_CONFIG_G  => AXI_STREAM_CONFIG_O_C
   )
   port map(
      sAxisClk    => axisClk,
      sAxisRst    => axisRst,
      sAxisMaster => sAxisMaster,
      sAxisSlave  => sAxisSlave,
      mAxisClk    => axisClk,
      mAxisRst    => axisRst,
      mAxisMaster => imAxisMaster,
      mAxisSlave  => imAxisSlave
   );



   comb : process (axilRst, axisRst, sAxilReadMaster, sAxilWriteMaster, sAxisSlave, r, s, decDataInt,
      acqNoSync, dFifoOut, dFifoExtData, dFifoValid, dFifoSof, dFifoEof, dFifoEofe, testTrig, errInhibit, adcStreamsEn_n) is
      variable sv       : StrType;
      variable rv       : RegType;
      variable regCon   : AxiLiteEndPointType;
   begin
      rv := r;    -- r is in AXI lite clock domain
      sv := s;    -- s is in AXI stream clock domain
      sv.dFifoRd := (others=>'0');
      sv.axisMaster := AXI_STREAM_MASTER_INIT_C;
      sv.testTrig(0) := testTrig;
      sv.testTrig(1) := s.testTrig(0);
      sv.testTrig(2) := s.testTrig(1);
      sv.errInhibit := errInhibit;
      
      -- cross clock sync
      
      rv.frmSize          := s.frmSize;
      rv.frmMax           := s.frmMax;
      rv.frmMin           := s.frmMin;
      rv.frmCnt           := s.frmCnt;
      rv.sofError         := s.sofError;
      rv.eofError         := s.eofError;
      rv.ovError          := s.ovError;
      sv.testMode         := r.testMode;
      sv.forceAdcData     := r.forceAdcData;
      sv.decDataBitOrder  := r.decDataBitOrder;
      sv.decBypass        := r.decBypass;
      sv.stopDataTx       := r.stopDataTx;
      sv.streamDataMode   := r.streamDataMode;
      sv.asicDataReq      := r.asicDataReq;
      sv.sampleCntOffset  := r.sampleCntOffset;

      
      if r.rstCnt /= "000" then
         sv.rstCnt := '1';
      else
         sv.rstCnt := '0';
      end if;
      
      -- axi lite logic 
      rv.rstCnt := r.rstCnt(1 downto 0) & '0';
      axiSlaveWaitTxn(regCon, sAxilWriteMaster, sAxilReadMaster, rv.sAxilWriteSlave, rv.sAxilReadSlave);
      
      axiSlaveRegisterR(regCon, x"00",  0, r.frmCnt);
      axiSlaveRegisterR(regCon, x"04",  0, r.frmSize);
      axiSlaveRegisterR(regCon, x"08",  0, r.frmMax);
      axiSlaveRegisterR(regCon, x"0C",  0, r.frmMin);
      axiSlaveRegisterR(regCon, x"10",  0, r.sofError);
      axiSlaveRegisterR(regCon, x"14",  0, r.eofError);
      axiSlaveRegisterR(regCon, x"18",  0, r.ovError);
      axiSlaveRegister (regCon, x"1C",  0, rv.testMode);
      axiSlaveRegister (regCon, x"1C",  2, rv.forceAdcData);
      axiSlaveRegister (regCon, x"1C",  3, rv.decDataBitOrder);
      axiSlaveRegister (regCon, x"1C",  4, rv.decBypass);
      axiSlaveRegister (regCon, x"20",  0, rv.streamDataMode);
      axiSlaveRegister (regCon, x"20",  1, rv.stopDataTx);   
      axiSlaveRegister (regCon, x"24",  0, rv.rstCnt);
      axiSlaveRegister (regCon, x"24",  3, rv.rstSt);      
      axiSlaveRegister (regCon, x"28",  0, rv.asicDataReq);
      axiSlaveRegister (regCon, x"2C",  0, rv.sampleCntOffset);

      for i in 0 to STREAMS_PER_ASIC_G-1 loop
        axiSlaveRegisterR(regCon, X"80"+toSlv((i*4), 8), 0, decDataInt(i));
      end loop;
      
      axiSlaveDefault(regCon, rv.sAxilWriteSlave, rv.sAxilReadSlave, AXIL_ERR_RESP_G);
      
      -- axi stream logic
      
      -- sync acquisition number
      sv.acqNo(0) := acqNoSync;
      
      -- report an error when sAxisSlave.tReady is dropped (overflow)
      sv.tReady := sAxisSlave.tReady;
      if sAxisSlave.tReady = '0' and s.tReady = '1' then
         sv.ovError := s.ovError + 1;
      end if;
      
      case s.state is
         when IDLE_S =>
           --------------------------------------------------------------------
           -- FIRST frame mode trigger option
           --------------------------------------------------------------------
           -- starts data tx when at least one stream  have DV and SOF, different data
           -- delays should be acommodate by the fifo
           --------------------------------------------------------------------
           -- SECOND frame mode trigger option
           --------------------------------------------------------------------
           -- in test mode and rising edge of testTrig signal
           --------------------------------------------------------------------
           -- STREAM mode
           --------------------------------------------------------------------
           -- when a frame is finished, the next one starts. 
           --------------------------------------------------------------------
           -- OBS
           --------------------------------------------------------------------
           -- if module is desabled data is not set ever as indicated by
           -- stopDataTx signal
            if (
                ((dFifoValid(STREAMS_PER_ASIC_G-1 downto 0) /= VECTOR_OF_ZEROS_C(STREAMS_PER_ASIC_G-1 downto 0) and dFifoSof(STREAMS_PER_ASIC_G-1 downto 0) /= VECTOR_OF_ZEROS_C(STREAMS_PER_ASIC_G-1 downto 0))
                 or (s.testMode(STREAMS_PER_ASIC_G-1 downto 0) /= VECTOR_OF_ZEROS_C(STREAMS_PER_ASIC_G-1 downto 0) and s.testTrig(1) = '1' and s.testTrig(2) = '0'))
                and (s.stopDataTx='0'))  then
               -- start sending the header
               -- do not read the fifo yet
               sv.acqNo(1) := s.acqNo(0);
               sv.state := WAIT_SOF_S;
               sv.testBitFlip := '0';
               sv.testColCnt := 0;
               sv.testRowCnt := 0;
            elsif s.streamDataMode='1' and s.testTrig(1) = '1' and s.testTrig(2) = '0' then
               sv.acqNo(1) := s.acqNo(0);
               sv.state := WAIT_SAMPLE_ZERO;
               sv.testBitFlip := '0';
               sv.testColCnt := 0;
               sv.testRowCnt := 0;
            elsif dFifoValid /= VECTOR_OF_ZEROS_C(STREAMS_PER_ASIC_G-1 downto 0) then
               -- should only happen if we are in stream mode, else in frame
               -- mode it should not happen
               -- ADD logic for stream mode
               -- In frame mode:
               -- dump data and report an error
               if s.errInhibit = '0' then
                  sv.sofError := s.sofError + 1;
               end if;
               if dFifoValid = VECTOR_OF_ONES_C(STREAMS_PER_ASIC_G-1 downto 0) then
                 sv.dFifoRd := (others=>'1');
               end if;
            else
               if dFifoValid = VECTOR_OF_ONES_C(STREAMS_PER_ASIC_G-1 downto 0) then                
                 sv.dFifoRd := (others=>'1');
               end if;
            end if;

         when WAIT_SOF_S =>
           if ((dFifoSof(STREAMS_PER_ASIC_G-1 downto 0) or adcStreamsEn_n) = VECTOR_OF_ONES_C(STREAMS_PER_ASIC_G-1 downto 0)) then
             if s.streamDataMode='1' then
               sv.state := IDLE_S;
             else
               sv.state := HDR_S;
             end if;
           end if;
           --keeps flushing data until all SOF show up
           for i in 0 to (STREAMS_PER_ASIC_G-1) loop
             if dFifoSof(i) = '0' then
               sv.dFifoRd(i) := '1';
             end if;             
           end loop;  -- i

           
        when WAIT_SAMPLE_ZERO =>
           if (s.sampleCounter = s.sampleCntOffset) then -- and (dFifoSof(STREAMS_PER_ASIC_G-1 downto 0) = VECTOR_OF_ZEROS_C(STREAMS_PER_ASIC_G-1 downto 0)) then
             sv.state := HDR_S;
           else
             --keeps flushing data until all SOF show up
             if dFifoValid = VECTOR_OF_ONES_C(STREAMS_PER_ASIC_G-1 downto 0) then
               sv.dFifoRd := (others=>'1');
             end if;
           end if;
            
         -- header is 6 x 16 bit words
         when HDR_S =>
           --------------------------------------------------------------------
           -- HEADER for 1 STREAM
           --------------------------------------------------------------------
           if STREAMS_PER_ASIC_G = 1 then        
             sv.axisMaster.tValid := '1';
             if s.stCnt = 5 then
               sv.stCnt := 1;
               sv.state := DATA_S;
             else
               sv.stCnt := s.stCnt + 1;
             end if;
             if s.stCnt = 0 then
               sv.axisMaster.tData(15 downto 0) := x"00" & LANE_NO_G & VC_NO_G;
               ssiSetUserSof(AXI_STREAM_CONFIG_I_C, sv.axisMaster, '1');
             elsif s.stCnt = 1 then
               sv.axisMaster.tData(15 downto 0) := x"0000";
             elsif s.stCnt = 2 then
               sv.axisMaster.tData(15 downto 0) := s.acqNo(1)(15 downto 0);
             elsif s.stCnt = 3 then
               sv.axisMaster.tData(15 downto 0) := s.acqNo(1)(31 downto 16);
             elsif s.stCnt = 4 then
               if s.testMode /= VECTOR_OF_ONES_C(STREAMS_PER_ASIC_G-1 downto 0) then
                 sv.axisMaster.tData(15 downto 0) := x"000" & '0' & ASIC_NO_G;
               else
                 sv.axisMaster.tData(15 downto 0) := x"000" & '0' & ASIC_NO_G;
               end if;
             else
               sv.axisMaster.tData(15 downto 0) := toSlv(STREAMS_PER_ASIC_G, 16);
             end if;
           end if;
           ------------------------------------------------------------------
           -- HEADER for 2 Streams
           ------------------------------------------------------------------
           if STREAMS_PER_ASIC_G = 2 then        
             sv.axisMaster.tValid := '1';
             if s.stCnt = 2 then
               sv.stCnt := 1;
               sv.state := DATA_S;
             else
               sv.stCnt := s.stCnt + 1;
             end if;
             if s.stCnt = 0 then
               sv.axisMaster.tData(31 downto 0) := x"0000" & x"00" & LANE_NO_G & VC_NO_G;
               ssiSetUserSof(AXI_STREAM_CONFIG_I_C, sv.axisMaster, '1');
             elsif s.stCnt = 1 then
               sv.axisMaster.tData(31 downto 0) := s.acqNo(1)(31 downto 0);
             elsif s.stCnt = 2 then
               if s.testMode /= VECTOR_OF_ONES_C(STREAMS_PER_ASIC_G-1 downto 0) then
                 sv.axisMaster.tData(15 downto 0) := x"000" & '0' & ASIC_NO_G;
               else
                 sv.axisMaster.tData(15 downto 0) := x"000" & '0' & ASIC_NO_G;
               end if;
               sv.axisMaster.tData(31 downto 16) := toSlv(STREAMS_PER_ASIC_G, 16);
             else
               sv.axisMaster.tData(31 downto 0) := x"00000000";
             end if;
           end if;
           ------------------------------------------------------------------
           -- HEADER for 6 Streams
           ------------------------------------------------------------------
           if STREAMS_PER_ASIC_G = 6 then        
             sv.axisMaster.tValid := '1';
             sv.state := DATA_S;
             sv.axisMaster.tData(31 downto  0) := x"0000" & x"00" & LANE_NO_G & VC_NO_G;
             sv.axisMaster.tData(63 downto 32) := s.acqNo(1)(31 downto 0);
             if s.testMode /= VECTOR_OF_ONES_C(STREAMS_PER_ASIC_G-1 downto 0) then
               sv.axisMaster.tData(79 downto 64) := x"000" & '0' & ASIC_NO_G;
             else
               sv.axisMaster.tData(79 downto 64) := x"000" & '0' & ASIC_NO_G;
             end if;
             sv.axisMaster.tData(95 downto 80) := x"0000";
             ssiSetUserSof(AXI_STREAM_CONFIG_I_C, sv.axisMaster, '1');                    
           end if;
             
         when DATA_S =>
           if (s.testMode /= VECTOR_OF_ZEROS_C(STREAMS_PER_ASIC_G-1 downto 0)) or
             (s.forceAdcData = '1' and  dFifoValid /= VECTOR_OF_ZEROS_C(STREAMS_PER_ASIC_G-1 downto 0)) then
             -- test mode row and col counters
             if s.testColCnt < (ASIC_WIDTH_G/STREAMS_PER_ASIC_G)-1 then  -- why -1 here??
               sv.testColCnt := s.testColCnt + 1;
             else
               sv.testColCnt := 0;
               sv.testRowCnt := s.testRowCnt + 1;
             end if;
               
             sv.axisMaster.tValid := '1';
             -- test or real data readout
             if s.forceAdcData = '1' then
               sv.axisMaster.tData(16*STREAMS_PER_ASIC_G-1 downto 0) := dFifoExtData;
             else
               sv.axisMaster.tData(16*STREAMS_PER_ASIC_G-1 downto 0) := (others => '0');
               if s.testColCnt = ASIC_WIDTH_G/(2*STREAMS_PER_ASIC_G)-1 then
                 sv.axisMaster.tData(15 downto 0) := ASIC_NO_G(1 downto 0) & s.testBitFlip & s.acqNo(1)(4 downto 0) & "00" & toSlv(s.testRowCnt, 6);
               elsif s.testRowCnt = 15 then
                 sv.axisMaster.tData(15 downto 0) := ASIC_NO_G(1 downto 0) & s.testBitFlip & s.acqNo(1)(4 downto 0) & "00" & toSlv(s.testColCnt, 6);
               else
                 sv.axisMaster.tData(15 downto 0) := ASIC_NO_G(1 downto 0) & s.testBitFlip & "00000" & x"ff";
               end if;
             end if;
             sv.dFifoRd := (others=>'1');                           
             sv.stCnt := s.stCnt + 1;
             -- data package completed
             if s.stCnt = s.asicDataReq then 
               sv.frmSize := toSlv(s.stCnt, 32);
               sv.stCnt := 0;
               if s.frmMax <= sv.frmSize then
                 sv.frmMax := sv.frmSize;
               end if;
               if s.frmMin >= sv.frmSize then
                 sv.frmMin := sv.frmSize;
               end if;                 
               sv.frmCnt := s.frmCnt + 1;                 
               sv.axisMaster.tLast := '1';           
               sv.state := IDLE_S;
             end if;               
           else
             --read data
             if (dFifoValid or adcStreamsEn_n) = VECTOR_OF_ONES_C(STREAMS_PER_ASIC_G-1 downto 0) then             
               sv.axisMaster.tValid := '1';
               sv.axisMaster.tData(16*STREAMS_PER_ASIC_G-1 downto 0) := dFifoExtData;                           
               sv.dFifoRd := (others=>'1');                       
               sv.stCnt := s.stCnt + 1;
               if ((dFifoEof /= VECTOR_OF_ZEROS_C(STREAMS_PER_ASIC_G-1 downto 0) or dFifoEofe /= VECTOR_OF_ZEROS_C(STREAMS_PER_ASIC_G-1 downto 0)) or s.stCnt = s.asicDataReq) then 
                 sv.frmSize := toSlv(s.stCnt, 32);
                 sv.stCnt := 0;
                 if s.frmMax <= sv.frmSize then
                   sv.frmMax := sv.frmSize;
                 end if;
                 if s.frmMin >= sv.frmSize then
                   sv.frmMin := sv.frmSize;
                 end if;
                     
                 if dFifoEofe /= VECTOR_OF_ZEROS_C(STREAMS_PER_ASIC_G-1 downto 0) or sv.frmSize /= s.asicDataReq then
                   ssiSetUserEofe(AXI_STREAM_CONFIG_I_C, sv.axisMaster, '1');
                   sv.eofError := s.eofError + 1;
                 else
                   sv.frmCnt := s.frmCnt + 1;
                 end if;
                 sv.axisMaster.tLast := '1';
                 --change of state is required if running in frame mode
                 if s.streamDataMode = '0' then
                   sv.state := IDLE_S;
                 else
                   if s.stCnt = s.asicDataReq then
                     sv.state := IDLE_S;
                   end if;
                 end if;
               end if;               
             end if;
           end if;
         when others =>
            sv.state := IDLE_S;
      end case;

      -- sampleCounter
      if s.dFifoRd = VECTOR_OF_ONES_C(STREAMS_PER_ASIC_G-1 downto 0) then
        sv.sampleCounter := s.sampleCounter + '1';
      elsif dFifoSof(STREAMS_PER_ASIC_G-1 downto 0) /= VECTOR_OF_ZEROS_C(STREAMS_PER_ASIC_G-1 downto 0) then       
        sv.sampleCounter := (others=>'0');
      end if;
      
      -- reset counters
      if s.rstCnt = '1' then
         sv.frmCnt   := (others=>'0');
         sv.frmSize  := (others=>'0');
         sv.frmMax   := (others=>'0');
         sv.frmMin   := (others=>'1');
         sv.eofError := (others=>'0');
         sv.sofError := (others=>'0');
         sv.ovError  := (others=>'0');
      end if;

      -- reset state
      if r.rstSt = '1' then
        sv.state := IDLE_S;
      end if;
      
      -- reset logic    
      if (axilRst = '1') then
         rv := REG_INIT_C;
      end if;
      if (axisRst = '1') then
         sv := STR_INIT_C;
      end if;

      -- outputs
      
      rin <= rv;
      sin <= sv;

      sAxilWriteSlave   <= r.sAxilWriteSlave;
      sAxilReadSlave    <= r.sAxilReadSlave;
      sAxisMaster       <= s.axisMaster;
      dFifoRd           <= sv.dFifoRd;

   end process comb;

   rseq : process (axilClk) is
   begin
      if (rising_edge(axilClk)) then
         r <= rin after TPD_G;
      end if;
   end process rseq;
   
   sseq : process (axisClk) is
   begin
      if (rising_edge(axisClk)) then
         s <= sin after TPD_G;
      end if;
   end process sseq;
   

end RTL;

