-- Copyright (c) 2017 Nuand LLC
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this software and associated documentation files (the "Software"), to deal
-- in the Software without restriction, including without limitation the rights
-- to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
-- copies of the Software, and to permit persons to whom the Software is
-- furnished to do so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in
-- all copies or substantial portions of the Software.
--
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
-- OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
-- THE SOFTWARE.

library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

library work;
    use work.fifo_readwrite_p.all;
    use work.fx3_gpif_p.all;

entity fifo_writer is
    generic (
        NUM_STREAMS           : natural := 1;
        FIFO_USEDW_WIDTH      : natural := 12;
        FIFO_DATA_WIDTH       : natural := 32;
        META_FIFO_USEDW_WIDTH : natural := 5;
        META_FIFO_DATA_WIDTH  : natural := 128
    );
    port (
        clock               :   in      std_logic;
        reset               :   in      std_logic;
        enable              :   in      std_logic;

        usb_speed           :   in      std_logic;
        meta_en             :   in      std_logic;
        packet_en           :   in      std_logic;
        eight_bit_mode_en   :   in      std_logic := '0';
        highly_packed_mode_en : in      std_logic;
        timestamp           :   in      unsigned(63 downto 0);
        mini_exp            :   in      std_logic_vector(1 downto 0);

        in_sample_controls  :   in      sample_controls_t(0 to NUM_STREAMS-1) := (others => SAMPLE_CONTROL_DISABLE);
        in_samples          :   in      sample_streams_t(0 to NUM_STREAMS-1)  := (others => ZERO_SAMPLE);

        fifo_usedw          :   in      std_logic_vector(FIFO_USEDW_WIDTH-1 downto 0);
        fifo_clear          :   buffer  std_logic;
        fifo_write          :   buffer  std_logic := '0';
        fifo_full           :   in      std_logic;
        fifo_data           :   out     std_logic_vector(FIFO_DATA_WIDTH-1 downto 0) := (others => '0');

        packet_control      :   in      packet_control_t;
        packet_ready        :   out     std_logic;

        meta_fifo_full      :   in     std_logic;
        meta_fifo_usedw     :   in     std_logic_vector(META_FIFO_USEDW_WIDTH-1 downto 0);
        meta_fifo_data      :   out    std_logic_vector(META_FIFO_DATA_WIDTH-1 downto 0) := (others => '0');
        meta_fifo_write     :   out    std_logic := '0';

        overflow_led        :   buffer  std_logic;
        overflow_count      :   buffer  unsigned(63 downto 0);
        overflow_duration   :   in      unsigned(15 downto 0)
    );
end entity;

architecture simple of fifo_writer is

    constant DMA_BUF_SIZE_SS   : natural   := GPIF_BUF_SIZE_SS;
    constant DMA_BUF_SIZE_HS   : natural   := GPIF_BUF_SIZE_HS;

    signal dma_buf_size        : natural range DMA_BUF_SIZE_HS to DMA_BUF_SIZE_SS := DMA_BUF_SIZE_SS;

    signal fifo_enough         : boolean   := false;
    signal overflow_detected   : std_logic := '0';

    type meta_state_t is (
        IDLE,
        PACKET_WAIT_EOP
    );

    type meta_fsm_t is record
        state           : meta_state_t;
        dma_downcount   : natural range 0 to 65536;
        meta_write      : std_logic;
        meta_data       : std_logic_vector(meta_fifo_data'range);
        meta_written    : std_logic;
    end record;

    constant META_FSM_RESET_VALUE : meta_fsm_t := (
        state           => IDLE,
        dma_downcount   => 0,
        meta_write      => '0',
        meta_data       => (others => '-'),
        meta_written    => '0'
    );

    signal meta_current : meta_fsm_t := META_FSM_RESET_VALUE;
    signal meta_future  : meta_fsm_t := META_FSM_RESET_VALUE;

    type fifo_state_t is (
        CLEAR,
        WRITE_SAMPLES,
        WRITE_PACKET_PAYLOAD
    );

    -- Sample sets (one sample from each enabled channel) per DMA buffer. The
    -- largest is 8-bit mode with one channel: 2 samples per 32-bit word.
    constant UNITS_PER_BUF_MAX : natural := 2 * DMA_BUF_SIZE_SS;

    signal units_per_buf       : natural range 1 to UNITS_PER_BUF_MAX := 1;

    type ch_offsets_t is array( natural range <> ) of natural range fifo_data'low to fifo_data'high;

    type fifo_fsm_t is record
        state               : fifo_state_t;
        fifo_clear          : std_logic;
        fifo_write          : std_logic;
        fifo_data           : unsigned(fifo_data'range);
        fifo_12b_buf        : unsigned(95 downto 0);
        write_cycle         : natural range 0 to 3;
        samples_left        : natural range 0 to in_sample_controls'length;
        in_sample_controls_r: sample_controls_t(0 to NUM_STREAMS-1);
        in_samples_r        : sample_streams_t(0 to NUM_STREAMS-1);
        timestamp_r         : unsigned(timestamp'range);
        eight_bit_delay     : std_logic;
        units_left          : natural range 0 to UNITS_PER_BUF_MAX;
        dropping            : std_logic;
        meta_write          : std_logic;
        meta_ts             : unsigned(timestamp'range);
    end record;

    constant FIFO_FSM_RESET_VALUE : fifo_fsm_t := (
        state               => CLEAR,
        fifo_clear          => '1',
        fifo_write          => '0',
        fifo_data           => (others => '-'),
        fifo_12b_buf        => (others => '-'),
        write_cycle         => 0,
        samples_left        => 0,
        in_sample_controls_r=> (others => SAMPLE_CONTROL_DISABLE),
        in_samples_r        => (others => ZERO_SAMPLE),
        timestamp_r         => (others => '0'),
        eight_bit_delay     => '0',
        units_left          => 0,
        dropping            => '0',
        meta_write          => '0',
        meta_ts             => (others => '0')
    );

    signal fifo_current : fifo_fsm_t := FIFO_FSM_RESET_VALUE;
    signal fifo_future  : fifo_fsm_t := FIFO_FSM_RESET_VALUE;

    signal sync_mini_exp: std_logic_vector(1 downto 0);

    signal meta_fifo_used_v_r : unsigned(meta_fifo_usedw'length downto 0) := (others => '0');
    signal fifo_used_v_r      : unsigned(fifo_usedw'length downto 0) := (others => '0');
    signal fifo_room_thresh   : natural range 0 to 2**fifo_usedw'length := 0;
    signal fifo_room          : boolean := false;
    signal meta_room          : boolean := false;

begin

    -- Throw an error if port widths don't make sense
    assert (fifo_data'length >= (NUM_STREAMS*2*in_samples(in_samples'low).data_i'length) )
        report "fifo_data port width too narrow to support " & integer'image(NUM_STREAMS) & " MIMO streams."
        severity failure;

    -- Determine the DMA buffer size based on USB speed
    calc_buf_size : process( clock, reset )
    begin
        if( reset = '1' ) then
            dma_buf_size <= DMA_BUF_SIZE_SS;
        elsif( rising_edge(clock) ) then
            if( usb_speed = '0' ) then
                dma_buf_size <= DMA_BUF_SIZE_SS;
            else
                dma_buf_size <= DMA_BUF_SIZE_HS;
            end if;
        end if;
    end process;

    -- Determine how many sample sets (one sample from each enabled channel) fit
    -- in the payload of a DMA buffer. Metadata is written and samples are
    -- dropped only on these boundaries, so the GPIF always pairs each meta
    -- with the samples it describes and every buffer starts on a word (and,
    -- in highly packed mode, a 3-word group) boundary.
    calc_units_per_buf : process( clock, reset )
        variable payload : natural range 0 to DMA_BUF_SIZE_SS;  -- 32-bit words
        variable n_sets   : natural range 0 to UNITS_PER_BUF_MAX;
    begin
        if( reset = '1' ) then
            units_per_buf <= 1;
        elsif( rising_edge(clock) ) then
            if( highly_packed_mode_en = '1' ) then
                payload := dma_buf_size - 8;
            else
                payload := dma_buf_size - 4;
            end if;

            if( eight_bit_mode_en = '1' ) then
                n_sets := 2 * payload;               -- 16 bits per sample
            elsif( highly_packed_mode_en = '1' ) then
                -- 24 bits per sample: 4/3 samples per word. Only the
                -- SuperSpeed buffer size is a multiple of 3 words.
                if( dma_buf_size = DMA_BUF_SIZE_SS ) then
                    n_sets := (DMA_BUF_SIZE_SS - 8) * 4 / 3;
                else
                    n_sets := (DMA_BUF_SIZE_HS - 8) * 4 / 3;
                end if;
            else
                n_sets := payload;                   -- 32 bits per sample
            end if;

            if( count_enabled_channels(fifo_current.in_sample_controls_r) = 2 ) then
                n_sets := n_sets / 2;
            end if;

            if( n_sets = 0 ) then
                n_sets := 1;
            end if;

            units_per_buf <= n_sets;
        end if;
    end process;

    -- Calculate whether there's enough room in the destination FIFO
    --
    -- This is pipelined over three clock cycles so the comparisons stay short
    -- enough for the RX clock in OVERSAMPLE mode (245.76 MHz at 122.88 Msps):
    --   1. register the FIFO fill levels (the dcfifo outputs are not registered)
    --   2. compare each fill level against its threshold
    --   3. combine the comparison results
    -- The thresholds leave a full DMA buffer of sample space and 4 entries of
    -- meta space, so a couple of cycles of latency cannot overflow either FIFO.
    calc_fifo_free : process( clock, reset )
        constant FIFO_MAX    : natural := 2**fifo_usedw'length;
        constant META_MAX    : natural := 2**meta_fifo_usedw'length;
    begin
        if( reset = '1' ) then
            fifo_enough <= false;
            fifo_room <= false;
            meta_room <= false;
            meta_fifo_used_v_r <= (others => '0');
            fifo_used_v_r <= (others => '0');
            fifo_room_thresh <= 0;
        elsif( rising_edge(clock) ) then
            fifo_room_thresh   <= FIFO_MAX - dma_buf_size;

            fifo_used_v_r      <= unsigned(fifo_full & fifo_usedw);
            meta_fifo_used_v_r <= unsigned(meta_fifo_full & meta_fifo_usedw);

            fifo_room <= fifo_used_v_r < fifo_room_thresh;
            meta_room <= (META_MAX - 4) > meta_fifo_used_v_r or meta_en = '0';

            fifo_enough <= fifo_room and meta_room;
        end if;
    end process;

    -- ------------------------------------------------------------------------
    -- MINI EXP PIN SYNCHRONIZER
    -- ------------------------------------------------------------------------
    generate_sync_mimo_rx_en : for i in mini_exp'range generate
        U_sync_mini_exp : entity work.synchronizer
            generic map (
                RESET_LEVEL         =>  '0'
                )
            port map (
                reset               =>  '0',
                clock               =>  clock,
                async               =>  mini_exp(i),
                sync                =>  sync_mini_exp(i)
            );
    end generate;

    -- ------------------------------------------------------------------------
    -- META FIFO WRITER
    -- ------------------------------------------------------------------------

    -- Meta FIFO synchronous process
    meta_fsm_sync : process( clock, reset )
    begin
        if( reset = '1' ) then
            meta_current <= META_FSM_RESET_VALUE;
        elsif( rising_edge(clock) ) then
            meta_current <= meta_future;
        end if;
    end process;

    packet_ready <= '1' when fifo_enough else '0';

    -- Meta FIFO combinatorial process
    meta_fsm_comb : process( all )
        variable packet_flags : std_logic_vector(7 downto 0);
    begin

        meta_future            <= meta_current;

        meta_future.meta_write <= '0';
        -- currently the GPIF modules overwrites the bottom 16 bits of the flags field
        packet_flags := packet_control.pkt_flags;
        meta_future.meta_data  <= x"FFF" & "11" & sync_mini_exp & x"FFFF" & std_logic_vector(timestamp) &
                       packet_control.pkt_core_id & packet_flags &
                       std_logic_vector(to_unsigned(integer(meta_current.dma_downcount), 16));


        case meta_current.state is
            when IDLE =>

                if( highly_packed_mode_en = '1' ) then
                    meta_future.dma_downcount <= dma_buf_size - 8;
                else
                    meta_future.dma_downcount <= dma_buf_size - 4;
                end if;

                if( fifo_enough ) then
                    if( packet_en = '1' ) then
                       -- use downcount to count number of DWORDS for packets

                       if( packet_control.pkt_sop = '1' ) then
                          meta_future.state  <= PACKET_WAIT_EOP;

                          -- meta is not written yet, but there should be space
                          meta_future.meta_written <= '1';

                          -- EOP and VALID must be asserted together, count the last VALID now
                          if( packet_control.data_valid = '1') then
                             meta_future.dma_downcount <= 2;
                          else
                             meta_future.dma_downcount <= 1;
                          end if;
                       end if;
                    end if;
                else
                    meta_future.meta_written <= '0';
                end if;

            when PACKET_WAIT_EOP =>

                if( packet_control.data_valid = '1' ) then
                   meta_future.dma_downcount <= meta_current.dma_downcount + 1;

                   if( packet_control.pkt_eop = '1' ) then
                       meta_future.meta_write  <= '1';
                       meta_future.state       <= IDLE;
                   end if;
                end if;

            when others =>

                meta_future.state <= IDLE;

        end case;

        -- Abort?
        if( (enable = '0') or (meta_en = '0') or (packet_en = '0') ) then
            meta_future.meta_write    <= '0';
            meta_future.meta_written  <= '0';
            meta_future.state         <= IDLE;
        end if;

    end process;

    -- In packet mode the meta FSM above writes the metadata. In sample mode
    -- the sample FIFO writer writes it at the start of each DMA buffer.
    meta_fifo_write <= meta_current.meta_write when packet_en = '1' else
                       fifo_current.meta_write;
    meta_fifo_data  <= meta_current.meta_data when packet_en = '1' else
                       x"FFF" & "11" & sync_mini_exp & x"FFFF" &
                       std_logic_vector(fifo_current.meta_ts) & x"12344321";


    -- ------------------------------------------------------------------------
    -- SAMPLE FIFO WRITER
    -- ------------------------------------------------------------------------

    -- Sample FIFO synchronous process
    fifo_fsm_sync : process( clock, reset )
    begin
        if( reset = '1' ) then
            fifo_current <= FIFO_FSM_RESET_VALUE;
        elsif( rising_edge(clock) ) then
            fifo_current <= fifo_future;
        end if;
    end process;

    -- Sample FIFO combinatorial process
    fifo_fsm_comb : process( all )

        -- --------------------------------------------------------------------
        -- MIMO PACKER: STEP 1 of 3
        -- --------------------------------------------------------------------
        -- This block receives samples as an array of sample streams, one
        -- element per channel. These streams need to be packed into a single,
        -- wide bus that is written to a FIFO and eventually delivered to the
        -- host. When all channels are enabled, this wide bus will contain one
        -- I/Q sample pair for each channel. When channels are disabled, the
        -- wide bus may contain multiple samples from the remaining enabled
        -- channels in order to more efficiently use the available USB bandwidth.
        -- For example, a 2x2 MIMO design with 16-bit samples will pack its data
        -- into a 64-bit wide bus in one of the following ways:
        --      | 63:48 | 47:32 | 31:16 | 15:0 | Bit indices
        --   1. |   Q1  |   I1  |   Q0  |  I0  | Channels 0 & 1 enabled
        --   2. |   Q0' |   I0' |   Q0  |  I0  | Channel 0 only enabled
        --   3. |   Q1' |   I1' |   Q1  |  I1  | Channel 1 only enabled
        function pack( sc : sample_controls_t;
                       ss : sample_streams_t;
                       d  : unsigned ) return unsigned is
            constant LEN  : natural           := ss(ss'low).data_i'length + ss(ss'low).data_q'length;
            variable rv   : unsigned(d'range) := (others => '0');
        begin
            rv := d;
            for i in sc'range loop
                if( (sc(i).enable = '1') and (ss(i).data_v = '1') ) then
                    rv := unsigned(ss(i).data_q) & unsigned(ss(i).data_i) &
                          rv(rv'high downto rv'low+LEN);
                end if;
            end loop;
            return rv;
        end function;

        -- --------------------------------------------------------------------
        -- MIMO PACKER: STEP 1 of 3 (8bit mode)
        -- --------------------------------------------------------------------
        -- This block receives samples as an array of sample streams, one
        -- element per channel. These streams need to be packed into a single,
        -- wide bus that is written to a FIFO and eventually delivered to the
        -- host. When all channels are enabled, this wide bus will contain one
        -- I/Q sample pair for each channel. When channels are disabled, the
        -- wide bus may contain multiple samples from the remaining enabled
        -- channels in order to more efficiently use the available USB bandwidth.
        -- For example, a 2x2 MIMO design with 16-bit samples will pack its data
        -- into a 64-bit wide bus in one of the following ways:
        --      |          Sample Set 1         |          Sample Set 0        |
        --      | 63:56 | 55:48 | 47:40 | 39:32 | 31:24 | 23:16 |  15:8 |  7:0 | Bit indices
        --   1. |   Q1  |   I1  |   Q0  |  I0   |   Q1  |   I1  |   Q0  |  I0  | Channels 0 & 1 enabled
        --   2. |   Q0' |   I0' |   Q0  |  I0   |   Q0' |   I0' |   Q0  |  I0  | Channel 0 only enabled
        --   3. |   Q1' |   I1' |   Q1  |  I1   |   Q1' |   I1' |   Q1  |  I1  | Channel 1 only enabled
        function pack_eight_bit_mode( sc : sample_controls_t;
                                      ss : sample_streams_t;
                                      d  : unsigned ) return unsigned is
            constant IQ_PAIR_LEN  : natural := ss(ss'low).data_i'length/2 + ss(ss'low).data_q'length/2;
            variable rv           : unsigned(d'range) := (others => '0');
        begin
            rv := d;
            for i in sc'range loop
                if( (sc(i).enable = '1') and (ss(i).data_v = '1') ) then
                    rv := unsigned(ss(i).data_q(11 downto 4)) &
                          unsigned(ss(i).data_i(11 downto 4)) &
                          rv(rv'high downto rv'low+IQ_PAIR_LEN);
                end if;
            end loop;
            return rv;
        end function;


        -- --------------------------------------------------------------------
        -- MIMO PACKER: STEP 1 of 3 (SC12Q11 Highly Packed Mode)
        -- --------------------------------------------------------------------
        -- This block receives samples as an array of sample streams, one
        -- element per channel. These streams need to be packed into a single,
        -- wide bus that is written to a FIFO and delivered to the host.
        -- For SC12Q11 (SC16_Q11_PACKED) format, each I and Q sample is 12 bits wide.
        --
        -- This packing scheme efficiently utilizes the available bandwidth
        -- for the SC12Q11 format, allowing for 8 complete IQ pairs every 3 cycles.
        --
        -- The packing process fills 3 cycles of a 64-bit buffer with 8 24b IQ pairs:
        -- Cycle 1: | 63:60 | 59:48 | 47:36 | 35:24 | 23:12 | 11:0  | Bit indices
        --          |  Q0'  |   I0  |   Q1  |   I1  |   Q0  |  I0   | Samples
        --           [-----][--------full 12-bit samples-----------]
        --            4-bit
        --           overlap
        --
        -- Cycle 2: | 63:56 | 55:44 | 43:32 | 31:20 | 19:8  |  7:0  | Bit indices
        --          |  I1'  |   Q0  |   I0  |   Q1  |  I1   |  Q0'  | Samples
        --           [-----][--------full 12-bit samples--------][---]
        --            8-bit                                       8-bit
        --           overlap                                     overlap
        --
        -- Cycle 3: | 63:52 | 51:40 | 39:28 | 27:16 | 15:4  |  3:0  | Bit indices
        --          |   Q1  |   I1  |   Q0  |   I0  |  Q1   |  I1'  | Samples
        --           [--------full 12-bit samples-----------][-----]
        --                                                    4-bit
        --                                                   overlap
        --
        -- Note: Samples with apostrophes (') indicate partial samples that span
        -- across cycle boundaries. The complete samples are reconstructed by
        -- combining these partial segments from different cycles.
        function pack_sc12q11( sc : sample_controls_t;
                               ss : sample_streams_t;
                               d  : unsigned ) return unsigned is
            constant IQ_PAIR_LEN  : positive := 24;
            variable rv           : unsigned(d'range) := (others => '0');
        begin
            rv := d;
            for i in sc'range loop
                if( (sc(i).enable = '1') and (ss(i).data_v = '1') ) then
                    rv := unsigned(ss(i).data_q(11 downto 0)) &
                          unsigned(ss(i).data_i(11 downto 0)) &
                          rv(rv'high downto rv'low+IQ_PAIR_LEN);
                end if;
            end loop;
            return rv;
        end function;

        -- --------------------------------------------------------------------
        -- Accept one sample set (one sample from each enabled channel)
        -- --------------------------------------------------------------------
        -- Sample sets are counted per DMA buffer. The first one of a buffer
        -- is only accepted if there is room for the whole buffer; it then
        -- writes the buffer's metadata with its own timestamp. Otherwise
        -- sample sets are dropped until there is room. Because drops only
        -- happen on buffer boundaries, the packers are always empty and in
        -- phase when they occur.
        procedure accept_samples(
            signal fifo_current : in  fifo_fsm_t;
            signal fifo_future  : out fifo_fsm_t
        ) is
            variable sc         : sample_controls_t(0 to NUM_STREAMS-1);
            variable ss         : sample_streams_t(0 to NUM_STREAMS-1);
            variable write_req  : std_logic;
            variable buf_next   : unsigned(fifo_current.fifo_12b_buf'range);
        begin
            sc := fifo_current.in_sample_controls_r;
            ss := fifo_current.in_samples_r;

            -- Check for valid data
            write_req := '0';
            for i in sc'range loop
                if( sc(i).enable = '1' ) then
                    write_req := write_req or ss(i).data_v;
                end if;
            end loop;

            if( write_req = '1' ) then
                if( fifo_current.units_left = 0 ) then
                    if( not fifo_enough ) then
                        -- Overflow: drop sample sets until a buffer fits
                        fifo_future.dropping <= '1';
                        return;
                    end if;

                    -- First sample set of a new buffer
                    fifo_future.dropping   <= '0';
                    fifo_future.units_left <= units_per_buf - 1;
                    fifo_future.meta_write <= meta_en;
                    fifo_future.meta_ts    <= fifo_current.timestamp_r;
                else
                    fifo_future.units_left <= fifo_current.units_left - 1;
                end if;

                -- MIMO PACKER: STEP 1 of 3
                if( eight_bit_mode_en = '1' ) then
                    fifo_future.fifo_data <= pack_eight_bit_mode(sc, ss, fifo_current.fifo_data);
                elsif( highly_packed_mode_en = '1' ) then
                    buf_next := pack_sc12q11(sc, ss, fifo_current.fifo_12b_buf);
                    fifo_future.fifo_12b_buf <= buf_next;

                    case fifo_current.write_cycle is
                        when 1 =>
                            fifo_future.fifo_data <= buf_next(fifo_data'length-1 downto 0);
                        when 2 =>
                            fifo_future.fifo_data <= buf_next(fifo_data'length-1+16 downto 16);
                        when 3 =>
                            fifo_future.fifo_data <= buf_next(fifo_data'length-1+32 downto 32);
                        when others =>
                            null;
                    end case;
                else
                    fifo_future.fifo_data <= pack(sc, ss, fifo_current.fifo_data);
                end if;

                if( fifo_current.samples_left = 0 ) then
                    fifo_future.samples_left <= NUM_STREAMS - count_enabled_channels(sc);
                    fifo_future.fifo_write <= '1';

                    if( eight_bit_mode_en = '1' ) then
                        fifo_future.fifo_write <= fifo_current.eight_bit_delay;
                        fifo_future.eight_bit_delay <= not fifo_current.eight_bit_delay;
                    end if;

                    if( highly_packed_mode_en = '1' ) then
                        fifo_future.write_cycle <= (fifo_current.write_cycle + 1) mod 4;
                        if fifo_current.write_cycle = 0 then
                            fifo_future.fifo_write <= '0';
                        end if;
                    end if;
                else
                    -- MIMO PACKER: STEP 3 of 3
                    fifo_future.samples_left <= fifo_current.samples_left - 1;
                end if;
            end if;
        end;

    begin

        fifo_future            <= fifo_current;

        fifo_future.fifo_clear <= '0';
        fifo_future.fifo_write <= '0';
        fifo_future.meta_write <= '0';

        fifo_future.in_samples_r <= in_samples;
        fifo_future.in_sample_controls_r <= in_sample_controls;
        fifo_future.timestamp_r <= timestamp;

        case fifo_current.state is

            when CLEAR =>

                -- MIMO PACKER: STEP 2 of 3
                --   Compute "samples left" to fill up the fifo_data bus
                fifo_future.samples_left      <= NUM_STREAMS - count_enabled_channels(in_sample_controls);

                if( enable = '1' ) then
                    fifo_future.fifo_clear <= '0';
                    if( packet_en = '1' ) then
                       fifo_future.state      <= WRITE_PACKET_PAYLOAD;
                       fifo_future.samples_left <= NUM_STREAMS - 1;
                    else
                       fifo_future.state      <= WRITE_SAMPLES;
                    end if;
                else
                    fifo_future.fifo_clear <= '1';
                end if;

                fifo_future.eight_bit_delay <= '0';
                fifo_future.write_cycle     <= 0;
                fifo_future.units_left      <= 0;
                fifo_future.dropping        <= '0';

            when WRITE_PACKET_PAYLOAD =>

                if( meta_current.meta_written = '1' or (fifo_enough and packet_control.pkt_sop = '1' )) then
                    -- This code converts DWORD packet data into a fifo_data std_logic_vector
                    -- that is controlled by a generic.
                    if( packet_control.data_valid = '1' ) then
                        if( packet_control.pkt_eop = '1' and fifo_current.samples_left /= 0 ) then
                           -- End of packet asserted, however fifo_data is not full so
                           -- zero out the unset bits, commit what has been accumulated
                           fifo_future.samples_left <= NUM_STREAMS - 1;
                           fifo_future.fifo_write <= '1';

                           if (fifo_current.fifo_data'high > 31) then
                               fifo_future.fifo_data(fifo_future.fifo_data'high
                                           downto fifo_future.fifo_data'high - 31) <= (others => '0');
                           end if;
                           fifo_future.fifo_data(31 downto 0) <= unsigned(packet_control.data);
                        elsif( fifo_current.samples_left = 0 ) then
                           -- DWORDs perfectly filled fifo_data, commit fifo_data
                           fifo_future.samples_left <= NUM_STREAMS - 1;
                           fifo_future.fifo_write <= '1';

                           if (fifo_current.fifo_data'high > 31) then
                              fifo_future.fifo_data <= unsigned(packet_control.data) &
                                 fifo_current.fifo_data(fifo_current.fifo_data'high downto 32);
                           else
                              fifo_future.fifo_data <= unsigned(packet_control.data);
                           end if;
                        else
                           fifo_future.fifo_data(fifo_future.fifo_data'high
                                       downto fifo_future.fifo_data'high - 31) <=
                                             unsigned(packet_control.data);

                           if (fifo_current.fifo_data'high > 31) then
                               fifo_future.fifo_data(fifo_future.fifo_data'high - 32 downto 0) <=
                                        (others => '0');
                           end if;
                           fifo_future.samples_left <= fifo_current.samples_left - 1;
                        end if;

                    end if;
                end if;

            when WRITE_SAMPLES =>

                accept_samples(fifo_current, fifo_future);

            when others =>

                fifo_future.state <= CLEAR;

        end case;

        -- Abort?
        if( enable = '0' ) then
            fifo_future.fifo_clear <= '1';
            fifo_future.fifo_write <= '0';
            fifo_future.state      <= CLEAR;
        end if;

        -- Output assignments
        fifo_clear <= fifo_current.fifo_clear;
        fifo_write <= fifo_current.fifo_write;
        fifo_data  <= std_logic_vector(fifo_current.fifo_data);

    end process;


    -- ------------------------------------------------------------------------
    -- OVERFLOW
    -- ------------------------------------------------------------------------

    -- Overflow detection
    detect_overflows : process( clock, reset )
    begin
        if( reset = '1' ) then
            overflow_detected <= '0';
        elsif( rising_edge( clock ) ) then
            overflow_detected <= '0';
            if( enable = '1' and fifo_current.dropping = '1' ) then
                overflow_detected <= '1';
            end if;
        end if;
    end process;

    -- Count the number of times we overflow, but only if they are discontinuous
    -- meaning we have an overflow condition, a non-overflow condition, then
    -- another overflow condition counts as 2 overflows, but an overflow condition
    -- followed by N overflow conditions counts as a single overflow condition.
    count_overflows : process( clock, reset )
        variable prev_overflow : std_logic := '0';
    begin
        if( reset = '1' ) then
            prev_overflow  := '0';
            overflow_count <= (others =>'0');
        elsif( rising_edge( clock ) ) then
            if( prev_overflow = '0' and overflow_detected = '1' ) then
                overflow_count <= overflow_count + 1;
            end if;
            prev_overflow := overflow_detected;
        end if;
    end process;

    -- Active high assertion for overflow_duration when the overflow
    -- condition has been detected.  The LED will stay asserted
    -- if multiple overflows have occurred
    blink_overflow_led : process( clock, reset )
        variable downcount : natural range 0 to 2**overflow_duration'length-1;
    begin
        if( reset = '1' ) then
            downcount    := 0;
            overflow_led <= '0';
        elsif( rising_edge(clock) ) then
            -- Default to not being asserted
            overflow_led <= '0';

            -- Countdown so we can see what happened
            if( overflow_detected = '1' ) then
                downcount := to_integer(overflow_duration);
            elsif( downcount /= 0 ) then
                downcount := downcount - 1;
                overflow_led <= '1';
            end if;
        end if;
    end process;

end architecture;
