`timescale 1ns / 10ps

module i2c_if
  (bus_clk, quiesce, user_w_write_8_wren, user_w_write_8_data,
   user_w_write_8_full, user_w_write_8_open, user_r_read_8_rden,
   user_r_read_8_data, user_r_read_8_empty, user_r_read_8_eof,
   user_r_read_8_open,
   i2c_clk, i2c_data);

   input         bus_clk;
   input 	 quiesce;
   input 	 user_w_write_8_wren;
   input [7:0] 	 user_w_write_8_data;
   input 	 user_w_write_8_open;
   input 	 user_r_read_8_rden;
   input 	 user_r_read_8_open;
   output 	 user_w_write_8_full;
   output [7:0]  user_r_read_8_data;
   output 	 user_r_read_8_empty;
   output 	 user_r_read_8_eof;
   inout 	 i2c_clk;
   inout 	 i2c_data;

   reg [15:0] 	 div_counter;
   reg 		 sclk_logic;
   reg 		 sdata_logic;
   reg 		 sdata_sample;
   reg 		 i2c_en;
   reg 		 pre_en;
   reg [3:0] 	 state;
   reg 		 first;
   reg 		 dir_write;
   reg 		 save_direction;
   reg [7:0] 	 write_byte;
   reg [7:0] 	 read_byte;
   reg [2:0] 	 idx;
   reg 		 write_byte_valid;
   reg 		 fifo_wr_en;
   reg 		 open_d;
   reg 		 stop_pending;
   reg 		 stop_deferred;
   reg 		 do_restart;

   wire 	 i2c_data_in;

   parameter 	 clk_freq = 100; // In MHz, nearest integer

   parameter st_idle = 0,
	       st_start = 1,
	       st_fetch = 2,
	       st_bit0 = 3,
	       st_bit1 = 4,
	       st_bit2 = 5,
	       st_ack0 = 6,
	       st_ack1 = 7,
	       st_ack2 = 8,
	       st_stop0 = 9,
	       st_stop1 = 10,
	       st_stop2 = 11, // Represented by "default"
	       st_startstop = 12;

   assign i2c_clk = sclk_logic;

   // Emulated open collector output
   (* PULLUP = "TRUE" *) IOBUF i2c_iobuf
     (.I(1'b0), .O(i2c_data_in), .T(sdata_logic), .IO(i2c_data));

   assign user_r_read_8_eof = 0;
   assign user_w_write_8_full = write_byte_valid || stop_pending;

   // i2c_en should be high every 10 us
   // This allows a huge multicycle path constraint on i2c_en

   // A stop condition is presented on the bus when
   // * in a write access, the write stream closes, and the read stream
   //   is already closed, or
   // * in a read access, the write stream closes
   // * a stop condition was prevented previously by an open read stream,
   //   and the read stream closes (in which case a start-stop is presented).

   always @(posedge bus_clk)
     begin
	i2c_en <= pre_en;
	sdata_sample <= i2c_data_in;
	fifo_wr_en <= i2c_en && (state == st_ack0) && !dir_write;
	open_d <= user_w_write_8_open;

	if (open_d && !user_w_write_8_open)
	  begin
	     stop_pending <= 1;
	     do_restart <= user_r_read_8_open;
	  end

	if (user_w_write_8_wren)
	  begin
	     write_byte <= user_w_write_8_data;
	     write_byte_valid <= 1; // Zeroed by state machine
	  end

	if (div_counter == ((clk_freq * 10) - 1))
	  begin
	     div_counter <= 0;
	     pre_en <= 1;
	  end
	else
	  begin
	     div_counter <= div_counter + 1;
	     pre_en <= 0;
	  end

	// State machine

 	if (i2c_en)
	  case (state)
	    st_idle:
	      begin
		 sclk_logic <= 1;
		 sdata_logic <= 1;

		 stop_pending <= 0;

		 if (write_byte_valid)
		   state <= st_start;

		 // st_startstop is invoked only if the stream for reading data
		 // was open during the write session, which indicates that the
		 // next cycle will be a read session. This prevented a
		 // stop condition, so a restart can takes place. But then
		 // this stream closed suddenly without this read session,
		 // so a dirty stop condition needs to be inserted.

		 if (stop_deferred && !user_r_read_8_open)
		   state <= st_startstop;
	      end

	    st_start:
	      begin
		 sdata_logic <= 0; // Start condition
		 first <= 1;
		 dir_write <= 1;
		 stop_deferred <= 0;

		 state <= st_fetch;
	      end

	    st_fetch:
	      begin
		 sclk_logic <= 0;
		 idx <= 7;

		 state <= st_bit0;
	      end

	    st_bit0:
	      begin
		 if (dir_write)
		   sdata_logic <= write_byte[idx];
		 else
		   sdata_logic <= 1; // Keep clear when reading

		 state <= st_bit1;
	      end

	    st_bit1:
	      begin
		 sclk_logic <= 1;
		 read_byte[idx] <= sdata_sample;

		 state <= st_bit2;
	      end

	    st_bit2:
	      begin
		 sclk_logic <= 0;

		 idx <= idx - 1;

		 if (idx != 0)
		   state <= st_bit0;
		 else
		   state <= st_ack0;
	      end

	    st_ack0:
	      // Don't handle the last ACK cycle until the outcome is known.
	      // This allows a NAK on the last received byte, and also ensures
	      // a stop condition at the end of a write cycle (and not a
	      // restart if the file was reopened by host for the next cycle).

	      if (write_byte_valid || stop_pending)
		begin
		   if (dir_write)
		     sdata_logic <= 1; // The slave should ack
		   else
		     sdata_logic <= stop_pending; // We ack on read

		   save_direction <= !write_byte[0];
		   state <= st_ack1;
		end

	    st_ack1:
	      if (!dir_write || !sdata_sample || stop_pending)
		begin
		   state <= st_ack2; // Read mode or slave acked. Or Quit.
		   write_byte_valid <= 0;
		end

	    st_ack2:
	      begin
		 sclk_logic <= 1;

		 if (write_byte_valid)
		   begin
		      if (first)
			dir_write <= save_direction;
		      first <= 0;

		      state <= st_fetch;
		   end
		 else if (stop_pending)
		   state <= st_stop0;
	      end

	    // The three stop states toggle the clock once, so that
	    // we're sure that the slave has released the bus, leaving
	    // its acknowledge state. Used only in write direction.

	    st_stop0:
	      begin
		 sclk_logic <= 0;
		 state <= st_stop1;
	      end

	    st_stop1:
	      begin
		 if (do_restart && dir_write)
		   begin
		      sdata_logic <= 1; // Avoid stop condition
		      stop_deferred <= 1;
		   end
		 else
		   begin
		      sdata_logic <= 0;
		   end
		 state <= st_stop2;
	      end

	    st_startstop:
	      begin
		 sdata_logic <= 0;
		 stop_deferred <= 0;
		 state <= st_idle;
	      end

	    default: // Normally this is st_stop2
	      begin
		 sclk_logic <= 1;

		 write_byte_valid <= 0;

		 // st_idle will raise sdata to '1', making a stop condition
		 // unless sdata_logic was driven low in st_stop1
		 state <= st_idle;
	      end
	  endcase

	if (quiesce) // Override all above.
	  begin
	     state <= st_idle;
	     stop_pending <= 0;
	     write_byte_valid <= 0;
	     stop_deferred <= 0;
	  end
     end

   fifo_8x2048 fifo
     (
      .clk(bus_clk),
      .srst(!user_r_read_8_open),
      .din(read_byte),
      .wr_en(fifo_wr_en),
      .rd_en(user_r_read_8_rden),
      .dout(user_r_read_8_data),
      .full(),
      .empty(user_r_read_8_empty));
endmodule
