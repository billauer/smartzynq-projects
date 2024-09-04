module xillydemo
  (
  input clk_50,

  inout [33:0] J5,  //BANK35 V3V3
  inout [33:0] J6,  //BANK33 VADJ

  output  hdmi_clk_p,
  output  hdmi_clk_n,
  output [2:0] hdmi_d_p,
  output [2:0] hdmi_d_n,

  input  uart_rxd,
  output uart_txd,

  output MDIO_PHY_mdc,
  inout  MDIO_PHY_mdio_io,
  input  [3:0]RGMII_rd,
  input  RGMII_rx_ctl,
  input  RGMII_rxc,
  output [3:0]RGMII_td,
  output RGMII_tx_ctl,
  output RGMII_txc,

  inout EEPROM_SCL, EEPROM_SDA,

  inout LCD_SCL, LCD_SDA, LCD_CS,
  output LCD_DC, LCD_RES, LCD_BLK,

  output LED1, LED2,
  input KEY1, KEY2
  );

   localparam gpio_width = 34;
   wire [3:0]  GPIO_LED;
   wire [63:0] gpio_tri_i, gpio_tri_o, gpio_tri_t;

   assign  LED1 = GPIO_LED[0];
   assign  LED2 = gpio_tri_t[63] ? 0 : gpio_tri_o[63];
   assign  gpio_tri_i[62] = KEY2;
   assign  gpio_tri_i[61] = KEY1;

   // Wires for the vivado_system module
   wire        bus_rst_n;
   wire [31:0] S_AXI_AWADDR;
   wire        S_AXI_AWVALID;
   wire [31:0] S_AXI_WDATA;
   wire [3:0]  S_AXI_WSTRB;
   wire        S_AXI_WVALID;
   wire        S_AXI_BREADY;
   wire [31:0] S_AXI_ARADDR;
   wire        S_AXI_ARVALID;
   wire        S_AXI_RREADY;
   wire        S_AXI_ARREADY;
   wire [31:0] S_AXI_RDATA;
   wire [1:0]  S_AXI_RRESP;
   wire        S_AXI_RVALID;
   wire        S_AXI_WREADY;
   wire [1:0]  S_AXI_BRESP;
   wire        S_AXI_BVALID;
   wire        S_AXI_AWREADY;
   wire        M_AXI_ACP_ARREADY;
   wire        M_AXI_ACP_ARVALID;
   wire [31:0] M_AXI_ACP_ARADDR;
   wire [3:0]  M_AXI_ACP_ARLEN;
   wire [2:0]  M_AXI_ACP_ARSIZE;
   wire [1:0]  M_AXI_ACP_ARBURST;
   wire [2:0]  M_AXI_ACP_ARPROT;
   wire [3:0]  M_AXI_ACP_ARCACHE;
   wire        M_AXI_ACP_RREADY;
   wire        M_AXI_ACP_RVALID;
   wire [63:0] M_AXI_ACP_RDATA;
   wire [1:0]  M_AXI_ACP_RRESP;
   wire        M_AXI_ACP_RLAST;
   wire        M_AXI_ACP_AWREADY;
   wire        M_AXI_ACP_AWVALID;
   wire [31:0] M_AXI_ACP_AWADDR;
   wire [3:0]  M_AXI_ACP_AWLEN;
   wire [2:0]  M_AXI_ACP_AWSIZE;
   wire [1:0]  M_AXI_ACP_AWBURST;
   wire [2:0]  M_AXI_ACP_AWPROT;
   wire [3:0]  M_AXI_ACP_AWCACHE;
   wire        M_AXI_ACP_WREADY;
   wire        M_AXI_ACP_WVALID;
   wire [63:0] M_AXI_ACP_WDATA;
   wire [7:0]  M_AXI_ACP_WSTRB;
   wire        M_AXI_ACP_WLAST;
   wire        M_AXI_ACP_BREADY;
   wire        M_AXI_ACP_BVALID;
   wire [1:0]  M_AXI_ACP_BRESP;
   wire        host_interrupt;

   genvar      i;

  /*
   * PART 1
   * ======
   *
   * When the Xillybus IP core is replaced, this part should be replaced
   * with the new IP core's instantiation template (the content of
   * template.v)
   */

  // Clock and quiesce
  wire  bus_clk;
  wire  quiesce;


  // Wires related to /dev/xillybus_mem_8
  wire  user_r_mem_8_rden;
  wire  user_r_mem_8_empty;
  wire [7:0] user_r_mem_8_data;
  wire  user_r_mem_8_eof;
  wire  user_r_mem_8_open;
  wire  user_w_mem_8_wren;
  wire  user_w_mem_8_full;
  wire [7:0] user_w_mem_8_data;
  wire  user_w_mem_8_open;
  wire [4:0] user_mem_8_addr;
  wire  user_mem_8_addr_update;

  // Wires related to /dev/xillybus_read_32
  wire  user_r_read_32_rden;
  wire  user_r_read_32_empty;
  wire [31:0] user_r_read_32_data;
  wire  user_r_read_32_eof;
  wire  user_r_read_32_open;

  // Wires related to /dev/xillybus_read_8
  wire  user_r_read_8_rden;
  wire  user_r_read_8_empty;
  wire [7:0] user_r_read_8_data;
  wire  user_r_read_8_eof;
  wire  user_r_read_8_open;

  // Wires related to /dev/xillybus_write_32
  wire  user_w_write_32_wren;
  wire  user_w_write_32_full;
  wire [31:0] user_w_write_32_data;
  wire  user_w_write_32_open;

  // Wires related to /dev/xillybus_write_8
  wire  user_w_write_8_wren;
  wire  user_w_write_8_full;
  wire [7:0] user_w_write_8_data;
  wire  user_w_write_8_open;


  xillybus xillybus_ins (

    // Ports related to /dev/xillybus_mem_8
    // FPGA to CPU signals:
    .user_r_mem_8_rden(user_r_mem_8_rden),
    .user_r_mem_8_empty(user_r_mem_8_empty),
    .user_r_mem_8_data(user_r_mem_8_data),
    .user_r_mem_8_eof(user_r_mem_8_eof),
    .user_r_mem_8_open(user_r_mem_8_open),

    // CPU to FPGA signals:
    .user_w_mem_8_wren(user_w_mem_8_wren),
    .user_w_mem_8_full(user_w_mem_8_full),
    .user_w_mem_8_data(user_w_mem_8_data),
    .user_w_mem_8_open(user_w_mem_8_open),

    // Address signals:
    .user_mem_8_addr(user_mem_8_addr),
    .user_mem_8_addr_update(user_mem_8_addr_update),


    // Ports related to /dev/xillybus_read_32
    // FPGA to CPU signals:
    .user_r_read_32_rden(user_r_read_32_rden),
    .user_r_read_32_empty(user_r_read_32_empty),
    .user_r_read_32_data(user_r_read_32_data),
    .user_r_read_32_eof(user_r_read_32_eof),
    .user_r_read_32_open(user_r_read_32_open),


    // Ports related to /dev/xillybus_read_8
    // FPGA to CPU signals:
    .user_r_read_8_rden(user_r_read_8_rden),
    .user_r_read_8_empty(user_r_read_8_empty),
    .user_r_read_8_data(user_r_read_8_data),
    .user_r_read_8_eof(user_r_read_8_eof),
    .user_r_read_8_open(user_r_read_8_open),


    // Ports related to /dev/xillybus_write_32
    // CPU to FPGA signals:
    .user_w_write_32_wren(user_w_write_32_wren),
    .user_w_write_32_full(user_w_write_32_full),
    .user_w_write_32_data(user_w_write_32_data),
    .user_w_write_32_open(user_w_write_32_open),


    // Ports related to /dev/xillybus_write_8
    // CPU to FPGA signals:
    .user_w_write_8_wren(user_w_write_8_wren),
    .user_w_write_8_full(user_w_write_8_full),
    .user_w_write_8_data(user_w_write_8_data),
    .user_w_write_8_open(user_w_write_8_open),


    // General signals
    .M_AXI_ACP_ARREADY(M_AXI_ACP_ARREADY),
    .M_AXI_ACP_AWREADY(M_AXI_ACP_AWREADY),
    .M_AXI_ACP_BRESP(M_AXI_ACP_BRESP),
    .M_AXI_ACP_BVALID(M_AXI_ACP_BVALID),
    .M_AXI_ACP_RDATA(M_AXI_ACP_RDATA),
    .M_AXI_ACP_RLAST(M_AXI_ACP_RLAST),
    .M_AXI_ACP_RRESP(M_AXI_ACP_RRESP),
    .M_AXI_ACP_RVALID(M_AXI_ACP_RVALID),
    .M_AXI_ACP_WREADY(M_AXI_ACP_WREADY),
    .S_AXI_ARADDR(S_AXI_ARADDR),
    .S_AXI_ARVALID(S_AXI_ARVALID),
    .S_AXI_AWADDR(S_AXI_AWADDR),
    .S_AXI_AWVALID(S_AXI_AWVALID),
    .S_AXI_BREADY(S_AXI_BREADY),
    .S_AXI_RREADY(S_AXI_RREADY),
    .S_AXI_WDATA(S_AXI_WDATA),
    .S_AXI_WSTRB(S_AXI_WSTRB),
    .S_AXI_WVALID(S_AXI_WVALID),
    .bus_clk(bus_clk),
    .bus_rst_n(bus_rst_n),
    .GPIO_LED(GPIO_LED),
    .M_AXI_ACP_ARADDR(M_AXI_ACP_ARADDR),
    .M_AXI_ACP_ARBURST(M_AXI_ACP_ARBURST),
    .M_AXI_ACP_ARCACHE(M_AXI_ACP_ARCACHE),
    .M_AXI_ACP_ARLEN(M_AXI_ACP_ARLEN),
    .M_AXI_ACP_ARPROT(M_AXI_ACP_ARPROT),
    .M_AXI_ACP_ARSIZE(M_AXI_ACP_ARSIZE),
    .M_AXI_ACP_ARVALID(M_AXI_ACP_ARVALID),
    .M_AXI_ACP_AWADDR(M_AXI_ACP_AWADDR),
    .M_AXI_ACP_AWBURST(M_AXI_ACP_AWBURST),
    .M_AXI_ACP_AWCACHE(M_AXI_ACP_AWCACHE),
    .M_AXI_ACP_AWLEN(M_AXI_ACP_AWLEN),
    .M_AXI_ACP_AWPROT(M_AXI_ACP_AWPROT),
    .M_AXI_ACP_AWSIZE(M_AXI_ACP_AWSIZE),
    .M_AXI_ACP_AWVALID(M_AXI_ACP_AWVALID),
    .M_AXI_ACP_BREADY(M_AXI_ACP_BREADY),
    .M_AXI_ACP_RREADY(M_AXI_ACP_RREADY),
    .M_AXI_ACP_WDATA(M_AXI_ACP_WDATA),
    .M_AXI_ACP_WLAST(M_AXI_ACP_WLAST),
    .M_AXI_ACP_WSTRB(M_AXI_ACP_WSTRB),
    .M_AXI_ACP_WVALID(M_AXI_ACP_WVALID),
    .S_AXI_ARREADY(S_AXI_ARREADY),
    .S_AXI_AWREADY(S_AXI_AWREADY),
    .S_AXI_BRESP(S_AXI_BRESP),
    .S_AXI_BVALID(S_AXI_BVALID),
    .S_AXI_RDATA(S_AXI_RDATA),
    .S_AXI_RRESP(S_AXI_RRESP),
    .S_AXI_RVALID(S_AXI_RVALID),
    .S_AXI_WREADY(S_AXI_WREADY),
    .host_interrupt(host_interrupt),
    .quiesce(quiesce)
  );

   /*
    * PART 2
    * ======
    *
    * Example code for a Xillybus stream with a 32-bit word.
    * This code demonstrates a loopback to the host.
    */

   // 32-bit loopback
   fifo_32x512 fifo_32
     (
      .clk(bus_clk),
      .srst(!user_w_write_32_open && !user_r_read_32_open),
      .din(user_w_write_32_data),
      .wr_en(user_w_write_32_wren),
      .rd_en(user_r_read_32_rden),
      .dout(user_r_read_32_data),
      .full(user_w_write_32_full),
      .empty(user_r_read_32_empty)
      );

   assign  user_r_read_32_eof = 0;

   /*
    * PART 3
    * ======
    *
    * Example code for a Xillybus stream with a 8-bit word.
    * This code demonstrates a loopback to the host.
    */

   fifo_8x2048 fifo_8
     (
      .clk(bus_clk),
      .srst(!user_w_write_8_open && !user_r_read_8_open),
      .din(user_w_write_8_data),
      .wr_en(user_w_write_8_wren),
      .rd_en(user_r_read_8_rden),
      .dout(user_r_read_8_data),
      .full(user_w_write_8_full),
      .empty(user_r_read_8_empty)
      );

   assign  user_r_read_8_eof = 0;

   assign  user_r_mem_8_empty = 0;
   assign  user_r_mem_8_eof = 0;
   assign  user_w_mem_8_full = 0;

   reg [7:0] reg0, reg1, reg2, reg3, reg4, reg5, reg6, reg7;

   always @(posedge bus_clk)
     if (user_w_mem_8_wren)
       case (user_mem_8_addr[2:0])
	 0: reg0 <= user_w_mem_8_data;
	 1: reg1 <= user_w_mem_8_data;
	 2: reg2 <= user_w_mem_8_data;
	 3: reg3 <= user_w_mem_8_data;
	 4: reg4 <= user_w_mem_8_data;
	 5: reg5 <= user_w_mem_8_data;
	 6: reg6 <= user_w_mem_8_data;
	 7: reg7 <= user_w_mem_8_data;
       endcase

   reg [7:0] mem_8_data_reg;
   assign user_r_mem_8_data = mem_8_data_reg;

   always @(posedge bus_clk)
     if (user_r_mem_8_rden)
       case (user_mem_8_addr[2:0])
	 0: mem_8_data_reg <= reg0;
	 1: mem_8_data_reg <= reg1;
	 2: mem_8_data_reg <= reg2;
	 3: mem_8_data_reg <= reg3;
	 4: mem_8_data_reg <= reg4;
	 5: mem_8_data_reg <= reg5;
	 6: mem_8_data_reg <= reg6;
	 7: mem_8_data_reg <= reg7;
       endcase

   servo_pwm servo_pwm_i [7:0]
     (
      .clk(bus_clk),
      .rst(quiesce),
      .pwm_signal(J6[7:0]),
      .pwm_width( { reg7, reg6, reg5, reg4, reg3, reg2, reg1, reg0 } )
      );

   /*
    * PART 5
    * ======
    *
    * Example code for Xillybus Lite
    */

   wire        user_clk;
   wire        user_wren;
   wire [3:0]  user_wstrb;
   wire        user_rden;
   reg [31:0]  user_rd_data;
   wire [31:0] user_wr_data;
   wire [31:0] user_addr;
   wire        user_irq;

   reg [7:0] litearray0[0:31];
   reg [7:0] litearray1[0:31];
   reg [7:0] litearray2[0:31];
   reg [7:0] litearray3[0:31];

   assign      user_irq = 0; // No interrupts for now

   always @(posedge user_clk)
     begin
	if (user_wstrb[0])
	  litearray0[user_addr[6:2]] <= user_wr_data[7:0];

	if (user_wstrb[1])
	  litearray1[user_addr[6:2]] <= user_wr_data[15:8];

	if (user_wstrb[2])
	  litearray2[user_addr[6:2]] <= user_wr_data[23:16];

	if (user_wstrb[3])
	  litearray3[user_addr[6:2]] <= user_wr_data[31:24];

	if (user_rden)
	  user_rd_data <= { litearray3[user_addr[6:2]],
			    litearray2[user_addr[6:2]],
			    litearray1[user_addr[6:2]],
			    litearray0[user_addr[6:2]] };
     end

   /*
    * PART 6
    * ======
    *
    * Interface with the Zynq processor's block design
    */

   wire MDIO_PHY_mdio_i, MDIO_PHY_mdio_o, MDIO_PHY_mdio_t;
   wire eeprom_scl_i, eeprom_scl_o, eeprom_scl_t;
   wire eeprom_sda_i, eeprom_sda_o, eeprom_sda_t;
   wire spi_sck_o, spi_io0_o, spi_ss_o;
   wire spi_sck_i, spi_io0_i, spi_ss_i;
   wire spi_sck_t, spi_io0_t, spi_ss_t;

   IOBUF MDIO_PHY_0_mdio_iobuf
     (.I(MDIO_PHY_mdio_o),
      .IO(MDIO_PHY_mdio_io),
      .O(MDIO_PHY_mdio_i),
      .T(MDIO_PHY_mdio_t));

   IOBUF eeprom_iobuf [1:0]
     (.I({eeprom_scl_o, eeprom_sda_o}),
      .O({eeprom_scl_i, eeprom_sda_i}),
      .T({eeprom_scl_t, eeprom_sda_t}),
      .IO({EEPROM_SCL, EEPROM_SDA}));

   IOBUF gpio_iobuf [gpio_width-1:0]
     (.I(gpio_tri_o[gpio_width-1:0]),
      .O(gpio_tri_i[gpio_width-1:0]),
      .T(gpio_tri_t[gpio_width-1:0]),
      .IO(J5));

   IOBUF lcd_iobuf [2:0]
     (.I({spi_sck_o, spi_io0_o, spi_ss_o}),
      .O({spi_sck_i, spi_io0_i, spi_ss_i}),
      .T({spi_sck_t, spi_io0_t, spi_ss_t}),
      .IO({LCD_SCL, LCD_SDA, LCD_CS}));

   assign LCD_RES = gpio_tri_t[60] ? 0 : gpio_tri_o[60];
   assign LCD_DC = gpio_tri_t[59] ? 0 : gpio_tri_o[59];
   assign LCD_BLK = gpio_tri_t[58] ? 0 : gpio_tri_o[58];

   (* BOX_TYPE = "user_black_box" *)
     vivado_system vivado_system_i
		 (
	      .GPIO_0_tri_i(gpio_tri_i),
		  .GPIO_0_tri_o(gpio_tri_o),
		  .GPIO_0_tri_t(gpio_tri_t),

		  .dvi_clk_n(hdmi_clk_n),
		  .dvi_clk_p(hdmi_clk_p),
		  .dvi_d_n(hdmi_d_n),
		  .dvi_d_p(hdmi_d_p),
		  .user_addr(user_addr),
		  .user_clk(user_clk),
		  .user_irq(user_irq),
		  .user_rd_data(user_rd_data),
		  .user_rden(user_rden),
		  .user_wr_data(user_wr_data),
		  .user_wren(user_wren),
		  .user_wstrb(user_wstrb),

		  .MDIO_PHY_0_mdc(MDIO_PHY_mdc),
		  .MDIO_PHY_0_mdio_i(MDIO_PHY_mdio_i),
		  .MDIO_PHY_0_mdio_o(MDIO_PHY_mdio_o),
		  .MDIO_PHY_0_mdio_t(MDIO_PHY_mdio_t),
		  .RGMII_0_rd(RGMII_rd),
		  .RGMII_0_rx_ctl(RGMII_rx_ctl),
		  .RGMII_0_rxc(RGMII_rxc),
		  .RGMII_0_td(RGMII_td),
		  .RGMII_0_tx_ctl(RGMII_tx_ctl),
		  .RGMII_0_txc(RGMII_txc),

		  .SPI_0_0_io0_i(spi_io0_i),
		  .SPI_0_0_io0_o(spi_io0_o),
		  .SPI_0_0_io0_t(spi_io0_t),
		  .SPI_0_0_io1_i(1'b0),
		  .SPI_0_0_io1_o(),
		  .SPI_0_0_io1_t(),
		  .SPI_0_0_sck_i(spi_sck_i),
		  .SPI_0_0_sck_o(spi_sck_o),
		  .SPI_0_0_sck_t(spi_sck_t),
		  .SPI_0_0_ss1_o(),
		  .SPI_0_0_ss2_o(),
		  .SPI_0_0_ss_i(1'b0),
		  .SPI_0_0_ss_o(spi_ss_o),
		  .SPI_0_0_ss_t(spi_ss_t),

		  .IIC_0_0_scl_i(eeprom_scl_i),
		  .IIC_0_0_scl_o(eeprom_scl_o),
		  .IIC_0_0_scl_t(eeprom_scl_t),
		  .IIC_0_0_sda_i(eeprom_sda_i),
		  .IIC_0_0_sda_o(eeprom_sda_o),
		  .IIC_0_0_sda_t(eeprom_sda_t),

		  .IIC_1_0_scl_i(1'b0),
		  .IIC_1_0_scl_o(),
		  .IIC_1_0_scl_t(),
		  .IIC_1_0_sda_i(1'b0),
		  .IIC_1_0_sda_o(),
		  .IIC_1_0_sda_t(),

		  .xillybus_M_AXI_araddr(M_AXI_ACP_ARADDR),
		  .xillybus_M_AXI_arburst(M_AXI_ACP_ARBURST),
		  .xillybus_M_AXI_arcache(M_AXI_ACP_ARCACHE),
		  .xillybus_M_AXI_arlen(M_AXI_ACP_ARLEN),
		  .xillybus_M_AXI_arprot(M_AXI_ACP_ARPROT),
		  .xillybus_M_AXI_arready(M_AXI_ACP_ARREADY),
		  .xillybus_M_AXI_arsize(M_AXI_ACP_ARSIZE),
		  .xillybus_M_AXI_arvalid(M_AXI_ACP_ARVALID),
		  .xillybus_M_AXI_awaddr(M_AXI_ACP_AWADDR),
		  .xillybus_M_AXI_awburst(M_AXI_ACP_AWBURST),
		  .xillybus_M_AXI_awcache(M_AXI_ACP_AWCACHE),
		  .xillybus_M_AXI_awlen(M_AXI_ACP_AWLEN),
		  .xillybus_M_AXI_awprot(M_AXI_ACP_AWPROT),
		  .xillybus_M_AXI_awready(M_AXI_ACP_AWREADY),
		  .xillybus_M_AXI_awsize(M_AXI_ACP_AWSIZE),
		  .xillybus_M_AXI_awvalid(M_AXI_ACP_AWVALID),
		  .xillybus_M_AXI_bready(M_AXI_ACP_BREADY),
		  .xillybus_M_AXI_bresp(M_AXI_ACP_BRESP),
		  .xillybus_M_AXI_bvalid(M_AXI_ACP_BVALID),
		  .xillybus_M_AXI_rdata(M_AXI_ACP_RDATA),
		  .xillybus_M_AXI_rlast(M_AXI_ACP_RLAST),
		  .xillybus_M_AXI_rready(M_AXI_ACP_RREADY),
		  .xillybus_M_AXI_rresp(M_AXI_ACP_RRESP),
		  .xillybus_M_AXI_rvalid(M_AXI_ACP_RVALID),
		  .xillybus_M_AXI_wdata(M_AXI_ACP_WDATA),
		  .xillybus_M_AXI_wlast(M_AXI_ACP_WLAST),
		  .xillybus_M_AXI_wready(M_AXI_ACP_WREADY),
		  .xillybus_M_AXI_wstrb(M_AXI_ACP_WSTRB),
		  .xillybus_M_AXI_wvalid(M_AXI_ACP_WVALID),
		  .xillybus_S_AXI_araddr(S_AXI_ARADDR),
		  .xillybus_S_AXI_arready(S_AXI_ARREADY),
		  .xillybus_S_AXI_arvalid(S_AXI_ARVALID),
		  .xillybus_S_AXI_awaddr(S_AXI_AWADDR),
		  .xillybus_S_AXI_awready(S_AXI_AWREADY),
		  .xillybus_S_AXI_awvalid(S_AXI_AWVALID),
		  .xillybus_S_AXI_bready(S_AXI_BREADY),
		  .xillybus_S_AXI_bresp(S_AXI_BRESP),
		  .xillybus_S_AXI_bvalid(S_AXI_BVALID),
		  .xillybus_S_AXI_rdata(S_AXI_RDATA),
		  .xillybus_S_AXI_rready(S_AXI_RREADY),
		  .xillybus_S_AXI_rresp(S_AXI_RRESP),
		  .xillybus_S_AXI_rvalid(S_AXI_RVALID),
		  .xillybus_S_AXI_wdata(S_AXI_WDATA),
		  .xillybus_S_AXI_wready(S_AXI_WREADY),
		  .xillybus_S_AXI_wstrb(S_AXI_WSTRB),
		  .xillybus_S_AXI_wvalid(S_AXI_WVALID),
		  .xillybus_bus_clk(bus_clk),
		  .xillybus_bus_rst_n(bus_rst_n),
		  .xillybus_host_interrupt(host_interrupt),
		  .UART_0_0_rxd(uart_rxd),
		  .UART_0_0_txd(uart_txd)
		  );
endmodule
