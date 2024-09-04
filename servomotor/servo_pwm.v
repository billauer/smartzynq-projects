module servo_pwm
  (
   input clk,
   input rst,

   output reg pwm_signal,
   input [7:0] pwm_width
  );

   reg [9:0]  enable_count;
   reg 	      enable;

   reg [10:0] pwm_count;
   reg [8:0]  threshold;

   always @(posedge clk)
     begin
	if (enable_count == 999)
	  begin
	     enable_count <= 0;
	     enable <= 1;
	  end
	else
	  begin
	     enable_count <= enable_count + 1;
	     enable <= 0;
	  end

	threshold <= pwm_width + 50; // Add 0.5 ms to PWM width

	if (enable)
	  begin
	     if (pwm_count == 1999)
	       pwm_count <= 0;
	     else
	       pwm_count <= pwm_count + 1;

	     pwm_signal <= (pwm_count < threshold);
	  end

	if (rst)
	  begin
	     enable_count <= 0;
	     pwm_count <= 0;
	     pwm_signal <= 0;
	  end
     end
endmodule
