module anc_axis_stream(

input clk,
input rst_n,

input sample_valid,

input signed [31:0] x_in,
input signed [31:0] e_in,
input signed [31:0] y_in,

output reg [127:0] tdata,
output reg tvalid,
input tready

);

always @(posedge clk or negedge rst_n)
begin
    if(!rst_n)
    begin
        tdata  <= 0;
        tvalid <= 0;
    end
    else
    begin
        if(sample_valid && tready)
        begin
            tdata <= {32'd0, x_in, e_in, y_in}; // padding
            tvalid <= 1'b1;
        end
        else
        begin
            tvalid <= 0;
        end
    end
end

endmodule