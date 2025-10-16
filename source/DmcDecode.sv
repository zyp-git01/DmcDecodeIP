
//CDNS_ASYNC_FIFO_DATA_W is 5
//this module should be instanced to DmcDecodeCtrl_A/DmcDecodeCtrl_B
module DmcDecode #(
    parameter FPGA = 0
) (
	input                                   clk_i,//this clock is 98MHz
    input                                   clk_48K,
    input                                   enable,
    input                                   clk_or_data,//0:clock , 1:data
    input                                   reset_n,//connect to (reset_n && pll_lock)
    input                                   receive_line_rst_n,

    input signed [CDNS_ASYNC_FIFO_DATA_W:0] REG_TEN_JUDGE,//will be connected to {1'b0,REG_TEN_JUDGE}
    input signed [CDNS_ASYNC_FIFO_DATA_W:0] REG_FIF_JUDGE,//will be connected to {1'b0,REG_FIF_JUDGE} , FIF is fifteen
    input signed [CDNS_ASYNC_FIFO_DATA_W:0] REG_TEN_JUDGE_MARGIN,//will be connected to {1'b0,REG_TEN_JUDGE_MARGIN}

    input [3:0]                             DCTR_M,//from 

    input [4:0]                             ASYNC_FIFO_THRESHOLD,
    input [7:0]                             JUDGE_UPDATE_THRESHOLD,


    input [CDNS_ASYNC_FIFO_ADDR_W:0]        async_fifo_pop_size,//from async-fifo
    input                                   async_fifo_pop_empty,//from async-fifo
    input signed [CDNS_ASYNC_FIFO_DATA_W:0] async_fifo_popd_data,//will be connected to {1'b0,async_fifo_popd_data}

    output                                  async_fifo_pop_en,
    output logic                            async_fifo_clr,//in order to clear garbage
    output logic                            early_receive_done,

    output logic                            data_o,
    output logic                            sync_o, //output-sync
    output logic                            data_vld,//DeScrambleDeMux_enable

    input  [3:0]                            dbg_bus_sub_module_sel,
    output logic [12:0]                     dbg_bus_from_DmcDecode
);

logic signed [CDNS_ASYNC_FIFO_DATA_W:0] TDC_OFFSET;

//DCTR_M[0]\DCTR_M[1]\DCTR_M[2] : step 2
//DCTR_M[3]: step 4
always @(*) begin
    case (DCTR_M[3:0])
        4'b0000: TDC_OFFSET = 'd0;
        4'b0001: TDC_OFFSET = 'd2;
        4'b0010: TDC_OFFSET = 'd2;
        4'b0011: TDC_OFFSET = 'd4;
        4'b0100: TDC_OFFSET = 'd2;
        4'b0101: TDC_OFFSET = 'd4;
        4'b0110: TDC_OFFSET = 'd4;
        4'b0111: TDC_OFFSET = 'd6;
        4'b1000: TDC_OFFSET = 'd4;
        4'b1001: TDC_OFFSET = 'd6;
        4'b1010: TDC_OFFSET = 'd6;
        4'b1011: TDC_OFFSET = 'd8;
        4'b1100: TDC_OFFSET = 'd6;
        4'b1101: TDC_OFFSET = 'd8;
        4'b1110: TDC_OFFSET = 'd8;
        4'b1111: TDC_OFFSET = 'd10;
        default: TDC_OFFSET = 'd0;
    endcase
end

enum logic [3:0] { st_idle = 4'b0001,
                   st_short_edge = 4'b0010,
                   st_long_edge = 4'b0100,
                   st_done = 4'b1000 } state;

logic [5:0] async_fifo_pop_empty_d;
logic       async_fifo_pop_valid;

logic signed [CDNS_ASYNC_FIFO_DATA_W+2:0] data_sum;//data_sum will 
logic signed [CDNS_ASYNC_FIFO_DATA_W+2:0] data_sum_d;//data_sum will 


logic signed [CDNS_ASYNC_FIFO_DATA_W:0] ten_judge;
logic signed [CDNS_ASYNC_FIFO_DATA_W:0] long_judge;
logic signed [CDNS_ASYNC_FIFO_DATA_W:0] short_judge;

logic signed [CDNS_ASYNC_FIFO_DATA_W:0] judge_result_temp;
wire signed [CDNS_ASYNC_FIFO_DATA_W:0] judge_result = ( async_fifo_pop_valid ? async_fifo_popd_data : judge_result_temp ) - ( async_fifo_pop_valid ? ( state[1] || state[0] ? short_judge : ( REG_DFE_SEL ? long_judge : short_judge ) ) : 
                                                                                                                                                     ten_judge );
wire judge_result_bigger_zero = !judge_result[CDNS_ASYNC_FIFO_DATA_W] && |judge_result;//judge_result > 0

logic [7:0] judge_update_cnt;

wire reset_n_period = reset_n && receive_line_rst_n;

always @(posedge clk_i or negedge reset_n_period) begin
    if (!reset_n_period) begin
        state <= st_idle;
        async_fifo_pop_valid <= 1'b0;
        data_o <= 1'b0;
        sync_o <= 1'b0;
        early_receive_done <= 1'b0;
    end else begin
        case (state)
            st_idle: begin
                if ( ( async_fifo_pop_size >= ASYNC_FIFO_THRESHOLD ) && clk_or_data && enable ) begin //async_fifo_pop_size >= 3
                    state <= st_short_edge;
                    async_fifo_pop_valid <= 1'b1;
                end
            end
            st_long_edge:begin
                if ( !async_fifo_pop_valid ) begin
                    async_fifo_pop_valid <= !judge_result_bigger_zero ;
                    
                    if ( !clk_or_data )
                        data_o <= judge_result_bigger_zero ;
                    else 
                        sync_o <= judge_result_bigger_zero ;
                end else if ( !async_fifo_pop_empty ) begin
                    if ( judge_result_bigger_zero ) begin//next edge is long-edge
                        state <= st_long_edge;
                        async_fifo_pop_valid <= 1'b0;
                        if ( !clk_or_data )
                            data_o <= 1'b1;
                        else 
                            sync_o <= 1'b1;
                    end else begin
                        state <= st_short_edge;
                        if ( !clk_or_data )
                            data_o <= 1'b0;
                        else 
                            sync_o <= 1'b0;
                    end
                end else if ( clk_or_data ) begin//async_fifo_pop_empty
                    state <= st_done;
                    early_receive_done <= 1'b1;
                    async_fifo_pop_valid <= 1'b0;
                    sync_o <= 1'b0;
                end
            end
            st_short_edge:begin
                if ( async_fifo_pop_empty ) begin
                    state <= st_done;
                    early_receive_done <= 1'b1;
                    async_fifo_pop_valid <= 1'b0;
                    sync_o <= 1'b0;
                end else if ( judge_result_bigger_zero ) begin//next edge is long-edge
                    state <= st_long_edge;
                    async_fifo_pop_valid <= 1'b0;
                    if ( !clk_or_data )
                        data_o <= 1'b1;
                    else
                        sync_o <= 1'b1 ;
                end else if ( !clk_or_data ) 
                    data_o <= 1'b0;
            end
            default: state <= state;
        endcase
    end
end

always @(posedge clk_i or negedge reset_n_period) begin
    if (!reset_n_period)
        data_vld <= 1'b0;
    else if ( clk_or_data && async_fifo_pop_empty )
        data_vld <= 1'b0;
    else if ( async_fifo_pop_valid )
        data_vld <= 1'b1;
end

always @(posedge clk_i or negedge reset_n_period) begin
    if (!reset_n_period)
        judge_result_temp <= '0;
    else 
        judge_result_temp <= judge_result;
end


always @(posedge clk_i or negedge reset_n_period) begin
    if (!reset_n_period)
        async_fifo_pop_empty_d <= {6{1'b1}};
    else 
        async_fifo_pop_empty_d <= {async_fifo_pop_empty_d[4:0],async_fifo_pop_empty};
end


always @(posedge clk_i or negedge reset_n_period) begin
    if (!reset_n_period)
        async_fifo_clr <= 1'b0;
    else if ( async_fifo_pop_empty )
        async_fifo_clr <= 1'b0;
    else if ( !(|async_fifo_pop_empty_d) && |async_fifo_pop_size[1:0] && !(|async_fifo_pop_size[CDNS_ASYNC_FIFO_ADDR_W:2]) && state[0] )//state == st_wait
        async_fifo_clr <= 1'b1;
end


always @(posedge clk_48K or negedge reset_n) begin
    if (!reset_n)
        judge_update_cnt <= 'd1;
    else if ( judge_update_cnt == JUDGE_UPDATE_THRESHOLD )//judge_update_cnt is equal to JUDGE_UPDATE_THRESHOLD
        judge_update_cnt <= 'd1;
    else if ( |JUDGE_UPDATE_THRESHOLD )//JUDGE_UPDATE_THRESHOLD is not zero
        judge_update_cnt <= judge_update_cnt + 'd1;
end

logic [CDNS_ASYNC_FIFO_DATA_W+2:0] data_sum;
logic [5:0] data_o_dly;//data_o's delay
always @(posedge clk_or_data or reset_n_period) begin
    if (!reset_n_period)
        data_o_dly <= '0;
    else if ( data_vld && ( data_o_dly != 6'b001100 ) )
        data_o_dly <= {data_o_dly[4:0],data_o};
    else if (  )
end

always @(posedge clk_48K or negedge reset_n) begin
    if (!reset_n)
        ten_judge <= 'd1;
    else if ( judge_update_cnt == JUDGE_UPDATE_THRESHOLD )
        ten_judge <= 'd1;
    else if ( |JUDGE_UPDATE_THRESHOLD )
        ten_judge <= ten_judge + 'd1;
end


generate
    if ( FPGA == 1 ) begin
        assign async_fifo_pop_en = async_fifo_pop_valid;
    end else begin
        assign async_fifo_pop_en = async_fifo_pop_valid && !async_fifo_pop_empty;
    end
endgenerate


always @(*) begin
    case (dbg_bus_sub_module_sel)
        4'd0 : dbg_bus_from_DmcDecode = { 1'b0, TDC_OFFSET, ten_judge};
        4'd1 : dbg_bus_from_DmcDecode = { 1'b0, short_judge, long_judge};
        4'd2 : dbg_bus_from_DmcDecode = { 1'b0, judge_result_temp, judge_result};
        4'd3 : dbg_bus_from_DmcDecode = {{(13-CDNS_ASYNC_FIFO_DATA_W-3){1'b0}}, data_sum};
        4'd4 : dbg_bus_from_DmcDecode = {{(13-CDNS_ASYNC_FIFO_DATA_W-3){1'b0}}, data_sum_d};
        4'd5 : dbg_bus_from_DmcDecode = {clk_i, clk_or_data, enable, reset_n, data_o, sync_o, data_vld, early_receive_done};
        4'd6 : dbg_bus_from_DmcDecode = {2'b00, async_fifo_pop_en, pop_cnt, async_fifo_pop_empty, async_fifo_clr, async_fifo_pop_size};
        4'd7 : dbg_bus_from_DmcDecode = {2'b00, judge_result_bigger_zero, async_fifo_pop_en, data_o, sync_o, data_vld, async_fifo_popd_data};
        4'd8 : dbg_bus_from_DmcDecode = {5'b0, judge_result_bigger_zero, pop_cnt, state};
        default: dbg_bus_from_DmcDecode = '0;
    endcase
end

endmodule