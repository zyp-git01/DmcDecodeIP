
module DmcDecodeCtrl#(
    parameter FPGA = 0
)(
    input                                   TDC_DATA_SCKA,
    input                                   TDC_DATA_SCKB,

    input [5:0]                             BINARY_TDCA,
    input [5:0]                             BINARY_TDCB,

    input                                   RST_N_DEMODA,

    input                                   clk_i,//this clock is 98MHz
    input                                   clk_or_data,//connect to !clk_49MHz
    input                                   reset_n,//ext_rst_n_delayed && !SOFTRST && SUSTAIN_RST_N && !ASYNC_FIFO_RST
    input                                   pll_lock,

    input                                   scf_srf,//receive_scf_srf
    input                                   receive_line_rst_n,
//below signals is config-settings
    input                                   MSTR,

    input [CDNS_ASYNC_FIFO_DATA_W-1:0]      REG_TEN_JUDGE_A,//will be connected to {1'b0,REG_TEN_JUDGE}
    input [CDNS_ASYNC_FIFO_DATA_W-1:0]      REG_FIF_JUDGE_A,//will be connected to {1'b0,REG_FIF_JUDGE} , FIF is fifteen
    input [CDNS_ASYNC_FIFO_DATA_W-1:0]      REG_TEN_JUDGE_MARGIN_A,//will be connected to {1'b0,REG_TEN_JUDGE_MARGIN}
    input [CDNS_ASYNC_FIFO_DATA_W-1:0]      REG_FIF_JUDGE_MARGIN_A,//will be connected to {1'b0,REG_FIF_JUDGE_MARGIN} , Efuse
    input [CDNS_ASYNC_FIFO_DATA_W-1:0]      REG_TEN_JUDGE_CAL_THRESHOLD_A,//will be connected to {1'b0,REG_TEN_JUDGE_CAL_THRESHOLD} , Efuse
    input [CDNS_ASYNC_FIFO_DATA_W-1:0]      REG_FIF_JUDGE_CAL_THRESHOLD_A,//will be connected to {1'b0,REG_FIF_JUDGE_CAL_THRESHOLD} , Efuse

    input [CDNS_ASYNC_FIFO_DATA_W-1:0]      REG_TEN_JUDGE_B,//will be connected to {1'b0,REG_TEN_JUDGE}
    input [CDNS_ASYNC_FIFO_DATA_W-1:0]      REG_FIF_JUDGE_B,//will be connected to {1'b0,REG_FIF_JUDGE} , FIF is fifteen
    input [CDNS_ASYNC_FIFO_DATA_W-1:0]      REG_TEN_JUDGE_MARGIN_B,//will be connected to {1'b0,REG_TEN_JUDGE_MARGIN}
    input [CDNS_ASYNC_FIFO_DATA_W-1:0]      REG_FIF_JUDGE_MARGIN_B,//will be connected to {1'b0,REG_FIF_JUDGE_MARGIN} , Efuse
    input [CDNS_ASYNC_FIFO_DATA_W-1:0]      REG_TEN_JUDGE_CAL_THRESHOLD_B,//will be connected to {1'b0,REG_TEN_JUDGE_CAL_THRESHOLD} , Efuse
    input [CDNS_ASYNC_FIFO_DATA_W-1:0]      REG_FIF_JUDGE_CAL_THRESHOLD_B,//will be connected to {1'b0,REG_FIF_JUDGE_CAL_THRESHOLD} , Efuse

    input                                   REG_JUDGE_SEL_A,//default value is 1'b0
    input                                   REG_JUDGE_SEL_B,//default value is 1'b0

    input                                   REG_DFE_SEL_A,
    input                                   REG_DFE_SEL_B,

    input [3:0]                             DCTR_M_A,//from DMC_CTLA_1[7:4]
    input [3:0]                             DCTR_M_B,//from DMC_CTLB_1[7:4]

    input [4:0]                             ASYNC_FIFO_THRESHOLD,
    input [7:0]                             JUDGE_UPDATE_THRESHOLD,

    input                                   tdc_sum_vld_for_ten,
    input                                   tdc_sum_vld_for_fif,
    input                                   scf_preamble_check,
    input                                   srf_preamble_check,
//below signals is config-settings

    input                                   sync_from_pad,
    input                                   ARX_EN,
    input                                   BRX_EN,

    output                                  data_o,
    output                                  sync_o, //output-sync
    output                                  data_vld,//used as DeScrambleDeMux_enable
    output                                  bit_cnt_begin,//output to DeScrambleDeMux
    output logic                            early_receive_done,

    output logic                            SYNMCLK,//output to PLL

    input     [3:0]                         dbg_bus_sub_module_sel,
    output logic [12:0]                     dbg_bus_from_DmcDecodeCtrl,
    output       [12:0]                     dbg_bus_from_async_fifo
);

//***************************************//
//***************************************//
//              ASYNC FIFO               //
//***************************************//
//***************************************//
wire                                async_fifo_clr;
wire                                async_fifo_push_clk;
wire                                async_fifo_rst_n = reset_n && pll_lock && receive_line_rst_n && !async_fifo_clr ;
wire [CDNS_ASYNC_FIFO_DATA_W:0]     async_fifo_pushd_data = ARX_EN ? BINARY_TDCA : ( BRX_EN ? BINARY_TDCB : {(CDNS_ASYNC_FIFO_DATA_W+1){1'b0}} ) ;
wire                                async_fifo_push_full;
wire                                async_fifo_push_overflow;
wire [CDNS_ASYNC_FIFO_ADDR_W:0]     async_fifo_push_size;
wire                                async_fifo_pop_empty;
wire                                async_fifo_pop_en;
wire                                async_fifo_underflow;
wire [CDNS_ASYNC_FIFO_ADDR_W:0]     async_fifo_pop_size;
wire [CDNS_ASYNC_FIFO_DATA_W:0]     async_fifo_popd_data;
wire                                async_fifo_push_en;

`ifdef SIM
wire async_fifo_pop_en_temp;
assign #1 async_fifo_pop_en = async_fifo_pop_en_temp;
`endif

`ifdef FPGA_CODE
assign async_fifo_push_clk = ARX_EN ? TDC_DATA_SCKA : BRX_EN ? TDC_DATA_SCKB : 1'b0;
`else
wire async_fifo_push_clk_temp;
CKMUX2M6 CKMUX_DMC_FIFO_CLK_U0 ( .S( BRX_EN ), .A( 1'b0                     ), .B( TDC_DATA_SCKB ), .Z( async_fifo_push_clk_temp ) );
CKMUX2M6 CKMUX_DMC_FIFO_CLK_U1 ( .S( ARX_EN ), .A( async_fifo_push_clk_temp ), .B( TDC_DATA_SCKA ), .Z( async_fifo_push_clk      ) );
`endif


generate
    if ( FPGA ) begin:fpga_decoded_to_demux_fifo
        decode_to_demux_fifo u_decoded_to_demux_fifo(
            .aclr               (!async_fifo_rst_n      ),
            .data               (async_fifo_pushd_data  ),
            .wrclk              (async_fifo_push_clk    ),
            .rdclk              (clk_i                  ),
            .wrreq              (async_fifo_push_en     ),
            .rdreq              (async_fifo_pop_en      ),
            .q                  (async_fifo_popd_data   ),
            .wrfull             (async_fifo_push_full   ), 
            .rdempty            (async_fifo_pop_empty   ),
            .wrusedw            (async_fifo_push_size   ),
            .rdusedw            (async_fifo_pop_size    )
        );
    end else begin:non_fpga_decoded_to_demux_fifo
        cdns_async_fifo # (
            .CDNS_ASYNC_FIFO_DATA_W(CDNS_ASYNC_FIFO_DATA_W+1), // Data width
            .CDNS_ASYNC_FIFO_ADDR_W(CDNS_ASYNC_FIFO_ADDR_W), // Address width. Note. If a single location FIFO
                                                                // is required then set this to 0.
            .DEPTH(DEPTH), // Note. Depth should only be used in the case
                            // when a 1 deep FIFO is need - i.e. when CDNS_ASYNC_FIFO_ADDR_W
                            // is set to 0. In this case depth should be set to
                            // 1. Otherwise depth is the default 2**CDNS_ASYNC_FIFO_ADDR_W
            .CDNS_ASYNC_FIFO_REG_OUTPUT_STAGE(CDNS_ASYNC_FIFO_REG_OUTPUT_STAGE), // Create an additional register stage on the
                                                                                    // output of the FIFO. When a read takes place
                                                                                    // the ouptut will be placed in this register
                                                                                    // until another read takes place.
            .CDNS_ASYNC_FIFO_RESET_VAL({(CDNS_ASYNC_FIFO_DATA_W+1){1'b0}}) // Only valid when ADD_REG_OUTPUT_STAGE is set.
                                                                        // Sets the reset value of the output register
                                                                        // when this option is set.
        )
        async_fifo_dmc_2_demux_U0 (//dmc_ctrl to demux
            // Clock and Reset
            .clk_push            ( async_fifo_push_clk      ),
            .clk_pop             ( clk_i                    ),
            .reset_push_n        ( async_fifo_rst_n         ),
            .reset_pop_n         ( async_fifo_rst_n         ),
            // Push Interface 
            .push                ( async_fifo_push_en       ),// Push Data to the FIFO
            .pushd               ( async_fifo_pushd_data    ),// Push Data
            .push_full           ( async_fifo_push_full     ),// Full (push side)
            .push_overflow       ( async_fifo_push_overflow ),// Overflow
            .push_size           ( async_fifo_push_size     ),// Number of entries (push side) in FIFO
            // Pop Interface 
            .pop                 ( async_fifo_pop_en        ),// Pop Data from the FIFO
            .popd                ( async_fifo_popd_data     ),// Pop Data
            .pop_empty           ( async_fifo_pop_empty     ),// Empty (pop side)
            .pop_underflow       ( async_fifo_underflow     ),// Underflow
            .pop_size            ( async_fifo_pop_size      ),// Number of entries (pop side) in FIFO
            
            .dbg_bus_sub_module_sel ( {1'b0,dbg_bus_sub_module_sel[2:0]}),
            .dbg_bus_from_async_fifo( dbg_bus_from_async_fifo           )
        );
    end
endgenerate

wire            REG_DFE_SEL = ARX_EN ? REG_DFE_SEL_A : ( BRX_EN ? REG_DFE_SEL_B : 1'b0 );//from 

wire [3:0]      DCTR_M = ARX_EN ? DCTR_M_A : ( BRX_EN ? DCTR_M_B : '0 );//from 
logic           longz;
logic [15:0]    longz_R;
logic           DmcDecode_enable;

wire [12:0]   dbg_bus_from_DmcDecode;
//for LVDS-TDC-A
DmcDecode #(
    .FPGA                   ( FPGA                          )
) DmcDecode_U0(
	.clk_i                  ( clk_i                         ),//this clock is 98MHz
    .enable                 ( DmcDecode_enable              ),
    .clk_or_data            ( clk_or_data                   ),
    .reset_n                ( reset_n && pll_lock           ),
    .receive_line_rst_n     ( receive_line_rst_n            ),
    .scf_srf                ( scf_srf                       ),

    .REG_TEN_JUDGE_A                ( {1'b0,REG_TEN_JUDGE_A}               ),
    .REG_FIF_JUDGE_A                ( {1'b0,REG_FIF_JUDGE_A}               ),
    .REG_TEN_JUDGE_MARGIN_A         ( {1'b0,REG_TEN_JUDGE_MARGIN_A}        ),
    .REG_FIF_JUDGE_MARGIN_A         ( {1'b0,REG_FIF_JUDGE_MARGIN_A}        ),
    .REG_TEN_JUDGE_CAL_THRESHOLD_A  ( {1'b0,REG_TEN_JUDGE_CAL_THRESHOLD_A} ),
    .REG_FIF_JUDGE_CAL_THRESHOLD_A  ( {1'b0,REG_FIF_JUDGE_CAL_THRESHOLD_A} ),
    
    .REG_TEN_JUDGE_B                ( {1'b0,REG_TEN_JUDGE_B}               ),
    .REG_FIF_JUDGE_B                ( {1'b0,REG_FIF_JUDGE_B}               ),
    .REG_TEN_JUDGE_MARGIN_B         ( {1'b0,REG_TEN_JUDGE_MARGIN_B}        ),
    .REG_FIF_JUDGE_MARGIN_B         ( {1'b0,REG_FIF_JUDGE_MARGIN_B}        ),
    .REG_TEN_JUDGE_CAL_THRESHOLD_B  ( {1'b0,REG_TEN_JUDGE_CAL_THRESHOLD_B} ),
    .REG_FIF_JUDGE_CAL_THRESHOLD_B  ( {1'b0,REG_FIF_JUDGE_CAL_THRESHOLD_B} ),

    .REG_JUDGE_SEL_A        ( REG_JUDGE_SEL_A               ),
    .REG_JUDGE_SEL_B        ( REG_JUDGE_SEL_B               ),

    .REG_DFE_SEL            ( REG_DFE_SEL                   ),
    
    .DCTR_M                 ( DCTR_M                        ),//from

    .ASYNC_FIFO_THRESHOLD   ( ASYNC_FIFO_THRESHOLD          ),
    .JUDGE_UPDATE_THRESHOLD ( JUDGE_UPDATE_THRESHOLD        ),

    .tdc_sum_vld_for_ten    ( tdc_sum_vld_for_ten           ),
    .tdc_sum_vld_for_fif    ( tdc_sum_vld_for_fif           ),
    .scf_preamble_check     ( scf_preamble_check            ),
    .srf_preamble_check     ( srf_preamble_check            ),

    .async_fifo_pop_size    ( async_fifo_pop_size           ),//from async-fifo
    `ifdef FPGA_CODE
    .async_fifo_pop_empty   ( async_fifo_pop_size == '0     ),//from async-fifo
    `else
    .async_fifo_pop_empty   ( async_fifo_pop_empty          ),//from async-fifo
    `endif
    .async_fifo_popd_data   ( {1'b0,async_fifo_popd_data[CDNS_ASYNC_FIFO_DATA_W-1:0]}   ),//will be connected to {1'b0,async_fifo_popd_data}
    .async_fifo_popd_data_pol( async_fifo_popd_data[CDNS_ASYNC_FIFO_DATA_W] ),

    `ifdef SIM
    .async_fifo_pop_en      ( async_fifo_pop_en_temp        ),
    `else
    .async_fifo_pop_en      ( async_fifo_pop_en             ),
    `endif
    .async_fifo_clr         ( async_fifo_clr                ),//in order to clear garbage
    .early_receive_done     ( early_receive_done            ),

    .data_o                 ( data_o                        ),
    .sync_o                 ( sync_o                        ), //output-sync
    .data_vld               ( data_vld                      ),//DeScrambleDeMux_enable
    .bit_cnt_begin          ( bit_cnt_begin                 ),

    .*
);

`ifdef new_dmc_decode
DmcSpecialCheck #(
    .FPGA                   ( FPGA )
) u_DmcSpecialCheck (
    .reset_n                ( reset_n && DmcDecode_enable && receive_line_rst_n                     ),
    .TDC_DATA_SCK           ( async_fifo_push_clk                                                   ),
    .BINARY_TDC             ( async_fifo_pushd_data                                                 ),
    .REG_FIF_JUDGE          ( ARX_EN ? {1'b0,REG_FIF_JUDGE_A} : {1'b0,REG_FIF_JUDGE_B}              ),
    .REG_FIF_JUDGE_MARGIN   ( ARX_EN ? {1'b0,REG_FIF_JUDGE_MARGIN_A} : {1'b0,REG_FIF_JUDGE_MARGIN_B}),
    .special_check_done     (                                                                       ),
    .async_fifo_push_en     ( async_fifo_push_en                                                    )
);
`endif

logic [14:0] sync_counter;
wire         sync_counter_rst_n = reset_n && RST_N_DEMODA;
always @(posedge TDC_DATA_SCKA or negedge sync_counter_rst_n) begin
    if (!sync_counter_rst_n)
        sync_counter <= {{14{1'b0}},1'b1};
    else
        sync_counter <= {sync_counter[13:0],1'b0};
end

assign SYNMCLK = MSTR ? sync_from_pad : |sync_counter[13:8];

always @(posedge TDC_DATA_SCKA or negedge sync_counter_rst_n) begin
    if (!sync_counter_rst_n)
        longz <= 1'b1;
    else
        longz <= 1'b0;
end


always @(posedge clk_i or negedge reset_n) begin
    if (!reset_n)
        longz_R <= '1;
    else
        longz_R <= {longz_R[14:0],longz};
end

wire reset_n_longz = reset_n && pll_lock;
always @(posedge clk_i or negedge reset_n_longz) begin
    if (!reset_n_longz)
        DmcDecode_enable <= 1'b0;
    else if ( !DmcDecode_enable )
        DmcDecode_enable <= !(|longz_R[15:8]) && (&(longz_R[7:2])) || MSTR ;
end

`ifndef new_dmc_decode
assign async_fifo_push_en = DmcDecode_enable && ( ARX_EN || BRX_EN );
`endif

always @(*) begin
    case (dbg_bus_sub_module_sel)
        4'd9 :  dbg_bus_from_DmcDecodeCtrl = { TDC_DATA_SCKA, TDC_DATA_SCKB, BINARY_TDCA[4:0], BINARY_TDCA[4:0], RST_N_DEMODA };
        4'd10 :  dbg_bus_from_DmcDecodeCtrl = { async_fifo_push_en, DmcDecode_enable, longz, sync_from_pad, ARX_EN, BRX_EN, data_o, sync_o, data_vld, early_receive_done, SYNMCLK, pll_lock, sync_counter_rst_n };
        4'd11 :  dbg_bus_from_DmcDecodeCtrl = sync_counter[12:0];
        4'd12 :  dbg_bus_from_DmcDecodeCtrl = longz_R[14:2];
        4'd13 :  dbg_bus_from_DmcDecodeCtrl = {sync_from_pad, 1'b0, 1'b0, DCTR_M_A, DCTR_M_B, ARX_EN, BRX_EN};
        default: dbg_bus_from_DmcDecodeCtrl = dbg_bus_from_DmcDecode;
    endcase
end

endmodule



//CDNS_ASYNC_FIFO_DATA_W is 5
//this module should be instanced to DmcDecodeCtrl_A/DmcDecodeCtrl_B
module DmcDecode #(
    parameter FPGA = 0
) (
	input                                   clk_i,//this clock is 98MHz
    input                                   enable,
    input                                   clk_or_data,//0:clock , 1:data
    input                                   reset_n,//connect to (reset_n && pll_lock)
    input                                   receive_line_rst_n,
    input                                   scf_srf,//from receive_scf_srf

    input signed [CDNS_ASYNC_FIFO_DATA_W:0] REG_TEN_JUDGE_A,//will be connected to {1'b0,REG_TEN_JUDGE} , Efuse
    input signed [CDNS_ASYNC_FIFO_DATA_W:0] REG_FIF_JUDGE_A,//will be connected to {1'b0,REG_FIF_JUDGE} , FIF is fifteen , Efuse
    input signed [CDNS_ASYNC_FIFO_DATA_W:0] REG_TEN_JUDGE_MARGIN_A,//will be connected to {1'b0,REG_FIF_JUDGE_MARGIN} , Efuse
    input signed [CDNS_ASYNC_FIFO_DATA_W:0] REG_FIF_JUDGE_MARGIN_A,//will be connected to {1'b0,REG_FIF_JUDGE_MARGIN} , Efuse
    input signed [CDNS_ASYNC_FIFO_DATA_W:0] REG_TEN_JUDGE_CAL_THRESHOLD_A,//will be connected to {1'b0,REG_TEN_JUDGE_CAL_THRESHOLD} , Efuse
    input signed [CDNS_ASYNC_FIFO_DATA_W:0] REG_FIF_JUDGE_CAL_THRESHOLD_A,//will be connected to {1'b0,REG_FIF_JUDGE_CAL_THRESHOLD} , Efuse

    input signed [CDNS_ASYNC_FIFO_DATA_W:0] REG_TEN_JUDGE_B,//will be connected to {1'b0,REG_TEN_JUDGE} , Efuse
    input signed [CDNS_ASYNC_FIFO_DATA_W:0] REG_FIF_JUDGE_B,//will be connected to {1'b0,REG_FIF_JUDGE} , FIF is fifteen , Efuse
    input signed [CDNS_ASYNC_FIFO_DATA_W:0] REG_TEN_JUDGE_MARGIN_B,//will be connected to {1'b0,REG_FIF_JUDGE_MARGIN} , Efuse
    input signed [CDNS_ASYNC_FIFO_DATA_W:0] REG_FIF_JUDGE_MARGIN_B,//will be connected to {1'b0,REG_FIF_JUDGE_MARGIN} , Efuse
    input signed [CDNS_ASYNC_FIFO_DATA_W:0] REG_TEN_JUDGE_CAL_THRESHOLD_B,//will be connected to {1'b0,REG_TEN_JUDGE_CAL_THRESHOLD} , Efuse
    input signed [CDNS_ASYNC_FIFO_DATA_W:0] REG_FIF_JUDGE_CAL_THRESHOLD_B,//will be connected to {1'b0,REG_FIF_JUDGE_CAL_THRESHOLD} , Efuse

    input                                   REG_JUDGE_SEL_A,//from Efuse
    input                                   REG_JUDGE_SEL_B,//from Efuse

    input                                   REG_DFE_SEL,

    input [3:0]                             DCTR_M,//from 

    input [4:0]                             ASYNC_FIFO_THRESHOLD,//Efuse
    input [7:0]                             JUDGE_UPDATE_THRESHOLD,//Efuse

    input                                   tdc_sum_vld_for_ten,//from DeScrambleDeMux
    input                                   tdc_sum_vld_for_fif,//from DeScrambleDeMux
    input                                   scf_preamble_check,//from DeScrambleDeMux
    input                                   srf_preamble_check,//from DeScrambleDeMux

    input [CDNS_ASYNC_FIFO_ADDR_W:0]        async_fifo_pop_size,//from async-fifo
    input                                   async_fifo_pop_empty,//from async-fifo
    input signed [CDNS_ASYNC_FIFO_DATA_W:0] async_fifo_popd_data,//will be connected to {1'b0,async_fifo_popd_data}
    input                                   async_fifo_popd_data_pol,
    input                                   async_fifo_push_en,

    output                                  async_fifo_pop_en,
    output logic                            async_fifo_clr,//in order to clear garbage
    output logic                            early_receive_done,

    output logic                            data_o,
    output logic                            sync_o, //output-sync
    output logic                            data_vld,//DeScrambleDeMux_enable
    output logic                            bit_cnt_begin,//bit_cnt_begin == 0 , means bit_cnt begin from 'd1
                                                          //bit_cnt_begin == 1 , means bit_cnt begin from 'd2 , and ScrambleMux should data[2] , not data[1]

    input  [3:0]                            dbg_bus_sub_module_sel,
    output logic [12:0]                     dbg_bus_from_DmcDecode
);

logic signed [CDNS_ASYNC_FIFO_DATA_W:0] TDC_OFFSET;

`ifdef SIM
parameter clk_i_delay = 1;
`else
parameter clk_i_delay = 0;
`endif

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

logic [1:0] async_fifo_push_en_R;
logic [1:0] async_fifo_push_en_nR;
logic [5:0] async_fifo_pop_empty_R;
logic       async_fifo_pop_valid;
logic       receive_line_rst_n_R;
logic       first_data_always_short;


logic signed [CDNS_ASYNC_FIFO_DATA_W:0] scf_ten_judge_cal;
logic signed [CDNS_ASYNC_FIFO_DATA_W:0] scf_fif_judge_cal;
logic signed [CDNS_ASYNC_FIFO_DATA_W:0] srf_ten_judge_cal;
logic signed [CDNS_ASYNC_FIFO_DATA_W:0] srf_fif_judge_cal;
logic signed [CDNS_ASYNC_FIFO_DATA_W:0] ten_judge;
logic signed [CDNS_ASYNC_FIFO_DATA_W:0] fif_long_judge;
logic signed [CDNS_ASYNC_FIFO_DATA_W:0] fif_short_judge;

logic scf_judge_sel;
logic srf_judge_sel;

logic signed [CDNS_ASYNC_FIFO_DATA_W:0] judge_result_R;
wire signed [CDNS_ASYNC_FIFO_DATA_W:0] judge_result = ( async_fifo_pop_valid || state[0] ? async_fifo_popd_data : judge_result_R ) - ( async_fifo_pop_valid || state[0] ? ( state[1] ? fif_short_judge : ( REG_DFE_SEL ? fif_long_judge : fif_short_judge ) ) : 
                                                                                                                                                     ten_judge );
wire judge_result_bigger_zero = !judge_result[CDNS_ASYNC_FIFO_DATA_W] && |judge_result;//judge_result > 0

logic [7:0] scf_judge_update_cnt;
logic [7:0] srf_judge_update_cnt;

wire reset_n_period = reset_n && receive_line_rst_n;

always @(posedge clk_i or negedge reset_n_period) begin
    if (!reset_n_period) begin
        state <= st_idle;
        async_fifo_pop_valid <= 1'b0;
        data_o <= 1'b0;
        sync_o <= 1'b0;
        early_receive_done <= 1'b0;
        first_data_always_short <= 1'b0;
    end else begin
        case (state)
            st_idle: begin
            `ifdef new_dmc_decode
                if ( ( async_fifo_pop_size >= ASYNC_FIFO_THRESHOLD ) && enable && clk_or_data ) begin //async_fifo_pop_size >= 3
                    state <= st_short_edge;
                    async_fifo_pop_valid <= #(clk_i_delay) 1'b1;
                end 
            `else //not new_dmc_decode
                if ( ( async_fifo_pop_size >= ASYNC_FIFO_THRESHOLD ) && enable && ( clk_or_data && async_fifo_popd_data_pol || !clk_or_data && !async_fifo_popd_data_pol ) ) begin //async_fifo_pop_size >= 3
                    state <= st_short_edge;
                    async_fifo_pop_valid <= #(clk_i_delay) 1'b1;
                    first_data_always_short <= 1'b1; //if ARX_EN is too early , will make TDC measure a long-level , so don't care first-level
                end 
            `endif
            end
            st_long_edge:begin
                if ( !clk_or_data && async_fifo_pop_empty ) begin//async_fifo_pop_empty
                    state <= st_done;
                    early_receive_done <= #(clk_i_delay) 1'b1;
                    async_fifo_pop_valid <= #(clk_i_delay) 1'b0;
                    sync_o <= 1'b0;
                end else if ( !async_fifo_pop_empty ) begin
                    if ( !async_fifo_pop_valid ) begin
                        async_fifo_pop_valid <= #(clk_i_delay) !judge_result_bigger_zero ;
                        
                        if ( !clk_or_data )
                            data_o <= #(clk_i_delay) judge_result_bigger_zero ;
                        else 
                            sync_o <= #(clk_i_delay) judge_result_bigger_zero ;
                    end else begin //async_fifo_pop_valid == 1
                        if ( judge_result_bigger_zero ) begin//next edge is long-edge
                            state <= st_long_edge;
                            async_fifo_pop_valid <= #(clk_i_delay) 1'b0;
                            if ( !clk_or_data )
                                data_o <= #(clk_i_delay) 1'b1;
                            else 
                                sync_o <= #(clk_i_delay) 1'b1;
                        end else begin
                            state <= st_short_edge;
                            if ( !clk_or_data )
                                data_o <= #(clk_i_delay) 1'b0;
                            else 
                                sync_o <= #(clk_i_delay) 1'b0;
                        end
                    end 
                end
            end
            st_short_edge:begin
                first_data_always_short <= 1'b0;
                if ( async_fifo_pop_empty ) begin
                    state <= st_done;
                    early_receive_done <= #(clk_i_delay) 1'b1;
                    async_fifo_pop_valid <= #(clk_i_delay) 1'b0;
                    sync_o <= 1'b0;
                end else if ( judge_result_bigger_zero && !first_data_always_short ) begin//next edge is long-edge
                    state <= st_long_edge;
                    async_fifo_pop_valid <= #(clk_i_delay) 1'b0;
                    if ( !clk_or_data )
                        data_o <= #(clk_i_delay) 1'b1;
                    else
                        sync_o <= #(clk_i_delay) 1'b1 ;
                end else if ( !clk_or_data ) 
                    data_o <= #(clk_i_delay) 1'b0;
            end
            default: state <= state;
        endcase
    end
end

always @(posedge clk_or_data or negedge reset_n_period) begin
    if (!reset_n_period)
        data_vld <= 1'b0;
    else if ( async_fifo_pop_empty )
        data_vld <= 1'b0;
    else if ( async_fifo_pop_valid )
        data_vld <= 1'b1;
end

always @(posedge clk_or_data or negedge reset_n_period) begin
    if (!reset_n_period)
        data_vld <= 1'b0;
    else if ( async_fifo_pop_empty )
        data_vld <= 1'b0;
    else if ( async_fifo_pop_valid )
        data_vld <= 1'b1;
end

`ifndef new_dmc_decode

always @(posedge clk_i or negedge reset_n) begin
    if (!reset_n)
        bit_cnt_begin <= 1'b0;
    else if ( !async_fifo_pop_empty && !async_fifo_pop_valid && state[0] )
        bit_cnt_begin <= #(clk_i_delay) ~async_fifo_popd_data_pol;
end

`else //no new_dmc_decode

always @(posedge clk_i or negedge reset_n_period) begin
    if (!reset_n_period)
        async_fifo_push_en_R <= 2'b00;
    else
        async_fifo_push_en_R <= #(clk_i_delay) {async_fifo_push_en_R[0],async_fifo_push_en};
end

always @(negedge clk_i or negedge reset_n_period) begin
    if (!reset_n_period)
        async_fifo_push_en_nR <= 2'b00;
    else
        async_fifo_push_en_nR <= #(clk_i_delay) {async_fifo_push_en_nR[0],async_fifo_push_en};
end

logic async_fifo_push_en_nR_e1;
always @(posedge clk_or_data or negedge reset_n) begin
    if (!reset_n)
        async_fifo_push_en_nR_e1 <= 1'b0;
    else
        async_fifo_push_en_nR_e1 <= async_fifo_push_en_nR[1] || async_fifo_push_en_R[1];
end

assign bit_cnt_begin = !async_fifo_push_en_nR_e1 && ( async_fifo_push_en_nR[1] || async_fifo_push_en_R[1] );

`endif




always @(posedge clk_i or negedge reset_n_period) begin
    if (!reset_n_period)
        judge_result_R <= '0;
    else 
        judge_result_R <= judge_result;
end


always @(posedge clk_i or negedge reset_n_period) begin
    if (!reset_n_period)
        async_fifo_pop_empty_R <= {6{1'b1}};
    else 
        async_fifo_pop_empty_R <= {async_fifo_pop_empty_R[4:0],async_fifo_pop_empty};
end


always @(posedge clk_i or negedge reset_n_period) begin
    if (!reset_n_period)
        async_fifo_clr <= 1'b0;
    else if ( async_fifo_pop_empty )
        async_fifo_clr <= 1'b0;
    else if ( !(|async_fifo_pop_empty_R) && |async_fifo_pop_size[1:0] && !(|async_fifo_pop_size[CDNS_ASYNC_FIFO_ADDR_W:2]) && state[0] )//state == st_idle
        async_fifo_clr <= 1'b1;
end

//only detect the "10101100" , can update judge_update_cnt
always @(posedge clk_or_data or negedge reset_n) begin
    if (!reset_n)
        scf_judge_update_cnt <= 'd1;
    else if ( (scf_judge_update_cnt == JUDGE_UPDATE_THRESHOLD) && scf_preamble_check )//scf_judge_update_cnt is equal to JUDGE_UPDATE_THRESHOLD
        scf_judge_update_cnt <= 'd1;
    else if ( (|JUDGE_UPDATE_THRESHOLD) && scf_preamble_check )//JUDGE_UPDATE_THRESHOLD is not zero & preamble_check is a pulse
        scf_judge_update_cnt <= scf_judge_update_cnt + 'd1;
end

always @(posedge clk_or_data or negedge reset_n) begin
    if (!reset_n)
        srf_judge_update_cnt <= 'd1;
    else if ( (srf_judge_update_cnt == JUDGE_UPDATE_THRESHOLD) && srf_preamble_check )//scf_judge_update_cnt is equal to JUDGE_UPDATE_THRESHOLD
        srf_judge_update_cnt <= 'd1;
    else if ( (|JUDGE_UPDATE_THRESHOLD) && srf_preamble_check )//JUDGE_UPDATE_THRESHOLD is not zero & preamble_check is a pulse
        srf_judge_update_cnt <= srf_judge_update_cnt + 'd1;
end

logic [CDNS_ASYNC_FIFO_DATA_W+2:0] data_sum_for_ten_judge;
logic [CDNS_ASYNC_FIFO_DATA_W+2:0] data_sum_for_fif_judge;//sum the of "110010" "1010110010"
wire                               scf_calculate_judge = ( scf_judge_update_cnt == JUDGE_UPDATE_THRESHOLD ) && ( receive_line_rst_n_R && !receive_line_rst_n );
wire                               srf_calculate_judge = ( srf_judge_update_cnt == JUDGE_UPDATE_THRESHOLD ) && ( receive_line_rst_n_R && !receive_line_rst_n );
always @(posedge clk_or_data or negedge reset_n) begin
    if (!reset_n)
        receive_line_rst_n_R <= 1'b1;
    else
        receive_line_rst_n_R <= receive_line_rst_n;
end

always @(posedge clk_or_data or negedge reset_n) begin
    if (!reset_n)
        data_sum_for_ten_judge <= '0;
    else if ( scf_calculate_judge || srf_calculate_judge )//state == st_idle
        data_sum_for_ten_judge <= '0;
    else if ( tdc_sum_vld_for_ten )
        data_sum_for_ten_judge <= data_sum_for_ten_judge + {2'b00,async_fifo_popd_data};
end

always @(posedge clk_or_data or negedge reset_n) begin
    if (!reset_n)
        data_sum_for_fif_judge <= '0;
    else if ( scf_calculate_judge || srf_calculate_judge )//state == st_idle
        data_sum_for_fif_judge <= '0;
    else if ( tdc_sum_vld_for_fif )
        data_sum_for_fif_judge <= data_sum_for_fif_judge + {2'b00,async_fifo_popd_data};
end

always @(posedge clk_or_data or negedge reset_n) begin
    if (!reset_n)
        scf_judge_sel <= 1'b0;
    else if ( REG_JUDGE_SEL_A )//if mannually choose REG_JUDGE
        scf_judge_sel <= 1'b0;
    else if ( !receive_line_rst_n && ( scf_judge_update_cnt == JUDGE_UPDATE_THRESHOLD ) )
        scf_judge_sel <= 1'b1;
end

always @(posedge clk_or_data or negedge reset_n) begin
    if (!reset_n)
        srf_judge_sel <= 1'b0;
    else if ( REG_JUDGE_SEL_B )//if mannually choose REG_JUDGE
        srf_judge_sel <= 1'b0;
    else if ( !receive_line_rst_n && ( srf_judge_update_cnt == JUDGE_UPDATE_THRESHOLD ) )
        srf_judge_sel <= 1'b1;
end

always @(posedge clk_or_data or negedge reset_n) begin
    if (!reset_n)
        scf_ten_judge_cal <= 'd1;
    else if ( !scf_judge_sel )//if mannually choose REG_JUDGE or judge_update_cnt != JUDGE_UPDATE_THRESHOLD
        scf_ten_judge_cal <= REG_TEN_JUDGE_A;
    else if ( scf_calculate_judge )
        scf_ten_judge_cal <= ( data_sum_for_ten_judge[CDNS_ASYNC_FIFO_DATA_W+2:2] < scf_ten_judge_cal - REG_TEN_JUDGE_CAL_THRESHOLD_A ) || ( data_sum_for_ten_judge[CDNS_ASYNC_FIFO_DATA_W+2:2] > scf_ten_judge_cal + REG_TEN_JUDGE_CAL_THRESHOLD_A ) ? scf_ten_judge_cal : data_sum_for_ten_judge[CDNS_ASYNC_FIFO_DATA_W+2:2];//set multicycle path from data_sum_for_ten_judge to scf_ten_judge_cal
end

always @(posedge clk_or_data or negedge reset_n) begin
    if (!reset_n)
        scf_fif_judge_cal <= 'd1;//fifteen judge
    else if ( !scf_judge_sel )//if mannually choose REG_JUDGE or judge_update_cnt != JUDGE_UPDATE_THRESHOLD
        scf_fif_judge_cal <= REG_FIF_JUDGE_A;
    else if ( scf_calculate_judge )
        scf_fif_judge_cal <= ( data_sum_for_fif_judge[CDNS_ASYNC_FIFO_DATA_W+2:2] < scf_fif_judge_cal - REG_FIF_JUDGE_CAL_THRESHOLD_A ) || ( data_sum_for_fif_judge[CDNS_ASYNC_FIFO_DATA_W+2:2] > scf_fif_judge_cal + REG_FIF_JUDGE_CAL_THRESHOLD_A ) ? scf_fif_judge_cal : data_sum_for_fif_judge[CDNS_ASYNC_FIFO_DATA_W+2:2];//set multicycle path from data_sum_for_fif_judge to scf_fif_judge_cal
end

always @(posedge clk_or_data or negedge reset_n) begin
    if (!reset_n)
        srf_ten_judge_cal <= 'd1;
    else if ( !srf_judge_sel )//if mannually choose REG_JUDGE or judge_update_cnt != JUDGE_UPDATE_THRESHOLD
        srf_ten_judge_cal <= REG_TEN_JUDGE_B;
    else if ( srf_calculate_judge )
        srf_ten_judge_cal <= ( data_sum_for_ten_judge[CDNS_ASYNC_FIFO_DATA_W+2:2] < srf_ten_judge_cal - REG_TEN_JUDGE_CAL_THRESHOLD_B ) || ( data_sum_for_ten_judge[CDNS_ASYNC_FIFO_DATA_W+2:2] > srf_ten_judge_cal + REG_TEN_JUDGE_CAL_THRESHOLD_B ) ? srf_ten_judge_cal : data_sum_for_ten_judge[CDNS_ASYNC_FIFO_DATA_W+2:2];//set multicycle path from data_sum_for_ten_judge to srf_ten_judge_cal
end

always @(posedge clk_or_data or negedge reset_n) begin
    if (!reset_n)
        srf_fif_judge_cal <= 'd1;//fifteen judge
    else if ( !srf_judge_sel )//if mannually choose REG_JUDGE or judge_update_cnt != JUDGE_UPDATE_THRESHOLD
        srf_fif_judge_cal <= REG_FIF_JUDGE_B;
    else if ( srf_calculate_judge )
        srf_fif_judge_cal <= ( data_sum_for_fif_judge[CDNS_ASYNC_FIFO_DATA_W+2:2] < srf_fif_judge_cal - REG_FIF_JUDGE_CAL_THRESHOLD_B ) || ( data_sum_for_fif_judge[CDNS_ASYNC_FIFO_DATA_W+2:2] > srf_fif_judge_cal + REG_FIF_JUDGE_CAL_THRESHOLD_B ) ? srf_fif_judge_cal : data_sum_for_fif_judge[CDNS_ASYNC_FIFO_DATA_W+2:2];//set multicycle path from data_sum_for_fif_judge to srf_fif_judge_cal
end

assign ten_judge = scf_srf ? ( srf_judge_sel ? srf_ten_judge_cal : REG_TEN_JUDGE_B ) + REG_TEN_JUDGE_MARGIN_B : ( scf_judge_sel ? scf_ten_judge_cal : REG_TEN_JUDGE_A ) + REG_TEN_JUDGE_MARGIN_A;
assign fif_short_judge = scf_srf ? ( srf_judge_sel ? srf_fif_judge_cal : REG_FIF_JUDGE_B ) : ( scf_judge_sel ? scf_fif_judge_cal : REG_FIF_JUDGE_A );
assign fif_long_judge = scf_srf ? ( srf_judge_sel ? srf_fif_judge_cal : REG_FIF_JUDGE_B ) + REG_FIF_JUDGE_MARGIN_B : ( scf_judge_sel ? scf_fif_judge_cal : REG_FIF_JUDGE_A ) + REG_FIF_JUDGE_MARGIN_A ;


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
        4'd1 : dbg_bus_from_DmcDecode = { 1'b0, fif_short_judge, fif_long_judge};
        4'd2 : dbg_bus_from_DmcDecode = { 1'b0, judge_result_R, judge_result};
        4'd3 : dbg_bus_from_DmcDecode = {{(13-CDNS_ASYNC_FIFO_DATA_W-2){1'b0}}, data_sum_for_fif_judge};
        4'd4 : dbg_bus_from_DmcDecode = {{(13-CDNS_ASYNC_FIFO_DATA_W-2){1'b0}}, data_sum_for_ten_judge};
        4'd5 : dbg_bus_from_DmcDecode = {clk_i, clk_or_data, enable, reset_n, data_o, sync_o, data_vld, early_receive_done};
        4'd6 : dbg_bus_from_DmcDecode = {2'b00, async_fifo_pop_en, async_fifo_pop_empty, async_fifo_clr, async_fifo_pop_size};
        4'd7 : dbg_bus_from_DmcDecode = {2'b00, judge_result_bigger_zero, async_fifo_pop_en, data_o, sync_o, data_vld, async_fifo_popd_data};
        4'd8 : dbg_bus_from_DmcDecode = {5'b0, judge_result_bigger_zero, state};
        default: dbg_bus_from_DmcDecode = '0;
    endcase
end

endmodule




module DmcSpecialCheck #(
    parameter FPGA = 0
)(
    input                                   reset_n,//reset_n && receive_line_rst_n
    
    input                                   TDC_DATA_SCK,//connect to TDC_DATA_SCKA or TDC_DATA_SCKB, mux from scf_srf

    input [CDNS_ASYNC_FIFO_DATA_W:0]        BINARY_TDC,//connect to BINARY_TDCA or BINARY_TDCB, mux from scf_srf

    input signed [CDNS_ASYNC_FIFO_DATA_W:0] REG_FIF_JUDGE,//will be connected to {1'b0,REG_FIF_JUDGE} , FIF is fifteen , Efuse
    input signed [CDNS_ASYNC_FIFO_DATA_W:0] REG_FIF_JUDGE_MARGIN,//will be connected to {1'b0,REG_FIF_JUDGE_MARGIN} , Efuse                          

    output logic                            special_check_done,
    output logic                            async_fifo_push_en
);

wire signed [CDNS_ASYNC_FIFO_DATA_W:0] judge_result = {1'b0,BINARY_TDC[CDNS_ASYNC_FIFO_DATA_W-1:0]} - ( REG_FIF_JUDGE + REG_FIF_JUDGE_MARGIN ) ;

wire judge_result_bigger_zero = !judge_result[CDNS_ASYNC_FIFO_DATA_W] && |judge_result;//judge_result > 0

logic [3:0] data_R;
wire        wait_for_long_one = data_R == 4'h0;

always @(posedge TDC_DATA_SCK or negedge reset_n) begin
    if (!reset_n)
        data_R <= '1;
    else
        data_R <= {data_R[2:0], judge_result_bigger_zero};
end


assign special_check_done = wait_for_long_one && judge_result_bigger_zero;

always @(posedge special_check_done or negedge reset_n) begin
    if (!reset_n)
        async_fifo_push_en <= 1'b0;
    else
        async_fifo_push_en <= 1'b1;
end


endmodule