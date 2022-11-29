/*
 *  Bounding Box Sample Test Iteration
 *
 *  Inputs:
 *    BBox and triangle information
 *
 *  Outputs:
 *    Subsample location and triangle information
 *
 *  Function:
 *    Iterate from left to right bottom to top
 *    across the bounding box.
 *
 *    While iterating set the halt signal in
 *    order to hold the bounding box pipeline in
 *    place.
 *
 *
 * Long Description:
 *    The iterator starts in the waiting state,
 *    when a valid triangle bounding box
 *    appears at the input. It will enter the
 *    testing state the next cycle with a
 *    sample equivelant to the lower left
 *    cooridinate of the bounding box.
 *
 *    While in the testing state, the next sample
 *    for each cycle should be one sample interval
 *    to the right, except when the current sample
 *    is at the right edge.  If the current sample
 *    is at the right edge, the next sample should
 *    be one row up.  Additionally, if the current
 *    sample is on the top row and the right edge,
 *    next cycles sample should be invalid and
 *    equivelant to the lower left vertice and
 *    next cycles state should be waiting.
 *
 *
 *   Author: John Brunhaver
 *   Created:      Thu 07/23/09
 *   Last Updated: Tue 10/01/10
 *
 *   Copyright 2009 <jbrunhaver@gmail.com>
 *
 */

/* ***************************************************************************
 * Change bar:
 * -----------
 * Date           Author    Description
 * Sep 19, 2012   jingpu    ported from John's original code to Genesis
 *
 * ***************************************************************************/

/* A Note on Signal Names:
 *
 * Most signals have a suffix of the form _RxxN
 * where R indicates that it is a Raster Block signal
 * xx indicates the clock slice that it belongs to
 * and N indicates the type of signal that it is.
 * H indicates logic high, L indicates logic low,
 * U indicates unsigned fixed point, and S indicates
 * signed fixed point.
 *
 * For all the signed fixed point signals (logic signed [`$sig_fig`-1:0]),
 * their highest `$sig_fig-$radix` bits, namely [`$sig_fig-1`:`$radix`]
 * represent the integer part of the fixed point number,
 * while the lowest `$radix` bits, namely [`$radix-1`:0]
 * represent the fractional part of the fixed point number.
 *
 *
 *
 * For signal subSample_RnnnnU (logic [3:0])
 * 1000 for  1x MSAA eq to 1 sample per pixel
 * 0100 for  4x MSAA eq to 4 samples per pixel,
 *              a sample is half a pixel on a side
 * 0010 for 16x MSAA eq to 16 sample per pixel,
 *              a sample is a quarter pixel on a side.
 * 0001 for 64x MSAA eq to 64 samples per pixel,
 *              a sample is an eighth of a pixel on a side.
 *
 */

module test_iterator
#(
    parameter SIGFIG = 24,      // Integer Bits in color and position.
    parameter RADIX = 10,       // Fraction bits in color and position
    parameter VERTS = 3,        // Maximum Vertices in triangle
    parameter AXIS = 3,         // Number of axis for each vertex 3 is (x,y,z).
    parameter COLORS = 3,       // Number of color channels
    parameter PIPE_DEPTH = 1,   // How many pipe stages are in this block
    parameter MOD_FSM = 0       // Use Modified FSM to eliminate a wait state
)
(
    //Input Signals
    input logic signed [SIGFIG-1:0]     tri_R13S[VERTS-1:0][AXIS-1:0],     // Triangle to iterate over
    input logic unsigned [SIGFIG-1:0]   color_R13U[COLORS-1:0],            // Color of triangle
    input logic signed [SIGFIG-1:0]     box_R13S[1:0][1:0],                // Box to iterate for subsamples
    input logic                         validTri_R13H,                     // Triangle is valid

    //Control Signals
    input logic [3:0]   subSample_RnnnnU,  // Subsample width
    output logic        halt_RnnnnL,       // Halt -> hold current microtriangle
    
    //Note that this block generates
    //Global Signals
    input logic clk, // Clock
    input logic rst, // Reset

    //Outputs
    output logic signed [SIGFIG-1:0]    tri_R14S[VERTS-1:0][AXIS-1:0],  //triangle to Sample Test
    output logic unsigned [SIGFIG-1:0]  color_R14U[COLORS-1:0],         //Color of triangle
    output logic signed [SIGFIG-1:0]    sample_R14S[1:0],               //Sample Location to Be Tested
    output logic                        validSamp_R14H                  //Sample and triangle are Valid
);

    // This module implements a Moore machine to iterate sample points in bbox
    // Recall: a Moore machine is an FSM whose output values are determined
    // solely by its current state.
    //
    // A simple way to build a Moore machine is to make states for every output
    // and the values of the current states are the outputs themselves

    // Now we create the signals for the next states of each outputs and
    // then instantiate registers for storing these states
    logic signed [SIGFIG-1:0]       next_tri_R14S[VERTS-1:0][AXIS-1:0];
    logic unsigned  [SIGFIG-1:0]    next_color_R14U[COLORS-1:0] ;
    logic signed [SIGFIG-1:0]       next_sample_R14S[1:0];
    logic                           next_validSamp_R14H;
    logic                           next_halt_RnnnnL;

    // Instantiate registers for storing these states
    dff3 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE1(VERTS),
        .ARRAY_SIZE2(AXIS),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0)
    )
    d301
    (
        .clk    (clk            ),
        .reset  (rst            ),
        .en     (1'b1           ),
        .in     (next_tri_R14S  ),
        .out    (tri_R14S       )
    );

    dff2 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE(COLORS),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0)
    )
    d302
    (
        .clk    (clk            ),
        .reset  (rst            ),
        .en     (1'b1           ),
        .in     (next_color_R14U),
        .out    (color_R14U     )
    );

    dff2 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE(2),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0)
    )
    d303
    (
        .clk    (clk                ),
        .reset  (rst                ),
        .en     (1'b1               ),
        .in     (next_sample_R14S   ),
        .out    (sample_R14S        )
    );

    dff #(
        .WIDTH(2),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0) // No retime
    )
    d304
    (
        .clk    (clk                                    ),
        .reset  (rst                                    ),
        .en     (1'b1                                   ),
        .in     ({next_validSamp_R14H, next_halt_RnnnnL}),
        .out    ({validSamp_R14H, halt_RnnnnL}          )
    );
    // Instantiate registers for storing these states

    typedef enum logic {
                            WAIT_STATE,
                            TEST_STATE
                        } state_t;
generate
if(MOD_FSM == 0) begin // Using baseline FSM
    //////
    //////  RTL code for original FSM Goes Here
    //////

    // To build this FSM we want to have two more states (not additional FSM states but objects to remember): one is the working
    // status of this FSM, and the other is the current bounding box where we iterate sample points

    // Define two more states, box_R14S and state_R14H
    logic signed [SIGFIG-1:0]   box_R14S[1:0][1:0];    		//The state for current bounding box
    logic signed [SIGFIG-1:0]   next_box_R14S[1:0][1:0];    //State for the next Bounding box's state

    state_t                     state_R14H;                 //State Designation (Waiting or Testing)
    state_t                     next_state_R14H;            //Next Cycles State

    dff3 #(
        .WIDTH(SIGFIG),
        .ARRAY_SIZE1(2),
        .ARRAY_SIZE2(2),
        .PIPE_DEPTH(1),
        .RETIME_STATUS(0)
    )
    d305
    (
        .clk    (clk            ),
        .reset  (rst            ),
        .en     (1'b1           ),
        .in     (next_box_R14S  ),
        .out    (box_R14S       )
    );

    always_ff @(posedge clk, posedge rst) begin
        if(rst) begin
            state_R14H <= WAIT_STATE;
        end
        else begin
            state_R14H <= next_state_R14H;
        end
    end

    // define some helper signals
    logic signed [SIGFIG-1:0]   next_up_samp_R14S[1:0]; //If jump up, next sample
    logic signed [SIGFIG-1:0]   next_rt_samp_R14S[1:0]; //If jump right, next sample
    logic                       at_right_edg_R14H;      //Current sample at right edge of bbox?
    logic                       at_top_edg_R14H;        //Current sample at top edge of bbox?
    logic                       at_end_box_R14H;        //Current sample at end of bbox?

    //////
    ////// First calculate the values of the helper signals using CURRENT STATES
    //////

    // check the comments 'A Note on Signal Names'
    // at the begining of the module for the help on
    // understanding the signals here

    logic signed [SIGFIG-1:0] sum;

    always_comb begin
        // START CODE HERE
        next_up_samp_R14S[0] = box_R14S[0][0];
        next_rt_samp_R14S[1] = sample_R14S[1];
        sum = '0;
        unique case ( 1'b1 )
            (subSample_RnnnnU[3]): sum[RADIX] = 1'b1;
            (subSample_RnnnnU[2]): sum[RADIX-1] = 1'b1;
            (subSample_RnnnnU[1]): sum[RADIX-2] = 1'b1;
            (subSample_RnnnnU[0]): sum[RADIX-3] = 1'b1;
        endcase // case ( 1'b1 )
        next_up_samp_R14S[1] = sample_R14S[1] + sum;
        next_rt_samp_R14S[0] = sample_R14S[0] + sum;      
        at_right_edg_R14H = (next_rt_samp_R14S[0] > box_R14S[1][0]) ? 1'b1 : 1'b0;
        at_top_edg_R14H = (next_up_samp_R14S[1] > box_R14S[1][1]) ? 1'b1 : 1'b0;
        at_end_box_R14H = (at_right_edg_R14H && at_top_edg_R14H) ? 1'b1 : 1'b0;
        // END CODE HERE
    end

    //////
    ////// Then complete the following combinational logic defining the
    ////// next states
    //////

    ////// COMPLETE THE FOLLOWING ALWAYS_COMB BLOCK

    // Combinational logic for state transitions
    always_comb begin
        // START CODE HERE
        // Try using a case statement on state_R14H
        case(state_R14H)
            WAIT_STATE : begin
                if (validTri_R13H) begin                    // if valid triangle present then... 
                    next_tri_R14S = tri_R13S;                   // read in the triangle information
                    next_color_R14U = color_R13U;               // read in the triangle's color information
                    next_sample_R14S = box_R13S[0][1:0];        // set the first sample at LL coordinate of bbox
                    next_validSamp_R14H = 1'b1;                 // declare the first sample valid
                    next_halt_RnnnnL = 1'b0;                    // set halt signal low to indicate NOT ready to recieve another bbox
                    next_state_R14H = TEST_STATE;               // transition to testing state
                    next_box_R14S = box_R13S;                   // initialize the box state
                end
                else begin                                  // otherwise, invalid triangle present then...
                    next_tri_R14S = tri_R13S;                   // read in the triangle information (disregarded as sample not valid)
                    next_color_R14U = color_R13U;               // read in the triangle's color information (disregarded as sample not valid)
                    next_sample_R14S = box_R13S[0][1:0];        // set the first sample at LL coordinate of bbox (disregarded as sample not valid)
                    next_validSamp_R14H = 1'b0;                 // declare the sample NOT valid
                    next_halt_RnnnnL = 1'b1;                    // set halt signal high to indicate ready to recieve a bbox
                    next_state_R14H = WAIT_STATE;               // remain in wait state
                    next_box_R14S = box_R13S;                   // read in the box (disregarded as sample not valid)
                end
            end
            TEST_STATE : begin
                if (at_end_box_R14H) begin                  // if at the end of the bbox...
                    next_tri_R14S = tri_R14S;                   // triangle information stays the same
                    next_color_R14U = color_R14U;               // triangle's color information stays the same
                    next_sample_R14S = box_R14S[0][1:0];        // set the subsample location to the lower left vertices
                    next_validSamp_R14H = 1'b1;                 // (???) declare the last sample invalid (???)
                    next_halt_RnnnnL = 1'b1;                    // set halt signal high to indicate ready to recieve a bbox
                    next_state_R14H = WAIT_STATE;               // transition to waiting state
                    next_box_R14S = box_R14S;                   // box stays the same
                end
                else if (at_right_edg_R14H) begin           // otherwise, if at then end of the row but not at the end of the bbox and ...
                    next_tri_R14S = tri_R14S;                   // triangle information stays the same
                    next_color_R14U = color_R14U;               // triangle's color information stays the same
                    next_sample_R14S = next_up_samp_R14S;       // move up and all the way left to the next row's first subsample location
                    next_validSamp_R14H = 1'b1;                 // declare the last sample valid
                    next_halt_RnnnnL = 1'b0;                    // keep halt signal low as testing still in progress
                    next_state_R14H = TEST_STATE;               // remain in test state
                    next_box_R14S[1][1] = box_R14S[1][1];       // shrink the box by cutting off one row at the bottom
                    next_box_R14S[1][0] = box_R14S[1][0];
                    next_box_R14S[0][0] = box_R14S[0][0];       
                    next_box_R14S[0][1] = box_R14S[0][1] + sum;
                end
                else begin                                  // otherwise, still in the same row and not at the end of the box or row so...
                    next_tri_R14S = tri_R14S;                   // triangle information stays the same
                    next_color_R14U = color_R14U;               // triangle's color information stays the same
                    next_sample_R14S = next_rt_samp_R14S;       // move right to the next subsample location
                    next_validSamp_R14H = 1'b1;                 // declare the sample valid
                    next_halt_RnnnnL = 1'b0;                    // keep halt signal low as testing still in progress
                    next_state_R14H = TEST_STATE;               // remain in test state
                    next_box_R14S = box_R14S;                   // box stays the same
                end
            end
        endcase
        // END CODE HERE
    end // always_comb

    //Assertions for testing FSM logic (HOW DO I CREATE ASSERTIONS?)

    // Write assertions to verify your FSM transition sequence
    // Can you verify that:
    // 1) A validTri_R13H signal causes a transition from WAIT state to TEST state
    // 2) An end_box_R14H signal causes a transition from TEST state to WAIT state
    // 3) What are you missing? (not a valid triangle in the TEST state?)

    //Your assertions goes here
    // START CODE HERE
     // verfies validTri_R13H causes WAIT->TEST
    assert property (@(posedge clk) (validTri_R13H && !rst && (state_R14H == WAIT_STATE)) |-> ##[1:2] (state_R14H == TEST_STATE));
    // verfies at_end_box_R14H causes TEST->WAIT
    assert property (@(posedge clk) (at_end_box_R14H && !rst && (state_R14H == TEST_STATE)) |-> ##[1:2] (state_R14H == WAIT_STATE));
    // verifies if no valid triangle received keep waiting (i.e., WAIT->WAIT)
    assert property (@(posedge clk) (!validTri_R13H && !rst && (state_R14H == WAIT_STATE)) |-> ##[1:2] (state_R14H == WAIT_STATE));
    // verifies if not at the end of the box then remain in the test state (i.e., TEST->TEST)
    assert property (@(posedge clk) (!at_end_box_R14H && !rst && (state_R14H == TEST_STATE)) |-> ##[1:2] (state_R14H == TEST_STATE));
    // END CODE HERE
    // Assertion ends 

    //////
    //////  RTL code for original FSM Finishes
    //////

    //Some Error Checking Assertions

    //Define a Less Than Property
    //
    //  a should be less than or equal to b
    property rb_lt( rst, a , b , c );
        @(posedge clk) rst | ((a<=b) | !c);
    endproperty
    
    //Define a Greater Than Property
    //
    //  a should be greater than or equal to b
    property rb_gt( rst, a , b , c );
        @(posedge clk) rst | ((a>=b) | !c);
    endproperty

    //Check that Proposed Sample is in BBox
    // START CODE HERE
    assert property( rb_gt( rst, sample_R14S[0], box_R14S[0][0], validSamp_R14H )); //check if Sample's x position is greater than or equal to bbox LL's x position
    assert property( rb_gt( rst, sample_R14S[1], box_R14S[0][1], validSamp_R14H )); //check if Sample's y position is greater than or equal to bbox LL's y position
    assert property( rb_lt( rst, sample_R14S[0], box_R14S[1][0], validSamp_R14H )); //check if Sample's x position is less than or equal to bbox UR's x position
    assert property( rb_lt( rst, sample_R14S[1], box_R14S[1][1], validSamp_R14H )); //check if Sample's y position is less than or equal to bbox UR's y position
    // END CODE HERE

    //Error Checking Assertions
end 
else begin // Use modified FSM

    //////
    //////  RTL code for modified FSM Goes Here
    //////

    ////// PLACE YOUR CODE HERE

    //////
    //////  RTL code for modified FSM Finishes
    //////

end
endgenerate

endmodule
