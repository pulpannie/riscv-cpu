//
//  Author: Prof. Taeweon Suh
//          Computer Science & Engineering
//          Korea University
//  Date: July 14, 2020
//  Description: Skeleton design of RV32I Single-cycle CPU
//

`timescale 1ns/1ns
`define simdelay 1

module rv32i_cpu (
		      input         clk, reset,
            output [31:0] pc,		  		// program counter for instruction fetch
            input  [31:0] inst, 			// incoming instruction
            output        Memwrite, 	// 'memory write' control signal
            output [31:0] Memaddr,  	// memory address 
            output [31:0] MemWdata, 	// data to write to memory
            input  [31:0] MemRdata); 	// data read from memory

  wire        auipc, lui;
  wire        alusrc, regwrite;
  wire [4:0]  alucontrol;
  wire        memtoreg, memwrite, mem_memwrite;
  wire        branch, jal, jalr;

  assign Memwrite = mem_memwrite ;

  // Instantiate Controller
  controller i_controller(
      .opcode		(inst[6:0]), 
		.funct7		(inst[31:25]), 
		.funct3		(inst[14:12]), 
		.auipc		(auipc),
		.lui			(lui),
		.memtoreg	(memtoreg),
		.memwrite	(memwrite),
		.branch		(branch),
		.alusrc		(alusrc),
		.regwrite	(regwrite),
		.jal			(jal),
		.jalr			(jalr),
		.alucontrol	(alucontrol));

  // Instantiate Datapath
  datapath i_datapath(
		.clk				(clk),
		.reset			(reset),
		.auipc			(auipc),
		.lui				(lui),
		.memtoreg		(memtoreg),
		.memwrite		(memwrite),
		.branch			(branch),
		.alusrc			(alusrc),
		.regwrite		(regwrite),
		.jal				(jal),
		.jalr				(jalr),
		.alucontrol		(alucontrol),
		.pc				(pc),
		.inst				(inst),
		.aluout			(Memaddr), 
		.MemWdata		(MemWdata),
		.mem_memwrite  (mem_memwrite),
		.MemRdata		(MemRdata));

endmodule


//
// Instruction Decoder 
// to generate control signals for datapath
//
module controller(input  [6:0] opcode,
                  input  [6:0] funct7,
                  input  [2:0] funct3,
                  output       auipc,
                  output       lui,
                  output       alusrc,
                  output [4:0] alucontrol,
                  output       branch,
                  output       jal,
                  output       jalr,
                  output       memtoreg,
                  output       memwrite,
                  output       regwrite);

	maindec i_maindec(
		.opcode		(opcode),
		.auipc		(auipc),
		.lui			(lui),
		.memtoreg	(memtoreg),
		.memwrite	(memwrite),
		.branch		(branch),
		.alusrc		(alusrc),
		.regwrite	(regwrite),
		.jal			(jal),
		.jalr			(jalr));

	aludec i_aludec( 
		.opcode     (opcode),
		.funct7     (funct7),
		.funct3     (funct3),
		.alucontrol (alucontrol));


endmodule


//
// RV32I Opcode map = Inst[6:0]
//
`define OP_R			7'b0110011
`define OP_I_Arith	7'b0010011
`define OP_I_Load  	7'b0000011
`define OP_I_JALR  	7'b1100111
`define OP_S			7'b0100011
`define OP_B			7'b1100011
`define OP_U_LUI		7'b0110111
`define OP_J_JAL		7'b1101111

//
// Main decoder generates all control signals except alucontrol 
//
//
module maindec(input  [6:0] opcode,
               output       auipc,
               output       lui,
               output       regwrite,
               output       alusrc,
               output       memtoreg, memwrite,
               output       branch, 
               output       jal,
               output       jalr);

  reg [8:0] controls;

  assign {auipc, lui, regwrite, alusrc, 
			 memtoreg, memwrite, branch, jal, 
			 jalr} = controls;

  always @(*)
  begin
    case(opcode)
      `OP_R: 			controls <= #`simdelay 9'b0010_0000_0; // R-type
      `OP_I_Arith: 	controls <= #`simdelay 9'b0011_0000_0; // I-type Arithmetic
      `OP_I_Load: 	controls <= #`simdelay 9'b0011_1000_0; // I-type Load
		`OP_I_JALR:		controls <= #`simdelay 9'b0011_0000_1; // JALR
      `OP_S: 			controls <= #`simdelay 9'b0001_0100_0; // S-type Store
      `OP_B: 			controls <= #`simdelay 9'b0000_0010_0; // B-type Branch
      `OP_U_LUI: 		controls <= #`simdelay 9'b0111_0000_0; // LUI
      `OP_J_JAL: 		controls <= #`simdelay 9'b0011_0001_0; // JAL
      default:    	controls <= #`simdelay 9'b0000_0000_0; // ???
    endcase
  end

endmodule

//
// ALU decoder generates ALU control signal (alucontrol)
//
module aludec(input      [6:0] opcode,
              input      [6:0] funct7,
              input      [2:0] funct3,
              output reg [4:0] alucontrol);

  always @(*)

    case(opcode)

      `OP_R:   		// R-type
		begin
			case({funct7,funct3})
			 10'b0000000_000: alucontrol <= #`simdelay 5'b00000; // addition (add)
			 10'b0100000_000: alucontrol <= #`simdelay 5'b10000; // subtraction (sub)
			 10'b0000000_111: alucontrol <= #`simdelay 5'b00001; // and (and)
			 10'b0000000_110: alucontrol <= #`simdelay 5'b00010; // or (or)
          default:         alucontrol <= #`simdelay 5'bxxxxx; // ???
        endcase
		end

      `OP_I_Arith:   // I-type Arithmetic
		begin
			case(funct3)
			 3'b000:  alucontrol <= #`simdelay 5'b00000; // addition (addi)
			 3'b110:  alucontrol <= #`simdelay 5'b00001; // and (andi)
			 3'b111:  alucontrol <= #`simdelay 5'b00010; // or (ori)
			 3'b100:  alucontrol <= #`simdelay 5'b00011; // xor (xori)
			 3'b001:  alucontrol <= #`simdelay 5'b00100; //slli
          default: alucontrol <= #`simdelay 5'bxxxxx; // ???
        endcase
		end
		

      `OP_I_Load: 	// I-type Load (LW, LH, LB...)
      	alucontrol <= #`simdelay 5'b00000;  // addition 
		`OP_I_JALR:
			alucontrol <= #`simdelay 5'b00000;  // addition
		`OP_J_JAL: 		
		   alucontrol <= #`simdelay 5'bxxxxx; // JAL


      `OP_B:   		// B-type Branch (BEQ, BNE, ...)
		begin
			case(funct3)
			 3'b000:  alucontrol <= #`simdelay 5'b10000;  // subtraction 
			 3'b111:	 alucontrol <= #`simdelay 5'b10000;  // subtraction
			 3'b001:  alucontrol <= #`simdelay 5'b10000;  // subtraction
			 //############## Hyokyung Kim : Start ##################	
			 3'b101:  alucontrol <= #`simdelay 5'b10000;  // subtraction
			 3'b110:  alucontrol <= #`simdelay 5'b10000;  // subtraction
			 default: alucontrol <= #`simdelay 5'bxxxxx;
		  endcase 
		end
		//############## Hyokyung Kim : End ##################
      `OP_S:   		// S-type Store (SW, SH, SB)
      	alucontrol <= #`simdelay 5'b00000;  // addition 

      `OP_U_LUI: 		// U-type (LUI)
      	alucontrol <= #`simdelay 5'b00000;  // addition

      default: 
      	alucontrol <= #`simdelay 5'b00000;  // 

    endcase
    
endmodule





//
// CPU datapath
//
module datapath(input         clk, reset,
                input  [31:0] inst,
                input         auipc,
                input         lui,
                input         regwrite,
                input         memtoreg,
                input         memwrite,
                input         alusrc, 
                input  [4:0]  alucontrol,
                input         branch,
                input         jal,
                input         jalr,

                output reg [31:0] pc,
                output reg [31:0] aluout,
                output reg [31:0] MemWdata,
					 output reg mem_memwrite,
                input  [31:0] MemRdata);

  wire [4:0]  rs1, rs2;
  reg  [4:0]  rd;
  wire [2:0]  funct3;
  wire [31:0] rs1_data, rs2_data;
  reg  [31:0] tmp_rs1_data;
  reg  [31:0] tmp_rs2_data;
  reg  [31:0] rd_data;
  wire [20:1] jal_imm;
  wire [31:0] se_jal_imm;
  wire [12:1] br_imm;
  wire [31:0] se_br_imm;
  wire [31:0] se_imm_itype;
  wire [31:0] se_imm_stype;
  wire [31:0] se_jalr_imm;
  wire [31:0] auipc_lui_imm;
  reg  [31:0] alusrc1;
  reg  [31:0] alusrc2;
  reg  [31:0] id_inst;
  reg  [31:0] id_pc;
  reg  [4:0] id_rd;
  reg  [4:0] id_rs1;
  reg  [4:0] id_rs2;
  reg  [4:0] id_alucontrol;
  reg  id_auipc;
  reg  id_lui;	 		
  reg  id_regwrite;
  reg  id_alusrc;
  reg  id_memtoreg;
  reg  id_memwrite;
  reg  id_branch;
  reg  id_jal;
  reg  id_jalr;
  reg  id_f3beq;
  reg  id_f3bgeu;
  reg  id_f3blt;
  reg  id_f3bne;
  reg  id_f3bge;
  reg  id_f3bltu;
  reg	 [4:0] ctrl_alucontrol;
  reg  ctrl_auipc;
  reg	 ctrl_lui;	 		
  reg	 ctrl_regwrite;
  reg  ctrl_alusrc;
  reg	 ctrl_memtoreg;
  reg	 ctrl_memwrite;
  reg  ctrl_branch;
  reg  ctrl_jal;
  reg  ctrl_jalr;
  reg  [4:0] ctrl_rs1;
  reg  [4:0] ctrl_rs2;
  reg  [4:0] ctrl_rd;
  reg  [31:0] ex_pc;
  reg  [31:0] ex_inst;
  reg  [31:0] ex_rs1_data;
  reg  [31:0] ex_rs2_data;
  reg  [31:0] ex_auipc_lui_imm;
  reg  [31:0] ex_se_imm_itype;
  reg  [31:0] ex_se_imm_stype;
  reg  [4:0]  ex_alucontrol;
  reg  [4:0] ex_rd;
  reg  [4:0] ex_rs1;
  reg  [4:0] ex_rs2;
  reg ex_auipc;
  reg ex_lui;	
  reg ex_regwrite;
  reg ex_alusrc;
  reg ex_memtoreg;
  reg ex_memwrite;
  reg ex_branch;
  reg ex_jal;
  reg ex_jalr;
  reg  [31:0] ex_MemWdata;
  wire [31:0] ex_aluout;
  reg [31:0] tmp_ex_rs2_data;
  reg  [4:0] mem_rd;
  reg mem_regwrite;
  reg mem_alusrc; 
  reg mem_memtoreg; 
  reg mem_jal;
  reg mem_jalr;
  reg mem_lui;
  reg mem_auipc;
  reg  [31:0] wb_pc;
  reg  [31:0] wb_MemRdata;
  reg  [31:0] wb_aluout;
  reg wb_memtoreg;
  reg wb_regwrite;
  reg wb_alusrc;
  reg wb_jal;
  reg wb_jalr;
  reg wb_lui;
  reg wb_auipc;
  wire [4:0] tmp_rd;  
  wire [4:0] tmp_rs1;  
  wire [4:0] tmp_rs2;
  reg ex_mem_fwd_rs1;
  reg ex_mem_fwd_rs2;
  reg ex_wb_fwd_rs1;
  reg ex_wb_fwd_rs2;
  reg id_mem_fwd_rs2;
  reg id_wb_fwd_rs1;
  reg id_wb_fwd_rs2;
  reg enable;
  reg ex_enable;
  reg ex_f3beq;
  reg ex_f3blt;
  reg ex_f3bgeu;
  reg ex_f3bne;
  reg ex_f3bge;
  reg ex_f3bltu;
  reg [31:0] mem_pc;
  wire [2:0] id_check_load;
  wire [2:0] ex_check_load;
  wire [2:0] mem_check_load;
  wire [2:0] wb_check_load;
  wire if_id_write;
  wire b_enable;
  wire bne_taken;
  wire bge_taken;
  
  wire [31:0] branch_dest, jal_dest, jalr_dest;
  wire		  Nflag, Zflag, Cflag, Vflag;
  wire		  f3beq, f3blt, f3bgeu, f3bge, f3bltu;
  wire		  beq_taken;
  wire		  blt_taken;
  wire 	     bgeu_taken;
  wire        bltu_taken;

  assign rs1 = id_inst[19:15];
  assign rs2 = id_inst[24:20];
  assign tmp_rd  = id_inst[11:7];
  assign funct3  = id_inst[14:12];
  
//############## Hyokyung Kim : Start ##################
  //
  // PC (Program Counter) logic 
  //
  assign f3beq  = (funct3 == 3'b000);
  assign f3blt  = (funct3 == 3'b100);
  assign f3bgeu = (funct3 == 3'b111);
  assign f3bne  = (funct3 == 3'b001);
  assign f3bge  = (funct3 == 3'b101);
  assign f3bltu = (funct3 == 3'b110);

  assign beq_taken  =  ex_branch & ex_f3beq & Zflag;
  assign blt_taken  =  ex_branch & ex_f3blt & (Nflag != Vflag);
  assign bgeu_taken =  ex_branch & ex_f3bgeu & Cflag;
  assign bne_taken  =  ex_branch & ex_f3bne & (!Zflag);
  assign bge_taken  =  ex_branch & ex_f3bge & (Nflag == Vflag);
  assign bltu_taken =  ex_branch & ex_f3bltu & (!Cflag);
  
  assign b_enable   =  beq_taken | blt_taken | bgeu_taken | bne_taken | bge_taken | bltu_taken;
  
  assign branch_dest = (ex_pc + se_br_imm);
  assign jal_dest 	= (ex_pc + se_jal_imm);
  assign jalr_dest   = ex_aluout;

  always @(posedge clk, posedge reset)
  begin
     if (reset)          pc <= 32'b0;
	  else if (enable)
	  begin
	      if (b_enable) // branch_taken
				             pc <= #`simdelay branch_dest;
		   else if (ex_jal) // jal
				             pc <= #`simdelay jal_dest;
			else if (ex_jalr) //jalr
				             pc <= #`simdelay jalr_dest;
		   else 
				             pc <= #`simdelay (pc + 4);
	  end
  end


  // JAL immediate
  assign jal_imm[20:1] = {ex_inst[31],ex_inst[19:12],ex_inst[20],ex_inst[30:21]};
  assign se_jal_imm[31:0] = {{11{jal_imm[20]}},jal_imm[20:1],1'b0};

  //JALR immediate
  assign se_jalr_imm[31:0] = {{20{ex_inst[31]}},ex_inst[31:20]};
  
  // Branch immediate
  assign br_imm[12:1] = {ex_inst[31],ex_inst[7],ex_inst[30:25],ex_inst[11:8]};
  assign se_br_imm[31:0] = {{19{br_imm[12]}},br_imm[12:1],1'b0};

//############## Hyokyung Kim : End ##################

  // 
  // Register File 
  //
  regfile i_regfile(
    .clk			(clk),
    .we			(wb_regwrite),
    .rs1			(rs1),
    .rs2			(rs2),
    .rd			(rd),
    .rd_data	(rd_data),
    .rs1_data	(rs1_data),
    .rs2_data	(rs2_data));


	//
	// ALU 
	//
	alu i_alu(
		.a			(alusrc1),
		.b			(alusrc2),
		.alucont	(ex_alucontrol),
		.result	(ex_aluout),
		.N			(Nflag),
		.Z			(Zflag),
		.C			(Cflag),
		.V			(Vflag));

	// 1st source to ALU (alusrc1)
	always@(*)
	begin
		if      (ex_auipc)	               alusrc1[31:0]  =  ex_pc;
		else if (ex_lui) 		               alusrc1[31:0]  =  32'b0;
		else if (ex_mem_fwd_rs1 & !mem_memwrite) alusrc1[31:0]  =  aluout;
		else if (ex_wb_fwd_rs1)             alusrc1[31:0]  =  rd_data;
		else          		                  alusrc1[31:0]  =  ex_rs1_data;
	end
	
	//############## Hyokyung Kim : Start ##################
	// 2nd source to ALU (alusrc2)
	always@(*)
	begin
		if	     (ex_auipc | ex_lui)			alusrc2[31:0] = ex_auipc_lui_imm[31:0];
		else if (ex_alusrc & ex_memwrite)	alusrc2[31:0] = ex_se_imm_stype[31:0];
		else if (ex_alusrc)					   alusrc2[31:0] = ex_se_imm_itype[31:0];
		else if (ex_jalr)                   alusrc2[31:0] = se_jalr_imm[31:0];
		else if (ex_mem_fwd_rs2 & enable)   alusrc2[31:0] = aluout;
		else if (ex_wb_fwd_rs2)             alusrc2[31:0] = rd_data;//wb_aluout;
		else									      alusrc2[31:0] = ex_rs2_data;
	end
		//############## Hyokyung Kim : End ##################
	assign se_imm_itype[31:0]  = {{20{id_inst[31]}},id_inst[31:20]};
	assign se_imm_stype[31:0]  = {{20{id_inst[31]}},id_inst[31:25],id_inst[11:7]};
	assign auipc_lui_imm[31:0] = {id_inst[31:12],12'b0};

	//############## Hyokyung Kim : Start ##################
	// Data selection for writing to RF
	always@(*)
	begin
		if	     (wb_jal | wb_jalr)			rd_data[31:0] = wb_pc + 4;
		else if (wb_memtoreg)	         rd_data[31:0] = wb_MemRdata;
		else						            rd_data[31:0] = wb_aluout;
	end
		//############## Hyokyung Kim : End ##################
	//*********************FORWARDING MODULES**************************//
	always @(*)
	begin
		if (id_mem_fwd_rs2)
			//ex_MemWdata = aluout;
			tmp_rs2_data = aluout;
		else if (id_wb_fwd_rs2)
			tmp_rs2_data = rd_data;
		else 
		   tmp_rs2_data = rs2_data;
   end
	
	always @(*)
	begin
		if (id_wb_fwd_rs1)
			tmp_rs1_data = rd_data;
		else
			tmp_rs1_data = rs1_data;
	end
	
		//edited///
	always @(*)
	begin
	if (ex_wb_fwd_rs2 & !ex_mem_fwd_rs2)
		//for MemWdata;
		tmp_ex_rs2_data[31:0] = rd_data;
	else if (ex_mem_fwd_rs2)
		tmp_ex_rs2_data[31:0] = aluout;
	else 	   
		tmp_ex_rs2_data[31:0] = ex_rs2_data;
	   //tmp_ex_rs2_data[31:0] = ex_MemWdata;
	end

	
	//############## Hyokyung Kim : Start ##################
	//*******************************************************************
	//************************FLIPFLOP***********************************
	//*******************************************************************
				//if-id FLIP FLOP
	always @(posedge clk, posedge reset)    // clock의 rising edge마다 활성화
	begin
		if(reset)
			begin
			id_inst[31:0]       <= #`simdelay 32'b0;
			id_pc[31:0]         <= #`simdelay 32'b0;
			id_alucontrol[4:0]  <= #`simdelay 5'b0;
			id_auipc            <= #`simdelay 1'b0;
			id_lui      		  <= #`simdelay 1'b0;	 		
			id_regwrite         <= #`simdelay 1'b0;
			id_alusrc           <= #`simdelay 1'b0;
			id_memtoreg         <= #`simdelay 1'b0;
			id_memwrite         <= #`simdelay 1'b0;
			id_branch           <= #`simdelay 1'b0;
			id_jal              <= #`simdelay 1'b0;
			id_jalr             <= #`simdelay 1'b0;
			//id_f3beq				  <= #`simdelay 1'b0;
			//id_f3blt            <= #`simdelay 1'b0;
			//id_f3bgeu           <= #`simdelay 1'b0;
			//id_f3bne            <= #`simdelay 1'b0;
			//id_f3bge            <= #`simdelay 1'b0;
			//id_f3bltu            <= #`simdelay 1'b0;
			end
		else if (b_enable | ex_jal | ex_jalr)
		begin
			id_inst[31:0]       <= #`simdelay 32'b0;
			id_pc[31:0]         <= #`simdelay 32'b0;
			id_alucontrol[4:0]  <= #`simdelay 5'b0;
			id_auipc            <= #`simdelay 1'b0;
			id_lui      		  <= #`simdelay 1'b0;	 		
			id_regwrite         <= #`simdelay 1'b0;
			id_alusrc           <= #`simdelay 1'b0;
			id_memtoreg         <= #`simdelay 1'b0;
			id_memwrite         <= #`simdelay 1'b0;
			id_branch           <= #`simdelay 1'b0;
			id_jal              <= #`simdelay 1'b0;
			id_jalr             <= #`simdelay 1'b0;
			//id_f3beq				  <= #`simdelay 1'b0;
			//id_f3blt            <= #`simdelay 1'b0;
			//id_f3bgeu           <= #`simdelay 1'b0;
			//id_f3bne            <= #`simdelay 1'b0;
			//id_f3bge            <= #`simdelay 1'b0;
			//id_f3bltu           <= #`simdelay 1'b0;
		end
		else if (enable)
			begin
			id_inst[31:0]       <= #`simdelay inst;
			id_pc[31:0]         <= #`simdelay pc;
			id_alucontrol[4:0]  <= #`simdelay alucontrol;
			id_auipc            <= #`simdelay auipc;
			id_lui              <= #`simdelay lui;	 		
			id_regwrite         <= #`simdelay regwrite;
			id_alusrc           <= #`simdelay alusrc;
			id_memtoreg         <= #`simdelay memtoreg;
			id_memwrite         <= #`simdelay memwrite;
			id_branch           <= #`simdelay branch;
			id_jal              <= #`simdelay jal;
			id_jalr             <= #`simdelay jalr;
			//id_f3beq				  <= #`simdelay f3beq;
			//id_f3blt            <= #`simdelay f3blt;
			//id_f3bgeu           <= #`simdelay f3bgeu;
			//id_f3bne            <= #`simdelay f3bne;
			//id_f3bge            <= #`simdelay f3bge;
			//id_f3bltu            <= #`simdelay f3bltu;
			end		
	end

	//hazard detection unit
	always @(*)
	begin
		if (ex_check_load == 3'b111)
		begin
			if (ex_rd == rs1 )       enable = 1'b0;
			else if (ex_rd == rs2 )  enable = 1'b0;
			else                     enable = 1'b1;
		end
		else
	   begin
			                         enable = 1'b1;
		end
	end

	assign id_check_load = {id_regwrite, id_alusrc, id_memtoreg};
	assign ex_check_load = {ex_regwrite, ex_alusrc, ex_memtoreg};
	assign mem_check_load = {mem_regwrite, mem_alusrc, mem_memtoreg};
	assign wb_check_load = {wb_regwrite, wb_alusrc, wb_memtoreg};
	
	
	always @(*)
	begin
		if (enable)
		begin
			//ctrl_alucontrol[4:0] =  id_alucontrol;
			ctrl_auipc           =  id_auipc;
			ctrl_lui             =  id_lui;	 		
			ctrl_regwrite        =  id_regwrite;
			ctrl_alusrc          =  id_alusrc;
			ctrl_memtoreg        =   id_memtoreg;
			ctrl_memwrite        =  id_memwrite;
			ctrl_rs1[4:0]        =  rs1;
			ctrl_rs2[4:0]        =  rs2;
			ctrl_rd[4:0]         =  tmp_rd;
			ctrl_branch          =  id_branch;
			ctrl_jal             =  id_jal;
			ctrl_jalr            =  id_jalr;
		end
		else
		begin
			//ctrl_alucontrol[4:0] =  5'b0;
			ctrl_auipc         =  1'b0;
			ctrl_lui           =  1'b0;	 		
			ctrl_regwrite      =  1'b0;
			ctrl_alusrc        =  1'b0;
			ctrl_memtoreg      =  1'b0;
			ctrl_memwrite      =  1'b0;
			ctrl_rs1[4:0]      =  5'b0;
			ctrl_rs2[4:0]      =  5'b0;
			ctrl_rd[4:0]       =  5'b0;
			ctrl_branch          =  1'b0;
			ctrl_jal             =  1'b0;
			ctrl_jalr            =  1'b0;
		end
	end
	
			//id-ex FLIP FLOP
	always @(posedge clk, posedge reset)    // clock의 rising edge마다 활성화
	begin
		if(reset)
			begin
			ex_pc[31:0]           <= #`simdelay 32'b0;
			ex_inst[31:0]         <= #`simdelay 32'b0;
			ex_rs1_data[31:0]     <= #`simdelay 32'b0;
			ex_rs2_data[31:0]     <= #`simdelay 32'b0;
			ex_auipc_lui_imm[31:0]<= #`simdelay 32'b0;
			ex_se_imm_itype[31:0] <= #`simdelay 32'b0;
			ex_se_imm_stype[31:0] <= #`simdelay 32'b0;
			ex_rs1[4:0]           <= #`simdelay 5'b0;
			ex_rs2[4:0]           <= #`simdelay 5'b0;
			ex_rd[4:0]            <= #`simdelay 5'b0;
			ex_alucontrol[4:0]    <= #`simdelay 5'b0;
			ex_auipc              <= #`simdelay 1'b0;
			ex_lui                <= #`simdelay 1'b0;	 		
			ex_regwrite           <= #`simdelay 1'b0;
			ex_alusrc             <= #`simdelay 1'b0;
			ex_memtoreg           <= #`simdelay 1'b0;
			ex_memwrite           <= #`simdelay 1'b0;
			ex_enable             <= #`simdelay 1'b0;
			ex_branch             <= #`simdelay 1'b0;
			ex_jal                <= #`simdelay 1'b0;
			ex_jalr               <= #`simdelay 1'b0;
			ex_f3beq              <= #`simdelay 1'b0;
			ex_f3blt              <= #`simdelay 1'b0;
			ex_f3bgeu             <= #`simdelay 1'b0;
			ex_f3bne              <= #`simdelay 1'b0;
			ex_f3bge              <= #`simdelay 1'b0;
			ex_f3bltu              <= #`simdelay 1'b0;
			end
		else if (b_enable | ex_jal | ex_jalr)
		begin
			ex_pc[31:0]           <= #`simdelay 32'b0;
			ex_inst[31:0]         <= #`simdelay 32'b0;
			ex_rs1_data[31:0]     <= #`simdelay 32'b0;
			ex_rs2_data[31:0]     <= #`simdelay 32'b0;
			ex_auipc_lui_imm[31:0]<= #`simdelay 32'b0;
			ex_se_imm_itype[31:0] <= #`simdelay 32'b0;
			ex_se_imm_stype[31:0] <= #`simdelay 32'b0;
			ex_rs1[4:0]           <= #`simdelay 5'b0;
			ex_rs2[4:0]           <= #`simdelay 5'b0;
			ex_rd[4:0]            <= #`simdelay 5'b0;
			ex_alucontrol[4:0]    <= #`simdelay 5'b0;
			ex_auipc              <= #`simdelay 1'b0;
			ex_lui                <= #`simdelay 1'b0;	 		
			ex_regwrite           <= #`simdelay 1'b0;
			ex_alusrc             <= #`simdelay 1'b0;
			ex_memtoreg           <= #`simdelay 1'b0;
			ex_memwrite           <= #`simdelay 1'b0;
			ex_enable             <= #`simdelay 1'b0;
			ex_branch             <= #`simdelay 1'b0;
			ex_jal                <= #`simdelay 1'b0;
			ex_jalr               <= #`simdelay 1'b0;
			ex_f3beq              <= #`simdelay 1'b0;
			ex_f3blt              <= #`simdelay 1'b0;
			ex_f3bgeu             <= #`simdelay 1'b0;
			ex_f3bne              <= #`simdelay 1'b0;
			ex_f3bge              <= #`simdelay 1'b0;	
			ex_f3bltu              <= #`simdelay 1'b0;
		end
		else
			begin
			ex_pc[31:0]           <= #`simdelay id_pc;
			ex_inst[31:0]         <= #`simdelay id_inst;
			ex_rs1_data[31:0]     <= #`simdelay tmp_rs1_data;
			ex_rs2_data[31:0]     <= #`simdelay tmp_rs2_data;
			ex_MemWdata[31:0]     <= #`simdelay tmp_rs2_data;
			ex_auipc_lui_imm[31:0]<= #`simdelay auipc_lui_imm;
			ex_se_imm_itype[31:0] <= #`simdelay se_imm_itype;
			ex_se_imm_stype[31:0] <= #`simdelay se_imm_stype;
			ex_rd[4:0]            <= #`simdelay ctrl_rd;
			ex_rs1[4:0]           <= #`simdelay ctrl_rs1;//ctrl_rs1;
			ex_rs2[4:0]           <= #`simdelay ctrl_rs2;//ctrl_rs2;
			ex_alucontrol[4:0]    <= #`simdelay id_alucontrol;//ctrl_alucontrol;
			ex_auipc              <= #`simdelay ctrl_auipc;
			ex_lui                <= #`simdelay ctrl_lui;	 		
			ex_regwrite           <= #`simdelay ctrl_regwrite;
			ex_alusrc             <= #`simdelay id_alusrc;//ctrl_alusrc;
			ex_memtoreg           <= #`simdelay ctrl_memtoreg;
			ex_memwrite           <= #`simdelay ctrl_memwrite;
			ex_enable             <= #`simdelay enable;
			ex_branch             <= #`simdelay ctrl_branch;
			ex_jal                <= #`simdelay ctrl_jal;
			ex_jalr               <= #`simdelay ctrl_jalr;
			ex_f3beq              <= #`simdelay f3beq;
			ex_f3blt              <= #`simdelay f3blt;
			ex_f3bgeu             <= #`simdelay f3bgeu;
			ex_f3bne              <= #`simdelay f3bne;
			ex_f3bge              <= #`simdelay f3bge;
			ex_f3bltu             <= #`simdelay f3bltu;
			end		
	end
	
	
		//ex-mem FLIP FLOP
	always @(posedge clk, posedge reset)    // clock의 rising edge마다 활성화
	begin
		if(reset)
			begin
			mem_pc[31:0]   <= #`simdelay 32'b0;
			MemWdata[31:0] <= #`simdelay 32'b0;
			aluout[31:0]   <= #`simdelay 32'b0;
			mem_rd[4:0]    <= #`simdelay 32'b0;
			mem_regwrite   <= #`simdelay 1'b0;
			mem_alusrc     <= #`simdelay 1'b0;
			mem_memtoreg   <= #`simdelay 1'b0;
			mem_memwrite   <= #`simdelay 1'b0; 
			mem_jal        <= #`simdelay 1'b0;
			mem_jalr       <= #`simdelay 1'b0;
			mem_lui        <= #`simdelay 1'b0;
			mem_auipc      <= #`simdelay 1'b0;
			end
		else
			begin
			mem_pc[31:0]   <= #`simdelay ex_pc;
			MemWdata[31:0] <= #`simdelay tmp_ex_rs2_data;
			aluout[31:0]   <= #`simdelay ex_aluout;
			mem_rd[4:0]    <= #`simdelay ex_rd;
			mem_regwrite   <= #`simdelay ex_regwrite;
			mem_alusrc     <= #`simdelay ex_alusrc;
			mem_memtoreg   <= #`simdelay ex_memtoreg;
			mem_memwrite   <= #`simdelay ex_memwrite;
			mem_jal        <= #`simdelay ex_jal;
			mem_jalr       <= #`simdelay ex_jalr;
			mem_lui        <= #`simdelay ex_lui;
			mem_auipc      <= #`simdelay ex_auipc;
			end		
	end
	
		//mem-wb FLIP FLOP
	always @(posedge clk, posedge reset)    // clock의 rising edge마다 활성화
	begin
		if(reset)
			begin
			wb_pc[31:0]       <= #`simdelay 32'b0;
			wb_MemRdata[31:0] <= #`simdelay 32'b0;
			wb_aluout[31:0]   <= #`simdelay 32'b0;
			rd[4:0]           <= #`simdelay 4'b0;
			wb_regwrite       <= #`simdelay 1'b0;
			wb_memtoreg       <= #`simdelay 1'b0;
			wb_alusrc         <= #`simdelay 1'b0;
			wb_jal            <= #`simdelay 1'b0;
			wb_lui            <= #`simdelay 1'b0;
			wb_auipc          <= #`simdelay 1'b0;
			end
		else
			begin
			wb_pc[31:0]       <= #`simdelay mem_pc;
			wb_MemRdata[31:0] <= #`simdelay MemRdata;
			wb_aluout[31:0]   <= #`simdelay aluout;
			rd[4:0]           <= #`simdelay mem_rd;
			wb_regwrite       <= #`simdelay mem_regwrite;
			wb_memtoreg       <= #`simdelay mem_memtoreg;
			wb_jal            <= #`simdelay mem_jal;
			wb_jalr           <= #`simdelay mem_jalr;
			wb_lui            <= #`simdelay mem_lui;
			wb_auipc          <= #`simdelay mem_auipc;
			wb_alusrc         <= #`simdelay mem_alusrc;
			end		
	end
	
	//############## Hyokyung Kim : End ##################
	
	//**************************FORWARDING BIT CHECK *****************************//
	always @(*)
	begin
		if (ex_rs1 == mem_rd & ex_rs1 != 5'b0 & ex_enable)     ex_mem_fwd_rs1 = 1'b1;
		else             			                               ex_mem_fwd_rs1 = 1'b0;

		if (ex_rs2 == mem_rd & ex_rs2 != 5'b0 & ex_enable)     ex_mem_fwd_rs2 = 1'b1;
		else			                                           ex_mem_fwd_rs2 = 1'b0;
	end
	
	always @(*)
	begin
		if (ex_rs1 == rd & ex_rs1 != 5'b0 & ex_enable)    ex_wb_fwd_rs1 = 1'b1;
		else 	                                            ex_wb_fwd_rs1 = 1'b0;
		
		if (ex_rs2 == rd & ex_rs2 != 5'b0 & ex_enable)    ex_wb_fwd_rs2 = 1'b1;
		else                                              ex_wb_fwd_rs2 = 1'b0;
	end
	
	always @(*)
	begin
		if (rs2 == mem_rd & rs2 != 5'b0)             id_mem_fwd_rs2 = 1'b1;
		else                                         id_mem_fwd_rs2 = 1'b0;
	end
	
		
	always @(*)
	begin
		if (rs1 == rd & rs1 != 5'b0)                  id_wb_fwd_rs1 = 1'b1;
		else                                          id_wb_fwd_rs1 = 1'b0;
		if (rs2 == rd & rs2 != 5'b0)                  id_wb_fwd_rs2 = 1'b1;
		else                                          id_wb_fwd_rs2 = 1'b0;
	end
   //############## Hyokyung Kim : End ##################

endmodule
