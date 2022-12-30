
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
library UNISIM;
use UNISIM.VComponents.all;

entity CCSiMP32 is
    Port ( I_CLK : in  STD_LOGIC;
           I_EN : in  STD_LOGIC);
end CCSiMP32;

architecture CCSiMP32_arch of CCSiMP32 is

component ACU is 
		Port(I_ACU_ALUOP : in  STD_LOGIC_VECTOR (1 downto 0);
           I_ACU_FUNCT : in  STD_LOGIC_VECTOR (5 downto 0);
           O_ACU_CTL : out  STD_LOGIC_VECTOR (3 downto 0));
end component;

component DEC is
    Port ( I_DEC_EN : in  STD_LOGIC;
           I_DEC_OPCODE : in  STD_LOGIC_VECTOR (5 downto 0);
           O_DEC_REGDST : out  STD_LOGIC;
           O_DEC_JUMP : out  STD_LOGIC;
           O_DEC_BEQ : out  STD_LOGIC;
           O_DEC_BNE : out  STD_LOGIC;
           O_DEC_MEMREAD : out  STD_LOGIC;
           O_DEC_MEMTOREG : out  STD_LOGIC;
           O_DEC_ALUOP : out  STD_LOGIC_VECTOR (1 downto 0);
           O_DEC_MEMWRITE : out  STD_LOGIC;
           O_DEC_ALUSRC : out  STD_LOGIC;
           O_DEC_REGWRITE : out  STD_LOGIC);
end component;

component FSM is
    Port ( I_FSM_CLK : in  STD_LOGIC;
           I_FSM_EN : in  STD_LOGIC;
           I_FSM_INST : in  STD_LOGIC_VECTOR (31 downto 0);
           O_FSM_IF : out  STD_LOGIC;
           O_FSM_ID : out  STD_LOGIC;
           O_FSM_EX : out  STD_LOGIC;
           O_FSM_ME : out  STD_LOGIC;
           O_FSM_WB : out  STD_LOGIC);
end component;

component RAM is
    Port ( I_RAM_EN : in  STD_LOGIC;
           I_RAM_RE : in  STD_LOGIC;
           I_RAM_WE : in  STD_LOGIC;
           I_RAM_ADDR : in  STD_LOGIC_VECTOR (31 downto 0);
           I_RAM_DATA : in  STD_LOGIC_VECTOR (31 downto 0);
           O_RAM_DATA : out  STD_LOGIC_VECTOR (31 downto 0));
end component;

component ROM is
    Port ( I_ROM_EN : in  STD_LOGIC;
           I_ROM_ADDR : in  STD_LOGIC_VECTOR (31 downto 0);
           O_ROM_DATA : out  STD_LOGIC_VECTOR (31 downto 0));
end component;

component Reg is
    Port ( I_REG_EN : in  STD_LOGIC;
           I_REG_WE : in  STD_LOGIC;
           I_REG_SEL_RS : in  STD_LOGIC_VECTOR (4 downto 0);
           I_REG_SEL_RT : in  STD_LOGIC_VECTOR (4 downto 0);
           I_REG_SEL_RD : in  STD_LOGIC_VECTOR (4 downto 0);
           I_REG_DATA_RD : in  STD_LOGIC_VECTOR (31 downto 0);
           O_REG_DATA_A : out  STD_LOGIC_VECTOR (31 downto 0);
           O_REG_DATA_B : out  STD_LOGIC_VECTOR (31 downto 0));
end component;

component add1 is
    Port ( I_ADD1_A : in  STD_LOGIC_VECTOR (31 downto 0);
           O_ADD1_OUT : out  STD_LOGIC_VECTOR (31 downto 0));
end component;

component add2 is
    Port ( I_ADD2_A : in  STD_LOGIC_VECTOR (31 downto 0);
           I_ADD2_B : in  STD_LOGIC_VECTOR (31 downto 0);
           O_ADD2_OUT : out  STD_LOGIC_VECTOR (31 downto 0));
end component;

component ext is
    Port ( I_EXT_16 : in  STD_LOGIC_VECTOR (15 downto 0);
           O_EXT_32 : out  STD_LOGIC_VECTOR (31 downto 0));
end component;

component mux32 is
    Port ( I_MUX_0 : in  STD_LOGIC_VECTOR (31 downto 0);
           I_MUX_1 : in  STD_LOGIC_VECTOR (31 downto 0);
           I_MUX_SEL : in  STD_LOGIC;
           O_MUX_OUT : out  STD_LOGIC_VECTOR (31 downto 0));
end component;

component mux5 is
    Port ( I_MUX_0 : in  STD_LOGIC_VECTOR (4 downto 0);
           I_MUX_1 : in  STD_LOGIC_VECTOR (4 downto 0);
           I_MUX_SEL : in  STD_LOGIC;
           O_MUX_OUT : out  STD_LOGIC_VECTOR (4 downto 0));
end component;

component pc is
    Port ( I_PC_UPDATE : in  STD_LOGIC;
           I_PC : in  STD_LOGIC_VECTOR (31 downto 0);
           O_PC : out  STD_LOGIC_VECTOR (31 downto 0));
end component;

component shift26 is
    Port ( I_SHIFT_A : in  STD_LOGIC_VECTOR (25 downto 0);
           O_SHIFT_OUT : out  STD_LOGIC_VECTOR (27 downto 0));
end component;

component shift32 is
    Port ( I_SHIFT_A : in  STD_LOGIC_VECTOR (31 downto 0);
           O_SHIFT_OUT : out  STD_LOGIC_VECTOR (31 downto 0));
end component;

component ALU is
    Port ( I_ALU_EN : in  STD_LOGIC;
           I_ALU_CTL : in  STD_LOGIC_VECTOR (3 downto 0);
           I_ALU_A : in  STD_LOGIC_VECTOR (31 downto 0);
           I_ALU_B : in  STD_LOGIC_VECTOR (31 downto 0);
           O_ALU_OUT : out  STD_LOGIC_VECTOR (31 downto 0);
           O_ALU_ZERO : out  STD_LOGIC);
end component;

component AND2 is 
		Port (I0 : in STD_LOGIC;
				I1: in STD_LOGIC;
				O: out STD_LOGIC);
end component;

component OR2 is 
		Port (I0 : in STD_LOGIC;
				I1: in STD_LOGIC;
				O: out STD_LOGIC);
end component;

component INV is 
		Port (I : in STD_LOGIC;
				O: out STD_LOGIC);
end component;



-- FSM outputs
signal IF_state, ID_state, EX_state, ME_state, WB_state : std_logic;

-- ROM ouputs
-- the actual instruction
signal inst_data: std_logic_vector (31 downto 0);

-- PC outputs
signal inst_addr: std_logic_vector (31 downto 0);

-- DEC ouputs
signal DEC_REGDST, DEC_JUMP, DEC_BEQ, DEC_BNE, DEC_MEMREAD, DEC_MEMTOREG: std_logic;
signal DEC_MEMWRITE, DEC_ALUSRC, DEC_REGWRITE: std_logic;
signal DEC_ALUOP: std_logic_vector (1 downto 0);

-- U5 (AND2) output
-- enables the register to write
signal regWrite_en: std_logic;

-- U6 (MUX5) output
-- enables the register to write
signal write_regNum: std_logic_vector (4 downto 0);

-- output of MUX32 component (MUX2 in slide 7 of CCSiMP slide deck)
-- data that needs to be written into the write register
-- passed as input to Reg
signal writeReg_data: std_logic_vector (31 downto 0);

-- Reg outputs
signal REG_DATA_A, REG_DATA_B: std_logic_vector (31 downto 0);

--ACU ouputs
signal ACU_CTL: std_logic_vector (3 downto 0);

-- ext output
signal ext_offset: std_logic_vector (31 downto 0);

-- U10 (MUX3) output --> input B for ALU
signal mux3_ALU_B: std_logic_vector (31 downto 0);

-- ALU Outputs
signal ALU_OUT: std_logic_vector (31 downto 0);
signal ALU_ZERO, ALU_ZERO_INV: std_logic;

-- RAM Output
signal RAM_DATA: std_logic_vector (31 downto 0);

-- ADD1 output
signal ADD1_OUT: std_logic_vector (31 downto 0);

-- shift32 Output --> branch offset
signal branch_offset: std_logic_vector (31 downto 0);

-- ADD2 Output
signal ADD2_OUT: std_logic_vector (31 downto 0);

-- U17 Output
-- checking if beq inst & zero are both asserted
signal beq_inst: std_logic;

-- U19 Output
-- checking if bnq inst & NOT(zero) are both asserted
signal bne_inst: std_logic;

-- U20 output
-- OR gate to decide whether it's a beq or a bne instruction
-- will be the selection bit for MUX1
signal mux1_sel: std_logic;

-- MUX1 output
signal mux1_out: std_logic_vector (31 downto 0);

-- jump address
signal jump_addr: std_logic_vector (31 downto 0);
signal shifted_jumpAddr: std_logic_vector (27 downto 0);

-- MUX5 output
-- the new value for pc
signal mux5_out: std_logic_vector (31 downto 0);

begin
	U1: FSM port map(
						I_FSM_CLK => I_CLK,
						I_FSM_EN =>  I_EN,
						I_FSM_INST => inst_data,
						O_FSM_IF => IF_state,
						O_FSM_ID => ID_state,
						O_FSM_EX => EX_state,
						O_FSM_ME => ME_state,
						O_FSM_WB => WB_state);
	-- pc + 4
	U2: add1 port map ( 
						I_ADD1_A => inst_addr,
						O_ADD1_OUT => ADD1_OUT);
	U3: PC port map ( 
						I_PC_UPDATE => IF_state,
						I_PC => mux5_out,
						O_PC => inst_addr);
	U4: ROM port map( 
						I_ROM_EN => IF_state,
						I_ROM_ADDR => inst_addr,
						O_ROM_DATA => inst_data);
	U5: DEC port map ( 
						I_DEC_EN => ID_state,
						I_DEC_OPCODE => inst_data(31 downto 26),
						O_DEC_REGDST => DEC_REGDST,
						O_DEC_JUMP => DEC_JUMP,
						O_DEC_BEQ => DEC_BEQ,
						O_DEC_BNE => DEC_BNE,
						O_DEC_MEMREAD => DEC_MEMREAD,
						O_DEC_MEMTOREG => DEC_MEMTOREG,
						O_DEC_ALUOP => DEC_ALUOP,
						O_DEC_MEMWRITE => DEC_MEMWRITE,
						O_DEC_ALUSRC => DEC_ALUSRC,
						O_DEC_REGWRITE => DEC_REGWRITE);
	-- AND gate to enable the register
	-- 2 inputs: WB_state (write back state) * DEC_REGWRITE is asserted
	U6: AND2 port map (
						I0 => DEC_REGWRITE,
						I1 => WB_state,
						O => regWrite_en);
						
	-- select the register to be the write register
	-- MUX4
	U7: mux5 port map ( 
						I_MUX_0 => inst_data(20 downto 16),
						I_MUX_1 => inst_data(15 downto 11),
						I_MUX_SEL => DEC_REGDST,
						O_MUX_OUT => write_regNum);
	U8: Reg port map (
						I_REG_EN => ID_state,
						I_REG_WE => regWrite_en,
						I_REG_SEL_RS => inst_data(25 downto 21),
						I_REG_SEL_RT => inst_data(20 downto 16),
						I_REG_SEL_RD => write_regNum,
						I_REG_DATA_RD => writeReg_data,
						O_REG_DATA_A => REG_DATA_A,
						O_REG_DATA_B => REG_DATA_B);
	U9: ACU port map (
						I_ACU_ALUOP => DEC_ALUOP,
						I_ACU_FUNCT => inst_data (5 downto 0),
						O_ACU_CTL => ACU_CTL);
	-- sign extension of offset
	U10: ext port map (
						I_EXT_16 => inst_data (15 downto 0),
						O_EXT_32 => ext_offset);	
	-- MUX3
	U11: mux32 port map ( 
						I_MUX_0 => REG_DATA_B,
						I_MUX_1 => ext_offset,
						I_MUX_SEL => DEC_ALUSRC,
						O_MUX_OUT => mux3_ALU_B);
	
	U12: ALU port map ( 
						I_ALU_EN => EX_state,
						I_ALU_CTL => ACU_CTL,
						I_ALU_A => REG_DATA_A,
						I_ALU_B => mux3_ALU_B,
						O_ALU_OUT => ALU_OUT,
						O_ALU_ZERO => ALU_ZERO);
	U13: RAM port map (
						I_RAM_EN => ME_state,
						I_RAM_RE => DEC_MEMREAD,
						I_RAM_WE => DEC_MEMWRITE,
						I_RAM_ADDR => ALU_OUT,
						I_RAM_DATA => REG_DATA_B,
						O_RAM_DATA => RAM_DATA);
						
	-- MUX2
	U14: mux32 port map (
						I_MUX_0 => ALU_OUT,
						I_MUX_1 => RAM_DATA,
						I_MUX_SEL => DEC_MEMTOREG,
						O_MUX_OUT => writeReg_data);
	U15: shift32 port map (
						I_SHIFT_A => ext_offset,
						O_SHIFT_OUT => branch_offset);
	
	U16: add2 port map (
						I_ADD2_A => ADD1_OUT,
						I_ADD2_B => branch_offset,
						O_ADD2_OUT => ADD2_OUT);
	-- AND gate to check if DEC_BEQ and ALU_ZERO are both asserted
	-- essentially checking if beq instr
	U17: AND2 port map (
						I0 => DEC_BEQ,
						I1 => ALU_ZERO,
						O => beq_inst);
						
	U18: INV port map (
						I => ALU_ZERO,
						O => ALU_ZERO_INV);
						
	-- AND gate to check if DEC_BNE and NOT(ALU_ZERO) are both asserted
	-- essentially checking if bne instr
	U19: AND2 port map (
						I0 => DEC_BNE,
						I1 => ALU_ZERO_INV,
						O => bne_inst);
	U20: OR2 port map (
						I0 => beq_inst,
						I1 => bne_inst,
						O => mux1_sel);
	-- MUX1
	U21: mux32 port map (
						I_MUX_0 => ADD1_OUT,
						I_MUX_1 => ADD2_OUT,
						I_MUX_SEL => mux1_sel,
						O_MUX_OUT => mux1_out);
						
	U22: shift26 port map ( 
						I_SHIFT_A => inst_data(25 downto 0),
						O_SHIFT_OUT => shifted_jumpAddr);
	-- build the jump address from first 4 bits of PC + 4 (saved in ADD1_OUT)
	-- and shifted jump address
	jump_addr <= ADD1_OUT (31 downto 28) & shifted_jumpAddr;
	
	-- MUX5
	U23: mux32 port map (
						I_MUX_0 => mux1_out,
						I_MUX_1 => jump_addr,
						I_MUX_SEL => DEC_JUMP,
						O_MUX_OUT => mux5_out);


end CCSiMP32_arch;