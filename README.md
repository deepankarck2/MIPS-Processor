# MIPS-Processor

In this project, I have designed and implemented the components for a 32-bit MIPS processor using VHDL and Xilinx software. 

# Processor Design:

![image](https://user-images.githubusercontent.com/52084764/210125109-fa56945e-9846-44d1-9119-3c25ffb9ee60.png)

An interesting video on how CPU works: https://youtu.be/cNN_tTXABUA 

# Components

Some of the components required for the design of a CPU are:
### Decoder:
The purpose of the instruction decoder is to generate proper control signals based
on the Opcode of an instruction fetched from the instruction memory

### ACU:
THE ACU stands for ALU control unit, and it determines what operations the ALU will perform based on the decoded instruction. 

### ALU:
The ALU, or Arithmetic Logic Unit, is a fundamental component of a processor that performs arithmetic and logical operations. It is responsible for carrying out the instructions that manipulate data in a computer's memory.

The ALU typically performs operations such as addition, subtraction, multiplication, division, and bitwise operations (e.g., AND, OR, XOR). It also performs comparison operations, such as testing for equality or inequality, which can be used to make decisions in control flow instructions.

The ALU receives input from the registers in the processor, performs the specified operation, and stores the result back in a register or memory location. The operation to be performed and the input data are typically specified by the instruction being executed.

## The processor will support the following instructions:
![image](https://user-images.githubusercontent.com/52084764/210022863-b27c7176-81b8-4fa6-bc73-da47d57173c6.png)
