Single instruction 5-stage pipelined MIPS CPU based on Verilog HDL.

## MIPS_CPU
Implement 5-Stage pipelined MIPS CPU architecture from David_Harris and Sarah_Harris's Skeleton code.
- Test environment
  - Altera(Intel) DE0 FPGA board
  - Quartus2 13.1
  - ModelSim-Altera 10.1d

- Added Features
  - 5 Stage pipeline (IF, DE, EX, MEM, WB)
  - Forwarding (Resolve Data hazard)
  - Stall and Flush with branch instruction
    
- Overall block diagram
![Figure1](https://github.com/JunyeonL/MIPS_CPU/blob/master/blockdgram.jpg)
