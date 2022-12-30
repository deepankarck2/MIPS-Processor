		.data
memory:	.space 50
		.align 2

		.text
		.globl	main

		# This is a MIPS assembly code to produce Fibonacci Series: F(n)=F(n-2)+F(n-1)
		# i.e. F(1)=1, F(2)=1, F(3)=2, F(4)=3, F(5)=5, ...
		# Generate at least the first two numbers, continue until we reach a predefined index number  
		# The output are written into the data segment of the system memory
		# The use of registers:
		# $t0: output memory address
		# $t1: F(n-2)
		# $t2: F(n-1)
		# $t3: F(n)
		# $t4: loop index 
		# $t5: total number of Fibonacci number to be produced (>=2)

main:	
		# initialize all registers
		la $t0, memory # pointer to the memeory address for writing
		# the memory address can also be passed to $t0 using, for example, 
		# li $t0, 0x10010000 or
		# addiu $t0, $zero, 0x10010000
		li $t1, 1 # initialize $t1
		li $t2, 0 # initialize $t2
		li $t3, 1 # initialize $t3
		li $t4, 1 # initialize loop index
		li $t5, 10 # number of Fibonacci numbers to produce
	
		# produce the first Fibonacci number
		sw $t3, 0($t0) # write F(1)
		addi $t0, $t0, 4 # increment the memory pointer
	
loop:	# loop until the loop index reaches the total number we need
		addu $t3, $t1, $t2 # update the value of F(n)
		sw $t3, 0($t0) # write the next Fibonacci number to memory
		addi $t0, $t0, 4 # increment the pointer
		move $t2, $t1 # update F(n-1)
		move $t1, $t3 # update F(n-2)
		addi $t4, $t4, 1 # decrement the loop index
		bne $t5, $t4, loop # continue unless we reach the target

exit:	li $v0, 10
		syscall
			
		
	
	
