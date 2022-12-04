.data
    n: .word 11
.text
.globl __start


jal x0, __start
#----------------------------------------------------Do not modify above text----------------------------------------------------
FUNCTION:
# Todo: Define your own function
# We store the input n in register a0, and you should store your result in register a1
	addi x31, x0, 10 
	addi x30, x0, 1
	addi x29,x0, 2
	addi x2, x2, -4		 
	sw   x1, 0(x2)       		# store pointer
	#sw  a5, 0(x2)			# store the previous a5 value 
	bge  a0, x31, greater_than_10   # if n > 10, go to greater_than_10 
	bge  a0, x30, middle   		# if 1 < n < 10, go to middle
	lw   x1,0(x2)			#return pointer
	addi a1, x0, 7     		# if n = 0, T(0)=7
	addi x2, x2, 4     		# reset stack point
	jalr x0, 0(x1)

greater_than_10:
	#compute n = 3n/4
	slli a2, a0, 1                  # a2 = 2n
	add  a3, a0, a2			# a3 = 3n
	srli a0,a3,2			# a0 = 3n/4
	#compute 0.875n(7n/8)
	slli a4,a2,1			# a4 = 4n
	add  a5,a4,a3			# a5 = 7n
	srli a5,a5,3			# a5 = 7n/8
	addi x2,x2 -4
	sw   a5,0(x2)			#store the a5 value 
	jal  x1, FUNCTION               # do recursive fuction for n =3n/4 
	lw   a5,0(x2)			#load the  a5 value
	addi x2,x2,4
	lw   x1, 0(x2)                  # load the return address
	addi x2, x2, 4                  # reset stack point
	mul  a1, a1, x29                # multiply by 2
	add  a1,a1,a5			# add 0.875n
	addi a1, a1, -137               # add -137
	jalr x0, 0(x1)                  # return
middle:
	#compute n = n-1
	addi a0,a0,-1
	jal  x1, FUNCTION               # do recursive fuction for n = n-1
	lw   x1, 0(x2)                  # load the return address
	addi x2, x2, 4                  # reset stack point
	mul  a1, a1, x29                # multiply by 2
	jalr x0, 0(x1)                  # return

#----------------------------------------------------Do not modify below text----------------------------------------------------
__start:
    la   t0, n
    lw   a0, 0(t0)
    jal  x1, FUNCTION
    la   t0, n
    sw   a1, 4(t0)
    li a7, 10
    ecall