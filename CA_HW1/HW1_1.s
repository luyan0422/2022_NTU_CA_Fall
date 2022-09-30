#----------------------------------------------------Do not modify below text----------------------------------------------------
.data
  str1: .string	"This is HW1_1:\nBefore sorting: \n"
  str2: .string	"\nAfter sorting:\n"
  str3: .string	"  "
  num: .dword  10, -2, 4, -7, 6, 9, 3, 1, -5, -8

.globl main

.text
main:
  # Print initiate
  li a7, 4
  la a0, str1
  ecall
  
  # a2 stores the num address, a3 stores the length of  num
  la a2, num
  li a3, 10
  jal prints
  
  la a2, num
  li a3, 10
  jal sort
  
  # Print result
  li a7, 4
  la a0, str2
  ecall

  la a2, num
  li a3, 10
  jal prints
  
  # End the program
  li a7, 10
  ecall
#----------------------------------------------------Do not modify above text----------------------------------------------------
sort:
### Start your code here ###
  addi sp, sp -40
  sw ra, 32(sp) #save return address on stack
  sw s6, 24(sp) #save s6 on stack
  sw s5, 16(sp) #save x21 on stack
  sw s4, 8(sp) #save x20 on stack
  sw s3, 0(sp) #save x19 on stack
  
  mv s5 a2 #copy num address to s5
  mv s6 a3 #copy length to s6
  
  li s3 0 #outer loop i = 0
for1tst:
  bge s3, s6, exit1 #if i > 10 exit
  addi s4, s3, -1 # j = i - 1
for2tst:
  bltz s4, exit2 #if j < 0 exit2
  slli t0, s4, 3 # t0 = j * 8
  add t0, s5, t0 # t0 = num[] + j * 8(num[j])
  lw t1, 0(t0) # t1 = num[j]
  lw t2, 8(t0) #x7 = num[j + 1]
  ble t1, t2, exit2 #go to exit2 if num[j] < num[j + 1]
  
  #if num[j] > num[j + 1] swap
  sw t1, 8(t0)
  sw t2, 0(t0)
  
  addi s4, s4, -1 # j--
  j for2tst

exit2:
  addi s3, s3, 1 # i++
  j for1tst

exit1:
  lw s3, 0(sp)
  lw s4, 8(sp)
  lw s5, 16(sp)
  lw s6, 24(sp)
  lw ra, 32(sp)
  addi sp, sp, 40
  
  jr ra
  

#----------------------------------------------------Do not modify below text----------------------------------------------------
# Print function	
prints:
  mv t0, zero # for(i=0)
  # a2 stores the num address, a3 stores the length of  num
  mv t1, a2
  mv t2, a3
printloop:
  bge t0, t2, printexit # if ( i>=length of num ) jump to printexit 
  slli t4, t0, 3
  add t5, t1, t4
  lw t3, 0(t5)
  li a7, 1 # print_int
  mv a0, t3
  ecall
	
  li a7, 4
  la a0, str3
  ecall 
	
  addi t0, t0, 1 # i = i + 1
  j printloop
printexit:
  jr ra
#----------------------------------------------------Do not modify above text----------------------------------------------------