**GPIO:**



**Configure GPIO pin as Output/Input:**

1. Enable GPIO port bus (AHB1) in RCC\_AHB1ENR
2. Configure ip, op, alternate fn in GPIO\_MODER REG
3. Also, configure GPIO OTYPER, OSPEED, PUPD regs
4. Use GPIO Input and output data regs to rea/ write (IDR, ODR)





**Configure GPIO pin as External Interrupt:**

1. Enable GPIO port peripheral clock (AHB1 bus) in RCC\_AHB1ENR
2. Configure the Edge trigger for the interrupt using EXTI\_FTSR(Falling Trigger) or EXTI\_RTSR(rising) or both
3. Enable SYSCFG clock (APB2 bus) in RCC\_APB2ENR
4. Configure the corresponding port pin as a source for EXTIT using SYSCFG\_EXTICR reg
5. Enable the gpio Interrupt using EXTI\_IMR
6. Configure the IRQ - Priority using NVIC\_IPR reg ( only upper 4 bits are implemented for each IRQ -> priority ranges from 0 - 15)
7. Enable IRQ interrupt using NVIC\_ISER (Interrupt set Enable Reg)
8. Make sure clear EXTI\_PR reg (Interrupt pending reg or it will go in a loop)







**SPI:**

1. Configure Gpio Pin behave as SPI pins using GPIO-Alternate Fn reg and GPIO peripheral clock
2. Configure SPI using SPI-Control regs ->SCLK speed, Full/Half Duplex, 8bit/16 bit, Master/Slave
    . IN SSM = 1, SW Slave Management mode to save NSS pin, You Must **set** SSI bit in CR1 to disable the Slave Slect Internal
    . IN SSM = 0 (default), HW Slave Management To Make NSS Pin low when SPI Enable and drive high when SPI disable, You must set SSOE bit (slave select output enable) in CR2
3. After all configuration are made, turn on the SPI mode using SPE in CR1 reg. After SPE bit is set you cant change settings
   
4. Check the SPI-Status Reg to send or Receive Data.
5. After Data transfer Disable SPE to terminate communication.



