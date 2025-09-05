//;=======================================================================================================================
//;=======================================================================================================================
//;File:			NY8TE64A.h
//;Description:	The Header File for NY8TE64A
//;Author:		Laurent-Chaung
//;Date:			2021/06/25
//;=======================================================================================================================
//;=======================================================================================================================
//;-----------------------------------------------------------------------------------------------------------------------
//;MOVR and MOVAR instrutions for access R-page Register (General Purpose Register)
//;-----------------------------------------------------------------------------------------------------------------------
//;R-page sregisters					;	bit7	|	bit6	|	bit5	|	bit4	|	bit3	|	bit2	|	bit1	|	bit0
//;-----------------------------------------------------------------------------------------------------------------------	
//; 0x00 --------- Indirect Addressing Register
//#define INDF				0x00	;	INDF[7]		INDF[6]		INDF[5]		INDF[4]		INDF[3]		INDF[2]		INDF[1]		INDF[0]
//; 0x01 --------- Timer0 Data Register	
//#define TMR0				0x01	;	TMR0_DATA7	TMR0_DATA6	TMR0_DATA5	TMR0_DATA4	TMR0_DATA3	TMR0_DATA2	TMR0_DATA1	TMR0_DATA0
//; 0x02 --------- Low Byte of Program Counter
//#define PCL					0x02	;	PCL[7]		PCL[6]		PCL[5]		PCL[4]		PCL[3]		PCL[2]		PCL[1]		PCL[0]
//; 0x03 --------- Status Register
//#define STATUS				0x03 	;	BK[1]		BK[0]		GP[5]		/TO			/PD			Z			DC			C	
//; 0x04 --------- File Selection Register (Include SRAM Bank Select)
//#define FSR					0x04	;	GP[7]		FSR[6]		FSR[5]		FSR[4]		FSR[3]		FSR[2]		FSR[1]		FSR[0]	         
//; 0x05 --------- PortA Data Register
//#define PORTA				0x05	;	PA[7]		PA[6]		PA[5]		PA[4]		PA[3]		PA[2]		PA[1]		PA[0]
//; 0x06 --------- PortB Data Register
//#define PORTB				0x06	;	PB[7]		PB[6]		PB[5]		PB[4]		PB[3]		PB[2]		PB[1]		PB[0]	
//; 0x07 --------- PortC Data Register
//#define PORTC				0x07	;	-			-			-			-			-			-			PC[1]		PC[0]
//; 0x08 --------- Power Control Register (WatchDog, LVD, LVR Control  )
//#define PCON				0x08 	;	WDTEn		/PAPL[4]	LVDEn		/PHPA[5]	LVREn		GP[2]		EEWR_FAL	LOCK
//; 0x09 --------- PortB Wakeup Control Register
//#define BWUCON				0x09 	;	WUPB[7]		WUPB[6]		WUPB[5]		WUPB[4]		WUPB[3]		WUPB[2]		WUPB[1]		WUPB[0]
//; 0x0A --------- High Byte of Program Counter (B'xxxxxDDD')
//#define PCHBUF				0x0A 	;	-			XSPD_STP	-			-			PCHBUF[3]	PCHBUF[2]	PCHBUF[1]	PCHBUF[0]
//; 0x0B --------- PortA&B Pull-Low Control Register
//#define ABPLCON				0x0B 	;	/PLPB[3]	/PLPB[2]	/PLPB[1]	/PLPB[0]	/PLPA[3]	/PLPA[2]	/PLPA[1]	/PLPA[0]	
//; 0x0C --------- PortB Pull-High Control Register
//#define BPHCON				0x0C 	;	/PHPB[7]	/PHPB[6]	/PHPB[5]	/PHPB[4]	/PHPB[3]    /PHPB[2]	/PHPB[1]	/PHPB[0]	
//; 0x0D --------- PortC Pull-High Control Register
//#define CPHCON				0x0D	;	-			-			-			-			-			-			/PHPC[1]	/PHPC[0]
//; 0x0E --------- Interrupt Enable Register
//#define INTE				0x0E 	;	INT1IE		WDTIE		-			LVDIE		T1IE		INT0IE		PABIE		T0IE	
//; 0x0F --------- Interrupt Flag	Register (Write '0' to Clear Flag)
//#define INTF				0x0F 	;	INT1IF		WDTIF		-			LVDIF		T1IF		INT0IF		PABIF		T0IF	
//; 0x10 --------- ADC mode Register
//#define ADMD				0x10 	;	ADEn		START		EOC			GCHS		CHS[3]		CHS[2]		CHS[1]		CHS[0]
//; 0x11 --------- ADC clock, ADC interrupt flag and ADC low 4-bit data output Register
//#define ADR					0x11 	;	ADIF		ADIE		ADCK[1]		ADCK[0]		AD[3]		AD[2]		AD[1]		AD[0]
//; 0x12 --------- ADC output data Register  (ADC high 8-bit data output Register)
//#define ADD					0x12 	;	AD[11]		AD[10]		AD[9]		AD[8]		AD[7]		AD[6]		AD[5]		AD[4]
//; 0x13 --------- ADC high reference voltage Register
//#define ADVREFH				0x13	;	EVHEnB		-			-			-			-			-			VHS[1]		VHS[0]
//; 0x14 --------- ADC Sampling pulse width and ADC conversion bit Register and AIN pin control Register
//#define ADCR				0x14	;	PBCON[7]	PBCON[6]	PBCON[5]	PBCON[4]	SHCK[1]		SHCK[0]		ADCR[1]		ADCR[0]
//; 0x15 --------- PortA Wakeup Control Register
//#define AWUCON				0x15	;	WUPA[7]		WUPA[6]		WUPA[5]		WUPA[4]		WUPA[3]		WUPA[2]		WUPA[1]		WUPA[0]
//; 0x16 --------- AIN pin control Register
//#define PACON				0x16	;	PBCON[3]	PBCON[2]	PBCON[1]	PACON[4]	PACON[3]	PACON[2]	PACON[1]	PACON[0]
//; 0x17 --------- ADOFFSET
//#define	ADJMD				0x17	;	-			-			ADJ_SIGN	ADJ[4]		ADJ[3]		ADJ[2]		ADJ[1]		ADJ[0]
//; 0x18 --------- External Interrupt Contorl Register
//#define INTEDG 				0x18	;	INT2DEG		ExINT2En	ExINT1En	ExINTEn		INT1_Edg[1]	INT1_Edg[0]	INT_Edg[1]	INT_Edg[0]	
//; 0x19 --------- TIMER1 Data and PWMDUTY1 msb 2 bits Register
//#define TMRH  				0x19	;	-			-			TMR1_DATA9	TMR1_DATA8	PWM2_DUTY9	PWM2_DUTY8	PWM1_DUTY9	PWM1_DUTY8	
//; 0x1A --------- Analog Circuit Enable Register
//#define ANAEN				0x1A 	;	CMPEn		-			-			-			-			-			-			-
//; 0x1B --------- Resistor to Frequency Converter Control Register
//#define RFC 				0x1B	;	RFCEn		-			-			-			PADSel[3]	PADSel[2]	PADSel[1]	PADSel[0]
//; 0x1C --------- TIMER3 Data and PWMDUTY3 msb 2 bits Register
//#define TMR4RH  			0x1C	;	TMR4_DATA9	TMR4_DATA8	-			-			PWM4_DUTY9	PWM4_DUTY8	PWM3_DUTY9	PWM3_DUTY8
//; 0x1D --------- I_HRC frequency Trim High Byte
//#define OSCCALH				0x1D	;	-			-			-			-			-			OSC[10]		OSC[9]		OSC[8]	
//; 0x1E --------- I_HRC frequency Trim Low Byte
//#define OSCCALL				0x1E 	;	OSC[7]		OSC[6]		OSC[5]		OSC[4]		OSC[3]		OSC[2]		OSC[1]		OSC[0]	
//; 0x1F --------- Interrupt2 Enable/Flag Register
//#define INTE2 				0x1F	;	INT2IF		T4IF			-			-		INT2IE		T4IE			-			-			
//;-----------------------------------------------------------------------------------------------------------------------
//;T0MD and T0MDR instrutions for access T0MD Register
//;-----------------------------------------------------------------------------------------------------------------------
//; 0xx --------- Timer0 Control Register (Only Accessed by Instruction T0MD)
//;T0MD				0x0xx			;	LClkSrc		GP[6]		ClkSel		EdgeSel		PS0WDT		PS0Div[2]	PS0Div[1]	PS0Div[0]	
//;-----------------------------------------------------------------------------------------------------------------------
//;IOST and IOSTR instrution for access F-page Register (IO Configuration Register)
//;-----------------------------------------------------------------------------------------------------------------------
//;F-page registers					;	bit7	|	bit6	|	bit5	|	bit4	|	bit3	|	bit2	|	bit1	|	bit0
//;-----------------------------------------------------------------------------------------------------------------------	
//; 0x01 --------- Reserved
//; 0x02 --------- Reserved
//; 0x03 --------- Reserved
//; 0x04 --------- Reserved
//; 0x05 --------- PortA Direction(1:Input/0:Output) Control Register
//#define IOSTA				0x05	;	IOPA[7]		IOPA[6]		IOPA[5]		IOPA[4]		IOPA[3]		IOPA[2]		IOPA[1]		IOPA[0]
//; 0x06 --------- PortB Direction(1:Input/0:Output) Control Register
//#define IOSTB				0x06	;	IOPB[7]		IOPB[6]		IOPB[5]		IOPB[4]		IOPB[3]		IOPB[2]		IOPB[1]		IOPB[0]
//; 0x07 --------- PortC Direction(1:Input/0:Output) Control Register
//#define IOSTC				0x07	;	-			-			-			-			-			-			IOPC[1]		IOPC[0]
//; 0x08 --------- Reserved
//; 0x09 --------- PortA Pull-High Control Register (/PA[5]: Pull-Low)
//#define APHCON				0X09	;	/PHPA[7]	/PHPA[6]	/PLPA[5]	/PHPA[4]	/PHPA[3]	/PHPA[2]	/PHPA[1]	/PHPA[0]	
//; 0x0A --------- Prescaler0 Counter Value Register
//#define PS0CV				0x0A	;	PS0CV[7]	PS0CV[6]	PS0CV[5]	PS0CV[4]	PS0CV[3]	PS0CV[2]	PS0CV[1]	PS0CV[0]
//; 0x0B --------- PortC Pull-Low Control Register
//#define CPLCON				0X0B	;	-			-			-			-			-			-			/PLPC[1]	/PLPC[0]
//; 0x0C --------- PortB Open-Drain Control Register
//#define BODCON				0x0C	;	/ODPB[7]	/ODPB[6]	/ODPB[5]	/ODPB[4]	/ODPB[3]	/ODPB[2]	/ODPB[1]	/ODPB[0]
//; 0x0D --------- PortC Open-Drain Control Register
//#define CODCON				0X0D	;	-			-			-			-			-			-			/ODPC[1]	/ODPC[0]
//; 0x0E --------- Comparator voltage select Control Register
//#define CMPCR				0x0E	;   -			RBIAS_H		RBIAS_L		CMP_INV		SELP[1]		SELP[0]		SELN[1]		SELN[0]
//; 0x0F --------- Power Control Register 1
//#define PCON1				0x0F	;	INTEn		LVDOUT		LVDS[3]		LVDS[2]		LVDS[1]		LVDS[0]		GP[1]		TMR0En	
//
//;-----------------------------------------------------------------------------------------------------------------------
//;SFUN and SFUNR instrution for access S-page Register (Special Function Register)
//;-----------------------------------------------------------------------------------------------------------------------
//;S-page registers					;	bit7	|	bit6	|	bit5	|	bit4	|	bit3	|	bit2	|	bit1	|	bit0
//;-----------------------------------------------------------------------------------------------------------------------											
//; 0x00 --------- Timer1 Data Register	
//#define TMR1				0x00	;	TMR1_DATA7	TMR1_DATA6	TMR1_DATA5	TMR1_DATA4	TMR1_DATA3	TMR1_DATA2	TMR1_DATA1	TMR1_DATA0
//; 0x01 --------- Timer1 Control Register 1
//#define T1CR1				0x01	;	PWM1En		PWM1ActB	TM1OE		VFSEL1		TM1_HRC		OneShot		Reload		TMR1En	
//; 0x02 --------- Timer1 Control Register 2
//#define T1CR2				0x02	;	-			-			ClkSel		EdgeSel		/PS1En		PS1Div[2]	PS1Div[1]	PS1Div[0]
//; 0x03 --------- PWM1 Duty Register
//#define PWM1DUTY			0x03	;	PWM1_DUTY7	PWM1_DUTY6	PWM1_DUTY5	PWM1_DUTY4	PWM1_DUTY3	PWM1_DUTY2	PWM1_DUTY1	PWM1_DUTY0
//; 0x04 --------- Prescaler1 Counter Value Register
//#define PS1CV				0x04	;	PS1CV[7]	PS1CV[6]	PS1CV[5]	PS1CV[4]	PS1CV[3]	PS1CV[2]	PS1CV[1]	PS1CV[0]
//; 0x05 --------- Buzzer1 Control Register
//#define BZ1CR				0x05	;	BZ1En		-			-			-			FSel[3]		FSel[2]		FSel[1]		FSel[0]	
//; 0x06 --------- IR Control Register
//#define IRCR				0x06	;	IROSC358M    -			-			-			-			IRCSEL  	IRF57K		IREn	
//; 0x07 --------- Table Access High Byte Address Pointer Register
//#define TBHP				0x07	;	-			-			-			GP[4]	    HPoint[3]    HPoint[2]	HPoint[1]	HPoint[0]	
//; 0x08 --------- Table Access High Byte Data Register
//#define TBHD				0x08 	;	-			-			HData[5]	HData[4]	HData[3]	HData[2]	HData[1]	HData[0]
//; 0x09 --------- Reserved
//; 0x0A --------- PWM2 Control Register	1
//#define P2CR1				0x0A	;	PWM2En		PWM2ActB	-			-			-			-			-			-	
//; 0x0B --------- Reserved
//; 0x0C --------- PWM2 Duty Register
//#define PWM2DUTY			0x0C	;	PWM2_DUTY7	PWM2_DUTY6	PWM2_DUTY5	PWM2_DUTY4	PWM2_DUTY3	PWM2_DUTY2	PWM2_DUTY1	PWM2_DUTY0
//; 0x0D --------- Reserved
//; 0x0E --------- Reserved
//; 0x0F --------- Oscillation Control Register (Include Switch Working Mode)
//#define OSCCR				0x0F	;	CMPOUT		CMPOE		CMPIF		CMPIE		Mode[1]		Mode[0]		HOSC_Stop	HOSC_Sel
//; 0x10 --------- Reserved
//; 0x11 --------- PWM3 Control Register	1
//#define P3CR1				0x11	;	PWM3En		PWM3ActB	-			-			-			-			-			-	
//; 0x12 --------- Reserved
//; 0x13 --------- PWM3 Duty Register
//#define PWM3DUTY			0x13	;	PWM3_DUTY7	PWM3_DUTY6	PWM3_DUTY5	PWM3_DUTY4	PWM3_DUTY3	PWM3_DUTY2	PWM3_DUTY1	PWM3_DUTY0
//; 0x14 --------- Reserved
//; 0x15 --------- Timer4 Data Register
//#define TMR4 				0x15	;	TMR4_DATA7	TMR4_DATA6	TMR4_DATA5	TMR4_DATA4	TMR4_DATA3	TMR4_DATA2	TMR4_DATA1	TMR4_DATA0
//; 0x16 --------- Timer4 Control Register 1
//#define T4CR1 				0x16	;	PWM4En		PWM4ActB	-			VFSEL4		TM4_HRC		OneShot		Reload		TMR4En
//; 0x17 --------- Timer4 Control Register 2
//#define T4CR2 				0x17	;	-			-			ClkSel		EdgeSel		/PS4En		PS4Div[2]	PS4Div[1]	PS4Div[0]
//; 0x18 --------- PWM3 Duty Register
//#define	PWM4DUTY			0x18	;	PWM4_DUTY7	PWM4_DUTY6	PWM4_DUTY5	PWM4_DUTY4	PWM4_DUTY3	PWM4_DUTY2	PWM4_DUTY1	PWM4_DUTY0
//; 0x19 --------- Prescaler4 Counter Value Register
//#define PS4CV 				0x19	;	PS4CV[7]	PS4CV[6]	PS4CV[5]	PS4CV[4]	PS4CV[3]	PS4CV[2]	PS4CV[1]	PS4CV[0]
//; 0x1A --------- Timer5 Data Register
//#define TMR5 				0x1A	;	TMR5_DATA7	TMR5_DATA6	TMR5_DATA5	TMR5_DATA4	TMR5_DATA3	TMR5_DATA2	TMR5_DATA1	TMR5_DATA0
//; 0x1B --------- Timer5 Control Register 1
//#define T5CR1 				0x1B	;	PWM5En		PWM5ActB	-			VFSEL5		TM5_HRC		OneShot		Reload		TMR5En
//; 0x1C --------- Timer5 Control Register	2
//#define T5CR2 				0x1C	;	-			-			ClkSel		EdgeSel		/PS5En		PS5Div[2]	PS5Div[1]	PS5Div[0]
//; 0x1D --------- PWM5 Duty Register
//#define PWM5DUTY 			0x1D	; 	PWM5_DUTY7	PWM5_DUTY6	PWM5_DUTY5	PWM5_DUTY4	PWM5_DUTY3	PWM5_DUTY2	PWM5_DUTY1	PWM5_DUTY0
//; 0x1E --------- Prescaler5 Counter Value Register
//#define PS5CV 				0x1E	;	PS5CV[7]	PS5CV[6]	PS5CV[5]	PS5CV[4]	PS5CV[3]	PS5CV[2]	PS5CV[1]	PS5CV[0]
//; 0x1F --------- TIMER5 & PWMDUTY5 msb 2 bits Register
//#define TMR5RH 				0x1F 	;	-			-			TMR5_DATA9	TMR5_DATA8	-			-			PWM5_DUTY9	PWM5_DUTY8
//;-----------------------------------------------------------------------------------------------------------------------
//;TFUN and TFUNR instrution for access T-page Register (T-Page Special Function Register)
//;-----------------------------------------------------------------------------------------------------------------------
//;T-page registers					;	bit7	|	bit6	|	bit5	|	bit4	|	bit3	|	bit2	|	bit1	|	bit0
//;-----------------------------------------------------------------------------------------------------------------------											
//; 0x00 --------- Serial interface mode control register
//#define SIMCR 				0X00	;	SIMC[1]		SIMC[0]		MSTA		SSB_PADEn	RX_PADEn	TX_PADEn	RCLK_PADEn	BAUDOZ_PADEn
//; 0x01 --------- IIC mode Address register
//#define MADR 				0X01	;	MAD[7]		MAD[6]		MAD[5]		MAD[4]		MAD[3]		MAD[2]		MAD[1]		-
//; 0x02 --------- IIC mode frequency register
//#define MFDR 				0X02	;	-			-			-			FD[4]		FD[3]		FD[2]		FD[1]		FD[0]
//; 0x03 --------- IIC mode control register
//#define MCR 				0X03	;	-			-			-			MTX			TXAK		-			-			-
//; 0x04 --------- IIC mode status register
//#define MSR 				0X04	;	MCF			MAAS		MBB			MAL			-			SRW			MIF			RXAK
//; 0x05 --------- Serial interface mode data register
//#define SIMDR 				0X05	;	SIMD[7]		SIMD[6]		SIMD[5]		SIMD[4]		SIMD[3]		SIMD[2]		SIMD[1]		SIMD[0]
//; 0x06 --------- SPI control & status register
//#define SPCR 				0X06	;	SPIF		WCOL		-			MODF		CPOL		CKEG		SPR[1]		SPR[0]
//; 0x07 --------- Interrupt Enable 3th register
//#define INTE3 				0x07	;	SIMIE		EEIE		T5IE/CCPIE	LSRIE		TXIE		RXIE		TPOVIE		TPCPIE
//; 0x08 --------- Interrupt Flag 3th register	
//#define INTF3 				0x08	;	SIMIF		EEIF		T5IF/CCPIF	LSRIF		THRE		READY		TPOVIF		TPCPIF
//; 0x09 --------- Touch pad clock register
//#define TPCKS				0x09	;	-			-			-			WKUPT		-			TPCK[2]		TPCK[1]		TPCK[0]
//; 0x0A --------- Touch pad extra capacitance select register
//#define CASR 				0x0A	;	-			CA[6]		CA[5]		CA[4]		CA[3]		CA[2]		CA[1]		CA[0]
//; 0x0B --------- Touch pad channel select register
//#define TPCHS 				0x0B	;	-			-			-			CHS[4]		CHS[3]		CHS[2]		CHS[1]		CHS[0]
//; 0x0C --------- Touch pad control register
//#define TPCR 				0x0C	;	-			-			-			-			-			TPMD[2]		TPMD[1]		TPMD[0]
//; 0x0D --------- Touch pad low counter register
//#define TPCNTL 				0x0D	;	TPCNT[7]	TPCNT[6]	TPCNT[5]	TPCNT[4]	TPCNT[3] 	TPCNT[2]	TPCNT[1]	TPCNT[0]
//; 0x0E --------- Touch pad high counter register
//#define TPCNTH 				0x0E 	;	TPCNT[111]	TPCNT[110]	TPCNT[19]	TPCNT[18]	TPCNT[11]	TPCNT[10]	TPCNT[9]	TPCNT[8]
//; 0x0F --------- Touch pad 0~7 enable register
//#define TPPADEN 			0x0F 	; 	TPPADEn[7]	TPPADEn[6]	TPPADEn[5]	TPPADEn[4]	TPPADEn[3]	TPPADEn[2]	TPPADEn[1]	TPPADEn[0]
//; 0x10 --------- Touch pad 8~11 enable register
//#define TPPADEN1			0x10	; 	-			-			-			-			TPPADEn[11]	TPPADEn[10]	TPPADEn[9]	TPPADEn[8]
//; 0x11 --------- Touch pad group1 extra capacitance select register
//#define CASR1  				0X11	;	-			CA1[6]		CA1[5]		CA1[4]		CA1[3]		CA1[2]		CA1[1]		CA1[0]
//; 0x12 --------- Touch pad group2 extra capacitance select register
//#define CASR2  				0X12	;	-			CA2[6]		CA2[5]		CA2[4]		CA2[3]		CA2[2]		CA2[1]		CA2[0]
//; 0x13 --------- Reserved
//; 0x14 --------- Touch pad group1 low counter register
//#define TPCNTL1 			0X14	;	TPCNT1[7]	TPCNT1[6]	TPCNT1[5]	TPCNT1[4]	TPCNT1[3] 	TPCNT1[2]	TPCNT1[1]	TPCNT1[0]
//; 0x15 --------- Touch pad group2 low counter register
//#define TPCNTL2 			0X15	;	TPCNT2[7]	TPCNT2[6]	TPCNT2[5]	TPCNT2[4]	TPCNT2[3] 	TPCNT2[2]	TPCNT2[1]	TPCNT2[0]
//; 0x16 --------- Reserved
//; 0x17 --------- Touch pad group2 high counter register
//#define TPCNTH2 			0X17	;	-			-			-			-			TPCNT2[11] 	TPCNT2[10]	TPCNT2[9]	TPCNT2[8]
//; 0x18 --------- Transmit holding register /Receive buffer register
//#define THR 				0X18	;	URD[7]		URD[6]		URD[5]		URD[4]		URD[3] 		URD[2]		URD[1]		URD[0]
//; 0x19 --------- Line control register
//#define LCR 				0X19	;	LOOP		SBRK		PSTUCK		PEVEn		PREn 		STPS		WL[1]		WL[0]
//; 0x1A --------- Line status register
//#define LSR 				0X1A	;	-			TSRE		-			BKINT		FERR 		PERR		OERR		-
//; 0x1B --------- Baud rate divisor latch LSB register
//#define DLL 				0X1B	;	DLL[7]		DLL[6]		DLL[5]		DLL[4]		DLL[3]		DLL[2]		DLL[1]		DLL[0]
//; 0x1C --------- Baud rate divisor latch MSB register
//#define DLH 				0X1C	;	DLH[7]		DLH[6]		DLH[5]		DLH[4]		DLH[3]		DLH[2]		DLH[1]		DLH[0]
//; 0x1D --------- Reserved
//; 0x1E --------- CCP control register	 			
//#define CCP1CON				0X1E	;	PWM5M[1]	PWM5M[0]	FBCH1		FBCH0		CPPM[3]		CPPM[2]		CPPM[1]		CPPM[0]
//; 0x1F --------- Dead band control register
//#define PWMDB 				0X1F	;	DB[7]		DB[6]		DB[5]		DB[4]		DB[3]		DB[2]		DB[1]		DB[0]


//;=======================================================================================================================
//;=======================================================================================================================
//;-----------------------------------------------------------------------------------------------------------------------
//; R-page Special Function Register (General Purpose Register)
//;-----------------------------------------------------------------------------------------------------------------------
//;------------------------------------------------------------
//; INDF (0x00)	--------- Indirect Addressing Register
//;------------------------------------------------------------
//;Bit[7:0] : Indirect Address
#define C_Indir_Addr		0xFF

//;------------------------------------------------------------
//; TMR0 (0x01)	--------- Timer0 Data Register	
//;------------------------------------------------------------
//;Bit[7:0] : Timer0 Data
#define C_TMR0_Data			0xFF

//;------------------------------------------------------------
//; PCL (0x02)	--------- Low Byte of Program Counter
//;------------------------------------------------------------
//;Bit[7:0] : Low Byte of Program Counter
#define C_PCLow_Data		0xFF

//;------------------------------------------------------------
//; STATUS (0x03)		--------- Status Register (Include RAM Bank Select)
//;------------------------------------------------------------
//;Bit[7:6] : RAM Bank Selection 
#define C_RAM_Bank			0xC0			//; Select Ram Bank
#define C_RAM_Bank0			0x00			//; Select Ram Bank0
#define C_RAM_Bank1			0x40			//; Select Ram Bank1
#define C_RAM_Bank2			0x80			//; Select Ram Bank2
#define C_RAM_Bank3			0xC0			//; Select Ram Bank3	
//;Bit[5] : GP_ROM
//;Bit[4:0] : System Status
#define C_Status_WDT		0x10			//; WatchDog Overflow Flag
#define C_Status_SLP		0x08			//; Sleep (Power Down) Flag
#define C_Status_Z			0x04			//; Zero Flag
#define C_Status_HalfC		0x02			//; Half Carry/Half Borrow Flag
#define C_Status_C			0x01			//; Carry/Borrow Flag

	
#define C_Status_WDT_Bit	4
#define C_Status_SLP_Bit	3
#define C_Status_Z_Bit		2
#define Z					2
#define C_Status_HalfC_Bit	1
#define C_Status_C_Bit		0
;------------------------------------------------------------ 
//; FSR (0x04)		--------- File Selection Register       
//;------------------------------------------------------------
//;Bit[7] : GP_ROM
//;Bit[6:0] : Select one Register out of 128 registers of specific Bank.
#define	C_SFR_RAM_Addr		0x7F			//; RAM Address Bit[6:0] 
//;------------------------------------------------------------
//; PORTA(0x05)		--------- PortA Data Register
//;------------------------------------------------------------
//;Bit[7:0] : PORTA Data Bit Define	
#define	C_PA_Data			0xFF			//; PA Data
#define	C_PA7_Data			0x80			//; PA7 Data
#define	C_PA6_Data			0x40			//; PA6 Data
#define	C_PA5_Data			0x20			//; PA5 Data
#define	C_PA4_Data			0x10			//; PA4 Data
#define	C_PA3_Data			0x08			//; PA3 Data
#define	C_PA2_Data			0x04			//; PA2 Data
#define	C_PA1_Data			0x02			//; PA1 Data
#define	C_PA0_Data			0x01			//; PA0 Data
//;------------------------------------------------------------
//; PORTB (0x06)		--------- PortB Data Register
//;------------------------------------------------------------
//;Bit[7:0] : PORTB Data Bit Define	
#define	C_PB_Data			0xFF			//; PB Data
#define	C_PB7_Data			0x80			//; PB7 Data
#define	C_PB6_Data			0x40			//; PB6 Data
#define	C_PB5_Data			0x20			//; PB5 Data
#define	C_PB4_Data			0x10			//; PB4 Data
#define	C_PB3_Data			0x08			//; PB3 Data
#define	C_PB2_Data			0x04			//; PB2 Data
#define	C_PB1_Data			0x02			//; PB1 Data
#define	C_PB0_Data			0x01			//; PB0 Data
//;------------------------------------------------------------
//; PORTC (0x07)		--------- PortC Data Register
//;------------------------------------------------------------
//;Bit[1:0] : PORTC Data Bit Define	
#define	C_PC_Data			0xFF			//; PC Data
#define	C_PC1_Data			0x02			//; PC1 Data
#define	C_PC0_Data			0x01			//; PC0 Data
//;------------------------------------------------------------	
//; PCON (0x08)		--------- Power Control Register (WatchDog, LVD, LVR Control ) 
//;------------------------------------------------------------
//;Bit[7] : WatchDog Eanble
#define	C_WDT_En			0x80			//; WatchDog Enable
#define	C_WDT_Dis			0x00			//; WatchDog Disable
//;Bit[6] : APLCON4
#define	C_PA4_PLB_Dis		0x40			//; Disable PA4 Pull-Low
#define	C_PA4_PLB_En		0x00			//; Enable PA4 Pull-Low
//;Bit[5] : LVD Eanble
#define	C_LVD_En			0x20			//; LVD Enable
#define	C_LVD_Dis			0x00			//; LVD Disable
//;Bit[4] : PA5 Pull-High 
#define	C_PA5_PHB_Dis		0x10			//; Disable PA5 Pull-High
#define	C_PA5_PHB_En		0x00			//; Enable PA5 Pull-High
//;Bit[3] : LVR Enable
#define	C_LVR_En			0x08			//; LVR Enable
#define	C_LVR_Dis			0x00			//; LVR Disable
//;Bit[2] : Reserved
//;Bit[1] : EEPROM Write Error Flag
#define	C_EEW_ERR			0X01			//; EEPROM Write Fail Set to High	
//;Bit[0] : EEPROM LOCK Flag
#define	C_EE_LOCK_Flag		0X00			//; EEPROM Locked Output High

#define	C_WDT_En_Bit		7	
#define	C_PA4_PHL_Bit		6
#define	C_LVD_En_Bit		5	
#define	C_PA5_PHB_Bit		4
#define	C_LVR_En_Bit		3
#define	C_EEW_ERR_Bit		1
#define	C_EE_LOCK_Flag_Bit	0
//;------------------------------------------------------------	
//; BWUCON (0x09)	--------- PortB Wakeup Control Register
//;------------------------------------------------------------
//;Bit[7:0] : BWUCON Bit Define
#define	C_PB_Wakeup			0xFF			//; PortB Wakeup Enable
#define	C_PB7_Wakeup		0x80			//; PB7 Wakeup Enable
#define	C_PB6_Wakeup		0x40			//; PB6 Wakeup Enable
#define	C_PB5_Wakeup		0x20			//; PB5 Wakeup Enable
#define	C_PB4_Wakeup		0x10			//; PB4 Wakeup Enable
#define	C_PB3_Wakeup		0x08			//; PB3 Wakeup Enable
#define	C_PB2_Wakeup		0x04			//; PB2 Wakeup Enable 
#define	C_PB1_Wakeup		0x02			//; PB1 Wakeup Enable
#define	C_PB0_Wakeup		0x01			//; PB0 Wakeup Enable
		
//;------------------------------------------------------------	
//; PCHBUF (0x0A) --------- High Byte of Program Counter
//;------------------------------------------------------------
//;Bit[7] : Reserved
//;Bit[6] : Write 1 to Stop Crystal 32.768K Speed-Up Function, Write-Only
#define	C_XSPD_STP			0x40
//;Bit[5:4] : Reserved
//;Bit[3:0] : High Byte of Program Counter 0x0F
#define	C_PCHigh_Data		0x0F
//;------------------------------------------------------------		
//; ABPLCON (0x0B)	--------- PortA/B Pull-Low Control Register
//;------------------------------------------------------------
//;Bit[7:4] : PortA/B Pull-Low Control Register (1:Disable, 0:Pull-LOW)
#define C_PB_PLB			0xF0			//; PortB Pull-Low Control bit
#define C_PB3_PLB			0x80			//; PB3 Pull-Low Control bit
#define C_PB2_PLB			0x40			//; PB2 Pull-Low Control bit	
#define C_PB1_PLB			0x20			//; PB1 Pull-Low Control bit 	
#define C_PB0_PLB			0x10			//; PB0 Pull-Low Control bit
#define C_PA_PLB			0x0F			//; PortA Pull-Low Control bit
#define C_PA3_PLB			0x08			//; PA3 Pull-Low Control bit
#define C_PA2_PLB			0x04			//; PA2 Pull-Low Control bit	
#define C_PA1_PLB			0x02			//; PA1 Pull-Low Control bit 	
#define C_PA0_PLB			0x01			//; PA0 Pull-Low Control bit

//;------------------------------------------------------------	
//; BPHCON (0x0C)	--------- PortB Pull-High Control Register
//;------------------------------------------------------------
//;Bit[7:0]: PortB Pull-High Control Register (1:Disable, 0:Pull-High)
#define C_PB_PHB			0xFF			//; PortB Pull-High Control bit 
#define C_PB7_PHB			0x80			//; PB7 Pull-High Control bit
#define C_PB6_PHB			0x40			//; PB6 Pull-High Control bit
#define C_PB5_PHB			0x20			//; PB5 Pull-High Control bit
#define C_PB4_PHB			0x10			//; PB4 Pull-High Control bit
#define C_PB3_PHB			0x08			//; PB3 Pull-High Control bit	
#define C_PB2_PHB			0x04			//; PB2 Pull-High Control bit
#define C_PB1_PHB			0x02			//; PB1 Pull-High Control bit
#define C_PB0_PHB			0x01			//; PB0 Pull-High Control bit	
//;------------------------------------------------------------	
//; CPHCON (0x0D)	--------- PortC Pull-High Control Register
//;------------------------------------------------------------
//;Bit[1:0]: PortC Pull-High Control Register (1:Disable, 0:Pull-High)
#define C_PC_PHB			0xFF			//; PortC Pull-High Control bit 
#define C_PC1_PHB			0x02			//; PC1 Pull-High Control bit
#define C_PC0_PHB			0x01			//; PC0 Pull-High Control bit	
//;------------------------------------------------------------	
//; INTE (0x0E)		--------- Interrupt Enable Register
//; INTF (0x0F)		--------- Interrupt Flag
//;------------------------------------------------------------
//;Bit[7] : External interrupt 1 enable bit
#define	C_INT_EXT1			0x80			//; Enable external interrupt 1
//;Bit[6] : WDT timeout interrupt enable bit
#define	C_INT_WDT			0x40			//; Enable WDT timeout interrupt
//;Bit[5] : Reserved	
//;Bit[4] : Low-voltage detector interrupt enable bit
#define	C_INT_LVD			0x10			//; Enable LVD interrupt
#define	C_INT_CMP			0x10			//; Enable comparator interrupt
//;Bit[3] : Timer1 underflow interrupt enable bit
#define	C_INT_TMR1			0x08			//; Enable timer1 underflow interrupt
//;Bit[2] : External interrupt 0 enable bit
#define	C_INT_EXT0			0x04			//; Enable external interrupt 0
//;Bit[1] : PortA/PortB input change interrupt enable bit
#define	C_INT_PABKey		0x02			//; Enable portA/B input change interrupt
//;Bit[0] : Timer0 overflow interrupt enable bit
#define	C_INT_TMR0			0x01			//; Enable timer0 overflow interrupt

#define	C_INT_EXT1_Bit		7
#define	C_INT_WDT_Bit		6
#define	C_INT_LVD_Bit		4
#define	C_INT_CMP_Bit		4
#define	C_INT_TMR1_Bit		3
#define	C_INT_EXT0_Bit		2
#define	C_INT_PABKey_Bit	1
#define	C_INT_TMR0_Bit		0	
//;------------------------------------------------------------	
//; Pr_ADC_ADMD (0x10) --------- ADC mode Register
//;------------------------------------------------------------			
//;Bit[7] : Enable/Disable ADC bit
#define	C_ADC_En			0x80		//; Enable ADC
#define	C_ADC_Dis			0x00		//; Disable ADC	
//;Bit[6] : Start a ADC conversion session 
#define	C_ADC_Start			0x40		//; Write 1 to this bit, start ADC converting
//;Bit[5] : ADC is end-of-convert
#define	C_ADC_Finish		0x20		//; (1:ADC is Finish, 0:ADC is in procession) Read-only
//;Bit[4] : Globe Enable/Disable  ADC input channel bit
#define	C_ADC_CH_En			0x10		//; Enable ADC input channel
#define	C_ADC_CH_Dis		0x00		//; Disable all ADC input channel
//;Bit[3:0] : Select ADC input channel
#define	C_Quarter_VDD		0x0C		//; 1/4 VDD as ADC input		
#define	C_ADC_PB7			0x0B		//; PB7(AIN11) pad as ADC input channel		
#define	C_ADC_PB6			0x0A		//; PB6(AIN10) pad as ADC input channel		
#define	C_ADC_PB5			0x09		//; PB5(AIN9) pad as ADC input channel	
#define	C_ADC_PB4			0x08		//; PB4(AIN8) pad as ADC input channel	
#define	C_ADC_PB3			0x07		//; PB3(AIN7) pad as ADC input channel	
#define	C_ADC_PB2			0x06		//; PB2(AIN6) pad as ADC input channel		
#define	C_ADC_PB1			0x05		//; PB1(AIN5) pad as ADC input channel	
#define	C_ADC_PA4			0x04		//; PA4(AIN4) pad as ADC input channel	
#define	C_ADC_PA3			0x03		//; PA3(AIN3) pad as ADC input channel	
#define	C_ADC_PA2			0x02		//; PA2(AIN2) pad as ADC input channel	
#define	C_ADC_PA1			0x01		//; PA1(AIN1) pad as ADC input channel		
#define	C_ADC_PA0			0x00		//; PA0(AIN0) pad as ADC input channel

#define	C_ADC_En_Bit		0x7
#define	C_Start_Bit			0x6	
#define	C_Finish_Bit		0x5
#define	C_ADC_CH_En_Bit		0x4	
	
//;------------------------------------------------------------	
//; Pr_ADC_ADR (0x11) 	--------- ADC clock, ADC interrupt control and ADC low 4-bit data output Register
//;------------------------------------------------------------	
//;Bit[7] : ADC interrupt flag
#define C_INTF_ADC			0x00		//; clear ADC interrupt flag
//;Bit[6] : Enable/Disable of ADC interrupt
#define C_INT_ADC			0x40		//; Enable ADC interrupt
//;Bit[5:4] : Select one of the 4 ADC clock
#define C_ADC_CLK_Div2		0x30		//; ADC clock is Fcpu/2
#define C_ADC_CLK_Div1		0x20		//; ADC clock is Fcpu/1	
#define C_ADC_CLK_Div8		0x10		//; ADC clock is Fcpu/8
#define C_ADC_CLK_Div16		0x00		//; ADC clock is Fcpu/16	
//;Bit[3:0] : ADC low 4-bit data output Register.
#define C_ADC_Data_L		0x0F		//; AD3 ~ AD0 ,Read-only	
	
#define C_INF_ADC_Bit		0x7
#define C_INT_ADC_Bit		0x6	
	
//;------------------------------------------------------------	
//; Pr_ADC_Data (0x12) 	--------- ADC high 8-bit data output Register 
//;------------------------------------------------------------		
//;Bit[7:0] : High 8-bit of ADC data buffer
#define C_ADC_Data_H		0xFF		//; AD11 ~ AD4 , Read-only	

//;------------------------------------------------------------	
//; Pr_ADC_Vrefh (0x13) 	--------- ADC high reference voltage Register
//;------------------------------------------------------------		
//;Bit[7] : ADC reference high voltage (VREFH) select control bit	
#define C_Vrefh_External	0x80	//; ADC reference high voltage is supplied by external pin PA0
#define C_Vrefh_Internal	0x00	//; ADC reference high voltage is internal generated, the voltage selected depends on VHS1~0.
//;Bit[6:2] : Reserved
//;Bit[1:0] : Select ADC internal high reference voltage
#define C_Vrefh_VDD			0x03		//; ADC reference high voltage is VDD
#define C_Vrefh_4V			0x02		//; ADC reference high voltage is 4V	
#define C_Vrefh_3V			0x01		//; ADC reference high voltage is 3V	
#define C_Vrefh_2V			0x00		//; ADC reference high voltage is 2V

//;------------------------------------------------------------	
//; Pr_ADC_ADCR (0x14) 	--------- ADC Sampling pulse width and select ADC conversion bit Register and AIN pin control Register
//;------------------------------------------------------------
//;Bit[7:4] : Select PortB as pure AIN or GPIO pin
#define C_PB7_AIN11 		0x80		//; Select PB7 as pure AIN11 pin 
#define C_PB6_AIN10 		0x40		//; Select PB6 as pure AIN10 pin 
#define C_PB5_AIN9 			0x20		//; Select PB5 as pure AIN9 pin 
#define C_PB4_AIN8 			0x10		//; Select PB4 as pure AIN8 pin 
//;Bit[3:2] : Select ADC sampling pulse width
#define C_Sample_8CLK		0x0C		//; ADC Sampling pulse width is 8 ADC clock
#define C_Sample_4CLK		0x08		//; ADC Sampling pulse width is 4 ADC clock
#define C_Sample_2CLK		0x04		//; ADC Sampling pulse width is 2 ADC clock
#define C_Sample_1CLK		0x00		//; ADC Sampling pulse width is 1 ADC clock
//;Bit[1:0] : Select ADC conversion bit numbers   
#define C_12BIT				0x03		//; 12-bit ADC
#define C_10BIT				0x01		//; 10-bit ADC
#define C_8BIT				0x00		//;  8-bit ADC
	
//;------------------------------------------------------------	
//; Pr_PA_WakeUp_Ctrl (0x15) 	--------- PortA Wakeup Control Register
//;------------------------------------------------------------	
//;Bit[7:0] : AWUCON Bit Define
#define	C_PA_Wakeup			0xFF			//; Port A Wakeup Enable		
#define	C_PA7_Wakeup		0x80			//; PA7 Wakeup Enable
#define	C_PA6_Wakeup		0x40			//; PA6 Wakeup Enable 
#define	C_PA5_Wakeup		0x20			//; PA5 Wakeup Enable
#define	C_PA4_Wakeup		0x10			//; PA4 Wakeup Enable	
#define	C_PA3_Wakeup		0x08			//; PA3 Wakeup Enable
#define	C_PA2_Wakeup		0x04			//; PA2 Wakeup Enable 
#define	C_PA1_Wakeup		0x02			//; PA1 Wakeup Enable
#define	C_PA0_Wakeup		0x01			//; PA0 Wakeup Enable

//;------------------------------------------------------------	
//; PACON (0x16) 	--------- AIN pin control Register
//;------------------------------------------------------------	
//;Bit[7:0] : Select PortA/B as pure AIN or GPIO pin
#define	C_PAB_AIN 			0xFF			//; Select Port A/B as pure AIN pin for power-saving
#define	C_PB3_AIN7 			0x80			//; Select PB3 as pure AIN7 pin 
#define	C_PB2_AIN6 			0x40			//; Select PB2 as pure AIN6 pin 	
#define	C_PB1_AIN5 			0x20			//; Select PB1 as pure AIN5 pin 	
#define	C_PA4_AIN4 			0x10			//; Select PA4 as pure AIN4 pin 
#define	C_PA3_AIN3 			0x08			//; Select PA3 as pure AIN3 pin 
#define	C_PA2_AIN2 			0x04			//; Select PA2 as pure AIN2 pin 
#define	C_PA1_AIN1 			0x02			//; Select PA1 as pure AIN1 pin 	
#define	C_PA0_AIN0 			0x01			//; Select PA0 as pure AIN0 pin 	

#define	C_PB3_AIN7_Bit		7
#define	C_PB2_AIN6_Bit		6	
#define	C_PB1_AIN5_Bit		5
#define	C_PA4_AIN4_Bit		4
#define	C_PA3_AIN3_Bit		3
#define	C_PA2_AIN2_Bit		2
#define	C_PA1_AIN1_Bit		1	
#define	C_PA0_AIN0_Bit		0	
//;------------------------------------------------------------	
//; ADOFFSET (0x17) 	--------- ADOFFSET control Register
//;------------------------------------------------------------	
#define C_ADOFFSET			0X3F
//;------------------------------------------------------------	
//; INTEDG (0x18) 	--------- External Interrupt Contorl Register
//;------------------------------------------------------------	
//;Bit[7] : INT2 edge trigger select 
#define C_INT2_FallingEdge	0x00			//; Falling edge trigger	
#define C_INT2_RisingEdge	0x80			//; Rising edge trigger	
//;Bit[6] : External interrupt 2 select bit
#define C_INT2_En			0x40			//; PA5 set as INT2 input
#define C_INT2_Dis			0x00			//; PA5 set as GPIO
//;Bit[5] : External interrupt 1 select bit
#define C_INT1_En			0x20			//; PA3 set as INT1 input
#define C_INT1_Dis			0x00			//; PA3 set as GPIO
//;Bit[4] : External interrupt 0 select bit
#define C_INT0_En			0x10			//; PA4 set as INT0 input
#define C_INT0_Dis			0x00			//; PA4 set as GPIO
//;Bit[3:2] : INT1 edge trigger select 
#define C_INT1_Edge			0x0C			//; INT1 trigger edge  --- 11b: Rising & Falling Edge, 10b:Falling Edge, 01b:Rising Edge    
#define C_INT1_TwoEdge		0x0C			//; Rising & Falling edge trigger
#define C_INT1_FallingEdge	0x08			//; Falling edge trigger	
#define C_INT1_RisingEdge	0x04			//; Rising edge trigger			
//;Bit[1:0] : INT0 edge trigger select 
#define C_INT0_Edge			0x03			//; INT0 trigger edge  --- 11b: Rising & Falling Edge, 10b:Falling Edge, 01b:Rising Edge  
#define C_INT0_TwoEdge		0x03			//; Rising & Falling edge trigger
#define C_INT0_FallingEdge	0x02			//; Falling edge trigger	
#define C_INT0_RisingEdge	0x01			//; Rising edge trigger	

#define C_INT2_En_Bit		0x6
#define C_INT1_En_Bit		0x5
#define C_INT0_En_Bit		0x4

//;------------------------------------------------------------	
//; TMRH (0x19) 	--------- TIMER1 Data and PWMDUTY1 msb 2 bits Register
//;------------------------------------------------------------	
//;Bit[7:6] : Reserved	
//;Bit[5] : Timer1 Data Bit9	
#define	C_TMR1_Data_b9		0x20			//; Timer1 data bit9 
//;Bit[4] : Timer1 Data Bit8	
#define	C_TMR1_Data_b8		0x10			//; Timer1 data bit8		
//;Bit[3] : PWM2 Duty Bit9	
#define	C_PWM2_Duty_b9		0x08			//; PWM2 Duty bit9 
//;Bit[2] : PWM2 Duty Bit8	
#define	C_PWM2_Duty_b8		0x04			//; PWM2 Duty bit8
//;Bit[1] : PWM1 Duty Bit9	
#define	C_PWM1_Duty_b9		0x02			//; PWM1 Duty bit9 
//;Bit[0] : PWM1 Duty Bit8	
#define	C_PWM1_Duty_b8		0x01			//; PWM1 Duty bit8	

#define	C_TMR1_Data_b9_Bit	5
#define	C_TMR1_Data_b8_Bit	4
#define	C_PWM2_Duty_b9_Bit	3
#define	C_PWM2_Duty_b8_Bit	2	
#define	C_PWM1_Duty_b9_Bit	1
#define	C_PWM1_Duty_b8_Bit	0
//;------------------------------------------------------------	
//; ANAEN (0x1A)	--------- Analog Circuit Enable Register
//;------------------------------------------------------------
//;Bit[7] : Enable/disable voltage comparator
#define C_CMPEN				0x80			//;CMPEN: Enable/disable voltage comparator.
//;Bit[6:0] : Reserved

#define C_CMPEN_Bit			0x7
//;------------------------------------------------------------	
//; RFC (0x1B) 	--------- Resistor to Frequency Converter Control Register
//;------------------------------------------------------------		
//;Bit[7] : Enable/disable RFC function
#define C_RFC_En			0x80			//; Enable RFC function
#define C_RFC_Dis			0x00			//; Disable RFC function	
//;Bit[6:4] : Reserved
//;Bit[3:0] : Select one of the RFC pad
#define C_RFC_PB7			0X0F				//; Select PB7 as RFC pad
#define C_RFC_PB6			0X0E				//; Select PB6 as RFC pad
#define C_RFC_PB5			0X0D				//; Select PB5 as RFC pad
#define C_RFC_PB4			0X0C				//; Select PB4 as RFC pad	
#define C_RFC_PB3			0X0B				//; Select PB3 as RFC pad
#define C_RFC_PB2			0X0A				//; Select PB2 as RFC pad	
#define C_RFC_PB1			0X09				//; Select PB1 as RFC pad	
#define C_RFC_PB0			0X08				//; Select PB0 as RFC pad	
#define C_RFC_PA7			0X07				//; Select PA7 as RFC pad
#define C_RFC_PA6			0X06				//; Select PA6 as RFC pad
#define C_RFC_PA5			0X05				//; Select PA5 as RFC pad
#define C_RFC_PA4			0X04				//; Select PA4 as RFC pad
#define C_RFC_PA3			0X03				//; Select PA3 as RFC pad
#define C_RFC_PA2			0X02				//; Select PA2 as RFC pad	
#define C_RFC_PA1			0X01				//; Select PA1 as RFC pad
#define C_RFC_PA0			0X00				//; Select PA0 as RFC pad	
		
#define C_RFC_En_Bit		7
#define C_PSEL3_Bit			3
#define C_PSEL2_Bit			2
#define C_PSEL1_Bit			1
#define C_PSEL0_Bit			0	

//;------------------------------------------------------------	
//; TMR4RH (0x1C) 	--------- TIMER4 Data and PWMDUTY3/4 msb 2 bits Register
//;------------------------------------------------------------		
//;Bit[7] : Timer4 Data Bit9	
#define	C_TMR4_Data_b9		0x80			//; Timer4 data bit9 
//;Bit[6] : Timer4 Data Bit8	
#define	C_TMR4_Data_b8		0x40			//; Timer4 data bit8	
//;Bit[5:4] : Reserved
//;Bit[3] : PWM4 Duty Bit9	
#define C_PWM4_Duty_b9		0x08			//; PWM4 Duty bit9 
//;Bit[2] : PWM4 Duty Bit8	
#define C_PWM4_Duty_b8		0x04			//; PWM4 Duty bit8			
//;Bit[1] : PWM3 Duty Bit9	
#define C_PWM3_Duty_b9		0x02			//; PWM3 Duty bit9 
//;Bit[0] : PWM3 Duty Bit8	
#define C_PWM3_Duty_b8		0x01			//; PWM3 Duty bit8	


#define C_TMR4_Data_b9_Bit	7
#define C_TMR4_Data_b8_Bit	6
#define C_PWM4_Duty_b9_Bit	3
#define C_PWM4_Duty_b8_Bit	2	
#define C_PWM3_Duty_b9_Bit	1
#define C_PWM3_Duty_b8_Bit	0		
//;------------------------------------------------------------	
//; OSCCALH (0x1D) 	--------- Variable High Oscillator Frequency adjustment
//;------------------------------------------------------------	
//;Bit[7:3] : Reserved
//;Bit[2:0] : V_HRC frequency trim bit[10:8]

#define C_VHRC_Freq_b10_Bit	 2
#define C_VHRC_Freq_b9_Bit	 1
#define C_VHRC_Freq_b8_Bit	 0					

//;------------------------------------------------------------	
//; OSCCALL (0x1E) 	--------- Variable High Oscillator Frequency adjustment
//;------------------------------------------------------------	
//;Bit[7:0] : V_HRC frequency trim bit[7:0]

#define C_VHRC_Freq_b7_Bit	 7
#define C_VHRC_Freq_b6_Bit	 6	
#define C_VHRC_Freq_b5_Bit	 5	
#define C_VHRC_Freq_b4_Bit	 4	
#define C_VHRC_Freq_b3_Bit	 3	
#define C_VHRC_Freq_b2_Bit	 2	
#define C_VHRC_Freq_b1_Bit	 1	
#define C_VHRC_Freq_b0_Bit	 0		
	
//;------------------------------------------------------------	
//; INTE2 (0x1F)		--------- Interrupt2 Enable Register
//;------------------------------------------------------------
//;Bit[7] : EX_INT2 Interrupt flag bit
#define C_INTF_EXT2			0x80		//; EX_INT2 interrupt flag bit
//;Bit[6] : Timer4 underflow interrupt flag bit
#define C_INTF_TMR4			0x40		//; Timer4 underflow interrupt flag
//;Bit[5:4] : Reserved
//;Bit[3] : EX_INT2 interrupt enable bit	
#define C_INT_EXT2			0x08		//; Enable external interrupt 2
//;Bit[2] : Timer4 underflow interrupt enable bit
#define C_INT_TMR4			0x04		//; Enable timer4 underflow interrupt 
//;Bit[1:0] : Reserved

#define C_INTF_EXT2_Bit		7		
#define C_INTF_TMR4_Bit		6
#define C_INT_EXT2_Bit		3		
#define C_INT_TMR4_Bit		2				
											
	
	
//;------------------------------------------------------------	
//; T0MD (xxH)		--------- 				//; T0MD Register
//; 		[5]T0CS			[7]LCKTM0
//;------------------------------------------------------------
//;		C_TMR0_Clk		C_TMR0_LowClk		//; Timer 0 Clock Source	
//;------------------------------------------------------------
//;		0				x					//; From Instruction Clock
//;		1				0					//; From External Pin
//;		1				1					//; From Low Oscillator Frequency (I_LRC )
//;------------------------------------------------------------
//;Bit[7&5&4] : Timer0 Clock Source Selection
#define	C_TMR0_LowClk		0x80			//; Same as LCKTM0
#define	C_TMR0_Clk			0x20			//; Same as T0CS
#define	C_TMR0_ExtClk_Edge	0x10			//; Timer0 External Clock Edge Select --- 1:Falling Edge, 0:Rising Edge
//;Bit[6] : Reserved
//;Bit[3] : Watchdog Interrupt Source Selection / Timer0 Prescaler0 Selection
#define	C_PS0_WDT			0x08			//; Prescaler0 is assigned to WDT
#define	C_PS0_TMR0			0x00			//; Prescaler0 is assigned to TMR0
//;Bit[2:0] : Prescaler0 Dividing Rate Selection
#define	C_PS0_Div			0x07			//; Prescaler0 Dividing Rate Selection
#define	C_PS0_Div2			0x00
#define	C_PS0_Div4			0x01
#define	C_PS0_Div8			0x02
#define	C_PS0_Div16			0x03
#define	C_PS0_Div32			0x04
#define	C_PS0_Div64			0x05
#define	C_PS0_Div128		0x06
#define	C_PS0_Div256		0x07

//;=======================================================================================================================
//;=======================================================================================================================
//;-----------------------------------------------------------------------------------------------------------------------
//; F-page Special Function Register (IO Configuration Register)
//;-----------------------------------------------------------------------------------------------------------------------	
//;------------------------------------------------------------
//; Pf_PA_Dir_Ctrl (0x05)	--------- PortA Direction(Input/Output) Control Register
//;------------------------------------------------------------
//;Bit[7:0] : Port A Input/Output Mode Selection (1:Input, 0:Output)
#define	C_PA_Input			0xFF			//; Port A Input Mode Control
#define	C_PA_Output			0x00			//; Port A Output Mode Control
                           
#define	C_PA7_Input			0x80			//; PA7 I/O mode Control bit
#define	C_PA6_Input			0x40			//; PA6 I/O mode Control bit
#define	C_PA5_Input			0x20			//; PA5 I/O mode Control bit
#define	C_PA4_Input			0x10			//; PA4 I/O mode Control bit 
#define	C_PA3_Input			0x08			//; PA3 I/O mode Control bit
#define	C_PA2_Input			0x04			//; PA2 I/O mode Control bit
#define	C_PA1_Input			0x02			//; PA1 I/O mode Control bit
#define	C_PA0_Input			0x01			//; PA0 I/O mode Control bit 
#define	C_PA7_Output		0x00			//; PA7 I/O mode Control bit
#define	C_PA6_Output		0x00			//; PA6 I/O mode Control bit
#define	C_PA5_Output		0x00			//; PA5 I/O mode Control bit
#define	C_PA4_Output		0x00			//; PA4 I/O mode Control bit 	
#define	C_PA3_Output		0x00			//; PA3 I/O mode Control bit
#define	C_PA2_Output		0x00			//; PA2 I/O mode Control bit
#define	C_PA1_Output		0x00			//; PA1 I/O mode Control bit
#define	C_PA0_Output		0x00			//; PA0 I/O mode Control bit 
//;------------------------------------------------------------	
//; Pf_PB_Dir_Ctrl (0x06)	--------- PortB Direction(Input/Output) Control Register
//;------------------------------------------------------------
//;Bit[7:6] : Reserved
//;Bit[5:0] : Port B Input/Output Mode Selection (1:Input, 0:Output)
#define	C_PB_Input			0XFF			//; Port B Input Mode Control
#define	C_PB_Output			0x00			//; Port B Output Mode Control

#define	C_PB7_Input			0X80			//; PB7 I/O mode Control bit
#define	C_PB6_Input			0X40			//; PB6 I/O mode Control bit	
#define	C_PB5_Input			0x20			//; PB5 I/O mode Control bit
#define	C_PB4_Input			0x10			//; PB4 I/O mode Control bit
#define	C_PB3_Input			0x08			//; PB3 I/O mode Control bit
#define	C_PB2_Input			0x04			//; PB2 I/O mode Control bit
#define	C_PB1_Input			0x02			//; PB1 I/O mode Control bit
#define	C_PB0_Input			0x01			//; PB0 I/O mode Control bit
#define	C_PB7_Output		0X00			//; PB7 I/O mode Control bit
#define	C_PB6_Output		0X00			//; PB6 I/O mode Control bit	
#define	C_PB5_Output		0x00			//; PB5 I/O mode Control bit
#define	C_PB4_Output		0x00			//; PB4 I/O mode Control bit
#define	C_PB3_Output		0x00			//; PB3 I/O mode Control bit
#define	C_PB2_Output		0x00			//; PB2 I/O mode Control bit
#define	C_PB1_Output		0x00			//; PB1 I/O mode Control bit
#define	C_PB0_Output		0x00			//; PB0 I/O mode Control bit
//;------------------------------------------------------------	
//; Pf_PC_Dir_Ctrl (0x07)	--------- PortC Direction(Input/Output) Control Register
//;------------------------------------------------------------
//;Bit[7:2] : Reserved
//;Bit[1:0] : Port C Input/Output Mode Selection (1:Input, 0:Output)
#define	C_PC_Input			0x03			//; Port C Input Mode Control
#define	C_PC_Output			0x00			//; Port C Output Mode Control

#define	C_PC1_Input			0x02			//; PC1 I/O mode Control bit
#define	C_PC0_Input			0x01			//; PC0 I/O mode Control bit
#define	C_PC1_Output		0x00			//; PC1 I/O mode Control bit
#define	C_PC0_Output		0x00			//; PC0 I/O mode Control bit
//;------------------------------------------------------------	
//; Pr_PA_PH_Ctrl (0x09)	--------- PortA Pull-High Control Register
//;------------------------------------------------------------
//;Bit[7:0] : PortA Pull-High Control Register (1:Disable, 0:Pull-High)
#define	C_PA_PHB			0xFF			//; PortA Pull-High Control bit 
#define	C_PA7_PHB			0x80			//; PA7 Pull-High Control bit 
#define	C_PA6_PHB			0x40			//; PA6 Pull-High Control bit   
#define	C_PA5_PLB			0x20			//; PA5 Pull-Low  Control bit ***********************
#define	C_PA4_PHB			0x10			//; PA4 Pull-High Control bit
#define	C_PA3_PHB			0x08			//; PA3 Pull-High Control bit	
#define	C_PA2_PHB			0x04			//; PA2 Pull-High Control bit
#define	C_PA1_PHB			0x02			//; PA1 Pull-High Control bit
#define	C_PA0_PHB			0x01			//; PA0 Pull-High Control bit
		
//;------------------------------------------------------------	
//; Pf_PS0_CNT (0x0A)		--------- Prescaler0 Counter Value Register
//;------------------------------------------------------------
//;Bit[7:0] : Prescaler0 Counter Value
#define	C_PS0_Cnt			0xFF			//; Read-only	

//;------------------------------------------------------------	
//; Pr_PC_PL_Ctrl (0x0B)	--------- PortC Pull-Low Control Register
//;------------------------------------------------------------
//;Bit[7:2] : Reserved
//;Bit[1:0] : PortC Pull-Low Control Register (1:Disable, 0:Pull-Low)
#define C_PC_PLB			0x03			//; PortC Pull-Low Control bit 
#define C_PC1_PLB			0x02			//; PC1 Pull-Low Control bit 
#define C_PC0_PLB			0x01			//; PC0 Pull-Low Control bit  
	
//;------------------------------------------------------------	
//; Pf_PB_OD_Ctrl (0x0C)	--------- PortB Open-Drain Control Register
//;------------------------------------------------------------
//;Bit[7:0] : PortB Open-Drain Control (1:Open-Drain, 0:Disable)
#define C_PB_OD				0xFF			//; Port B Open-Drain Control
#define C_PB7_OD			0x80			//; PB7 Open-Drain Control bit	
#define C_PB6_OD			0x40			//; PB6 Open-Drain Control bit		
#define C_PB5_OD			0x20			//; PB5 Open-Drain Control bit	
#define C_PB4_OD			0x10			//; PB4 Open-Drain Control bit	
#define C_PB3_OD			0x08			//; PB3 Open-Drain Control bit	
#define C_PB2_OD			0x04			//; PB2 Open-Drain Control bit
#define C_PB1_OD			0x02			//; PB1 Open-Drain Control bit 
#define C_PB0_OD			0x01			//; PB0 Open-Drain Control bit
	
//;------------------------------------------------------------	
//; Pf_PC_OD_Ctrl (0x0D)	--------- PortC Open-Drain Control Register
//;------------------------------------------------------------
//;Bit[7:2] : Reserved
//;Bit[1:0] : PortC Open-Drain Control (1:Open-Drain, 0:Disable)
#define C_PC_OD				0x03			//; Port C Open-Drain Control
#define C_PC1_OD			0x02			//; PC1 Open-Drain Control bit 
#define C_PC0_OD			0x01			//; PC0 Open-Drain Control bit
//;------------------------------------------------------------	
//; CMPCR (0x0E)	--------- Comparator voltage select Control Register
//;------------------------------------------------------------
//;Bit[7] : Reserved

//;Bit[6] : VDD_ref bias high enable bit  (1:Enable, 0: Disable)
#define C_RBias_High_Dis 	0x00
#define C_RBias_High_En 	0x40
//;LVDS[3:0]		VDD_ref
//;------------------------------
//;0000			0.51 * VDD  (about)
//;0001			0.48 * VDD	(about)		
//;0010			0.44 * VDD	(about)	
//;0011			0.40 * VDD	(about)	
//;0100			0.37 * VDD	(about)		
//;0101			0.34 * VDD	(about)		
//;0110			0.33 * VDD	(about)		
//;0111			0.32 * VDD	(about)		
//;1000			0.31 * VDD	(about)		
//;1001			0.29 * VDD	(about)		
//;1010			0.28 * VDD	(about)		
//;1011			0.27 * VDD	(about)		
//;1100			0.26 * VDD	(about)		
//;1101			0.25 * VDD	(about)		
//;1110			0.24 * VDD	(about)		
//;1111			0.23 * VDD	(about)

//;Bit[5] : VDD_ref bias low enable bit  (1:Enable, 0: Disable) 
#define C_RBias_Low_Dis 	0x00
#define C_RBias_Low_En 		0x20
//;LVDS[3:0]		VDD_ref
//;------------------------------
//;0000			0.24 * VDD	(about)	
//;0001			0.23 * VDD	(about)		
//;0010			0.20 * VDD	(about)	
//;0011			0.17 * VDD	(about)		
//;0100			0.15 * VDD	(about)		
//;0101			0.13 * VDD	(about)		
//;0110			0.124 * VDD	(about)		
//;0111			0.116 * VDD	(about)		
//;1000			0.106 * VDD	(about)		
//;1001			0.096 * VDD	(about)		
//;1010			0.088 * VDD	(about)		
//;1011			0.080 * VDD	(about)		
//;1100			0.072 * VDD	(about)		
//;1101			0.065 * VDD	(about)		
//;1110			0.059 * VDD	(about)		
//;1111			0.055 * VDD	(about)

//;Bit[4] : Comparator Interrupt  trigger edge control(Comparator output state need polling SFR bit "LVDOUT")
#define C_CMPFINV_Dis		0x00			//; LVDIF=1 when Vp > Vn
#define C_CMPFINV_En		0x10			//; LVDIF=1 when Vp < Vn	
	
//;Bit[3:2] :	Non-inverting input	pad select
//;Bit[1:0] :	Inverting input	pad	select

//;Comparator input select
//;Bit[3:0]	Non-inverting input		Inverting input
//;-------------------------------------------
//;0000		PB0						PB2
//;0001		PB0						PB4
//;0010		PB0						Bandgap
//;0011		PB0						VDD_ref
//;0100		PB5						PB2
//;0101		PB5						PB4
//;0110		PB5						Bandgap
//;0111		PB5						VDD_ref
//;1000		VDD_ref					PB2
//;1001		VDD_ref					PB4
//;1010		VDD_ref					Bandgap				(only for LVD)
//;1011		NA						NA
//;11XX		NA						NA

//; Bandgap voltage is about 0.6V	
//;------------------------------------------------------------	
//; PCON1 (0x0F)	--------- Power Control Register 1
//;------------------------------------------------------------
//;Bit[7] : All Interrupt Enable
#define	C_All_INT_En		0x80			//; Enable all unmasked interrupts
//;Bit[6] : Low voltage detector output
#define	C_LVDOUT			0x40			//; read-only
#define	C_CMPOUT			0x40			//; read-only
//;Bit[5:2] : Select one of the 16 LVD voltage
#define	C_LVD_4P15V			0x3C			//; LVD Voltage=4.15V
#define	C_LVD_4P05V			0x38			//; LVD Voltage=4.05V	
#define	C_LVD_3P9V			0x34			//; LVD Voltage=3.9V	
#define	C_LVD_3P75V			0x30			//; LVD Voltage=3.75V	
#define	C_LVD_3P6V			0x2C			//; LVD Voltage=3.6V
#define	C_LVD_3P45V			0x28			//; LVD Voltage=3.45V	
#define	C_LVD_3P3V			0x24			//; LVD Voltage=3.3V	
#define	C_LVD_3P15V			0x20			//; LVD Voltage=3.15V	
#define	C_LVD_3P0V			0x1C			//; LVD Voltage=3.0V	
#define	C_LVD_2P9V			0x18			//; LVD Voltage=2.9V	
#define	C_LVD_2P8V			0x14			//; LVD Voltage=2.8V
#define	C_LVD_2P6V			0x10			//; LVD Voltage=2.6V	
#define	C_LVD_2P4V			0x0C			//; LVD Voltage=2.4V
#define	C_LVD_2P2V			0x08			//; LVD Voltage=2.2V
#define	C_LVD_2P0V			0x04			//; LVD Voltage=2.0V
#define	C_LVD_1P9V			0x00			//; LVD Voltage=1.9V	
//;Bit[0] : Timer0 Enable
#define	C_TMR0_En			0x01			//; Enable Timer0
#define	C_TMR0_Dis			0x00			//; Disable Timer0

#define	C_LVDOUT_Bit		0x6
#define	C_CMPOUT_Bit		0x6
//;=======================================================================================================================
//;=======================================================================================================================
//;-----------------------------------------------------------------------------------------------------------------------
//; S-page Special Function Register (Special Function Register)
//;-----------------------------------------------------------------------------------------------------------------------		
//;------------------------------------------------------------
//; TMR1 (0x00)	--------- Timer1 Data Register
//;------------------------------------------------------------
//;Bit[7:0] : Timer1 Data
#define	C_TMR1_Data			0xFF	
	
//;------------------------------------------------------------	
//; T1CR1 (0x01)	--------- Timer1 Control Register 1
//;------------------------------------------------------------
//;Bit[7:6] : PWM1 Control
#define	C_PWM1_En			0x80 			//; PWM1 output will be present on PB1/PB4
#define	C_PWM1_Active		0x40 			//; PWM1 output is active
#define	C_PWM1_Active_Lo	0x40 			//; PWM1 output is active low 
#define	C_PWM1_Active_Hi	0x00 			//; PWM1 output is active high 
//;Bit[5] : Timer1 Match Output
#define	C_TMR1_Output		0x20			//; Enable T1OUT output to pad PB4
//;Bit[4] : Timer1 Special Clock Source Selection
#define	C_VFSEL1			0x10			//; PWM1, 2, 3 & Timer 1 clock source is Variable High Oscillator clock
//;Bit[3] : Timer1 clock source selection
#define	C_TMR1_IHRC_En		0x08			//; PWM1, 2, 3 & Timer 1 clock source is High Oscillator clock
#define	C_TMR1_IHRC_Dis		0x00			//; PWM1, 2, 3 & Timer 1 clock source selection depends on T1CS register bit
//;Bit[2:0] : Timer1 Control
#define	C_TMR1_OneShot		0x04 			//; One-Shot mode. Timer1 will count once from the initial value to 0x00
#define	C_TMR1_Reload		0x02 			//; Initial value is reloaded from reload register TMR1(T1OS=0)
#define	C_TMR1_En			0x01 			//; Enable Timer1
#define	C_TMR1_Dis			0x00 			//; Disable Timer1

//;------------------------------------------------------------	
//; T1CR2 (0x02)	--------- Timer1 Control Register	2
//;------------------------------------------------------------
//;Bit[7:6] : Reserved
//;Bit[5] : Timer1 Clock Source Select
#define	C_TMR1_ClkSrc_Ext	0x20			//; Timer1 clock source from External Clock Input
#define	C_TMR1_ClkSrc_Inst	0x00			//; Timer1 clock source from Internal Instruction Clock

	
//;Bit[4] : Timer1 external clock edge selection
#define	C_TMR1_ExtClk_Edge	0x10			//; Timer1 External Clock Edge Select --- 1:Falling Edge, 0:Rising Edge  
#define	C_TMR1_ExtFalling	0x10			//; Timer1 will decrease one while EX_CKI Falling Edge.
#define	C_TMR1_ExtRising	0x00			//; Timer1 will decrease one while EX_CKI Rising Edge.
//;Bit[3] :  Disable/enable Prescaler1
#define	C_PS1_Dis           0x08			//; Enable Prescaler1
#define	C_PS1_EN			0x00			//; Disable Prescaler1
//;Bit[2:0] : Prescaler 1 Dividing Rate Selection
#define C_PS1_Div			0x07			//; Prescaler1 Dividing Rate Selection
#define C_PS1_Div2			0x00
#define C_PS1_Div4			0x01
#define C_PS1_Div8			0x02
#define C_PS1_Div16			0x03
#define C_PS1_Div32			0x04
#define C_PS1_Div64			0x05
#define C_PS1_Div128		0x06
#define C_PS1_Div256		0x07

//;------------------------------------------------------------	
//; PWM1DUTY (0x03)	--------- PWM1 Duty Register
//;------------------------------------------------------------
//;Bit[7:0] : PWM1 Duty Value
#define	C_PWM1_Duty			0xFF

//;------------------------------------------------------------	
//; PS1CV (0x04)		--------- Prescaler1 Counter Value Register
//;------------------------------------------------------------
//;Bit[7:0] : rescaler1 Counter Value
#define C_PS1_Cnt			0xFF			//; 8-Bit register	

//;------------------------------------------------------------	
//; BZ1CR (0x05)		--------- Buzzer1 Control Register
//;------------------------------------------------------------
//;Bit[7] : BZ1 Enable
#define C_BZ1_En			0x80			//; Enable BZ1 output
#define C_BZ1_Dis			0x00			//; Disable BZ1 output

//;Bit[6:4] : Reserved
//;Bit[3:0] : BZ1 Frequency Selection
#define	C_BZ1_FSel			0x0F			//; BZ1 frequency selection

#define	C_BZ1_PS1Div2		0x00			//; Clock Soruce from Prescaler 1
#define	C_BZ1_PS1Div4		0x01
#define	C_BZ1_PS1Div8		0x02
#define	C_BZ1_PS1Div16		0x03
#define	C_BZ1_PS1Div32		0x04
#define	C_BZ1_PS1Div64		0x05
#define	C_BZ1_PS1Div128		0x06
#define	C_BZ1_PS1Div256		0x07

#define	C_BZ1_TMR1B0		0x08			//; Clock Source from Timer 1
#define	C_BZ1_TMR1B1		0x09
#define	C_BZ1_TMR1B2		0x0A
#define	C_BZ1_TMR1B3		0x0B
#define	C_BZ1_TMR1B4		0x0C
#define	C_BZ1_TMR1B5		0x0D
#define	C_BZ1_TMR1B6		0x0E
#define	C_BZ1_TMR1B7		0x0F

//;------------------------------------------------------------	
//; IRCR (0x06)		--------- IR Control Register
//;------------------------------------------------------------
//;Bit[7] : IR Clock Source Selection (1:3.58MHz, 0:455KHz)
#define	C_IR_ClkSrc_358M	0x80			//; IRCR[7]=1 , external crystal is 3.58MHz (This bit is ignored if internal high frequency oscillation is used)
#define	C_IR_ClkSrc_455K	0x00			//; IRCR[7]=0 , external crystal is 455KHz
//;Bit[6:3] : Reserved
//;Bit[2] : IR Polarity Selection
#define	C_IR_Pol_Sel		0x04
//;Bit[1] : IR Carrier Frequency Selection (1:5.7K, 0:38KHz)
#define	C_IR_57K			0x02			//; IRCR[1]=1 , IR carrier frequency is 57KHz
#define	C_IR_38K			0x00			//; IRCR[1]=0 , IR carrier frequency is 57KHz
//;Bit[0] : IR Enable
#define	C_IR_En				0x01			//; Enable IR carrier output
#define	C_IR_Dis			0x00			//; Disable IR carrier output

//;------------------------------------------------------------	
//; TBHP (0x07)	--------- Table Access High Byte Address Pointer Register
//;------------------------------------------------------------
//;Bit[7:2] : Reserved
//;Bit[1:0] : Table Access High Byte Address Pointer
#define	C_TbHigh_Addr		0x0F			//; When instruction CALLA, lgotoA or TABLEA is executed TBHP[1:0] is PC[11:8]

//;------------------------------------------------------------	
//; TBHD (0x08)	--------- Table Access High Byte Data Register(14-bit ROM Data, H-byte 6bits+ acc 8bits) 
//;------------------------------------------------------------
//;Bit[7:6] : Reserved
//;Bit[5:0] : Table Access High Byte Data
#define	C_TbHigh_Data		0x3F
	
//;------------------------------------------------------------	
//; P2CR1 (0x0A)	--------- PWM2 Control Register 1
//;------------------------------------------------------------
//;Bit[7:6] : PWM2 Control
#define	C_PWM2_En			0x80 			//; PWM2 output will be present on PB3/PB5
#define	C_PWM2_Active		0x40 			//; PWM2 output is active
#define	C_PWM2_Active_Lo	0x40 			//; PWM2 output is active low 
#define	C_PWM2_Active_Hi	0x00 			//; PWM2 output is active high 
//;Bit[5:0] : Reserved

//;------------------------------------------------------------	
//; PWM2DUTY (0x0C)	--------- PWM2 Duty Register
//;------------------------------------------------------------
//;Bit[7:0] : PWM2 Duty Value
#define	C_PWM2_Duty			0xFF
	
//;------------------------------------------------------------   
//; OSCCR (0x0F)	--------- Table Access High Byte Data Register
//;------------------------------------------------------------
//;Bit[7] : Comparator output status
//;#define C_CMPOUT			0X80			//; Read-only
//;Bit[6] : Compare Output Enable to PB3
#define C_CMPOPB3_Dis		0x00			//; Disable comparator output to pad PB3
#define C_CMPOPB3_En		0x40			//; Enable comparator output to pad PB3
//;Bit[5] : Comparator output change state interrupt flag
#define	C_INF_CMP			0x00			//; clear comparator interrupt flag
#define	C_CMP_INF			0x20			//; comparator interrupt flag		
//;Bit[4] : Enable/Disable of comparator interrupt
//;#define	C_INT_CMP			0x10			//; Enable comparator interrupt
//;Bit[3:2] : System Mode Select
#define	C_Mode				0x0C			//; System Operating Mode Selection
#define	C_Normal_Mode		0x00			//; Enter Normal mode
#define	C_Halt_Mode			0x04			//; Enter Halt mode
#define	C_Standby_Mode		0x08			//; Enter Standby mode
//;Bit[1] : Stop FHOSC
#define	C_FHOSC_Stop		0x02			//; Disable high-frequency oscillation (FHOSC)
//;Bit[0] : FOSC Seletction
#define	C_FHOSC_Sel			0x01			//; OSCCR[0]=1 , FOSC is high-frequency oscillation (FHOSC)
#define	C_FLOSC_Sel			0x00			//; OSCCR[0]=0 , FOSC is Low-frequency oscillation (FLOSC)	


#define	C_CMPOPB3_En_bit	6
#define	C_INTF_CMP_bit		5
#define	C_FHOSC_Stop_bit	1
#define	C_FHOSC_Sel_bit		0
//;------------------------------------------------------------	
//; P3CR1 (0x11)	--------- PWM3 Control Register 1
//;------------------------------------------------------------
//;Bit[7:6] : PWM3 Control
#define	C_PWM3_En			0x80 			//; PWM3 output will be present on PB1
#define	C_PWM3_Active		0x40 			//; PWM3 output is active
#define	C_PWM3_Active_Lo	0x40 			//; PWM3 output is active low 
#define	C_PWM3_Active_Hi	0x00 			//; PWM3 output is active high 
//;Bit[5:0] : Reserved

//;------------------------------------------------------------	
//; PWM3DUTY (0x13)	--------- PWM3 Duty Register
//;------------------------------------------------------------
//;Bit[7:0] : PWM3 Duty Value
#define	C_PWM3_Duty			0xFF	
//;------------------------------------------------------------	
//; Ps_PS3_Cnt (0x14)		--------- Prescaler3 Counter Value Register
//;------------------------------------------------------------
//;------------------------------------------------------------
//; TMR4 (0x15)	--------- Timer4 Data Register
//;------------------------------------------------------------
//;Bit[7:0] : Timer4 Data
#define C_TMR4_Data			0xFF	
//;------------------------------------------------------------	
//; T4CR1 (0x16)	--------- Timer4 Control Register 1
//;------------------------------------------------------------
//;Bit[7:6] : PWM4 Control
#define	C_PWM4_En			0x80 			//; PWM4 output will be present on PA4
#define	C_PWM4_Active		0x40 			//; PWM4 output is active
#define	C_PWM4_Active_Lo	0x40 			//; PWM4 output is active low 
#define	C_PWM4_Active_Hi	0x00 			//; PWM4 output is active high 
//;Bit[5] : Reserved
//;Bit[4] : Timer4 Special Clock Source Selection
#define	C_VFSEL4			0x10			//; PWM4 & Timer 4 clock source is Variable High Oscillator clock
//;Bit[3] : Timer4 clock source selection
#define	C_TMR4_IHRC_En		0x08			//; PWM4 & Timer 4 clock source is High Oscillator clock
#define	C_TMR4_IHRC_Dis		0x00			//; PWM4& Timer 4 clock source selection depends on T1CS register bit
//;Bit[2:0] : Timer4 Control
#define	C_TMR4_OneShot		0x04 			//; One-Shot mode. Timer4 will count once from the initial value to 0x00
#define	C_TMR4_Reload		0x02 			//; Initial value is reloaded from reload register TMR4(T1OS=0)

#define	C_TMR4_En			0x01 			//; Enable Timer4
#define	C_TMR4_Dis			0x00 			//; Disable Timer4

//;------------------------------------------------------------	
//; T4CR2 (0x17)	--------- Timer4 Control Register	2
//;------------------------------------------------------------
//;Bit[7:6] : Reserved
//;Bit[5] : Timer4 Clock Source Select
#define	C_TMR4_ClkSrc_Ext	0x20			//; Timer4 clock source from External Clock Input
#define	C_TMR4_ClkSrc_Inst	0x00			//; Timer4 clock source from Internal Instruction Clock
//;Bit[4] : Timer4 external clock edge selection
#define	C_TMR4_ExtClk_Edge	0x10			//; Timer4 External Clock Edge Select --- 1:Falling Edge, 0:Rising Edge  
#define	C_TMR4_ExtFalling	0x10			//; Timer4 will decrease one while EX_CKI Falling Edge.
#define	C_TMR4_ExtRising	0x00			//; Timer4 will decrease one while EX_CKI Rising Edge.
//;Bit[3] :  Disable/enable Prescaler4
#define	C_PS4_Dis           0x08			//; Enable Prescaler4
#define	C_PS4_EN			0x00			//; Disable Prescaler4
//;Bit[2:0] : Prescaler4 Dividing Rate Selection
#define C_PS4_Div			0x07			//; Prescaler4 Dividing Rate Selection
#define C_PS4_Div2			0x00
#define C_PS4_Div4			0x01
#define C_PS4_Div8			0x02
#define C_PS4_Div16			0x03
#define C_PS4_Div32			0x04
#define C_PS4_Div64			0x05
#define C_PS4_Div128		0x06
#define C_PS4_Div256		0x07
//;------------------------------------------------------------	
//; PWM4DUTY (0x18)	--------- PWM4 Duty Register
//;------------------------------------------------------------
//;Bit[7:0] : PWM4 Duty Value
#define	C_PWM4_Duty			0xFF	
//;------------------------------------------------------------	
//; PS4CV (0x19)		--------- Prescaler4 Counter Value Register
//;------------------------------------------------------------
//;Bit[7:0] : rescaler4 Counter Value
#define C_PS4_Cnt			0xFF			//; 8-Bit register	
//;------------------------------------------------------------
//; TMR5 (0x1A)	--------- Timer5 Data Register
//;------------------------------------------------------------
//;Bit[7:0] : Timer5 Data
#define C_TMR5_Data			0xFF	
//;------------------------------------------------------------	
//; T5CR1 (0x1B)	--------- Timer5 Control Register 1
//;------------------------------------------------------------
//;Bit[7:6] : PWM5 Control
#define	C_PWM5_En			0x80 			//; PWM5 output will be present on PB2
#define	C_PWM5_Active		0x40 			//; PWM5 output is active
#define	C_PWM5_Active_Lo	0x40 			//; PWM5 output is active low 
#define	C_PWM5_Active_Hi	0x00 			//; PWM5 output is active high 
//;Bit[5] : Reserved
//;Bit[4] : Timer5 Special Clock Source Selection
#define	C_VFSEL5			0x10			//; PWM5 & Timer 5 clock source is Variable High Oscillator clock
//;Bit[3] : Timer4 clock source selection
#define	C_TMR5_IHRC_En		0x08			//; PWM5 & Timer 5 clock source is High Oscillator clock
#define	C_TMR5_IHRC_Dis		0x00			//; PWM5& Timer 5 clock source selection depends on T1CS register bit
//;Bit[2:0] : Timer5 Control
#define	C_TMR5_OneShot		0x04 			//; One-Shot mode. Timer5 will count once from the initial value to 0x00
#define	C_TMR5_Reload		0x02 			//; Initial value is reloaded from reload register TMR5(T1OS=0)

#define	C_TMR5_En			0x01 			//; Enable Timer5
#define	C_TMR5_Dis			0x00 			//; Disable Timer5

//;------------------------------------------------------------	
//; T5CR2 (0x1C)	--------- Timer5 Control Register	2
//;------------------------------------------------------------
//;Bit[7:6] : Reserved
//;Bit[5] : Timer5 Clock Source Select
#define	C_TMR5_ClkSrc_Ext	0x20			//; Timer5 clock source from External Clock Input
#define	C_TMR5_ClkSrc_Inst	0x00			//; Timer5 clock source from Internal Instruction Clock
//;Bit[4] : Timer5 external clock edge selection
#define	C_TMR5_ExtClk_Edge	0x10			//; Timer5 External Clock Edge Select --- 1:Falling Edge, 0:Rising Edge  
#define	C_TMR5_ExtFalling	0x10			//; Timer5 will decrease one while EX_CKI Falling Edge.
#define	C_TMR5_ExtRising	0x00			//; Timer5 will decrease one while EX_CKI Rising Edge.
//;Bit[3] :  Disable/enable Prescaler5
#define	C_PS5_Dis           0x08			//; Enable Prescaler5
#define	C_PS5_EN			0x00			//; Disable Prescaler5
//;Bit[2:0] : Prescaler5 Dividing Rate Selection
#define C_PS5_Div			0x07			//; Prescaler5 Dividing Rate Selection
#define C_PS5_Div2			0x00
#define C_PS5_Div4			0x01
#define C_PS5_Div8			0x02
#define C_PS5_Div16			0x03
#define C_PS5_Div32			0x04
#define C_PS5_Div64			0x05
#define C_PS5_Div128		0x06
#define C_PS5_Div256		0x07
//;------------------------------------------------------------	
//; PWM5DUTY (0x1D)	--------- PWM5 Duty Register
//;------------------------------------------------------------
//;Bit[7:0] : PWM5 Duty Value
#define	C_PWM5_Duty			0xFF	
//;------------------------------------------------------------	
//; PS5CV (0x1E)		--------- Prescaler5 Counter Value Register
//;------------------------------------------------------------
//;Bit[7:0] : rescaler5 Counter Value
#define C_PS5_Cnt			0xFF			//; 8-Bit register	

//;------------------------------------------------------------	
//; TMR5RH (0x1F) 	--------- TIMER5 & PWMDUTY5 msb 2 bits Register
//;------------------------------------------------------------		
//;Bit[7] : Timer5 Data Bit9	
#define	C_TMR5_Data_b9		0X20			//; Timer5 data bit9 
//;Bit[6] : Timer5 Data Bit8	
#define	C_TMR5_Data_b8		0x10			//; Timer5 data bit8				
//;Bit[1] : PWM5 Duty Bit9	
#define C_PWM5_Duty_b9		0x02			//; PWM5 Duty bit9 
//;Bit[0] : PWM5 Duty Bit8	
#define C_PWM5_Duty_b8		0x01			//; PWM5 Duty bit8	


#define C_TMR5_Data_b9_Bit	5
#define C_TMR5_Data_b8_Bit	4	
#define C_PWM5_Duty_b9_Bit	1
#define C_PWM5_Duty_b8_Bit	0
//;=======================================================================================================================
//;=======================================================================================================================
//;-----------------------------------------------------------------------------------------------------------------------
//; T-page Special Function Register (Touch Function Register)
//;-----------------------------------------------------------------------------------------------------------------------		
//;------------------------------------------------------------
//; SIMCR (0x00)	--------- Serial interface mode control register
//;------------------------------------------------------------
//;Bit[7] : Enable/disable SPI function
#define C_SPI_En			0x80			//; Enable SPI function
#define C_SPI_Dis			0x00			//; Disable SPI function	
//;Bit[6] : Enable/disable IIC function
#define C_IIC_En			0x40			//; Enable IIC function
#define C_IIC_Dis			0x00			//; Disable IIC function
//;Bit[5] : Select master / slave mode (include SPI / IIC)
#define	C_Master_En			0x20			//; Select master mode
#define	C_Slave_En			0x00			//; Select slave mode
//;Bit[4] : Enable / disable SSB pin (used at SPI mode)
#define	C_SSB_En			0x10			//; Enable SSB (PB5)
#define	C_SSB_Dis			0x00			//; Disable SSB
//;Bit[3] : Enable / disable UART RX PAD
#define	C_RX_En				0x08			//; Enable UART RX (PB7)
#define	C_RX_Dis			0x00			//; Disable UART RX, PB7 is GPIO
//;Bit[2] : Enable / disable UART TX PAD
#define	C_TX_En				0x04			//; Enable UART TX (PB6)
#define	C_TX_Dis			0x00			//; Disable UART TX, PB6 is GPIO
//;Bit[1] : Enable / disable UART RCLK
#define	C_RCLK_En			0x02			//; Enable UART RCLK (PB5)
#define	C_RCLK_Dis			0x00			//; Disable UART RCLK, PB5 is GPIO
//;Bit[0] : Enable / disable UART BAUDOZ
#define	C_BAUDOZ_En			0x01			//; Enable UART BAUDOZ (PB4)
#define	C_BAUDOZ_Dis		0x00			//; Disable UART BAUDOZ, PB4 is GPIO

#define C_SPI_En_Bit		7
#define C_IIC_En_Bit		6
#define C_Master_En_Bit		5
#define C_SSB_En_Bit		4
#define C_UART_RX_En_Bit	3
#define C_UART_TX_En_Bit	2
#define C_RCLK_En_Bit		1
#define C_BAUDOZ_En_Bit		0
//;------------------------------------------------------------
//; MADR (0x01)	--------- IIC mode address register
//;------------------------------------------------------------
//;Bit[7:1] : IIC mode address register
#define	C_MADR				0xFE			//; IIC mode address register
//;Bit[0] : Reserved

#define	C_MADR7_Bit			7
#define	C_MADR6_Bit			6
#define	C_MADR5_Bit			5
#define	C_MADR4_Bit			4
#define	C_MADR3_Bit			3
#define	C_MADR2_Bit			2
#define	C_MADR1_Bit			1
//;------------------------------------------------------------
//; MFDR (0x02)	--------- IIC mode frequency register
//;------------------------------------------------------------
//;Bit[7:5] : Reserved
//;Bit[4:0] : IIC Mode Frequency Dividing Rate Selection
#define C_FD_Div			0x1F			//; IIC Mode Frequency Dividing Rate Selection
#define C_FD_Div22			0x00
#define C_FD_Div24			0x01
#define C_FD_Div28			0x02
#define C_FD_Div34			0x03
#define C_FD_Div44			0x04
#define C_FD_Div48			0x05
#define C_FD_Div56			0x06
#define C_FD_Div68			0x07
#define C_FD_Div88			0x08
#define C_FD_Div96			0x09
#define C_FD_Div112			0x0A
#define C_FD_Div136			0x0B
#define C_FD_Div176			0x0C
#define C_FD_Div192			0x0D
#define C_FD_Div224			0x0E
#define C_FD_Div272			0x0F
#define C_FD_Div352			0x10
#define C_FD_Div384			0x11
#define C_FD_Div448			0x12
#define C_FD_Div544			0x13
#define C_FD_Div704			0x14
#define C_FD_Div768			0x15
#define C_FD_Div896			0x16
#define C_FD_Div1088		0x17
#define C_FD_Div1408		0x18
#define C_FD_Div1536		0x19
#define C_FD_Div1792		0x1A
#define C_FD_Div2176		0x1B
#define C_FD_Div2816		0x1C
#define C_FD_Div3072		0x1D
#define C_FD_Div3584		0x1E
#define C_FD_Div4352		0x1F
//;------------------------------------------------------------
//; MCR (0x03)	--------- IIC mode control register
//;------------------------------------------------------------
//;Bit[7:5] : Reserved
//;Bit[4] : IIC mode transmit/receive mode Select
#define	C_IIC_TX			0x10			//; IIC mode transmit mode
#define	C_IIC_RX			0x00			//; IIC mode receive mode
//;Bit[3] : IIC mode acknowledge enable
#define	C_NACK				0x08			//; Do not send acknowledge signal
#define	C_ACK				0x00			//; Send acknowledge at 9th lock bit
//;Bit[2:0] : Reserved

#define	C_MTX_Bit			4
#define	C_TXAK_Bit			3
//;------------------------------------------------------------
//; MCR (0x04)	--------- IIC mode status register
//;------------------------------------------------------------
//;Bit[7] : IIC Data Transfer Complete
#define	C_MCF				0x80		//; (1:A byte has been completed , 0:A byte is being transfer) Read-only
//;Bit[6] : IIC Address Check as Slave mode 
#define	C_MAAS				0x40		//; (1:Currently addressed , 0:not currently addressed) Read-only
//;Bit[5] : IIC Bus Busy 
#define	C_MBB				0x20		//; (1:bus busy, 0:bus idle) Read-only
//;Bit[4] : IIC Arbitration Lost in Master Mode
#define	C_MAL				0x10		//; (1:Lost arbitration, 0:No arbitration lost)
//;Bit[3] : Reserved
//;Bit[2] : IIC Read / Write selection as Slave mode
#define	C_SRW				0x04		//; (1:Read from slave, from calling master, 0:Write to slave from calling master) Read-only
//;Bit[1] : IIC Interrupt Flag
#define	C_MIF				0x02		//; (1:IIC interrupt has occurred, 0:IIC interrupt has not occurred)
//;Bit[0] : Receive Acknowledge
#define	C_RXAK				0x01		//; (1:NACK, 0:ACK) Read-only

#define C_MAL_Bit			4
#define C_MIF_Bit			1
//;------------------------------------------------------------
//; SIMDR (0x05)	--------- Serial interface mode data register
//;------------------------------------------------------------
//;Bit[7:0] : Serial interface mode data register
#define	C_SIMDR				0xFF
//;------------------------------------------------------------
//; SPCR (0x06)	--------- SPI control & status register
//;------------------------------------------------------------
//;Bit[7] : SPI Data Transfer Complete
#define	C_SPIF				0x80		//; (1:Transmission complete , 0:Transmission not complete)
//;Bit[6] : Write Collision
#define	C_WCOL				0x40		//; (1:Invalid write to SIMDR , 0:No invalid write to SIMDR)
//;Bit[4] : Mode Fault
#define	C_MODF				0x10		//; (1:SSB pulled low while master mode , 0:SSB not pulled low while master mode)
//;Bit[3] : Clock Polarity
#define	C_SCK_High			0x08		//; SCK pin at logic 1 between transmissions, idle high
#define	C_SCK_Low			0x00		//; SCK pin at logic 0 between transmissions, idle low
//;Bit[2] : SPI SCK Active Edge Type Selection
#define	C_SCK_Rising		0x04		//; CPOL=1: Capture at SCK falling edge, CPOL=0: Capture at SCK rising edge
#define	C_SCK_Falling		0x00		//; CPOL=1: Capture at SCK rising edge, CPOL=0: Capture at SCK falling edge
//;Bit[1:0] : SPI Clock Rate
#define C_SPI_CLK_DIV32		0x03		//; System clock/32
#define C_SPI_CLK_DIV16		0x02		//; System clock/16
#define C_SPI_CLK_DIV4		0x01		//; System clock/4
#define C_SPI_CLK_DIV2		0x00		//; System clock/2

#define	C_SPIF_Bit			7
#define	C_WCOL_Bit			6
#define	C_MODF_Bit			4
//;------------------------------------------------------------
//; INTE3 (0x07)		--------- Interrupt 3th Enable Register
//; INTF3 (0x08)		--------- Interrupt 3th Flag
//;------------------------------------------------------------
//;Bit[7] : Serial interface mode interrupt enable bit
#define	C_INT_SIM			0x80			//; Serial interface mode interrupt enable bit
//;Bit[6] : End of EEPROM Writing interrupt enable bit
#define	C_INT_EE			0x40			//; End of EEPROM Writing interrupt enable bit
//;Bit[5] : Timer5 interrupt enable bit
#define	C_INT_TMR5			0x20			//; Timer5 interrupt enable bit
//;Bit[5] : CCP interrupt enable bit
#define	C_INT_CCP			0x20			//; CCP interrupt enable bit
//;Bit[4] : LSR interrupt enable bit
#define	C_INT_LSR			0x10			//; LSR interrupt enable bit
//;Bit[3] : Transmitt holding register (THR) empty interrupt enable bit
#define	C_INT_TX			0x08			//; Transmitt holding register (THR) empty interrupt enable bit
//;Bit[2] : Receive one bye completely interrupt enable bit
#define	C_INT_RX			0x04			//; Receive one bye completely interrupt enable bit
//;Bit[1] : Touch pad counter overflow interrupt enable bit
#define	C_INT_TPOVF			0x02			//; Touch pad counter overflow interrupt enable bit
//;Bit[0] : Touch pad compare completely interrupt enable bit
#define	C_INT_TPCMP			0x01			//; Touch pad compare completely interrupt enable bit

#define	C_INT_SIM_Bit		7
#define	C_INT_EE_Bit		6
#define	C_INT_TMR5_Bit		5
#define	C_INT_CCP_Bit		5
#define	C_INT_LSR_Bit		4
#define	C_INT_TX_Bit		3
#define	C_INT_RX_Bit		2		
#define	C_INT_TPOVF_Bit		1
#define	C_INT_TPCMP_Bit		0
//;------------------------------------------------------------
//; TPCKS (0x09)	--------- Touch pad clock register
//;------------------------------------------------------------
//;Bit[4] : Touch slow mode wakeup period select
#define	C_WKUPT64ms			0x10			//; Touch slow mode wakeup period select 16Hz
#define	C_WKUPT32ms			0x00			//; Touch slow mode wakeup period select 32Hz
//;Bit[2:0] : Touch pad scan frequency select
#define	C_TFQ1M1			0x04			//; Touch pad modulation clock select 1.12MHz
#define	C_TFQ1M3			0x03			//; Touch pad modulation clock select 1.31MHz
#define	C_TFQ1M0			0x02			//; Touch pad modulation clock select 1MHz
#define	C_TFQ08M			0x01			//; Touch pad modulation clock select 0.88MHz
#define	C_TFQ07M			0x00			//; Touch pad modulation clock select 0.75MHz

//;------------------------------------------------------------
//; CASR  (0x0A)	--------- Touch pad extra capacitance select register
//;------------------------------------------------------------	
//;Bit[7] : Reserved
#define C_Ex_CAP_0			0				//; Touch pad extra capacitance minimum
#define C_Ex_CAP_32			32				//; Touch pad extra capacitance 32*0.05p
#define C_Ex_CAP_64			64				//; Touch pad extra capacitance 64*0.05p
#define C_Ex_CAP_127		127				//; Touch pad extra capacitance maximum (127*0.05p)
#define C_Ex_CAP_Max		127				//; Touch pad extra capacitance maximum (127*0.05p)
//;------------------------------------------------------------
//; TPCHS (0x0B)	--------- Touch pad channel select register
//;------------------------------------------------------------
//;#define	C_Group3			0x13			//; select  TP12~TP15
#define	C_Allkey			0x17			//; select all touch pad
#define	C_Groupall			0x18			//; select  GP0~GP2 [sleep only]
#define	C_Group01			0x14			//; select  GP0~GP1 [sleep only]
#define	C_Group2			0x12			//; select  TP8~TP11
#define	C_Group1			0x11			//; select  TP4~TP7
#define	C_Group0			0x10			//; select  TP0~TP3
#define	C_Inkey				0x16			//; select  INKEY
#define	C_TP11				0x0B			//; select  pad TP11
#define	C_TP10				0x0A			//; select  pad TP10
#define	C_TP9 				0x09			//; select  pad TP9 
#define	C_TP8 				0x08			//; select  pad TP8 
#define	C_TP7 				0x07			//; select  pad TP7 
#define	C_TP6 				0x06			//; select  pad TP6 
#define	C_TP5 				0x05			//; select  pad TP5 
#define	C_TP4 				0x04			//; select  pad TP4 
#define	C_TP3 				0x03			//; select  pad TP3 
#define	C_TP2 				0x02			//; select  pad TP2 
#define	C_TP1 				0x01			//; select  pad TP1 
#define	C_TP0 				0x00			//; select  pad TP0 
//;------------------------------------------------------------
//; TPCR (0x0C)	--------- Touch pad control register
//;------------------------------------------------------------
#define	C_TPSLOW			0x05			//; 101 = TPRUN in TP slow mode(scan the channel with TPCHS select)
#define	C_TPDISCHARGE		0x03			//; 011 = Initialize (Discharge Cs)
#define	C_TPOFF				0x02			//; 010 = TPCHOFF(all channel not select)
#define	C_TPRUN				0x01			//; 001 = TPRUN (scan the channel which TPCHS select).
#define	C_TPSTOP			0x00			//; 000 = TPSTP (touch pad  all stop)	
//;------------------------------------------------------------
//; TPCNTL (0x0D)	--------- Touch pad low counter register
//;------------------------------------------------------------
//;------------------------------------------------------------
//; TPCNTH (0x0E)	--------- Touch pad high counter register
//;------------------------------------------------------------
#define	C_TPCNT_RST			0x0FFF			//; TPCNT[11:0] reset value, count will start from 0 

//;------------------------------------------------------------
//; TPEN0 (0x0F)	--------- Touch pad enable register
//;------------------------------------------------------------	
#define	C_TPPADEN7			0x80			//; TP7 enable register
#define	C_TPPADEN6			0x40			//; TP6 enable register
#define	C_TPPADEN5			0x20			//; TP5 enable register
#define	C_TPPADEN4			0x10			//; TP4 enable register
#define	C_TPPADEN3			0x08			//; TP3 enable register
#define	C_TPPADEN2			0x04			//; TP2 enable register
#define	C_TPPADEN1			0x02			//; TP1 enable register
#define	C_TPPADEN0			0x01			//; TP0 enable register
#define	C_TPPADEN7_BIT		7
#define	C_TPPADEN6_BIT		6
#define	C_TPPADEN5_BIT		5
#define	C_TPPADEN4_BIT		4
#define	C_TPPADEN3_BIT		3
#define	C_TPPADEN2_BIT		2
#define	C_TPPADEN1_BIT		1
#define	C_TPPADEN0_BIT		0
//;------------------------------------------------------------
//; TPEN1 (0x10)	--------- Touch pad enable register
//;------------------------------------------------------------	
//;Bit[7:4] : Reserved
#define	C_TPPADEN11			0x08			//; TP11 enable register
#define	C_TPPADEN10			0x04			//; TP10 enable register
#define	C_TPPADEN9			0x02			//; TP0 enable register
#define	C_TPPADEN8			0x01			//; TP8 enable register
#define	C_TPPADEN11_BIT		3
#define	C_TPPADEN10_BIT		2
#define	C_TPPADEN9_BIT		1
#define	C_TPPADEN8_BIT		0
//;------------------------------------------------------------
//; CASR1  (0x11)	--------- Touch pad extra capacitance select register
//;------------------------------------------------------------	
//;Bit[7] : Reserved
#define C_Ex_CAP1_0			0				//; Touch pad extra capacitance minimum
#define C_Ex_CAP1_32		32				//; Touch pad extra capacitance 32*0.05p
#define C_Ex_CAP1_64		64				//; Touch pad extra capacitance 64*0.05p
#define C_Ex_CAP1_Max		127				//; Touch pad extra capacitance maximum (127*0.05p)
//;------------------------------------------------------------
//; CASR2  (0x12)	--------- Touch pad extra capacitance select register
//;------------------------------------------------------------	
//;Bit[7] : Reserved
#define C_Ex_CAP2_0			0				//; Touch pad extra capacitance minimum
#define C_Ex_CAP2_32		32				//; Touch pad extra capacitance 32*0.05p
#define C_Ex_CAP2_64		64				//; Touch pad extra capacitance 64*0.05p
#define C_Ex_CAP2_Max		127				//; Touch pad extra capacitance maximum (127*0.05p)
//;------------------------------------------------------------
//; TPCNTL1 (0x14)	--------- Touch pad low counter register
//;------------------------------------------------------------
#define	C_TPCNT1_RST		0x0FFF			//; TPCNT[11:0] reset value, count will start from 0 
//;------------------------------------------------------------
//; TPCNTL2 (0x15)	--------- Touch pad low counter register
//;------------------------------------------------------------
#define	C_TPCNT2_RST		0x0FFF			//; TPCNT[11:0] reset value, count will start from 0 
//;------------------------------------------------------------
//; TPCNTL3 (0x16)	--------- Touch pad low counter register
//;------------------------------------------------------------
#define	C_TPCNT3_RST		0x0FFF			//; TPCNT[11:0] reset value, count will start from 0
//;------------------------------------------------------------
//; THR (0x18)	--------- Transmit holding register /Receive Buffer Register
//;------------------------------------------------------------
//;Bit[7:0] : Transmit holding register /Receive Buffer Register
//;------------------------------------------------------------
//; LCR (0x19)	--------- Line control register
//;------------------------------------------------------------
//;Bit[7] : Loop Back Test Enable
#define	C_LOOP_En			0x80			//; Enable loop back test
#define	C_LOOP_Dis			0x00			//; Disable loop back test
//;Bit[6] : Set Break
#define	C_SBRK_En			0x40			//; Enable set break
#define	C_SBRK_Dis			0x00			//; Disable set break
//;Bit[5] : Stuck Parity Enable
#define	C_PSTUCK_En			0x20			//; Enable stuck parity
#define	C_PSTUCK_Dis		0x00			//; Disable stuck parity
//;Bit[4] : Even Parity Select
#define	C_Even_Parity		0x10			//; Select even parity
#define	C_Odd_Parity		0x00			//; Select odd parity
//;Bit[3] : Parity Enable
#define	C_Parity_En			0x08			//; Enable parity
#define	C_Parity_Dis		0x00			//; Disable parity
//;Bit[2] : STOP Bit Number
#define	C_STOP_2			0x04			//; Select 2 stop bit
#define	C_STOP_1P5			0x04			//; Select 1.5 stop bit
#define	C_STOP_1			0x00			//; Select 1 stop bit
//;Bit[1:0] : Word Length Bits
#define	C_WL_8				0x03			//; Select word length 8 bit
#define	C_WL_7				0x02			//; Select word length 7 bit
#define	C_WL_6				0x01			//; Select word length 6 bit
#define	C_WL_5				0x00			//; Select word length 5 bit

#define	C_LOOP_Bit			7
#define	C_SBRK_Bit			6
#define	C_PSTUCK_Bit		5
#define	C_PEVEN_Bit			4
#define	C_PREN_Bit			3
#define	C_STPS_Bit			2
#define	C_WL1_Bit			1
#define	C_WL0_Bit			0
//;------------------------------------------------------------
//; LSR (0x1A)	--------- Line status register
//;------------------------------------------------------------
//;Bit[7] : Reserved
//;Bit[6] : Transmitter shift register(TSR) empty
#define	TSRE				0x40			//; Transmitter shift register(TSR) empty flag
//;Bit[5] : Reserved
//;Bit[4] : Break interrupt flag
#define	BKINT				0x10			//; Break interrupt flag
//;Bit[3] : Frame error flag
#define	FERR				0x08			//; Frame error flag
//;Bit[2] : Parity error flag
#define	PERR				0x04			//; Parity error flag
//;Bit[1] : Over run error flag
#define	OERR				0x02			//; Over run error flag
//;Bit[0] : Reserved
//;------------------------------------------------------------
//; DLL (0x1B)	--------- Baud rate divisor latch LSB register
//;------------------------------------------------------------
//;Bit[7:0] : Baud rate divider LSB bit

#define	C_DLL7_Bit			7
#define	C_DLL6_Bit			6
#define	C_DLL5_Bit			5
#define	C_DLL4_Bit			4
#define	C_DLL3_Bit			3
#define	C_DLL2_Bit			2
#define	C_DLL1_Bit			1
#define	C_DLL0_Bit			0
//;------------------------------------------------------------
//; DLH (0x1C)	--------- Baud rate divisor latch MSB register
//;------------------------------------------------------------
//;Bit[7:0] : Baud rate divider MSB bit

#define	C_DLH7_Bit			7
#define	C_DLH6_Bit			6
#define	C_DLH5_Bit			5
#define	C_DLH4_Bit			4
#define	C_DLH3_Bit			3
#define	C_DLH2_Bit			2
#define	C_DLH1_Bit			1
#define	C_DLH0_Bit			0
//;------------------------------------------------------------	
//; CCP1CON (0x1E)	--------- CCP Control Register (Enhanced Capture/ Compare/PWM Modules)
//;------------------------------------------------------------
//;Bit[7:6] : CCP1_PWM Output Configuration 
//; when CCP1M[3:2] = 00, 01, 10 //;			//; P1A(PB2) assigned as Capture input or Compare output. P1B(PA5), P1C(PA2) and P1D(PA4) assigned as GPIO pins.
//; when CCP1M[3:2] = 11 & 
//; PWM1M[1:0] =      
#define C_CCP_SingOut		0x00				//; 00 : Single output. P1A(PB2) modulated. P1B(PA5), P1C(PA2), P1D(PA4) assigned as GPIO pins.
#define C_CCP_Forward		0x40				//; 01 : Full-bridge output forward. P1D(PA4) modulated. P1A(PB2) active. P1B(PA5), P1C(PA2) inactive.
#define C_CCP_HalfOut		0x80				//; 10 : Half-bridge output. P1A(PB2), P1B(PA5) modulated with deadband control. P1C(PA2), P1D(PA4) assigned as GPIO pins.
#define C_CCP_Reverse		0xC0				//; 11 : Full-bridge output reverse. P1B(PA5) modulated. P1C(PA2) active. P1A(PB2), P1D(PA4) inactive.
										
//;Bit[5:4] : Full-bridge change direction gap(deadband)
#define	C_HBDel_1CPU		0x00				//; 1  CPU Cycle
#define	C_HBDel_4CPU		0x10				//; 4  CPU Cycle
#define	C_HBDel_16CPU		0x20				//; 16 CPU Cycle
	
//;Bit[3:0] : ECCP Mode Select bits
#define	C_CCP_Reset			0x00				//; 0000 = Capture/Compare/PWM off (resets ECCP modules)
												//; 0001 = Unused (reserved)
#define	C_CCP_Comp_Toggle	0x02				//; 0010 = Compare mode, toggle P1A(PB2) output on match (CCP1IF bit is set)
												//; 0011 = Unused (reserved)
#define	C_CCP_Capt_FallEdge	 	0x04			//; 0100 = Capture mode, every falling edge
#define	C_CCP_Capt_RiseEdge		0x05			//; 0101 = Capture mode, every rising edge
#define	C_CCP_Capt_4timeRise	0x06			//; 0110 = Capture mode, every 4th rising edge
#define	C_CCP_Capt_16timeRise	0x07			//; 0111 = Capture mode, every 16th rising edge
#define	C_CCP_Comp_High		0x08				//; 1000 = Compare mode, set P1A(PB2) output on match (CCP1IF bit is set)
#define	C_CCP_Comp_Low		0x09				//; 1001 = Compare mode, clear P1A(PB2) output on match (CCP1IF bit is set)
#define	C_CCP_Comp_INT		0x0A				//; 1010 = Compare mode, generate interrupt flag on match (CCP1IF bit is set, P1A pin is unaffected)
												//; 1011 = Compare mode, trigger special event event. (CCP1IF bit is set//; ECCP reload TMR1, and starts an A/D conversion, if the A/D module is enabled)
#define	C_CCP_PWM_All_ActHigh		0x0C		//; 1100 = PWM mode. P1A(PB2), P1C(PA2) active high. P1D(PA4), P1B(PA5) active high.
#define	C_CCP_PWM_P1A_P1C_ActHigh	0x0D		//; 1101 = PWM mode. P1A(PB2), P1C(PA2) active high. P1D(PA4), P1B(PA5) active low.
#define	C_CCP_PWM_P1B_P1D_ActHigh	0x0E		//; 1110 = PWM mode. P1A(PB2), P1C(PA2) active low.  P1D(PA4), P1B(PA5) active high.
#define	C_CCP_PWM_All_ActLow		0x0F		//; 1111 = PWM mode. P1A(PB2), P1C(PA2) active low.  P1D(PA4), P1B(PA5) active low.	
//;--------------------------------------------------------------------------------------------------	
//; PWMDB (0x1F)	--------- PWM Delay Register (Half-bridge output. P1A,P1B modulated with deadband control.)
//;--------------------------------------------------------------------------------------------------	
//;Bit[7:0] : PWM Delay Count for Half-Bridge Output Mode  ( Unit:1 CPU Cycle/Bit)
#define	C_CCP_PWM_Delay		0xFF		//; Number of Instruction clock cycles between the P1A(PB2) transition and the P1B(PA5) transition.
	
//;=======================================================================================================================
//;=======================================================================================================================
//;-----------------------------------------------------------------------------------------------------------------------
//;General Constant Define
//;-----------------------------------------------------------------------------------------------------------------------
#define	C_SaveToAcc			0x00	
#define	toacc				0x00
#define	C_SaveToReg			0x01	
#define	toreg				0x01
			
#define	C_Bit0				0x01
#define	C_Bit1				0x02
#define	C_Bit2				0x04
#define	C_Bit3				0x08
#define	C_Bit4				0x10
#define	C_Bit5				0x20
#define	C_Bit6				0x40
#define	C_Bit7				0x80
			
#define	C_Num0				0x0
#define	C_Num1				0x1
#define	C_Num2				0x2
#define	C_Num3				0x3
#define	C_Num4				0x4
#define	C_Num5				0x5
#define	C_Num6				0x6
#define	C_Num7				0x7

