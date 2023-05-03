/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "misc_config.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

struct flag_32bit flag_PROJ_CTL;
#define FLAG_PROJ_TIMER_PERIOD_1000MS                 	(flag_PROJ_CTL.bit0)
#define FLAG_PROJ_TIMER_PERIOD_TX                   	(flag_PROJ_CTL.bit1)
#define FLAG_UART_TX_FINISH                 			(flag_PROJ_CTL.bit2)
#define FLAG_UART_RX_FINISH                             (flag_PROJ_CTL.bit3)
#define FLAG_PROJ_REVERSE4                              (flag_PROJ_CTL.bit4)
#define FLAG_PROJ_REVERSE5                              (flag_PROJ_CTL.bit5)
#define FLAG_PROJ_REVERSE6                              (flag_PROJ_CTL.bit6)
#define FLAG_PROJ_REVERSE7                              (flag_PROJ_CTL.bit7)


/*_____ D E F I N I T I O N S ______________________________________________*/

#define ENALBE_PDMA_IRQ
// #define ENALBE_PDMA_POLLING

#define UART_PORT  						(UART4)
#define UART_TX_DMA_CH 				    (5)
#define UART_TX_PDMA_OPENED_CH   		((1 << UART_TX_DMA_CH))
#define TXBUFSIZE                       (64)
uint8_t tBuffer[TXBUFSIZE] = {0};
uint8_t tmpBuffer[TXBUFSIZE] = {0};

#define RXBUFSIZE                       (TXBUFSIZE)
uint8_t rBuffer[RXBUFSIZE] = {0};
uint8_t rx_count = 0;

volatile unsigned int counter_systick = 0;
volatile uint32_t counter_tick = 0;

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

unsigned int get_systick(void)
{
	return (counter_systick);
}

void set_systick(unsigned int t)
{
	counter_systick = t;
}

void systick_counter(void)
{
	counter_systick++;
}

void SysTick_Handler(void)
{

    systick_counter();

    if (get_systick() >= 0xFFFFFFFF)
    {
        set_systick(0);      
    }

    // if ((get_systick() % 1000) == 0)
    // {
       
    // }

    #if defined (ENABLE_TICK_EVENT)
    TickCheckTickEvent();
    #endif    
}

void SysTick_delay(unsigned int delay)
{  
    
    unsigned int tickstart = get_systick(); 
    unsigned int wait = delay; 

    while((get_systick() - tickstart) < wait) 
    { 
    } 

}

void SysTick_enable(unsigned int ticks_per_second)
{
    set_systick(0);
    if (SysTick_Config(SystemCoreClock / ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
        while (1);
    }

    #if defined (ENABLE_TICK_EVENT)
    TickInitTickEvent();
    #endif
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void tick_counter(void)
{
	counter_tick++;
    if (get_tick() >= 60000)
    {
        set_tick(0);
    }
}
void TMR1_IRQHandler(void)
{
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
            FLAG_PROJ_TIMER_PERIOD_1000MS = 1;//set_flag(flag_timer_period_1000ms ,ENABLE);
		}

		if ((get_tick() % 100) == 0)
		{
            FLAG_PROJ_TIMER_PERIOD_TX = 1;
		}
		
		if ((get_tick() % 50) == 0)
		{

		}	
    }
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);
	
    if (status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
		#if 1
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_GET_ABORT_STS(PDMA));
		#else
        if (PDMA_GET_ABORT_STS(PDMA) & UART_TX_PDMA_OPENED_CH)
        {
        }
        PDMA_CLR_ABORT_FLAG(PDMA, UART_TX_PDMA_OPENED_CH);
        if (PDMA_GET_ABORT_STS(PDMA) & (1 << QSPI_MASTER_RX_DMA_CH))
        {
        }
        PDMA_CLR_ABORT_FLAG(PDMA, (1 << QSPI_MASTER_RX_DMA_CH));
		#endif
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        if((PDMA_GET_TD_STS(PDMA) & UART_TX_PDMA_OPENED_CH ) == UART_TX_PDMA_OPENED_CH  )
        {
            /* Clear PDMA transfer done interrupt flag */
            PDMA_CLR_TD_FLAG(PDMA, UART_TX_PDMA_OPENED_CH );
            
			//insert process
            UART_PDMA_DISABLE(UART_PORT,UART_INTEN_TXPDMAEN_Msk);
            FLAG_UART_TX_FINISH = 1;

        }  
    }
    else if (status & (PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk))     /* Check the DMA time-out interrupt flag */
    {
        // PDMA_CLR_TMOUT_FLAG(PDMA,UART_TX_DMA_CH);
    }
    else
    {
        // printf("status : 0x%4X\r\n" ,status);
    }

}

void UART_TX_PDMA(uint8_t* Datain , uint16_t len)
{
    #if defined (ENALBE_PDMA_POLLING)      
	uint32_t u32RegValue = 0;
	uint32_t u32Abort = 0;	
    static uint8_t cnt = 0;    
    #endif

    FLAG_UART_TX_FINISH = 0;
	
	//U
    PDMA_SetTransferCnt(PDMA,UART_TX_DMA_CH, PDMA_WIDTH_8, len);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,UART_TX_DMA_CH, (uint32_t) (&Datain[0]), PDMA_SAR_INC, (uint32_t) UART4_BASE , PDMA_DAR_FIX);
    /* Set request source; set basic mode. */

    PDMA_SetTransferMode(PDMA,UART_TX_DMA_CH, PDMA_UART4_TX, FALSE, 0);

	UART_PDMA_ENABLE(UART_PORT,UART_INTEN_TXPDMAEN_Msk);

    #if defined (ENALBE_PDMA_POLLING)   
    while(1)
    {
        /* Get interrupt status */
        u32RegValue = PDMA_GET_INT_STATUS(PDMA);
        /* Check the DMA transfer done interrupt flag */
        if(u32RegValue & PDMA_INTSTS_TDIF_Msk)
        {
            /* Check the PDMA transfer done interrupt flags */
            if((PDMA_GET_TD_STS(PDMA) & UART_TX_PDMA_OPENED_CH) == UART_TX_PDMA_OPENED_CH)
            {
                /* Clear the DMA transfer done flags */
                PDMA_CLR_TD_FLAG(PDMA , UART_TX_PDMA_OPENED_CH);

                UART_PDMA_DISABLE(UART_PORT,UART_INTEN_TXPDMAEN_Msk);
                FLAG_UART_TX_FINISH = 1;
                break;              
            }
        }
        /* Check the DMA transfer abort interrupt flag */
        if(u32RegValue & PDMA_INTSTS_ABTIF_Msk)
        {
            /* Get the target abort flag */
            u32Abort = PDMA_GET_ABORT_STS(PDMA);
            /* Clear the target abort flag */
            PDMA_CLR_ABORT_FLAG(PDMA,u32Abort);
            break;
        }		
    }
    #endif

    // while(!FLAG_UART_TX_FINISH);
    while(!UART_IS_TX_EMPTY(UART_PORT));

}

void UART4_IRQHandler(void)
{
    uint8_t buffer = 0;

    if(UART_GET_INT_FLAG(UART_PORT, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART_PORT) == 0)
        {            
	        buffer = UART_READ(UART_PORT);

            if (rx_count < RXBUFSIZE)
            {
                rBuffer[rx_count] = buffer;
                rx_count++;

                if (rx_count == (RXBUFSIZE-1) )
                {
                    FLAG_UART_RX_FINISH = 1;
                }
            }            
        }
    }

    if(UART_PORT->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART_PORT, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	
}

void UART_PDMA_Init(void)
{

    FLAG_UART_TX_FINISH = 0;

    UART_Open(UART_PORT, 115200);

    PDMA_Open(PDMA, UART_TX_PDMA_OPENED_CH);

    PDMA_SetBurstType(PDMA,UART_TX_DMA_CH, PDMA_REQ_SINGLE, PDMA_BURST_128);
    /* Disable table interrupt */
    PDMA->DSCT[UART_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

	UART_PDMA_DISABLE(UART_PORT,UART_INTEN_TXPDMAEN_Msk);        

    #if defined (ENALBE_PDMA_IRQ)
    PDMA_EnableInt(PDMA, UART_TX_DMA_CH, PDMA_INT_TRANS_DONE);

    NVIC_SetPriority(PDMA_IRQn,0);
    NVIC_ClearPendingIRQ(PDMA_IRQn);
    NVIC_EnableIRQ(PDMA_IRQn);
    #endif
    
    FLAG_UART_RX_FINISH = 0;
    rx_count = 0;

	UART_SetTimeoutCnt(UART_PORT, 20);

	UART_PORT->FIFO &= ~UART_FIFO_RFITL_4BYTES;
	UART_PORT->FIFO |= UART_FIFO_RFITL_8BYTES;

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART_PORT, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);

    NVIC_SetPriority(UART4_IRQn,2);
	NVIC_EnableIRQ(UART4_IRQn);
}

void create_buffer(void)
{
    uint16_t i = 0;  
    for (i = 0 ; i < (TXBUFSIZE/2) ; i++)
    {
        tBuffer[i*2+0] = HIBYTE(i);
        tBuffer[i*2+1] = LOBYTE(i);
    }    
}

void loop(void)
{
	// static uint32_t LOG1 = 0;
	// static uint32_t LOG2 = 0;
	static uint32_t cnt = 0;  

    if ((get_systick() % 1000) == 0)
    {
        // printf("%s(systick) : %4d\r\n",__FUNCTION__,LOG2++);    
    }

    if (FLAG_PROJ_TIMER_PERIOD_TX)
    {
        FLAG_PROJ_TIMER_PERIOD_TX = 0;

        tBuffer[0] = 0x5A;
        tBuffer[1] = 0x5A;
        tBuffer[2] = cnt;       
        tBuffer[TXBUFSIZE-3] = cnt+1;         
        tBuffer[TXBUFSIZE-2] = 0xA5;        
        tBuffer[TXBUFSIZE-1] = 0xA5;
        
        copy_buffer(tmpBuffer,tBuffer,TXBUFSIZE);

		UART_TX_PDMA(tBuffer , TXBUFSIZE);
        cnt++;

        // printf("tx:\r\n");
        // dump_buffer_hex(tmpBuffer , TXBUFSIZE);
    }

    if (FLAG_PROJ_TIMER_PERIOD_1000MS)
    {
        FLAG_PROJ_TIMER_PERIOD_1000MS = 0;

        // printf("%s(timer) : %4d\r\n",__FUNCTION__,LOG1++);
        PH0 ^= 1;
    }

    if (FLAG_UART_RX_FINISH)
    {
        FLAG_UART_RX_FINISH = 0;
        rx_count = 0;

        // printf("[rx]\r\n");
        // dump_buffer_hex(rBuffer , RXBUFSIZE);
        compare_buffer(rBuffer,tmpBuffer,RXBUFSIZE);

        reset_buffer(rBuffer , 0x00 , RXBUFSIZE);
    }
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART0);

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		printf("press : %c\r\n" , res);
		switch(res)
		{
			case '1':
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
                SYS_UnlockReg();
				// NVIC_SystemReset();	// Reset I/O and peripherals , only check BS(FMC_ISPCTL[1])
                // SYS_ResetCPU();     // Not reset I/O and peripherals
                SYS_ResetChip();    // Reset I/O and peripherals ,  BS(FMC_ISPCTL[1]) reload from CONFIG setting (CBS)	
				break;
		}
	}
}

void UART0_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
			UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, 20);

	UART0->FIFO &= ~UART_FIFO_RFITL_4BYTES;
	UART0->FIFO |= UART_FIFO_RFITL_8BYTES;

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART0_IRQn);

	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());    	

//    printf("Product ID 0x%8X\n", SYS->PDID);
	
	#endif	

    #if 0
    printf("FLAG_PROJ_TIMER_PERIOD_1000MS : 0x%2X\r\n",FLAG_PROJ_TIMER_PERIOD_1000MS);
    printf("FLAG_PROJ_REVERSE1 : 0x%2X\r\n",FLAG_PROJ_REVERSE1);
    printf("FLAG_PROJ_REVERSE2 : 0x%2X\r\n",FLAG_PROJ_REVERSE2);
    printf("FLAG_PROJ_REVERSE3 : 0x%2X\r\n",FLAG_PROJ_REVERSE3);
    printf("FLAG_PROJ_REVERSE4 : 0x%2X\r\n",FLAG_PROJ_REVERSE4);
    printf("FLAG_PROJ_REVERSE5 : 0x%2X\r\n",FLAG_PROJ_REVERSE5);
    printf("FLAG_PROJ_REVERSE6 : 0x%2X\r\n",FLAG_PROJ_REVERSE6);
    printf("FLAG_PROJ_REVERSE7 : 0x%2X\r\n",FLAG_PROJ_REVERSE7);
    #endif

}

void GPIO_Init (void)
{
	SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH0MFP_Msk)) | (SYS_GPH_MFPL_PH0MFP_GPIO);
	SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH1MFP_Msk)) | (SYS_GPH_MFPL_PH1MFP_GPIO);
	SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH2MFP_Msk)) | (SYS_GPH_MFPL_PH2MFP_GPIO);

	//EVM LED
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);
	
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

    CLK_EnableModuleClock(UART4_MODULE);
    CLK_SetModuleClock(UART4_MODULE, CLK_CLKSEL3_UART4SEL_HIRC, CLK_CLKDIV4_UART4(1));

    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC6MFP_Msk | SYS_GPC_MFPL_PC7MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC6MFP_UART4_RXD | SYS_GPC_MFPL_PC7MFP_UART4_TXD);

    CLK_EnableModuleClock(PDMA_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    SYS_Init();

	GPIO_Init();
	UART0_Init();
	TIMER1_Init();

    SysTick_enable(1000);
    #if defined (ENABLE_TICK_EVENT)
    TickSetTickEvent(1000, TickCallback_processA);  // 1000 ms
    TickSetTickEvent(5000, TickCallback_processB);  // 5000 ms
    #endif

    UART_PDMA_Init();
    create_buffer();

    /* Got no where to go, just loop forever */
    while(1)
    {
        loop();

    }
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
