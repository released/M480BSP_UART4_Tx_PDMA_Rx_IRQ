# M480BSP_UART4_Tx_PDMA_Rx_IRQ
 M480BSP_UART4_Tx_PDMA_Rx_IRQ


update @ 2023/05/03

1. initial UART4 TX (PC.7) w/ PDMA function , RX  (PC.6) with UART IRQ

- with 2 define ENALBE_PDMA_IRQ ,  ENALBE_PDMA_POLLING , for PDMA transfer function

2. TX / RX Buffer length set as 64 , send TX buffer per 100 ms , 

- set buffer index 0 , 1 as 0x5A , for indicator

- set buffer last index n-1 , n-2 , as 0x A5 , for indicator

- set buffer last index 2 , n-3 , as a counter (increase per 100ms) , for indicator

- by connect RX pin to TX pin , to test interrupt RX function

- set FLAG_UART_RX_FINISH , when RX receive finish (meet target length)

below is log to dump TX / RX buffer

![image](https://github.com/released/M480BSP_UART4_Tx_PDMA_Rx_IRQ/blob/main/log_tx_rx.jpg)	

below is log to compare TX / RX buffer

![image](https://github.com/released/M480BSP_UART4_Tx_PDMA_Rx_IRQ/blob/main/log_compare.jpg)	


