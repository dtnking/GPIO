/*
 * Dma.c
 *
 *  Created on: Dec 19, 2017
 *      Author: user2
 */

/*
 * DMA2,stream2,channel4 = USART1_Rx
 * DMA2,stream7,channel4 = USART1_Tx
 *
 * PSIZE = 1 byte
 * MSIZE = 1 byte
 *
 * MBURST = in 4 beats increment
 * FIFO_BURST = 1/2 threshold
 *
 * PINC = no
 * MINC = yes
 * PINCOS = no
 *
 * Circular mode = no
 * Double buffer = no
 * Transfer direction = Memory-to-peripheral (Tx)
 * Transfer direction = Peripheral-to-memory (Rx)
 * Priority = high
 * Flow controller = DMA
 *
 *
 */

#include "Dma.h"

#include "stdio.h"




void dmaInitForUsart1(DmaReg *dma, int streamNum, int channel,int mb, int pb, int dbm, int p,int pinc, int ms, \
					 int ps,int mi, int pi, int circ, int dir, int fc){
	dma->S[streamNum].CR &= ~1;
	dmaStreamSetMChannel(dma,streamNum,channel);
	dmaStreamSetMBurst(dma,streamNum,mb);
	dmaStreamSetPBurst(dma,streamNum,pb);
	dmaStreamSetDBM(dma,streamNum,dbm);
	dmaStreamSetPriority(dma,streamNum,p);
	dmaStreamSetPINCOS(dma,streamNum,pinc);
	dmaStreamSetMSize(dma,streamNum,ms);
	dmaStreamSetPSize(dma,streamNum,ps);
	dmaStreamSetCirc(dma,streamNum,circ);
	dmaStreamSetMInc(dma,streamNum,mi);
	dmaStreamSetPInc(dma,streamNum,pi);
	dmaStreamSetDir(dma,streamNum,dir);
	dmaStreamSetFlowContr(dma,streamNum,fc);

}

void dmaSetAddressAndSize(DmaReg *dma,int streamNum,uint32_t memoryAddr,       \
						  uint32_t peripheralAddr,uint32_t size){
	dma->S[streamNum].PAR = peripheralAddr;
	dma->S[streamNum].M0AR = memoryAddr;
	dma->S[streamNum].NDTR = size;
	dma->S[streamNum].CR |= DMA_STREAM_EN;
}

int dmaStreamCheckFlag(DmaReg *dma, int streamNum, int flag){
	int isUpper16 = FALSE;
	volatile uint32_t *intrStatusReg = &dma->LISR;
	if(streamNum>=4){
		intrStatusReg = &dma->HISR;
		streamNum -= 4;
	}
	if(streamNum >= 2){
		streamNum -= 2;
		isUpper16 = TRUE;
	}
	return *intrStatusReg & ( flag << (6 * streamNum + 16 * isUpper16));
}


