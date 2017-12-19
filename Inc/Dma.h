/*
 * Dma.h
 *
 *  Created on: Dec 19, 2017
 *      Author: user2
 */

#ifndef DMA_H_
#define DMA_H_
#include "RCC.h"

#define DMA1_BASE_ADDRS		(0x40026000)
#define DMA2_BASE_ADDRS		(0x40026400)

#define dma1			((DmaReg *)(DMA1_BASE_ADDRS))
#define dma2			((DmaReg *)(DMA2_BASE_ADDRS))

typedef struct DmaStreamReg DmaStreamReg;

struct DmaStreamReg{
	volatile uint32_t CR;			// 0x0
	volatile uint32_t NDTR;			// 0x4
	volatile uint32_t PAR;			// 0x8
	volatile uint32_t M0AR;			// 0xc
	volatile uint32_t M1AR;			// 0x10
	volatile uint32_t FCR;			// 0x14
};


typedef struct DmaReg DmaReg;

struct DmaReg{
	volatile uint32_t LISR;
	volatile uint32_t HISR;
	volatile uint32_t LIFCR;
	volatile uint32_t HIFCR;
	DmaStreamReg S[8];
};

#define dmaStreamHasTransferCompleted(d,s)			dmaStreamCheckFlag(d,s,DMA_TCIF)
#define dmaStreamHasHalfTransferCompleted(d,s)		dmaStreamCheckFlag(d,s,DMA_HTIF)
#define dmaStreamIsTransferError(d,s)				dmaStreamCheckFlag(d,s,DMA_TEIF)
#define dmaStreamIsDirectModeError(d,s)				dmaStreamCheckFlag(d,s,DMA_DMEIF)
#define dmaStreamIsError(d,s)						dmaStreamCheckFlag(d,s,DMA_FEIF)

#define DMA1_DEV		21
#define DMA2_DEV		22

#define CH0				(0)
#define CH1				(1)
#define CH2				(2)
#define CH3				(3)
#define CH4				(4)
#define CH5				(5)
#define CH6				(6)
#define CH7				(7)

#define SIN_TRANFER		(0)
#define INCR4			(1)
#define INCR8			(2)
#define INCR16			(3)

#define DBM_EN			(1)
#define DBM_DIS			(0)

#define PL_LOW			(0)
#define PL_MED			(1)
#define PL_HI			(2)
#define PL_VHI			(3)

#define PINCOS_EN		(1)
#define PINCOS_DIS		(0)

#define MSIZE_BYTE		(0)
#define MSIZE_HALFWORD	(1)
#define MSIZE_WORD		(2)

#define PSIZE_BYTE		(0)
#define PSIZE_HALFWORD	(1)
#define PSIZE_WORD		(2)

#define MINC_EN			(1)
#define MINC_DIS		(0)

#define PINC_EN			(1)
#define PINC_DIS		(0)

#define CIRC_EN			(1)
#define CIRC_DIS		(0)

#define DIR_P_TO_M		(0)
#define DIR_M_TO_P		(1)
#define DIR_M_TO_M		(2)

#define PFCTRL_DMA		(0)
#define PFCTRL_PER		(1)

#define STREAM_EN		(1)
#define STREAM_DIS		(0)

// DMA interrupt Status Register (xISR)
#define DMA_FEIF		1
#define DMA_DMEIF		(1<<2)
#define DMA_TEIF		(1<<3)
#define DMA_HTIF		(1<<4)
#define DMA_TCIF		(1<<5)


#define dmaStreamSetMChannel(d,s,c)			\
			do{								\
				((d)->S[s].CR) &= ~(7<<25);		\
				((d)->S[s].CR) |= (c<<25);		\
			}while(0)

#define dmaStreamSetMBurst(d,s,mb)			\
			do{								\
				((d)->S[s].CR) &= ~(3<<23);		\
				((d)->S[s].CR) |= (mb<<23);		\
			}while(0)

#define dmaStreamSetPBurst(d,s,pb)			\
			do{								\
				((d)->S[s].CR) &= ~(3<<21);		\
				((d)->S[s].CR) |= (pb<<21);		\
			}while(0)

#define dmaStreamSetDBM(d,s,dbm)				\
			do{								\
				((d)->S[s].CR) &= ~(1<<18);		\
				((d)->S[s].CR)|= (dbm<<18);		\
			}while(0)

#define dmaStreamSetPriority(d,s,p)			\
			do{								\
				((d)->S[s].CR) &= ~(3<<16);		\
				((d)->S[s].CR) |= (p<<16);		\
			}while(0)

#define dmaStreamSetPINCOS(d,s,pinc)			\
			do{								\
				((d)->S[s].CR) &= ~(1<<15);		\
				((d)->S[s].CR) |= (pinc<<15);		\
			}while(0)

#define dmaStreamSetMSize(d,s,ms)			\
			do{								\
				((d)->S[s].CR) &= ~(3<<13);		\
				((d)->S[s].CR) |= (ms<<13);		\
			}while(0)

#define dmaStreamSetPSize(d,s,ps)			\
			do{								\
				((d)->S[s].CR) &= ~(3<<11);		\
				((d)->S[s].CR)|= (ps<<11);		\
			}while(0)

#define dmaStreamSetMInc(d,s,mi)				\
			do{								\
				((d)->S[s].CR) &= ~(1<<10);		\
				((d)->S[s].CR) |= (mi<<10);		\
			}while(0)

#define dmaStreamSetPInc(d,s,pi)				\
			do{								\
				((d)->S[s].CR) &= ~(1<<9);		\
				((d)->S[s].CR) |= (pi<<9);		\
			}while(0)

#define dmaStreamSetCirc(d,s,circ)				\
			do{								\
				((d)->S[s].CR) &= ~(1<<8);		\
				((d)->S[s].CR) |= (circ<<8);		\
			}while(0)

#define dmaStreamSetDir(d,s,dir)				\
			do{								\
				((d)->S[s].CR) &= ~(3<<6);		\
				((d)->S[s].CR) |= (dir<<6);		\
			}while(0)

#define dmaStreamSetFlowContr(d,s,fc)		\
			do{								\
				((d)->S[s].CR) &= ~(1<<5);		\
				((d)->S[s].CR) |= (fc<<5);		\
			}while(0)


#endif /* DMA_H_ */
