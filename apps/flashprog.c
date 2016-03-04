/*
/* flashprog - Programmer for flash on Digital Devices Octopus 
 *
 * Copyright (C) 2010-2011 Digital Devices GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 only, as published by the Free Software Foundation.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA
 * Or, point your browser to http://www.gnu.org/copyleft/gpl.html
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/types.h>

#define DDB_MAGIC 'd'

struct ddb_flashio {
	__u8 *write_buf;
	__u32 write_len;
	__u8 *read_buf;
	__u32 read_len;
};

#define IOCTL_DDB_FLASHIO  _IOWR(DDB_MAGIC, 0x00, struct ddb_flashio)


int flashio(int ddb, uint8_t *wbuf, uint32_t wlen, uint8_t *rbuf, uint32_t rlen)
{
	struct ddb_flashio fio = {
		.write_buf=wbuf,
		.write_len=wlen,
		.read_buf=rbuf,
		.read_len=rlen,
	};
	
	return ioctl(ddb, IOCTL_DDB_FLASHIO, &fio);
}

enum {
	UNKNOWN_FLASH = 0,
	ATMEL_AT45DB642D = 1,
	SSTI_SST25VF016B = 2,
	SSTI_SST25VF032B = 3,
};


int flashread(int ddb, uint8_t *buf, uint32_t addr, uint32_t len)
{
	uint8_t cmd[4]={0x03, (addr>>24)&0xff, 
			(addr>>16)&0xff, addr&0xff};
	
	return flashio(ddb, cmd, 4, buf, len);
}

int flashdump(int ddb, uint32_t addr, uint32_t len)
{
	int i;
	uint8_t buf[len];

	flashread(ddb, buf, addr, len);

	for (i=0; i<len; i++) {
		printf("%02x ", buf[i]);
	}
	printf("\n");
}


int FlashDetect(int dev)
{
	uint8_t Cmd = 0x9F;
	uint8_t Id[3];
	
	int r = flashio(dev, &Cmd,1,Id,3);
	if( r < 0 ) return r;
	
	if( Id[0] == 0xBF && Id[1] == 0x25 && Id[2] == 0x41 ) r = SSTI_SST25VF016B; 
	else if( Id[0] == 0xBF && Id[1] == 0x25 && Id[2] == 0x4A ) r = SSTI_SST25VF032B; 
	else if( Id[0] == 0x1F && Id[1] == 0x28 ) r = ATMEL_AT45DB642D; 
	else r = UNKNOWN_FLASH;
	
	switch(r)
	{
        case UNKNOWN_FLASH : printf("Unknown Flash Flash ID = %02x %02x %02x\n",Id[0],Id[1],Id[2]); break;
        case ATMEL_AT45DB642D : printf("Flash: Atmel AT45DB642D  64 MBit\n"); break;
        case SSTI_SST25VF016B : printf("Flash: SSTI  SST25VF016B 16 MBit\n"); break;
        case SSTI_SST25VF032B : printf("Flash: SSTI  SST25VF032B 32 MBit\n"); break;
	}
	
	return r;
}


int FlashWriteAtmel(int dev,uint32_t FlashOffset, uint8_t *Buffer,int BufferSize)
{
    int err = 0;
    int BlockErase = BufferSize >= 8192;
    int i;
    
    if( BlockErase )
    {
        for(i = 0; i < BufferSize; i += 8192 )
        {
		uint8_t Cmd[4];
		if( (i & 0xFFFF) == 0 )
		{
			printf(" Erase    %08x\n",FlashOffset + i);
		}
		Cmd[0] = 0x50; // Block Erase
		Cmd[1] = ( (( FlashOffset + i ) >> 16) & 0xFF );
		Cmd[2] = ( (( FlashOffset + i ) >>  8) & 0xFF );
		Cmd[3] = 0x00;
		err = flashio(dev,Cmd,4,NULL,0);
		if( err < 0 ) break;
		
		while( 1 )
		{
			Cmd[0] = 0xD7;  // Read Status register
			err = flashio(dev,Cmd,1,&Cmd[0],1);
			if( err < 0 ) break;
			if( (Cmd[0] & 0x80) == 0x80 ) break;
		}
        }
    }
    
    for(i = 0; i < BufferSize; i += 1024 )
    {
        uint8_t Cmd[4 + 1024];
        if( (i & 0xFFFF) == 0 )
        {
            printf(" Programm %08x\n",FlashOffset + i);
        }
        Cmd[0] = 0x84; // Buffer 1
        Cmd[1] = 0x00;
        Cmd[2] = 0x00;
        Cmd[3] = 0x00;
        memcpy(&Cmd[4],&Buffer[i],1024);

        err = flashio(dev,Cmd,4 + 1024,NULL,0);
        if( err < 0 ) break;

        Cmd[0] = BlockErase ? 0x88 : 0x83; // Buffer to Main Memory (with Erase)
        Cmd[1] = ( (( FlashOffset + i ) >> 16) & 0xFF );
        Cmd[2] = ( (( FlashOffset + i ) >>  8) & 0xFF );
        Cmd[3] = 0x00;

        err = flashio(dev,Cmd,4,NULL,0);
        if( err < 0 ) break;

        while( 1 )
        {
		Cmd[0] = 0xD7;  // Read Status register
		err = flashio(dev,Cmd,1,&Cmd[0],1);
            if( err < 0 ) break;
            if( (Cmd[0] & 0x80) == 0x80 ) break;
        }
        if( err < 0 ) break;
    }
    return err;
}

int FlashWriteSSTI(int dev,uint32_t FlashOffset,uint8_t *Buffer,int BufferSize)
{
    int err = 0;
    uint8_t Cmd[6];
    int i, j;

    // Must be multiple of sector size
    if( (BufferSize % 4096) != 0 ) 
	    return -1;   
    
    do {
	    Cmd[0] = 0x50;  // EWSR
	    err = flashio(dev,Cmd,1,NULL,0);
	    if( err < 0 ) 
		    break;

	    Cmd[0] = 0x01;  // WRSR
	    Cmd[1] = 0x00;  // BPx = 0, Unlock all blocks
	    err = flashio(dev,Cmd,2,NULL,0);
	    if( err < 0 )
		    break;
	    
	    for(i = 0; i < BufferSize; i += 4096 ) {
		    if( (i & 0xFFFF) == 0 )
			    printf(" Erase    %08x\n",FlashOffset + i);
		    Cmd[0] = 0x06;  // WREN
		    err = flashio(dev,Cmd,1,NULL,0);
		    if( err < 0 )
			    break;
		    
		    Cmd[0] = 0x20;  // Sector erase ( 4Kb)
		    Cmd[1] = ( (( FlashOffset + i ) >> 16) & 0xFF );
		    Cmd[2] = ( (( FlashOffset + i ) >>  8) & 0xFF );
		    Cmd[3] = 0x00;
		    err = flashio(dev,Cmd,4,NULL,0);
		    if( err < 0 )
			    break;
		    
		    while(1) {
			    Cmd[0] = 0x05;  // RDRS
			    err = flashio(dev,Cmd,1,&Cmd[0],1);
			    if( err < 0 ) break;
			    if( (Cmd[0] & 0x01) == 0 ) break;
		    }
		    if( err < 0 ) break;
	    }
	    if( err < 0 ) 
		    break;
	    for(j = BufferSize - 4096; j >= 0; j -= 4096 ) {
		    if( (j & 0xFFFF) == 0 )
			    printf(" Programm %08x\n",FlashOffset + j);
		    
		    for(i = 0; i < 4096; i += 2 ) {
			    if( i == 0 ) {
				    Cmd[0] = 0x06;  // WREN
				    err = flashio(dev,Cmd,1,NULL,0);
				    if( err < 0 ) 
					    break;
				    
				    Cmd[0] = 0xAD;  // AAI
				    Cmd[1] = ( (( FlashOffset + j ) >> 16) & 0xFF );
				    Cmd[2] = ( (( FlashOffset + j ) >>  8) & 0xFF );
				    Cmd[3] = 0x00;
				    Cmd[4] = Buffer[j+i];
				    Cmd[5] = Buffer[j+i+1];
				    err = flashio(dev,Cmd,6,NULL,0);
			    } else {
				    Cmd[0] = 0xAD;  // AAI
				    Cmd[1] = Buffer[j+i];
				    Cmd[2] = Buffer[j+i+1];
				    err = flashio(dev,Cmd,3,NULL,0);
			    }
			    if( err < 0 ) 
				    break;
			    
			    while(1) {
				    Cmd[0] = 0x05;  // RDRS
				    err = flashio(dev,Cmd,1,&Cmd[0],1);
				    if( err < 0 ) break;
				    if( (Cmd[0] & 0x01) == 0 ) break;
			    }
			    if( err < 0 ) break;
		    }
		    if( err < 0 ) break;
		    
		    Cmd[0] = 0x04;  // WDIS
		    err = flashio(dev,Cmd,1,NULL,0);
		    if( err < 0 ) break;
		    
	    }
	    if( err < 0 ) break;
	    
	    Cmd[0] = 0x50;  // EWSR
	    err = flashio(dev,Cmd,1,NULL,0);
	    if( err < 0 ) break;
	    
	    Cmd[0] = 0x01;  // WRSR
	    Cmd[1] = 0x1C;  // BPx = 0, Lock all blocks
	    err = flashio(dev,Cmd,2,NULL,0);
    } while(0);
    return err;
}


int main(int argc, char *argv[])
{

	uint8_t *Buffer;
	int BufferSize = 0;
	int BlockErase = 0;
	uint32_t FlashOffset = 0x10000;
	int ddb;
	int i, err;
	int SectorSize=0;
	int FlashSize=0;
	int Flash;
	uint8_t id[4];


	ddb=open("/dev/ddbridge/card0", O_RDWR);

	if (ddb < 0) {
		printf("Could not open device\n");
		return -1;
	}

	Flash=FlashDetect(ddb);

	printf("Flash=%d\n", Flash);

	switch(Flash) {
        case ATMEL_AT45DB642D: 
		SectorSize = 1024; 
		FlashSize = 0x800000; 
		break;
        case SSTI_SST25VF016B: 
		SectorSize = 4096; 
		FlashSize = 0x200000; 
		break;
        case SSTI_SST25VF032B: 
		SectorSize = 4096; 
		FlashSize = 0x400000; 
		break;
	}


	flashread(ddb, id, 0, 4);
	printf("%02x %02x %02x %02x\n", 
	       id[0], id[1], id[2], id[3]);


	if( SectorSize == 0 ) return 0;
	
	if( argc < 2 )
		return -1;

	if (strncmp("-SubVendorID",argv[1],12) == 0 )
	{
		uint32_t SubVendorID;

		printf("SubVendorID\n");

		if( argc < 3 ) return -1;

		SubVendorID = strtoul(argv[2],NULL,16);
		
		BufferSize = SectorSize;
		FlashOffset = 0;
		
		Buffer = malloc(BufferSize);
		if( Buffer == NULL )
		{
			printf("out of memory\n");
			return 0;
		}
		memset(Buffer,0xFF,BufferSize);
		
		Buffer[0] = ( ( SubVendorID >> 24 ) & 0xFF );
		Buffer[1] = ( ( SubVendorID >> 16 ) & 0xFF );
		Buffer[2] = ( ( SubVendorID >>  8 ) & 0xFF );
		Buffer[3] = ( ( SubVendorID       ) & 0xFF );
		
	} else if (strncmp("-Jump",argv[1],5) == 0 ) {
		uint32_t Jump;
		if( argc < 3 ) return -1;

		Jump = strtoul(argv[2],NULL,16);
		
		BufferSize = SectorSize;
		FlashOffset = FlashSize-SectorSize;
		
		Buffer = malloc(BufferSize);
		if (!Buffer) {
			printf("out of memory\n");
			return 0;
		}
		memset(Buffer,0xFF,BufferSize);
		memset(&Buffer[BufferSize - 256 + 0x10],0x00,16);
		
		Buffer[BufferSize - 256 + 0x10] = 0xbd;
		Buffer[BufferSize - 256 + 0x11] = 0xb3;
		Buffer[BufferSize - 256 + 0x12] = 0xc4;
		Buffer[BufferSize - 256 + 0x1a] = 0xfe;
		Buffer[BufferSize - 256 + 0x1e] = 0x03;
		Buffer[BufferSize - 256 + 0x1f] = ( ( Jump >> 16 ) & 0xFF );
		Buffer[BufferSize - 256 + 0x20] = ( ( Jump >>  8 ) & 0xFF );
		Buffer[BufferSize - 256 + 0x21] = ( ( Jump       ) & 0xFF );
		
	} else {
		int fh, i;
		int fsize;

		if (argc > 2)
			FlashOffset = strtoul(argv[2],NULL,16);
		
		fh = open(argv[1],O_RDONLY);

		if (fh < 0 ) {
			printf("File not found \n");
			return 0;
		}
		
		fsize = lseek(fh,0,SEEK_END);
		
		if( fsize > 4000000 || fsize < SectorSize )
		{
			close(fh);
			printf("Invalid File Size \n");
			return 0;
		}

		if( Flash == ATMEL_AT45DB642D ) {
			BlockErase = fsize >= 8192;
			if( BlockErase )
				BufferSize = (fsize + 8191) & ~8191;
			else
				BufferSize = (fsize + 1023) & ~1023;
		}
		else
		{
			BufferSize = (fsize + SectorSize - 1 ) & ~(SectorSize - 1);
		}
		printf(" Size     %08x\n",BufferSize);
		
		Buffer = malloc(BufferSize);

		if( Buffer == NULL ) {
			close(fh);
			printf("out of memory\n");
			return 0;
		}
	
		memset(Buffer, 0xFF, BufferSize);
		lseek(fh, 0, SEEK_SET);
		read(fh, Buffer, fsize);
		close(fh);
		
		if (BufferSize >= 0x10000) {
			// Clear header
			for(i = 0; i < 0x200; i += 1 ) {
				if ( *(uint16_t *) (&Buffer[i]) == 0xFFFF )
					break;
				Buffer[i] = 0xFF;
			}
		}
	}
	
	switch(Flash) {
        case ATMEL_AT45DB642D: 
		err = FlashWriteAtmel(ddb, FlashOffset, Buffer, BufferSize); 
		break;
        case SSTI_SST25VF016B: 
        case SSTI_SST25VF032B: 
		err = FlashWriteSSTI(ddb, FlashOffset, Buffer, BufferSize); 
		break;
	}
	
	if (err < 0) 
		printf("Program Error\n");
	else
		printf("Program Done\n");
	
	free(Buffer);
	return 0;
}
