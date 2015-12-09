/*
 * z64
 *
 * This program is free software; you can redistribute it and/
 * or modify it under the terms of the GNU General Public Li-
 * cence as published by the Free Software Foundation; either
 * version 2 of the Licence, or any later version.
 *
 * This program is distributed in the hope that it will be use-
 * ful, but WITHOUT ANY WARRANTY; without even the implied war-
 * ranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public Licence for more details.
 *
 * You should have received a copy of the GNU General Public
 * Licence along with this program; if not, write to the Free
 * Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139,
 * USA.
 *
**/

#include "Gfx #1.3.h"
#include "rdp.h"
#include "rgl.h"

#include <string.h>

rdpState_t rdpState;
uint32_t   rdpChanged;
//rdpColor_t rdpTlut[1024];
uint8_t    rdpTmem[0x1000];
int        rdpFbFormat;
int        rdpFbSize;
int        rdpFbWidth;
uint32_t   rdpFbAddress;
uint32_t   rdpZbAddress;
int        rdpTiFormat;
int        rdpTiSize;
int        rdpTiWidth;
uint32_t   rdpTiAddress;
rdpTile_t  rdpTiles[8];
int        rdpTileSet;

struct area_t {
  int start, stop;
  uint32_t from;
  int fromLine, fromFormat, fromSize;
};

#define MAX_TMEM_AREAS 16
static area_t tmemAreas[MAX_TMEM_AREAS];
static int nbTmemAreas;

#ifdef RDP_DEBUG
int rdp_dump;
#endif

#define MAXCMD 0x100000
static uint32_t rdp_cmd_data[MAXCMD+32];
static volatile int rdp_cmd_ptr = 0;
static volatile int rdp_cmd_cur = 0;
static int rdp_cmd_left = 0;

#ifdef RDP_DEBUG
uint32_t rdpTraceBuf[0x100000];
int rdpTracePos;
#endif


static void MarkTmemArea(int start, int stop, uint32_t from, uint32_t fromLine,
                         int fromFormat, int fromSize)
{
  int i;

  // remove areas that intersect
  for (i=0; i<nbTmemAreas; i++)
    while (i<nbTmemAreas &&
           tmemAreas[i].start<stop && tmemAreas[i].stop>start) {
      memmove(tmemAreas+i, tmemAreas+i+1, nbTmemAreas-i-1);
      nbTmemAreas--;
    }

  DUMP("marking tmem %x --> %x rdram %x\n", start, stop, from);

  // add new area
  //rglAssert(nbTmemAreas < MAX_TMEM_AREAS);
  if (nbTmemAreas == MAX_TMEM_AREAS) {
    LOG("tmem areas buffer full, clearing\n");
    nbTmemAreas = 0;
  }
  tmemAreas[nbTmemAreas].start = start;
  tmemAreas[nbTmemAreas].stop = stop;
  tmemAreas[nbTmemAreas].from = from;
  tmemAreas[nbTmemAreas].fromLine = fromLine;
  tmemAreas[nbTmemAreas].fromFormat = fromFormat;
  tmemAreas[nbTmemAreas].fromSize = fromSize;
  nbTmemAreas++;
}

uint32_t rdpGetTmemOrigin(int tmem, int * line, int * stop, int * format, int * size)
{
  int i;
  for (i=0; i<nbTmemAreas; i++)
    if (tmemAreas[i].start == tmem) {
      *line = tmemAreas[i].fromLine;
      *stop = tmemAreas[i].stop;
      *format = tmemAreas[i].fromFormat;
      *size = tmemAreas[i].fromSize;
      return tmemAreas[i].from;
    }

  return ~0;
}

inline uint32_t READ_RDP_DATA(uint32_t address)
{
	if (dp_status & 0x1)		// XBUS_DMEM_DMA enabled
	{
		return rsp_dmem[(address & 0xfff) / 4];
	}
	else
	{
		return rdram[(address / 4)];
	}
}

static const int rdp_command_length[64] =
{
	8,			// 0x00, No Op
	8,			// 0x01, ???
	8,			// 0x02, ???
	8,			// 0x03, ???
	8,			// 0x04, ???
	8,			// 0x05, ???
	8,			// 0x06, ???
	8,			// 0x07, ???
	32,			// 0x08, Non-Shaded Triangle
	32+16,		// 0x09, Non-Shaded, Z-Buffered Triangle
	32+64,		// 0x0a, Textured Triangle
	32+64+16,	// 0x0b, Textured, Z-Buffered Triangle
	32+64,		// 0x0c, Shaded Triangle
	32+64+16,	// 0x0d, Shaded, Z-Buffered Triangle
	32+64+64,	// 0x0e, Shaded+Textured Triangle
	32+64+64+16,// 0x0f, Shaded+Textured, Z-Buffered Triangle
	8,			// 0x10, ???
	8,			// 0x11, ???
	8,			// 0x12, ???
	8,			// 0x13, ???
	8,			// 0x14, ???
	8,			// 0x15, ???
	8,			// 0x16, ???
	8,			// 0x17, ???
	8,			// 0x18, ???
	8,			// 0x19, ???
	8,			// 0x1a, ???
	8,			// 0x1b, ???
	8,			// 0x1c, ???
	8,			// 0x1d, ???
	8,			// 0x1e, ???
	8,			// 0x1f, ???
	8,			// 0x20, ???
	8,			// 0x21, ???
	8,			// 0x22, ???
	8,			// 0x23, ???
	16,			// 0x24, Texture_Rectangle
	16,			// 0x25, Texture_Rectangle_Flip
	8,			// 0x26, Sync_Load
	8,			// 0x27, Sync_Pipe
	8,			// 0x28, Sync_Tile
	8,			// 0x29, Sync_Full
	8,			// 0x2a, Set_Key_GB
	8,			// 0x2b, Set_Key_R
	8,			// 0x2c, Set_Convert
	8,			// 0x2d, Set_Scissor
	8,			// 0x2e, Set_Prim_Depth
	8,			// 0x2f, Set_Other_Modes
	8,			// 0x30, Load_TLUT
	8,			// 0x31, ???
	8,			// 0x32, Set_Tile_Size
	8,			// 0x33, Load_Block
	8,			// 0x34, Load_Tile
	8,			// 0x35, Set_Tile
	8,			// 0x36, Fill_Rectangle
	8,			// 0x37, Set_Fill_Color
	8,			// 0x38, Set_Fog_Color
	8,			// 0x39, Set_Blend_Color
	8,			// 0x3a, Set_Prim_Color
	8,			// 0x3b, Set_Env_Color
	8,			// 0x3c, Set_Combine
	8,			// 0x3d, Set_Texture_Image
	8,			// 0x3e, Set_Mask_Image
	8			// 0x3f, Set_Color_Image
};

/*****************************************************************************/

////////////////////////
// RDP COMMANDS
////////////////////////

static void rdp_invalid(uint32_t w1, uint32_t w2)
{
  LOGERROR("RDP: invalid command  %d, %08X %08X\n", (w1 >> 24) & 0x3f, w1, w2);
}

static void rdp_noop(uint32_t w1, uint32_t w2)
{

}

static void triangle(uint32_t w1, uint32_t w2, int shade, int texture, int zbuffer)
{
  rglTriangle(w1, w2, shade, texture, zbuffer, rdp_cmd_data + rdp_cmd_cur);
}

static void rdp_tri_noshade(uint32_t w1, uint32_t w2)
{
	triangle(w1, w2, 0, 0, 0);
}

static void rdp_tri_noshade_z(uint32_t w1, uint32_t w2)
{
	triangle(w1, w2, 0, 0, 1);
}

static void rdp_tri_tex(uint32_t w1, uint32_t w2)
{
	triangle(w1, w2, 0, 1, 0);
}

static void rdp_tri_tex_z(uint32_t w1, uint32_t w2)
{
	triangle(w1, w2, 0, 1, 1);
}

static void rdp_tri_shade(uint32_t w1, uint32_t w2)
{
	triangle(w1, w2, 1, 0, 0);
}

static void rdp_tri_shade_z(uint32_t w1, uint32_t w2)
{
	triangle(w1, w2, 1, 0, 1);
}

static void rdp_tri_texshade(uint32_t w1, uint32_t w2)
{
	triangle(w1, w2, 1, 1, 0);
}

static void rdp_tri_texshade_z(uint32_t w1, uint32_t w2)
{
	triangle(w1, w2, 1, 1, 1);
}

static void rdp_tex_rect(uint32_t w1, uint32_t w2)
{
	uint32_t w3, w4;
	rdpTexRect_t rect;

	w3 = rdp_cmd_data[rdp_cmd_cur+2];
	w4 = rdp_cmd_data[rdp_cmd_cur+3];

	rect.tilenum	= (w2 >> 24) & 0x7;
	rect.xl			= (w1 >> 12) & 0xfff;
	rect.yl			= (w1 >>  0) & 0xfff;
	rect.xh			= (w2 >> 12) & 0xfff;
	rect.yh			= (w2 >>  0) & 0xfff;
	rect.s			= (w3 >> 16) & 0xffff;
	rect.t			= (w3 >>  0) & 0xffff;
	rect.dsdx		= (w4 >> 16) & 0xffff;
	rect.dtdy		= (w4 >>  0) & 0xffff;

  rglTextureRectangle(&rect, 0);
}

static void rdp_tex_rect_flip(uint32_t w1, uint32_t w2)
{
	uint32_t w3, w4;
	rdpTexRect_t rect;

	w3 = rdp_cmd_data[rdp_cmd_cur+2];
	w4 = rdp_cmd_data[rdp_cmd_cur+3];

	rect.tilenum	= (w2 >> 24) & 0x7;
	rect.xl			= (w1 >> 12) & 0xfff;
	rect.yl			= (w1 >>  0) & 0xfff;
	rect.xh			= (w2 >> 12) & 0xfff;
	rect.yh			= (w2 >>  0) & 0xfff;
	rect.t			= (w3 >> 16) & 0xffff;
	rect.s			= (w3 >>  0) & 0xffff;
	rect.dtdy		= (w4 >> 16) & 0xffff;
	rect.dsdx		= (w4 >>  0) & 0xffff;
  
  rglTextureRectangle(&rect, 1);
}

static void rdp_sync_load(uint32_t w1, uint32_t w2)
{
	// Nothing to do?
}

static void rdp_sync_pipe(uint32_t w1, uint32_t w2)
{
	// Nothing to do?
}

static void rdp_sync_tile(uint32_t w1, uint32_t w2)
{
	// Nothing to do?
}

void rdpSignalFullSync();
void rdpWaitFullSync();
#ifdef RDP_DEBUG
int nbFullSync;
#endif
static void rdp_sync_full(uint32_t w1, uint32_t w2)
{
  //printf("full sync\n");
  rglFullSync();

  if (rglSettings.async)
    rdpSignalFullSync();
  else {
    *gfx.MI_INTR_REG |= 0x20;
    gfx.CheckInterrupts();
  }
#ifdef RDP_DEBUG
  nbFullSync++;
#endif
}

static void rdp_set_key_gb(uint32_t w1, uint32_t w2)
{
	//osd_die("RDP: unhandled command set_key_gb, %08X %08X\n", w1, w2);
}

static void rdp_set_key_r(uint32_t w1, uint32_t w2)
{
	//osd_die("RDP: unhandled command set_key_r, %08X %08X\n", w1, w2);
}

static void rdp_set_convert(uint32_t w1, uint32_t w2)
{
  rdpState.k5 = w2&0xff;
	//osd_die("RDP: unhandled command set_convert, %08X %08X\n", w1, w2);
}

static void rdp_set_scissor(uint32_t w1, uint32_t w2)
{
  rdpChanged |= RDP_BITS_CLIP;
  rdpState.clipMode = (w2 >> 24) & 3;
	rdpState.clip.xh = (w1 >> 12) & 0xfff;
	rdpState.clip.yh = (w1 >>  0) & 0xfff;
	rdpState.clip.xl = (w2 >> 12) & 0xfff;
	rdpState.clip.yl = (w2 >>  0) & 0xfff;
	// TODO: handle f & o?
}

static void rdp_set_prim_depth(uint32_t w1, uint32_t w2)
{
  rdpChanged |= RDP_BITS_MISC;
	rdpState.primitiveZ = (uint16_t)(w2 >> 16);
	rdpState.primitiveDeltaZ = (uint16_t)(w1);
}

static void rdp_set_other_modes(uint32_t w1, uint32_t w2)
{
  rdpChanged |= RDP_BITS_OTHER_MODES;
  rdpState.otherModes.w1 = w1;
  rdpState.otherModes.w2 = w2;
}

static void rdp_load_tlut(uint32_t w1, uint32_t w2)
{
	int tilenum = (w2 >> 24) & 0x7;

  rdpChanged |= RDP_BITS_TILE_SETTINGS;

#define tile rdpTiles[tilenum]
  //rdpTile_t tile;
	tile.sl = (w1 >> 12) & 0xfff;
	tile.tl = (w1 >>  0) & 0xfff;
	tile.sh = (w2 >> 12) & 0xfff;
	tile.th = (w2 >>  0) & 0xfff;

	int i;

  rdpChanged |= RDP_BITS_TLUT;

  int count = ((tile.sh - tile.sl + 4) >>2) * ((tile.th - tile.tl + 4) >>2);

  switch (rdpTiSize)
  {
    case RDP_PIXEL_SIZE_16BIT:
    {
      uint16_t *src = (uint16_t *)&rdram[(rdpTiAddress + (tile.tl >>2) * rdpTiWidth * 2 + ((tile.sl >>2) << rdpTiSize >> 1))/4];
      uint16_t *dst = (uint16_t *)(rdpTmem + rdpTiles[tilenum].tmem);

//       printf("loading TLUT from %x --> %x\n",
//              tile.th * rdpTiWidth / 2 + (tile.sh << rdpTiSize >> 1)/4

      for (i=0; i < count; i++)
      {
        dst[i*4] = src[i^1];
      }
      break;
    }
    default:	LOGERROR("RDP: load_tlut: size = %d\n", rdpTiSize);
  }
#undef tile
}

static void rdp_set_tile_size(uint32_t w1, uint32_t w2)
{
	int tilenum = (w2 >> 24) & 0x7;

  rdpChanged |= RDP_BITS_TILE_SETTINGS;

#define tile rdpTiles[tilenum]
	tile.sl = (w1 >> 12) & 0xfff;
	tile.tl = (w1 >>  0) & 0xfff;
	tile.sh = (w2 >> 12) & 0xfff;
	tile.th = (w2 >>  0) & 0xfff;
#undef tile
}

static void rdp_load_block(uint32_t w1, uint32_t w2)
{
	int i, width;
	uint16_t sl, sh, tl, dxt;
	int tilenum = (w2 >> 24) & 0x7;
	uint16_t *tc;
	int tb;

  rdpChanged |= RDP_BITS_TMEM;

	sl	= ((w1 >> 12) & 0xfff);
	tl	= ((w1 >>  0) & 0xfff);
	sh	= ((w2 >> 12) & 0xfff);
	dxt	= ((w2 >>  0) & 0xfff);

	width = (sh - sl) + 1;

	width = (width << rdpTiSize) >> 1;
	if (width & 7)
		width = (width & ~7) + 8;
	width >>= 3;

	tc = (uint16_t*)rdpTmem;
	tb = rdpTiles[tilenum].tmem << 2;

	const INT32 tiwinwords = (rdpTiWidth << rdpTiSize) >> 2;
	const INT32 slinwords = (sl << rdpTiSize) >> 2;

	const UINT32 src = (rdpTiAddress >> 1) + (tl * tiwinwords) + slinwords;

	if (dxt != 0)
	{
		INT32 j = 0;
		INT32 t = 0;
		INT32 oldt = 0;

		if (rdpTiles[tilenum].size != RDP_PIXEL_SIZE_32BIT && rdpTiles[tilenum].format != RDP_FORMAT_YUV)
		{
			for (INT32 i = 0; i < width; i++)
			{
				oldt = t;
				t = ((j >> 11) & 1) ? WORD_XOR_DWORD_SWAP : WORD_ADDR_XOR;
				if (t != oldt)
				{
					i += rdpTiles[tilenum].line;
				}

				INT32 ptr = tb + (i << 2);
				INT32 srcptr = src + (i << 2);

				tc[(ptr ^ t) & 0x7ff] = U_RREADIDX16(srcptr);
				tc[((ptr + 1) ^ t) & 0x7ff] = U_RREADIDX16(srcptr + 1);
				tc[((ptr + 2) ^ t) & 0x7ff] = U_RREADIDX16(srcptr + 2);
				tc[((ptr + 3) ^ t) & 0x7ff] = U_RREADIDX16(srcptr + 3);
				j += dxt;
			}
		}
		else if (rdpTiles[tilenum].format == RDP_FORMAT_YUV)
		{
			for (INT32 i = 0; i < width; i++)
			{
				oldt = t;
				t = ((j >> 11) & 1) ? WORD_XOR_DWORD_SWAP : WORD_ADDR_XOR;
				if (t != oldt)
				{
					i += rdpTiles[tilenum].line;
				}

				INT32 ptr = ((tb + (i << 1)) ^ t) & 0x3ff;
				INT32 srcptr = src + (i << 2);

				INT32 first = U_RREADIDX16(srcptr);
				INT32 sec = U_RREADIDX16(srcptr + 1);
				tc[ptr] = ((first >> 8) << 8) | (sec >> 8);
				tc[ptr | 0x400] = ((first & 0xff) << 8) | (sec & 0xff);

				ptr = ((tb + (i << 1) + 1) ^ t) & 0x3ff;
				first = U_RREADIDX16(srcptr + 2);
				sec = U_RREADIDX16(srcptr + 3);
				tc[ptr] = ((first >> 8) << 8) | (sec >> 8);
				tc[ptr | 0x400] = ((first & 0xff) << 8) | (sec & 0xff);

				j += dxt;
			}
		}
		else
		{
			for (INT32 i = 0; i < width; i++)
			{
				oldt = t;
				t = ((j >> 11) & 1) ? WORD_XOR_DWORD_SWAP : WORD_ADDR_XOR;
				if (t != oldt)
					i += rdpTiles[tilenum].line;

				INT32 ptr = ((tb + (i << 1)) ^ t) & 0x3ff;
				INT32 srcptr = src + (i << 2);
				tc[ptr] = U_RREADIDX16(srcptr);
				tc[ptr | 0x400] = U_RREADIDX16(srcptr + 1);

				ptr = ((tb + (i << 1) + 1) ^ t) & 0x3ff;
				tc[ptr] = U_RREADIDX16(srcptr + 2);
				tc[ptr | 0x400] = U_RREADIDX16(srcptr + 3);

				j += dxt;
			}
		}
		rdpTiles[tilenum].th = tl + (j >> 11);
	}
	else
	{
		if (rdpTiles[tilenum].size != RDP_PIXEL_SIZE_32BIT && rdpTiles[tilenum].format != RDP_FORMAT_YUV)
		{
			for (INT32 i = 0; i < width; i++)
			{
				INT32 ptr = tb + (i << 2);
				INT32 srcptr = src + (i << 2);
				tc[(ptr ^ WORD_ADDR_XOR) & 0x7ff] = U_RREADIDX16(srcptr);
				tc[((ptr + 1) ^ WORD_ADDR_XOR) & 0x7ff] = U_RREADIDX16(srcptr + 1);
				tc[((ptr + 2) ^ WORD_ADDR_XOR) & 0x7ff] = U_RREADIDX16(srcptr + 2);
				tc[((ptr + 3) ^ WORD_ADDR_XOR) & 0x7ff] = U_RREADIDX16(srcptr + 3);
			}
		}
		else if (rdpTiles[tilenum].format == RDP_FORMAT_YUV)
		{
			for (INT32 i = 0; i < width; i++)
			{
				INT32 ptr = ((tb + (i << 1)) ^ WORD_ADDR_XOR) & 0x3ff;
				INT32 srcptr = src + (i << 2);
				INT32 first = U_RREADIDX16(srcptr);
				INT32 sec = U_RREADIDX16(srcptr + 1);
				tc[ptr] = ((first >> 8) << 8) | (sec >> 8);//UV pair
				tc[ptr | 0x400] = ((first & 0xff) << 8) | (sec & 0xff);

				ptr = ((tb + (i << 1) + 1) ^ WORD_ADDR_XOR) & 0x3ff;
				first = U_RREADIDX16(srcptr + 2);
				sec = U_RREADIDX16(srcptr + 3);
				tc[ptr] = ((first >> 8) << 8) | (sec >> 8);
				tc[ptr | 0x400] = ((first & 0xff) << 8) | (sec & 0xff);
			}
		}
		else
		{
			for (INT32 i = 0; i < width; i++)
			{
				INT32 ptr = ((tb + (i << 1)) ^ WORD_ADDR_XOR) & 0x3ff;
				INT32 srcptr = src + (i << 2);
				tc[ptr] = U_RREADIDX16(srcptr);
				tc[ptr | 0x400] = U_RREADIDX16(srcptr + 1);

				ptr = ((tb + (i << 1) + 1) ^ WORD_ADDR_XOR) & 0x3ff;
				tc[ptr] = U_RREADIDX16(srcptr + 2);
				tc[ptr | 0x400] = U_RREADIDX16(srcptr + 3);
			}
		}
		rdpTiles[tilenum].th = tl;
	}

	//rdpTiles[tilenum].sth = rgbaint_t(m_tiles[tilenum].sh, m_tiles[tilenum].sh, m_tiles[tilenum].th, m_tiles[tilenum].th);
	//rdpTiles[tilenum].stl = rgbaint_t(m_tiles[tilenum].sl, m_tiles[tilenum].sl, m_tiles[tilenum].tl, m_tiles[tilenum].tl);
}

static void rdp_load_tile(uint32_t w1, uint32_t w2)
{
	const INT32 tilenum = (w2 >> 24) & 0x7;

	rdpTiles[tilenum].sl = ((w1 >> 12) & 0xfff);
	rdpTiles[tilenum].tl = ((w1 >> 0) & 0xfff);
	rdpTiles[tilenum].sh = ((w2 >> 12) & 0xfff);
	rdpTiles[tilenum].th = ((w2 >> 0) & 0xfff);

	const INT32 sl = rdpTiles[tilenum].sl >> 2;
	const INT32 tl = rdpTiles[tilenum].tl >> 2;
	const INT32 sh = rdpTiles[tilenum].sh >> 2;
	const INT32 th = rdpTiles[tilenum].th >> 2;

	const INT32 width = (sh - sl) + 1;
	const INT32 height = (th - tl) + 1;
	/*
	INT32 topad;
	if (m_misc_state.m_ti_size < 3)
	{
	topad = (width * m_misc_state.m_ti_size) & 0x7;
	}
	else
	{
	topad = (width << 2) & 0x7;
	}
	topad = 0; // ????
	*/

	switch (rdpTiSize)
	{
	case RDP_PIXEL_SIZE_8BIT:
	{
		const UINT32 src = rdpTiAddress;
		const INT32 tb = rdpTiles[tilenum].tmem << 3;
		UINT8* tc = (uint8_t*)rdpTmem;

		for (INT32 j = 0; j < height; j++)
		{
			const INT32 tline = tb + ((rdpTiles[tilenum].line << 3) * j);
			const INT32 s = ((j + tl) * rdpTiWidth) + sl;
			const INT32 xorval8 = ((j & 1) ? BYTE_XOR_DWORD_SWAP : BYTE_ADDR_XOR);

			for (INT32 i = 0; i < width; i++)
			{
				tc[((tline + i) ^ xorval8) & 0xfff] = U_RREADADDR8(src + s + i);
			}
		}
		break;
	}
	case RDP_PIXEL_SIZE_16BIT:
	{
		const UINT32 src = rdpTiAddress >> 1;
		UINT16* tc = (uint16_t*)rdpTmem;

		if (rdpTiles[tilenum].format != RDP_FORMAT_YUV)
		{
			for (INT32 j = 0; j < height; j++)
			{
				const INT32 tb = rdpTiles[tilenum].tmem << 2;
				const INT32 tline = tb + ((rdpTiles[tilenum].line << 2) * j);
				const INT32 s = ((j + tl) * rdpTiWidth) + sl;
				const INT32 xorval16 = (j & 1) ? WORD_XOR_DWORD_SWAP : WORD_ADDR_XOR;

				for (INT32 i = 0; i < width; i++)
				{
					UINT32 taddr = (tline + i) ^ xorval16;
					tc[taddr & 0x7ff] = U_RREADIDX16(src + s + i);
				}
			}
		}
		else
		{
			for (INT32 j = 0; j < height; j++)
			{
				const INT32 tb = rdpTiles[tilenum].tmem << 3;
				const INT32 tline = tb + ((rdpTiles[tilenum].line << 3) * j);
				const INT32 s = ((j + tl) * rdpTiWidth) + sl;
				const INT32 xorval8 = (j & 1) ? BYTE_XOR_DWORD_SWAP : BYTE_ADDR_XOR;
				UINT8* tc8 = (uint8_t*)rdpTmem;

				for (INT32 i = 0; i < width; i++)
				{
					UINT32 taddr = ((tline + i) ^ xorval8) & 0x7ff;
					UINT16 yuvword = U_RREADIDX16(src + s + i);
					tc8[taddr] = yuvword >> 8;
					tc8[taddr | 0x800] = yuvword & 0xff;
				}
			}
		}
		break;
	}
	case RDP_PIXEL_SIZE_32BIT:
	{
		const UINT32 src = rdpTiAddress >> 2;
		const INT32 tb = (rdpTiles[tilenum].tmem << 2);
		UINT16* tc16 = (uint16_t*)rdpTmem;

		for (INT32 j = 0; j < height; j++)
		{
			const INT32 tline = tb + ((rdpTiles[tilenum].line << 2) * j);

			const INT32 s = ((j + tl) * rdpTiWidth) + sl;
			const INT32 xorval32cur = (j & 1) ? WORD_XOR_DWORD_SWAP : WORD_ADDR_XOR;
			for (INT32 i = 0; i < width; i++)
			{
				UINT32 c = U_RREADIDX32(src + s + i);
				UINT32 ptr = ((tline + i) ^ xorval32cur) & 0x3ff;
				tc16[ptr] = c >> 16;
				tc16[ptr | 0x400] = c & 0xffff;
			}
		}
		break;
	}

	default:   	FATAL("RDP: load_tile: size = %d\n", rdpTiSize);
	}

	//m_tiles[tilenum].sth = rgbaint_t(m_tiles[tilenum].sh, m_tiles[tilenum].sh, m_tiles[tilenum].th, m_tiles[tilenum].th);
	//m_tiles[tilenum].stl = rgbaint_t(m_tiles[tilenum].sl, m_tiles[tilenum].sl, m_tiles[tilenum].tl, m_tiles[tilenum].tl);
}

static void rdp_set_tile(uint32_t w1, uint32_t w2)
{
	int tilenum = (w2 >> 24) & 0x7;
  //int i;

  rdpChanged |= RDP_BITS_TILE_SETTINGS;
  rdpTileSet |= 1<<tilenum;

#define tile rdpTiles[tilenum]
	tile.format	= (w1 >> 21) & 0x7;
	tile.size		= (w1 >> 19) & 0x3;
	tile.line		= ((w1 >>  9) & 0x1ff) * 8;
	tile.tmem		= ((w1 >>  0) & 0x1ff) * 8;
	tile.palette= (w2 >> 20) & 0xf;
	tile.ct			= (w2 >> 19) & 0x1;
	tile.mt			= (w2 >> 18) & 0x1;
	tile.mask_t	= (w2 >> 14) & 0xf;
	tile.shift_t= (w2 >> 10) & 0xf;
  if (tile.shift_t >= 12) tile.shift_t -= 16;
	tile.cs			= (w2 >>  9) & 0x1;
	tile.ms			= (w2 >>  8) & 0x1;
	tile.mask_s	= (w2 >>  4) & 0xf;
	tile.shift_s= (w2 >>  0) & 0xf;
  if (tile.shift_s >= 12) tile.shift_s -= 16;
#undef tile
}

static void rdp_fill_rect(uint32_t w1, uint32_t w2)
{
	rdpRect_t rect;
	rect.xl = (w1 >> 12) & 0xfff;
	rect.yl = (w1 >>  0) & 0xfff;
	rect.xh = (w2 >> 12) & 0xfff;
	rect.yh = (w2 >>  0) & 0xfff;

  rglFillRectangle(&rect);
}

static void rdp_set_fill_color(uint32_t w1, uint32_t w2)
{
  rdpChanged |= RDP_BITS_FILL_COLOR;
	rdpState.fillColor = w2;
}

static void rdp_set_fog_color(uint32_t w1, uint32_t w2)
{
  rdpChanged |= RDP_BITS_FOG_COLOR;
	rdpState.fogColor = w2;
}

static void rdp_set_blend_color(uint32_t w1, uint32_t w2)
{
  rdpChanged |= RDP_BITS_BLEND_COLOR;
	rdpState.blendColor = w2;
}

static void rdp_set_prim_color(uint32_t w1, uint32_t w2)
{
  rdpChanged |= RDP_BITS_PRIM_COLOR;
	// TODO: prim min level, prim_level
	rdpState.primColor = w2;
}

static void rdp_set_env_color(uint32_t w1, uint32_t w2)
{
  rdpChanged |= RDP_BITS_ENV_COLOR;
	rdpState.envColor = w2;
}

static void rdp_set_combine(uint32_t w1, uint32_t w2)
{
  rdpChanged |= RDP_BITS_COMBINE_MODES;

  rdpState.combineModes.w1 = w1;
  rdpState.combineModes.w2 = w2;
}

static void rdp_set_texture_image(uint32_t w1, uint32_t w2)
{
  rdpChanged |= RDP_BITS_TI_SETTINGS;

  rdpTiFormat	= (w1 >> 21) & 0x7;
	rdpTiSize		= (w1 >> 19) & 0x3;
	rdpTiWidth	= (w1 & 0x3ff) + 1;
	rdpTiAddress	= w2 & 0x0ffffff;
}

static void rdp_set_mask_image(uint32_t w1, uint32_t w2)
{
  rdpChanged |= RDP_BITS_ZB_SETTINGS;
	rdpZbAddress	= w2 & 0x0ffffff;
}

static void rdp_set_color_image(uint32_t w1, uint32_t w2)
{
  rdpChanged |= RDP_BITS_FB_SETTINGS;
	rdpFbFormat 	= (w1 >> 21) & 0x7;
	rdpFbSize		= (w1 >> 19) & 0x3;
	rdpFbWidth	= (w1 & 0x3ff) + 1;
	rdpFbAddress	= w2 & 0x0ffffff;
}

/*****************************************************************************/

static void (* rdp_command_table[64])(uint32_t w1, uint32_t w2) =
{
	/* 0x00 */
	rdp_noop,			rdp_invalid,			rdp_invalid,			rdp_invalid,
	rdp_invalid,		rdp_invalid,			rdp_invalid,			rdp_invalid,
	rdp_tri_noshade,	rdp_tri_noshade_z,		rdp_tri_tex,			rdp_tri_tex_z,
	rdp_tri_shade,		rdp_tri_shade_z,		rdp_tri_texshade,		rdp_tri_texshade_z,
	/* 0x10 */
	rdp_invalid,		rdp_invalid,			rdp_invalid,			rdp_invalid,
	rdp_invalid,		rdp_invalid,			rdp_invalid,			rdp_invalid,
	rdp_invalid,		rdp_invalid,			rdp_invalid,			rdp_invalid,
	rdp_invalid,		rdp_invalid,			rdp_invalid,			rdp_invalid,
	/* 0x20 */
	rdp_invalid,		rdp_invalid,			rdp_invalid,			rdp_invalid,
	rdp_tex_rect,		rdp_tex_rect_flip,		rdp_sync_load,			rdp_sync_pipe,
	rdp_sync_tile,		rdp_sync_full,			rdp_set_key_gb,			rdp_set_key_r,
	rdp_set_convert,	rdp_set_scissor,		rdp_set_prim_depth,		rdp_set_other_modes,
	/* 0x30 */
	rdp_load_tlut,		rdp_invalid,			rdp_set_tile_size,		rdp_load_block,
	rdp_load_tile,		rdp_set_tile,			rdp_fill_rect,			rdp_set_fill_color,
	rdp_set_fog_color,	rdp_set_blend_color,	rdp_set_prim_color,		rdp_set_env_color,
	rdp_set_combine,	rdp_set_texture_image,	rdp_set_mask_image,		rdp_set_color_image
};

void rdp_process_list(void)
{
	//int i;
	uint32_t cmd;//, length, cmd_length;

  rglUpdateStatus();
  if (!rglSettings.threaded)
    rdp_store_list();
  
  if (rglStatus == RGL_STATUS_CLOSED)
    return;

	while (rdp_cmd_cur != rdp_cmd_ptr)
	{
		cmd = (rdp_cmd_data[rdp_cmd_cur] >> 24) & 0x3f;
	//  if (((rdp_cmd_data[rdp_cmd_cur] >> 24) & 0xc0) != 0xc0)
	//  {
	//      LOGERROR("rdp_process_list: invalid rdp command %08X at %08X\n", rdp_cmd_data[rdp_cmd_cur], dp_start+(rdp_cmd_cur * 4));
	//  }

		if ((((rdp_cmd_ptr-rdp_cmd_cur)&(MAXCMD-1)) * 4) < rdp_command_length[cmd])
		{
// 			LOGERROR("rdp_process_list: not enough rdp command data: cur = %d, ptr = %d, expected = %d\n", rdp_cmd_cur, rdp_cmd_ptr, rdp_command_length[cmd]);
// 			return;
      break;
		}

#ifdef RDP_DEBUG
    if (rdp_dump)
		{
			char string[4000];
      int rdp_dasm(uint32_t * rdp_cmd_data, int rdp_cmd_cur, int length, char *buffer);
			rdp_dasm(rdp_cmd_data, rdp_cmd_cur, rdp_command_length[cmd], string);

			fprintf(stderr, "%08X: %08X %08X   %s\n", dp_start+(rdp_cmd_cur * 4), rdp_cmd_data[rdp_cmd_cur+0], rdp_cmd_data[rdp_cmd_cur+1], string);
		}
#endif

#ifdef RDP_DEBUG
    memcpy(rdpTraceBuf+rdpTracePos, rdp_cmd_data+rdp_cmd_cur, rdp_command_length[cmd]);
#endif

    if (rdp_cmd_cur + rdp_command_length[cmd]/4 > MAXCMD)
      memcpy(rdp_cmd_data + MAXCMD, rdp_cmd_data, rdp_command_length[cmd] - (MAXCMD - rdp_cmd_cur)*4);
    
		// execute the command
		rdp_command_table[cmd](rdp_cmd_data[rdp_cmd_cur+0], rdp_cmd_data[rdp_cmd_cur+1]);

#ifdef RDP_DEBUG
    rdpTracePos += rdp_command_length[cmd] / 4;
    rglAssert(rdpTracePos < sizeof(rdpTraceBuf)/sizeof(rdpTraceBuf[0]));
#endif

		rdp_cmd_cur = (rdp_cmd_cur + rdp_command_length[cmd] / 4) & (MAXCMD-1);
	}

// 	dp_current = dp_end;
// 	dp_start = dp_end;
	dp_start = dp_current;

  dp_status &= ~0x0002;
}

int rdp_store_list(void)
{
	uint32_t i;
	uint32_t data, cmd, length;
  int sync = 0;

//   while (dp_current < dp_end) {
    
//   }
//   dp_status &= ~0x0002;

	length = dp_end - dp_current;

//   LOG("rdp start %x cur %x end %x length %d dp_status %x\n",
//       dp_start, dp_current, dp_end,
//       length, dp_status);

  if (dp_end <= dp_current) {
    return 0;
  }

	// load command data
	for (i=0; i < length; i += 4)
	{
    data = READ_RDP_DATA(dp_current + i);
    if (rglSettings.async) {
      if (rdp_cmd_left) {
        rdp_cmd_left--;
      } else {
        cmd = (data >> 24) & 0x3f;
        rdp_cmd_left = rdp_command_length[cmd]/4-1;
        if (cmd == 0x29) // full_sync
          sync = 1;
      }
    }
    rdp_cmd_data[rdp_cmd_ptr] = data;
    rdp_cmd_ptr = (rdp_cmd_ptr + 1) & (MAXCMD-1);
	}

	dp_current += length;

  return sync;
}


int rdp_init()
{
  rdp_cmd_cur = rdp_cmd_ptr = 0;
  rdp_cmd_left = 0;
#ifdef RDP_DEBUG
  rdpTracePos = 0;
#endif
  nbTmemAreas = 0;
  return rglInit();
}

